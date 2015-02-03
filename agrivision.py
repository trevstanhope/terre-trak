"""
Agri-Vision
McGill University, Department of Bioresource Engineering
"""

__author__ = 'Trevor Stanhope'
__version__ = '2.0.'

## Libraries
import cv2, cv
import serial
import pymongo
from bson import json_util
from pymongo import MongoClient
import json
import numpy
from matplotlib import pyplot as plt
import thread
import gps
import time 
import sys
from datetime import datetime
import ast

## Constants
try:
    CONFIG_FILE = 'modes/%s.json' % sys.argv[1]
except Exception as err:
    CONFIG_FILE = 'modes/default.json'

def pretty_print(task, msg, *args):
    date = datetime.strftime(datetime.now(), "%Y-%m-%d %H:%M:%S.%f")
    print "%s %s %s" % (date, task, msg)

## Class
class Cultivator:
    def __init__(self, config_file):

        # Load Config
        pretty_print("CONFIG", "Loading %s" % config_file)
        self.config = json.loads(open(config_file).read())
        for key in self.config:
            try:
                getattr(self, key)
            except AttributeError as error:
                setattr(self, key, self.config[key])
        
        # Initializers
        self.init_cameras()
        self.init_arduino()
        self.init_pid()
        self.init_db()
        self.init_gps()
        self.init_display()
        
    # Initialize Cameras
    def init_cameras(self):
        if self.VERBOSE:
            pretty_print('CAMERA', 'Initialing Cameras')
            pretty_print('CAMERA', 'Camera Height: %d px' % self.CAMERA_HEIGHT)
            pretty_print('CAMERA', 'Camera Depth: %d cm' % self.CAMERA_DEPTH)
            pretty_print('CAMERA', 'Camera FOV: %f rad' % self.CAMERA_FOV)
        self.CAMERA_CENTER = self.CAMERA_WIDTH / 2
        if self.VERBOSE: 
            pretty_print('INIT', 'Image Center: %d px' % self.CAMERA_CENTER)
        self.GROUND_WIDTH = 2 * self.CAMERA_DEPTH * numpy.tan(self.CAMERA_FOV / 2.0)
        pretty_print('CAMERA', 'Ground Width: %d cm' % self.GROUND_WIDTH)
        pretty_print('CAMERA', 'Brush Range: +/- %d cm' % self.BRUSH_RANGE)
        self.PIXEL_PER_CM = self.CAMERA_WIDTH / self.GROUND_WIDTH
        pretty_print('CAMERA', 'Pixel-per-cm: %d px/cm' % self.PIXEL_PER_CM)
        self.PIXEL_RANGE = int(self.PIXEL_PER_CM * self.BRUSH_RANGE) 
        pretty_print('CAMERA', 'Pixel Range: +/- %d px' % self.PIXEL_RANGE)
        self.PIXEL_MIN = self.CAMERA_CENTER - self.PIXEL_RANGE
        self.PIXEL_MAX = self.CAMERA_CENTER + self.PIXEL_RANGE
        self.cameras = []
        for i in self.CAMERAS:
            if self.VERBOSE: pretty_print('CAMERA', 'Initializing Camera: %d' % i)
            cam = cv2.VideoCapture(i)
            cam.set(cv.CV_CAP_PROP_FRAME_WIDTH, self.CAMERA_WIDTH)
            cam.set(cv.CV_CAP_PROP_FRAME_HEIGHT, self.CAMERA_HEIGHT)
            self.cameras.append(cam)
    
    # Initialize Database
    def init_db(self):
        self.LOG_NAME = datetime.strftime(datetime.now(), self.LOG_FORMAT)
        self.MONGO_NAME = datetime.strftime(datetime.now(), self.MONGO_FORMAT)
        if self.VERBOSE: pretty_print('DB', 'Initializing MongoDB')
        if self.VERBOSE: pretty_print('DB', 'Connecting to MongoDB: %s' % self.MONGO_NAME)
        if self.VERBOSE: pretty_print('DB', 'New session: %s' % self.LOG_NAME)
        try:
            self.client = MongoClient()
            self.database = self.client[self.MONGO_NAME]
            self.collection = self.database[self.LOG_NAME]
            self.log = open('logs/' + self.LOG_NAME + '.csv', 'w')
            self.log.write(','.join(['time', 'lat', 'long', 'speed', 'cam0', 'cam1', 'estimate', 'average', 'pwm','\n']))
        except Exception as error:
            pretty_print('\tERROR', str(error))
    
    # Initialize PID Controller
    def init_pid(self):
        if self.VERBOSE: pretty_print('PID', 'Initialing Electro-Hydraulics')
        self.MIN_PWM = 0
        if self.VERBOSE: pretty_print('PID', 'PWM Minimum: %d' % self.MIN_PWM)
        self.MAX_PWM = 255
        if self.VERBOSE: pretty_print('PID', 'PWM Maximum: %d' % self.MAX_PWM)
        self.CENTER_PWM = int(self.MIN_PWM + self.MAX_PWM / 2.0)
        if self.VERBOSE: pretty_print('PID', 'PWM Center: %d' % self.CENTER_PWM)
        if self.VERBOSE: pretty_print('PID', 'Default Number of Averages: %d' % self.NUM_AVERAGES)
        self.offset_history = [self.CAMERA_CENTER] * self.NUM_AVERAGES
    
    # Initialize Arduino
    def init_arduino(self):
        if self.VERBOSE: pretty_print('ARDUINO', 'Initializing Arduino')
        try:
            if self.VERBOSE: pretty_print('ARDUINO', 'Device: %s' % str(self.SERIAL_DEVICE))
            if self.VERBOSE: pretty_print('ARDUINO', 'Baud Rate: %s' % str(self.SERIAL_BAUD))
            self.arduino = serial.Serial(self.SERIAL_DEVICE, self.SERIAL_BAUD)
        except Exception as error:
            if self.VERBOSE: pretty_print('ERROR', str(error))
    
    # Initialize GPS
    def init_gps(self):
        if self.VERBOSE: pretty_print('GPS', 'Initializing GPS')
        if self.GPS_ENABLED:
            try:
                if self.VERBOSE: pretty_print('GPS', 'WARNING: Enabing GPS')
                self.gpsd = gps.gps()
                self.gpsd.stream(gps.WATCH_ENABLE)
                thread.start_new_thread(self.update_gps, ())
            except Exception as err:
                pretty_print('ERROR', 'GPS not available! %s' % str(err))
                self.latitude = 0
                self.longitude = 0
                self.speed = 0
        else:
            pretty_print('GPS', 'WARNING: GPS Disabled')
            self.latitude = 0
            self.longitude = 0
            self.speed = 0
    
    # Display
    def init_display(self):
        if self.VERBOSE: pretty_print('INIT', 'Initializing Display')
        if self.DISPLAY_ON:
            thread.start_new_thread(self.update_display, ())

    ## Capture Images
    """
    1. Attempt to capture an image
    2. Repeat for each capture interface
    """
    def capture_images(self):
        if self.VERBOSE: pretty_print('CAMERA', 'Capturing Images')
        images = []
        for cam in self.cameras:
            if self.VERBOSE: pretty_print('CAMERA', 'Camera ID: %s' % str(cam))
            (s, bgr) = cam.read() 
            if s:
                images.append(bgr)
        if self.VERBOSE: pretty_print('CAMERA', 'Images captured: %d' % len(images))
        return images
        
    ## Plant Segmentation Filter
    """
    1. RBG --> HSV
    2. Set minimum saturation equal to the mean saturation
    3. Set minimum value equal to the mean value
    4. Take hues within range from green-yellow to green-blue
    """
    def plant_filter(self, images):
        masks = []
        for bgr in images:
            try:
                hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
                hue_min = self.HUE_MIN # yellowish
                hue_max = self.HUE_MAX # bluish
                sat_min = hsv[:,:,1].mean() # cutoff for how saturated the color must be
                sat_max = self.SAT_MAX
                val_min = hsv[:,:,2].mean()
                val_max = self.VAL_MAX
                threshold_min = numpy.array([hue_min, sat_min, val_min], numpy.uint8)
                threshold_max = numpy.array([hue_max, sat_max, val_max], numpy.uint8)
                mask = cv2.inRange(hsv, threshold_min, threshold_max)
                masks.append(mask) 
            except Exception as error:
                pretty_print('CV', str(error))        
        if self.VERBOSE: pretty_print('CV', 'Number of Masks: %d mask(s) ' % len(masks))
        return masks
        
    ## Find Plants
    """
    1. Calculates the column summation of the mask
    2. Calculates the 95th percentile threshold of the column sum array
    3. Finds indicies which are greater than or equal to the threshold
    4. Finds the median of this array of indices
    5. Repeat for each mask
    """
    def find_indices(self, masks):
        indices = []
        for mask in masks:
            try:
                column_sum = mask.sum(axis=0) # vertical summation
                threshold = numpy.percentile(column_sum, self.THRESHOLD_PERCENTILE)
                probable = numpy.nonzero(column_sum >= threshold) # returns 1 length tuble
                num_probable = len(probable[0])
                centroid = int(numpy.median(probable[0]))
                indices.append(centroid)
            except Exception as error:
                pretty_print('ERROR', '%s' % str(error))
        if self.VERBOSE: pretty_print('CV', 'Detected Indices : %s' % str(indices))
        return indices
        
    ## Best Guess for row based on multiple offsets from indices
    """
    1. If outside bounds, default to edges
    2. If inside, use mean of detected indices from both cameras
    1. Takes the current assumed offset and number of averages
    2. Calculate weights of previous offsets
    3. Estimate the weighted position of the crop row (in pixels)
    """
    def estimate_row(self, indices):
        try:
            estimated =  int(numpy.mean(indices))
        except Exception as error:
            pretty_print('ERROR', str(error))
            estimated = self.CAMERA_CENTER
        if self.VERBOSE: pretty_print('ROW', 'Estimated Offset: %s' % str(estimated))
        self.offset_history.append(estimated)
        while len(self.offset_history) > self.NUM_AVERAGES:
            self.offset_history.pop(0)
        average = int(numpy.mean(self.offset_history)) #!TODO
        if self.VERBOSE: pretty_print('ROW', 'Moving Average: %s' % str(average)) 
        differential = estimated - average
        if self.VERBOSE: pretty_print('ROW', 'Differential : %s' % str(differential)) 
        return estimated, average, differential
         
    ## Control Hydraulics
    """
    1. Get PWM response corresponding to average offset
    2. Send PWM response over serial to controller
    """
    def control_hydraulics(self, estimate, average):
        p = (estimate - self.CAMERA_CENTER) * self.P_COEF
        i = (average - self.CAMERA_CENTER) * self.I_COEF
        d = (0)  * self.D_COEF
        if self.VERBOSE: pretty_print('PID', str([p,i,d]))
        pwm = int(p + i + d + self.CENTER_PWM)
        if pwm > self.MAX_PWM:
            pwm = self.MAX_PWM
        elif pwm < self.MIN_PWM:
            pwm = self.MIN_PWM
        try:
            self.arduino.write(str(pwm) + '\n')
        except Exception as error:
            pretty_print('ERROR', str(error))
        if self.VERBOSE: pretty_print('ARDUINO', 'PWM Output: %s' % str(pwm))
        return pwm
    
    ## Log to Mongo
    """
    1. Log results to the database
    2. Returns Doc ID
    """
    def log_db(self, sample):
        if self.VERBOSE: pretty_print('DB', 'Logging to Database')
        try:          
            doc_id = self.collection.insert(sample)
        except Exception as error:
            pretty_print('ERROR', str(error))
        if self.VERBOSE: pretty_print('DB', 'Doc ID: %s' % str(doc_id))
        return doc_id
    
    ## Log to File
    """
    1. Open new text file
    2. For each document in session, print parameters to file
    """
    def log_file(self, sample):
        if self.VERBOSE: pretty_print('LOG', 'Logging to File')
        try:
            time = str(sample['time'])
            latitude = str(sample['lat'])
            longitude = str(sample['long'])
            speed = str(sample['speed'])
            cam0 = str(sample['cam0'])
            cam1 = str(sample['cam1'])
            estimate = str(sample['estimate'])
            average = str(sample['average'])
            pwm = str(sample['pwm'])
            self.log.write(','.join([time, latitude, longitude, speed, cam0, cam1, estimate, average, pwm,'\n']))
        except Exception as error:
            pretty_print('ERROR', str(error))
                
    ## Displays 
    """
    1. Draw lines on RGB images
    2. Draw lines on EGI images (the masks)
    3. Output GUI display
    """
    def update_display(self):
        while True:
            if self.VERBOSE: pretty_print('DISPLAY', 'Displaying Images')
            try:
                pwm = self.pwm
                average = self.average
                estimated = self.estimated
                masks = self.masks
                images = self.images
                cam0 = self.cam0
                cam1 = self.cam1
                output_images = []
                distance = round((average - self.CAMERA_CENTER) / float(self.PIXEL_PER_CM), 1)
                volts = round((pwm * (self.MAX_VOLTAGE - self.MIN_VOLTAGE) / (self.MAX_PWM - self.MIN_PWM) + self.MIN_VOLTAGE), 2)
                blank = numpy.zeros((self.CAMERA_HEIGHT, self.CAMERA_WIDTH), numpy.uint8)
                for img,mask in zip(images, masks):
                    if True: img = numpy.dstack([mask, mask, mask])
                    cv2.line(img, (self.PIXEL_MIN, 0), (self.PIXEL_MIN, self.CAMERA_HEIGHT), (0,0,255), 1)
                    cv2.line(img, (self.PIXEL_MAX, 0), (self.PIXEL_MAX, self.CAMERA_HEIGHT), (0,0,255), 1)
                    cv2.line(img, (average, 0), (average, self.CAMERA_HEIGHT), (0,255,0), 2)
                    cv2.line(img, (self.CAMERA_CENTER, 0), (self.CAMERA_CENTER, self.CAMERA_HEIGHT), (255,255,255), 1)
                    output_images.append(numpy.vstack([img, numpy.zeros((20, self.CAMERA_WIDTH, 3), numpy.uint8)])) #add blank space
                output_small = numpy.hstack(output_images)
                output_large = cv2.resize(output_small, (1024, 768))
                if average - self.CAMERA_CENTER >= 0:
                    distance_str = str("+%2.1f cm" % distance)
                elif average - self.CAMERA_CENTER< 0:
                    distance_str = str("%2.1f cm" % distance)
                volts_str = str("%2.1f V" % volts)
                cv2.putText(output_large, distance_str, (340,760), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 4)
                cv2.putText(output_large, volts_str, (840,760), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 4)
                cv2.namedWindow('Agri-Vision', cv2.WINDOW_NORMAL)
                if self.FULLSCREEN: cv2.setWindowProperty('Agri-Vision', cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
                cv2.imshow('Agri-Vision', output_large)
                if cv2.waitKey(5) == 3:
                    pass
            except Exception as error:
                pretty_print('DISPLAY', str(error))
                    
    ## Update GPS
    """
    1. Get the most recent GPS data
    2. Set global variables for lat, long and speed
    """
    def update_gps(self):  
        while True:
            self.gpsd.next()
            self.latitude = self.gpsd.fix.latitude
            self.longitude = self.gpsd.fix.longitude
            self.speed = self.gpsd.fix.speed
            pretty_print('GPS', '%d N %d E' % (self.latitude, self.longitude))
    
    ## Close
    """
    Function to shutdown application safely
    1. Close windows
    2. Disable arduino
    3. Release capture interfaces 
    """
    def close(self):
        if self.VERBOSE: pretty_print('SYSTEM', 'Shutting Down')
        try:
            if self.VERBOSE: pretty_print('ARDUINO', 'Closing Arduino')
            self.arduino.close() ## Disable arduino
        except Exception as error:
            pretty_print('ARDUINO', str(error))
        for i in range(len(self.cameras)):
            try:
                if self.VERBOSE: pretty_print('CAMERA', 'Closing Camera #%d' % i)
                self.cameras[i].release() ## Disable cameras
            except Exception as error:
                pretty_print('\tCAMERA ERROR', str(error))
        cv2.destroyAllWindows() ## Close windows
        
    ## Run  
    """
    Function for Run-time loop
    1. Get initial time
    2. Capture images
    3. Generate mask filter for plant matter
    4. Calculate indices of rows
    5. Estimate row from both images
    6. Get number of averages
    7. Calculate moving average
    8. Send PWM response to arduino
    9. Throttle to desired frequency
    10. Log results to DB
    11. Display results
    """     
    def run(self):
        while True:
            try:
                images = self.capture_images()
                masks = self.plant_filter(images)
                indices = self.find_indices(masks)
                (estimated, average, differential) = self.estimate_row(indices)
                pwm = self.control_hydraulics(estimated, average)
                try:
                    cam0 = indices[0]
                except Exception:
                    cam0 = self.CAMERA_CENTER
                try:
                    cam1 = indices[1]
                except Exception:
                    cam1 = self.CAMERA_CENTER
                sample = {
                    'cam0' : cam0 - self.CAMERA_CENTER, 
                    'cam1' : cam1 - self.CAMERA_CENTER, 
                    'estimate' : estimated - self.CAMERA_CENTER,
                    'average' : average - self.CAMERA_CENTER,
                    'pwm': pwm,
                    'time' : datetime.strftime(datetime.now(), self.TIME_FORMAT),
                    'long' : self.longitude,
                    'lat' : self.latitude,
                    'speed' : self.speed,
                }
                self.pwm = pwm
                self.images = images
                self.masks = masks
                self.average = average
                self.estimated = estimated
                self.cam0 = cam0
                self.cam1 = cam1
                if self.MONGO_ON: doc_id = self.log_db(sample)
                if self.LOGFILE_ON: self.log_file(sample)
            except KeyboardInterrupt as error:
                self.close()    
                break
    
## Main
if __name__ == '__main__':
    session = Cultivator(CONFIG_FILE)
    session.run()
