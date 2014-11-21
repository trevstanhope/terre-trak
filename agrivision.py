"""
AutoTill 2.0
McGill University, Department of Bioresource Engineering
"""

__author__ = 'Trevor Stanhope'
__version__ = '2.0.'

## Libraries
import cv2, cv
import serial # Electro-hydraulic controller
import pymongo # DB
from bson import json_util # DB
from pymongo import MongoClient # DB
import json
import numpy # Curve
from matplotlib import pyplot as plt # Display
import thread # GPS
import gps # GPS
import time 
import sys
from datetime import datetime
import ast

## Constants
try:
    CONFIG_FILE = sys.argv[1]
except Exception as err:
    CONFIG_FILE = 'settings.json'

## Class
class Cultivator:
    def __init__(self, config_file):

        # Load Config
        print('\tLoading config file: %s' % config_file)
        self.config = json.loads(open(config_file).read())
        for key in self.config:
            try:
                getattr(self, key)
            except AttributeError as error:
                setattr(self, key, self.config[key])
        
        # Cameras
        if self.VERBOSE: print('[Initialing Cameras] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        print('\tImage Width: %d px' % self.PIXEL_WIDTH)
        print('\tImage Height: %d px' % self.PIXEL_HEIGHT)
        print('\tCamera Height: %d cm' % self.CAMERA_HEIGHT)
        print('\tCamera FOV: %f rad' % self.CAMERA_FOV)
        self.PIXEL_CENTER = self.PIXEL_WIDTH / 2
        if self.VERBOSE: print('\tImage Center: %d px' % self.PIXEL_CENTER)
        self.GROUND_WIDTH = 2 * self.CAMERA_HEIGHT * numpy.tan(self.CAMERA_FOV / 2.0)
        print('\tGround Width: %d cm' % self.GROUND_WIDTH)
        print('\tBrush Range: +/- %d cm' % self.BRUSH_RANGE)
        self.PIXEL_PER_CM = self.PIXEL_WIDTH / self.GROUND_WIDTH
        print('\tPixel-per-cm: %d px/cm' % self.PIXEL_PER_CM)
        self.PIXEL_RANGE = int(self.PIXEL_PER_CM * self.BRUSH_RANGE) 
        print('\tPixel Range: +/- %d px' % self.PIXEL_RANGE)
        self.PIXEL_MIN = self.PIXEL_CENTER - self.PIXEL_RANGE
        self.PIXEL_MAX = self.PIXEL_CENTER + self.PIXEL_RANGE
        self.cameras = []
        for i in self.CAMERAS:
            if self.VERBOSE: print('\tInitializing Camera: %d' % i)
            cam = cv2.VideoCapture(i)
            cam.set(cv.CV_CAP_PROP_FRAME_WIDTH, self.PIXEL_WIDTH)
            cam.set(cv.CV_CAP_PROP_FRAME_HEIGHT, self.PIXEL_HEIGHT)
            self.cameras.append(cam)
        
        # Initialize Database
        self.LOG_NAME = datetime.strftime(datetime.now(), self.LOG_FORMAT)
        self.MONGO_NAME = datetime.strftime(datetime.now(), self.MONGO_FORMAT)
        if self.VERBOSE: print('[Initialing MongoDB] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        if self.VERBOSE: print('\tConnecting to MongoDB: %s' % self.MONGO_NAME)
        if self.VERBOSE: print('\tNew session: %s' % self.LOG_NAME)
        try:
            self.client = MongoClient()
            self.database = self.client[self.MONGO_NAME]
            self.collection = self.database[self.LOG_NAME]
            self.log = open('logs/' + self.LOG_NAME + '.csv', 'w')
            self.log.write(','.join(['time', 'lat', 'long', 'speed', 'cam0', 'cam1', 'estimate', 'average', 'pwm','\n']))
        except Exception as error:
            print('\tERROR in __init__(): %s' % str(error))
        
        # PWM Response Range for Electrohyraulic-Control
        if self.VERBOSE: print('[Initialing Electro-Hydraulics] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        self.MIN_PWM = int(255 * self.MIN_VOLTAGE / self.SUPPLY_VOLTAGE)
        if self.VERBOSE: print('\tPWM at Minimum: %d' % self.MIN_PWM)
        self.MAX_PWM = int(255 * self.MAX_VOLTAGE / self.SUPPLY_VOLTAGE)
        if self.VERBOSE: print('\tPWM at Maximum: %d' % self.MAX_PWM)
        self.CENTER_PWM = int(self.MIN_PWM + self.MAX_PWM / 2.0)
        if self.VERBOSE: print('\tPWM at Center: %d' % self.CENTER_PWM)
        self.RANGE_PWM = int(self.MAX_PWM - self.MIN_PWM)
        if self.VERBOSE: print('\tRange PWM: %d' % self.RANGE_PWM)
        self.PWM_RESPONSE = []
        for i in range(self.PIXEL_WIDTH):
            if i <= (self.PIXEL_CENTER - self.PIXEL_RANGE):
                val = self.MAX_PWM
            elif i >= (self.PIXEL_CENTER + self.PIXEL_RANGE):
                val = self.MIN_PWM
            else:
                val = int(self.MAX_PWM - (self.CENTER_PWM + float(self.RANGE_PWM) * (i - self.PIXEL_CENTER - 2) / float(2 * self.PIXEL_RANGE)))
            self.PWM_RESPONSE.append(val)
        if self.VERBOSE: print(self.PWM_RESPONSE)
    
        # Offset History
        if self.VERBOSE: print('\tDefault Number of Averages: %d' % self.NUM_AVERAGES)
        self.offset_history = [self.PIXEL_CENTER] * self.NUM_AVERAGES
        
        # Arduino Connection
        if self.VERBOSE: print('[Initializing Arduino] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        try:
            if self.VERBOSE: print('\tDevice: %s' % str(self.SERIAL_DEVICE))
            if self.VERBOSE: print('\tBaud Rate: %s' % str(self.SERIAL_BAUD))
            self.arduino = serial.Serial(self.SERIAL_DEVICE, self.SERIAL_BAUD)
        except Exception as error:
            print('\tERROR in __init__(): %s' % str(error))
        
        # GPS
        if self.VERBOSE: print('[Initializing GPS] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        if self.GPS_ENABLED:
            try:
                if self.VERBOSE: print('\tWARNING: Enabing GPS')
                self.gpsd = gps.gps()
                self.gpsd.stream(gps.WATCH_ENABLE)
                thread.start_new_thread(self.update_gps, ())
            except Exception as err:
                print('\tERROR in __init__(): GPS not available! %s' % str(err))
                self.latitude = 0
                self.longitude = 0
                self.speed = 0
        else:
            print('\tWARNING: GPS Disabled')
            self.latitude = 0
            self.longitude = 0
            self.speed = 0

        # Display
        if self.VERBOSE: print('[Initializing Display] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        if self.DISPLAY_ON:
            thread.start_new_thread(self.update_display, ())

    ## Capture Images
    """
    1. Attempt to capture an image
    2. Repeat for each capture interface
    """
    def capture_images(self):
        if self.VERBOSE: print('[Capturing Images] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        images = []
        for cam in self.cameras:
            if self.VERBOSE: print('\tCamera ID: %s' % str(cam))
            (s, bgr) = cam.read() 
            if s:
                images.append(bgr)
        if self.VERBOSE: print('\tImages captured: %d' % len(images))
        return images
        
    ## Green Filter
    """
    1. RBG --> HSV
    2. Set minimum saturation equal to the mean saturation
    3. Set minimum value equal to the mean value
    4. Take hues within range from green-yellow to green-blue
    """
    def plant_filter(self, images):
        if self.VERBOSE: print('[Filtering for Plants] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
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
                print('\tERROR in plant_filter(): %s' % str(error))        
        if self.VERBOSE: print('\tNumber of Masks: %d mask(s) ' % len(masks))
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
        if self.VERBOSE: print('[Finding Offsets] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
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
                print('\tERROR in find_indices(): %s' % str(error))
        if self.VERBOSE: print('\tDetected Indices: %s' % str(indices))
        return indices
        
    ## Best Guess for row based on multiple offsets from indices
    """
    1. If outside bounds, default to edges
    2. If inside, use mean of detected indices from both cameras
    """
    def estimate_row(self, indices):
        if self.VERBOSE: print('[Making Best Guess of Crop Row] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        try:
            estimated =  int(numpy.mean(indices))
        except Exception as error:
            print('\tERROR in estimate_row(): %s' % str(error))
            estimated = self.PIXEL_CENTER
        print('\tEstimated Offset: %s' % str(estimated))
        return estimated
        
    ## Estimate Average Position
    """
    1. Takes the current assumed offset and number of averages
    2. Calculate weights of previous offsets
    3. Estimate the weighted position of the crop row (in pixels)
    """
    def average_row(self, offset):
        if self.VERBOSE: print('[Estimating Row Position] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        self.offset_history.append(offset)
        while len(self.offset_history) > self.NUM_AVERAGES:
            self.offset_history.pop(0)
        average = int(numpy.mean(self.offset_history)) #!TODO
        print('\tMoving Average: %s' % str(average)) 
        return average
         
    ## Control Hydraulics
    """
    1. Get PWM response corresponding to average offset
    2. Send PWM response over serial to controller
    """
    def control_hydraulics(self, estimate, average):
        if self.VERBOSE: print('[Controlling Hydraulics] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        pwm_avg = self.PWM_RESPONSE[average]
        pwm_est = self.PWM_RESPONSE[estimate]
        pwm = int(self.CENTER_PWM + self.I_COEF * (pwm_avg - self.CENTER_PWM) + self.P_COEF * (pwm_est - self.CENTER_PWM))
        if pwm > self.MAX_PWM:
            pwm = self.MAX_PWM
        elif pwm < self.MIN_PWM:
            pwm = self.MIN_PWM
        try:
            self.arduino.write(str(pwm) + '\n')
        except Exception as error:
            print('\tERROR in control_hydraulics(): %s' % str(error))
        print('\tPWM Output: %s' % str(pwm))
        return pwm
    
    ## Logs to Mongo
    """
    1. Log results to the database
    2. Returns Doc ID
    """
    def log_db(self, sample):
        if self.VERBOSE: print('[Logging to Database] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        try:          
            doc_id = self.collection.insert(sample)
        except Exception as error:
            print('\tERROR in log_db(): %s' % str(error))
        if self.VERBOSE: print('\tDoc ID: %s' % str(doc_id))
        return doc_id
    
    ## Log to File
    """
    1. Open new text file
    2. For each document in session, print parameters to file
    """
    def log_file(self, sample):
        if self.VERBOSE: print('[Logging to File] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
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
            print('\tERROR: %s' % str(error))
                
    ## Displays 
    """
    1. Draw lines on RGB images
    2. Draw lines on EGI images (the masks)
    3. Output GUI display
    """
    def update_display(self):
	while True:
            if self.VERBOSE: print('[Displaying Images] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
            try:
                average = self.average
                estimated = self.estimated
                masks = self.masks
                images = self.images
                cam0 = self.cam0
                cam1 = self.cam1
                output_images = []
                for img,mask in zip(images, masks):
                    cv2.line(img, (self.PIXEL_MIN, 0), (self.PIXEL_MIN, self.PIXEL_HEIGHT), (0,0,255), 1)
                    cv2.line(img, (self.PIXEL_MAX, 0), (self.PIXEL_MAX, self.PIXEL_HEIGHT), (0,0,255), 1)
                    cv2.line(img, (average, 0), (average, self.PIXEL_HEIGHT), (0,255,0), 2)
                    cv2.line(img, (self.PIXEL_CENTER, 0), (self.PIXEL_CENTER, self.PIXEL_HEIGHT), (255,255,255), 1)
                    output_images.append(numpy.vstack([img, numpy.zeros((20, self.PIXEL_WIDTH, 3), numpy.uint8)]))
                output_small = numpy.hstack(output_images)
                output_large = cv2.resize(output_small, (1024, 768))
                if average - self.PIXEL_CENTER >= 0:
                    average_str = str("+%2.2f cm" % ((average - self.PIXEL_CENTER) / float(self.PIXEL_PER_CM)))
                elif average - self.PIXEL_CENTER< 0:
                    average_str = str("%2.2f cm" % ((average - self.PIXEL_CENTER) / float(self.PIXEL_PER_CM)))
                cv2.putText(output_large, average_str, (340,735), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 5)
                cv2.namedWindow('AutoTill', cv2.WINDOW_NORMAL)
                cv2.setWindowProperty('AutoTill', cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)
                cv2.imshow('AutoTill', output_large)
                if cv2.waitKey(5) == 3:
                    pass
            except Exception as error:
                print('\tERROR in display(): %s' % str(error))
                    
    ## Update GPS
    """
    1. Get the most recent GPS data
    2. Set global variables for lat, long and speed
    """
    def update_gps(self):  
        while True:
            print('[Updating GPS] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
            self.gpsd.next()
            self.latitude = self.gpsd.fix.latitude
            self.longitude = self.gpsd.fix.longitude
            self.speed = self.gpsd.fix.speed
    
    ## Close
    """
    Function to shutdown application safely
    1. Close windows
    2. Disable arduino
    3. Release capture interfaces 
    """
    def close(self):
        if self.VERBOSE: print('[Shutting Down] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        ## Close windows
        cv2.destroyAllWindows()
        ## Disable arduino
        try:
            if self.VERBOSE: print('\tClosing Arduino')
            self.arduino.close()
        except Exception as error:
            print('\tERROR in close(): %s' % str(error))
        ## Disable cameras
        for i in range(len(self.cameras)):
            try:
                if self.VERBOSE: print('\tClosing Camera #%d' % i)
                self.cameras[i].release()
            except Exception as error:
                print('\tERROR in close(): %s' % str(error))
                
    ## Throttle Frequency
    """
    1. While the frequency is less than the limit, wait
    """ 
    def throttle_frequency(self, start_time):
        if self.VERBOSE: print('[Throttling Frequency] %s' % datetime.strftime(datetime.now(), self.TIME_FORMAT))
        while (1 / (time.time() - start_time)) > self.FREQUENCY_LIMIT:
            time.sleep(0.01)
        frequency = 1/(time.time() - start_time)
        if self.VERBOSE: print('\tFrequency: ' + str(frequency))
        return frequency
        
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
                print('---------------------------------------')
                start_time = time.time()
                images = self.capture_images()
                masks = self.plant_filter(images)
                indices = self.find_indices(masks)
                estimated = self.estimate_row(indices)
                average = self.average_row(estimated)
                pwm = self.control_hydraulics(estimated, average)
                frequency = self.throttle_frequency(start_time)
                try:
                    cam0 = indices[0]
                except Exception:
                    cam0 = self.PIXEL_CENTER
                try:
                    cam1 = indices[1]
                except Exception:
                    cam1 = self.PIXEL_CENTER
                sample = {
                    'cam0' : cam0 - self.PIXEL_CENTER, 
                    'cam1' : cam1 - self.PIXEL_CENTER, 
                    'estimate' : estimated - self.PIXEL_CENTER,
                    'average' : average - self.PIXEL_CENTER,
                    'pwm': pwm,
                    'time' : datetime.strftime(datetime.now(), self.TIME_FORMAT),
                    'frequency' : frequency,
                    'long' : self.longitude,
                    'lat' : self.latitude,
                    'speed' : self.speed,
                }
                self.images = images
                self.masks = masks
                self.average = average
                self.estimated = estimated
                self.cam0 = cam0
                self.cam1 = cam1
                if self.MONGO_ON: doc_id = self.log_db(sample)
                if self.LOGFILE_ON: self.log_file(sample)
            except KeyboardInterrupt as error:
                break    
    
## Main
if __name__ == '__main__':
    session = Cultivator(CONFIG_FILE)
    session.run()
