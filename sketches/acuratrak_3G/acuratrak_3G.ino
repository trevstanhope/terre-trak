/*  
  SunCo AcuTrak Adaptor (Version 1)
  Electro-Hydraulic PWM Controller
  Developed by Trevor Stanhope
  Receives serial commands to adjust the hydraulics or run calibration sweep
  
  Calibration Momentary Switch Wiring:
  VCC --> 20KOhm Internal Pullup Resistor --> Input Pin
*/

/* --- Pin Definitions --- */
#define CALIBRATE_PIN 13
#define OUTPUT_PIN 5

/* --- Constants --- */ 
const unsigned long BAUD = 9600;
const unsigned int RESOLUTION = 255;
const unsigned int PWM_MIN = 93; // 1.25 V + 4 bits = 1.33 V
const unsigned int PWM_MAX = 169; // 3.75 V - 4 bits = 1.69 V
const unsigned int CALIBRATION_DELAY = 5000; // ms for each voltage interval
const unsigned int PWM_CENTER = (PWM_MAX + PWM_MIN) / 2; // start at zero of effective range from PWM_MIN to PWM_MAX

/* --- Variables --- */
int duty_cycle = PWM_CENTER;

/* --- Setup --- */
void setup(void) {
    pinMode(OUTPUT_PIN, OUTPUT);
    pinMode(CALIBRATE_PIN, INPUT); digitalWrite(CALIBRATE_PIN, HIGH);
    Serial.begin(BAUD);
}

/* --- Loop --- */
void loop(void) {
  
  // Parse serial input
  int val = Serial.parseInt();
  if (val == 0) { val = PWM_CENTER; }
  
  // Run calibrate sequence if the blue button is pressed
  if (!digitalRead(CALIBRATE_PIN)) {
    analogWrite(OUTPUT_PIN, PWM_MIN);
    delay(CALIBRATION_DELAY);
    analogWrite(OUTPUT_PIN, PWM_MAX);
    delay(CALIBRATION_DELAY);
    duty_cycle = PWM_CENTER;
  }
  
  // Otherwise use convert the value read over serial to PWM range 
  else {
    duty_cycle = map(val, 1, RESOLUTION, PWM_MIN, PWM_MAX);
    if (duty_cycle > PWM_MAX) { duty_cycle = PWM_MAX; } // over-limit
    if (duty_cycle < PWM_MIN) { duty_cycle = PWM_MIN; } // under-limit (also handles no match
    analogWrite(OUTPUT_PIN, duty_cycle); // write to pwm output
  }
  Serial.println(duty_cycle);
  Serial.flush();
}
