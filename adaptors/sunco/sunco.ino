/*
  SunCo AcuTrak Adaptor
  Electro-Hydraulic PWM Controller
  Developed by Trevor Stanhope
  Receives serial commands to adjust the hydraulics or run calibration sweep
  
  Calibration Momentary Switch Wiring:
  VCC --> 20KOhm Internal Pullup Resistor --> Input Pin
  
*/

/* --- Pin Definitions --- */
#define CALIBRATE_PIN 4
#define OUTPUT_PIN 5

/* --- Constants --- */
const unsigned long BAUD = 9600;
const unsigned int RESOLUTION = 255;
const unsigned int PWM_MIN = 69; // 1.25 V
const unsigned int PWM_MAX = 211; // 3.75 V
const unsigned int CALIBRATION_DELAY = 10; // ms for each voltage interval

/* --- Variables --- */
int duty = (PWM_MAX + PWM_MIN) / 2; // start at zero of effective range from PWM_MIN to PWM_MAX

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
  
  // Run calibrate sequence if the blue button is pressed
  if (!digitalRead(CALIBRATE_PIN)) {
    for (int i = PWM_MIN; i < PWM_MAX; i++) {
      analogWrite(OUTPUT_PIN, i);
      delay(CALIBRATION_DELAY);
    }
    for (int i = PWM_MAX; i > PWM_MIN; i--) {
      analogWrite(OUTPUT_PIN, i);
      delay(CALIBRATION_DELAY);
    }
  }
  
  // Otherwise use convert the value read over serial to PWM range 
  else {
    duty = map(val, 0, RESOLUTION, PWM_MIN, PWM_MAX);
    if (duty > PWM_MAX) { duty = PWM_MAX; } // over-limit
    if (duty < PWM_MIN) { duty = PWM_MIN; } // under-limit (also handles no match
    analogWrite(OUTPUT_PIN, duty); // write to pwm output
  }
}
