/*
  SunCo AcuTrak Adaptor
  Electro-Hydraulic PWM Controller
  Developed by Trevor Stanhope
  Receives serial commands to adjust the hydraulics
*/

/* --- Pin Definitions --- */
#define CALIBRATE_POUT 3
#define CALIBRATE_PIN 4
#define OUTPUT_PIN 5

/* --- Constants --- */
const unsigned long BAUD = 9600;
const unsigned int RESOLUTION = 255;
const unsigned int PWM_MIN = 69; // 1.25 V
const unsigned int PWM_MAX = 211; // 3.75 V
const unsigned int CALIBRATION_INTERVAL = 1000;

/* --- Variables --- */
int DUTY = (PWM_MAX + PWM_MIN) / 2; // start at zero of effective range from PWM_MIN to PWM_MAX

/* --- Setup --- */
void setup(void) {
    pinMode(OUTPUT_PIN, OUTPUT);
    pinMode(CALIBRATE_POUT, OUTPUT); digitalWrite(CALIBRATE_POUT, LOW);
    pinMode(CALIBRATE_PIN, INPUT); digitalWrite(CALIBRATE_PIN, HIGH);
    Serial.begin(BAUD);
}

/* --- Loop --- */
void loop(void) {
  int val = Serial.parseInt();
  if (!digitalRead(CALIBRATE_PIN)) {
    analogWrite(OUTPUT_PIN, PWM_MIN);
    delay(CALIBRATION_INTERVAL);
    analogWrite(OUTPUT_PIN, PWM_MAX);
    delay(CALIBRATION_INTERVAL);
  }
  DUTY = map(val, 0, RESOLUTION, PWM_MIN, PWM_MAX); // map input resolution to pwm output range
  analogWrite(OUTPUT_PIN, DUTY);
}
