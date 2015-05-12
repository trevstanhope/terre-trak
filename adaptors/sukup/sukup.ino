/*
  Sukup Auto-Guide Adaptor
  Electro-Hydraulic PWM Controller
  Developed by Trevor Stanhope
  Receives serial commands to adjust the hydraulics with user-inputted sensitivity settings
*/

/* --- Definitions --- */
#define REFERENCE_PIN 5
#define CONTROL_PIN 6 // 255 corresponds to reaction at max negative offset

/* --- Constants --- */
const unsigned long BAUD = 9600;
const unsigned int PWM_MIN = 0; // AnalogWrite(0) == 0% PWM
const unsigned int PWM_MAX = 210; // AnalogWrite(255) == 100% PWM

/* --- Variables --- */
int DUTY = 127; // effective range of 0 to 255
int PWM_CONTROL = 127; // neutral at 127
int PWM_REFERENCE = PWM_MAX; // neutral at 127

void setup(void) {
    pinMode(CONTROL_PIN, OUTPUT);
    pinMode(REFERENCE_PIN, OUTPUT);
    Serial.begin(BAUD);
}

void loop(void) {
    int val = Serial.parseInt();
    PWM_CONTROL = map(val, 0, 255, PWM_MIN, PWM_MAX);
    PWM_REFERENCE = PWM_MAX; 
    analogWrite(CONTROL_PIN, PWM_CONTROL);
    analogWrite(REFERENCE_PIN, PWM_REFERENCE);
}
