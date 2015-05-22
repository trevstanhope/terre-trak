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
const unsigned int RESOLUTION = 255;
const unsigned int PWM_MIN = 0; // AnalogWrite(0) == 0% PWM
const unsigned int PWM_MAX = 210; // AnalogWrite(255) == 100% PWM

/* --- Variables --- */
int pwm_control = (PWM_MAX + PWM_MIN) / 2;
int pwm_reference = PWM_MAX;

void setup(void) {
    pinMode(CONTROL_PIN, OUTPUT);
    pinMode(REFERENCE_PIN, OUTPUT);
    Serial.begin(BAUD);
}

void loop(void) {
    int val = Serial.parseInt();
    pwm_control = map(val, 0, RESOLUTION, PWM_MIN, PWM_MAX);
    if (pwm_control > PWM_MAX) { pwm_control = PWM_MAX; }
    if (pwm_control < PWM_MIN) { pwm_control = PWM_MIN; }
    analogWrite(CONTROL_PIN, pwm_control);
    analogWrite(REFERENCE_PIN, pwm_reference);
}
