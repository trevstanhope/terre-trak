/*
  Agri-Vision
  SunCo Adaptor
  Electro-Hydraulic PWM Controller
  Developed by Trevor Stanhope
  Receives serial commands to adjust the hydraulics
*/

/* --- Definitions --- */
#define OUTPUT_PIN 5

/* --- Constants --- */
const unsigned long BAUD = 9600;
const unsigned int PWM_MIN = 1; // AnalogWrite(0) == 0% PWM
const unsigned int PWM_MAX = 256; // AnalogWrite(255) == 100% PWM

/* --- Variables --- */
int DUTY = PWM_MAX / 2; // start at zero of effective range from PWM_MIN to PWM_MAX

/* --- Setup --- */
void setup(void) {
    pinMode(OUTPUT_PIN, OUTPUT);
    Serial.begin(BAUD);
}

/* --- Loop --- */
void loop(void) {
  DUTY = Serial.parseInt();
  if (DUTY == 0) {
    DUTY = PWM_MAX / 2;
  }
  analogWrite(OUTPUT_PIN, DUTY - 1);
}
