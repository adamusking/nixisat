#include<Arduino.h>


const int NUM_MOTORS = 4;

const int escPins[NUM_MOTORS] = {16, 17, 18, 15};

const int escChannels[NUM_MOTORS] = {0, 1, 2, 3};


const int PWM_FREQ = 50;    
const int PWM_RES  = 12;       
const int PWM_MAX  = (1 << PWM_RES) - 1;

const int PULSE_MIN = 1000;
const int PULSE_MAX = 2000;  

// Convert microseconds to LEDC dutya
int pulseToDuty(int pulse_us) {
  // 50 Hz period = 20,000 us
  return map(pulse_us, 0, 20000, 0, PWM_MAX);
}

void setAllMotors(int pulse_us) {
  int duty = pulseToDuty(pulse_us);
  for (int i = 0; i < NUM_MOTORS; i++) {
    ledcWrite(escChannels[i], duty);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Initializing PWM channels...");

  for (int i = 0; i < NUM_MOTORS; i++) {
    ledcSetup(escChannels[i], PWM_FREQ, PWM_RES);
    ledcAttachPin(escPins[i], escChannels[i]);
  }

  Serial.println("Setting minimum throttle (arming)...");
  setAllMotors(PULSE_MIN);

  delay(5000);

  Serial.println("ESC should now be armed.");
}

void loop() {
  Serial.println("Ramping up throttle...");

  for (int pulse = PULSE_MIN; pulse <= 1300; pulse += 20) {
    setAllMotors(pulse);
    Serial.print("Pulse: ");
    Serial.println(pulse);
    delay(500);
  }

  Serial.println("Holding...");
  delay(5000);

  Serial.println("Returning to minimum throttle...");
  setAllMotors(PULSE_MIN);

  while (true) {
    delay(1000); // stop here
  }
}
