#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver driver = Adafruit_PWMServoDriver();

#define SERVOMIN 150
#define SERVOMAX 600
#define USMIN 600
#define USMAX 2400
#define SERVO_FREQ 50

//servo counter pins
uint8_t RightHip = 0;
uint8_t LeftHip = 1;
uint8_t RightFoot = 2;
uint8_t LeftFoot = 15;

//pot pins
int pot1 = A0;
int pot2 = A1;
int pot3 = A2;
int pot4 = A3;

const int NUM_POTS = 4;

//starting servo positions
int RHIP_START_POS = 260;
int RHIP_MAX = 300;
int RHIP_MIN = 200;

int LHIP_START_POS = 340;
int LHIP_MAX = 400;
int LHIP_MIN = 275;

int RFOOT_START_POS = 350;
int RFOOT_MAX = 400;
int RFOOT_MIN = 250;

int LFOOT_START_POS = 320;
int LFOOT_MAX = 400;
int LFOOT_MIN = 250;

int currMap;
int reading, num;
int readings[NUM_POTS], servoPos[NUM_POTS], newPos[NUM_POTS], lastPos[NUM_POTS];

void readPot(void) {
  //int curr;
  reading = analogRead(pot1);
  readings[0] = min(reading, 100);
  reading = analogRead(pot2);
  readings[1] = min(reading, 100);
  reading = analogRead(pot3);
  readings[2] = min(reading, 100);
  reading = analogRead(pot4);
  readings[3] = min(reading, 100);

  //curr = reading;
  //return curr;
}

void mapServo(int pos[]) {

  //int mappings[4];
  servoPos[0] = map(pos[0], 0, 200, RHIP_MIN, RHIP_MAX);
  servoPos[1] = map(pos[1], 0, 200, LHIP_MIN, LHIP_MAX);
  servoPos[2] = map(pos[2], 0, 200, RFOOT_MIN, RFOOT_MAX);
  servoPos[3] = map(pos[3], 0, 200, LFOOT_MIN, LFOOT_MAX);

  //return mappings;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("INITIALIZE START ROBOT");

  driver.begin();
  driver.setOscillatorFrequency(27000000);
  driver.setPWMFreq(SERVO_FREQ);

  delay(10);

  uint8_t pulselen = 0;
  int average = ((SERVOMAX + SERVOMIN) / 2);

  driver.setPWM(0, 0, RHIP_START_POS);
  delay(500);
  driver.setPWM(1, 0, LHIP_START_POS);
  delay(500);
  driver.setPWM(2, 0, RFOOT_START_POS);
  delay(500);
  driver.setPWM(15, 0, LFOOT_START_POS);
  delay(500);

  //  for (uint16_t pulselen = 200; pulselen < 350; pulselen++) {
  //    driver.setPWM(2, 0, pulselen);
  //    delay(10);
  //  }

  delay(500);

  //POTENTIOMETER TEST DATA HERE
  pinMode(pot1, INPUT);
  readPot();
  mapServo(readings);

  for(int n = 0; n < 4; n++){
    lastPos[n] = servoPos[n];
  }
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print("DRIVING");

  readPot();
  mapServo(readings);
  //servoPos = mapServo(readings);
  delay(200);

  for (int k = 0; k < NUM_POTS; k++) {
    Serial.print("POT POS: "); Serial.println(readings[k]);
    Serial.print("SERVO POS: "); Serial.println(servoPos[k]);
    if (servoPos[k] - lastPos[k] > 0) {
      for (uint16_t pulselen = lastPos[k]; pulselen < servoPos[k]; pulselen++) {
        driver.setPWM(k, 0, pulselen);
        delay(10);
      }
    }
    else if (servoPos[k] - lastPos[k] < 0) {
      for (uint16_t pulselen = lastPos[k]; pulselen > servoPos[k]; pulselen--) {
        driver.setPWM(k, 0, pulselen);
        delay(10);
      }
    }
  }

  for(int n = 0; n < 4; n++){
    lastPos[n] = servoPos[n];
  }
}
