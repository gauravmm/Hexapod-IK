/*************************************************** 
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 16 servos, one after the other

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define NUM_PWM 2
#define NUM_SERVO_PER_PWM 16
#define NUM_SERVO 18

Adafruit_PWMServoDriver pwm[] = {Adafruit_PWMServoDriver(0x50), Adafruit_PWMServoDriver(0x41)};

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOENC(ms) ((int) ((float)(ms)*204.8))

// our servo # counter
uint8_t servonum = 0;
uint16_t initval [] = {225,204,163,194,174,163,204,194,194,215,245,215,221,260,205,215,245,215};
uint8_t mapping [] = {9, 10, 11, 3, 4, 5, 6, 7, 8, 16, 17, 18, 19, 20, 21, 22, 23, 24};
float ang2bits [] = {1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

void setServo(uint8_t servo, uint16_t val) {
  uint8_t mp = mapping[servo];
  pwm[mp / NUM_SERVO_PER_PWM].setPWM(mp % NUM_SERVO_PER_PWM, 0, val);
}

void servoAngle(uint8_t servo, float angle) {
  setServo(servo, (uint16_t) (ang2bits[servo] * angle + initval[servo]));
}

void servoAngleSet(int perturb []) {
  for(uint8_t i = 0; i < NUM_SERVO; ++i)
    servoAngle(i, perturb[i]);
}

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < NUM_PWM; i++) {
    pwm[i].begin();
    pwm[i].setPWMFreq(50);
  }
  
  for (uint16_t servonum = 0; servonum < NUM_SERVO; servonum++)
    setServo(servonum, initval[servonum]);
}

void loop() {
  // animate();
  changeAngle();
}

int num_frames = 7;
int frames[][NUM_SERVO] = {
//{0,  0,  0, 0,  0,  0, 0,  0,  0, 0,  0,  0, 0,  0,  0, 0,  0,  0},
  {0, 40, 20, 0, 40, 20, 0, 40, 20, 0, 40, 20, 0, 40, 20, 0, 40, 20},
  {0, 70, 40, 0, 70, 40, 0, 70, 40, 0, 70, 40, 0, 70, 40, 0, 70, 40},
//{0,  0, 20, 0,  0, 20, 0,  0, 20, 0,  0, 20, 0,  0, 20, 0,  0, 20},
  {0,-10,  0, 0,-10,  0, 0,-10,  0, 0,-10,  0, 0,-10,  0, 0,-10,  0},
  {0,  0, 90, 0,  0, 90, 0,  0, 90, 0,  0, 90, 0,  0, 90, 0,  0, 90},
  {-30,  0, 90, -30,  0, 90, -30,  0, 90, -30,  0, 90, -30,  0, 90, -30,  0, 90},
  {30,  0, 90, 30,  0, 90, 30,  0, 90, 30,  0, 90, 30,  0, 90, 30,  0, 90},
  {0,  0,  0, 0,  0,  0, 0,  0,  0, 0,  0,  0, 0,  0,  0, 0,  0,  0}};
 
void animate() {
  for(uint8_t i = 0; i < num_frames; ++i) {
    servoAngleSet(frames[i]);
    delay(500);
  }
  delay(1000);
}

void changeAngle() { 
  float inpval;
  int inpservo;
  
  while(!Serial.available()){
  } 
  inpservo = Serial.parseInt();
  inpval = Serial.parseFloat();
  
  if(inpservo >=0 && inpservo < NUM_SERVO) {
    Serial.print("Setting Servo ");
    Serial.print(inpservo);
    Serial.print(" to ");
    Serial.print((uint16_t) (ang2bits[inpservo] * inpval + initval[inpservo]));
    Serial.print("/4096\n");
    
    // setServo(inpservo, setval);
    servoAngle(inpservo, inpval);
  }
}
