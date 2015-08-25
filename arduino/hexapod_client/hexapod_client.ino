/*************************************************** 
  Hexapod client.
  Listens to angle settings from 

  Uses code written by Limor Fried/Ladyada for Adafruit Industries.
 ****************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//
// Configuration
//
#define NUM_PWM 2
#define NUM_SERVO_PER_PWM 16
#define NUM_SERVO 18

//
// PWM Driver
//

Adafruit_PWMServoDriver pwm[] = {Adafruit_PWMServoDriver(0x50), Adafruit_PWMServoDriver(0x41)};

uint8_t servonum = 0;
uint16_t initval [] = {225,204,163,194,174,163,204,194,194,215,245,215,221,260,205,215,245,215};
uint8_t mapping [] = {9, 10, 11, 3, 4, 5, 6, 7, 8, 25, 17, 18, 19, 20, 21, 22, 23, 24};

//
// Communication config.
//

#define ERR_NO_INPUT 1
#define ERR_CHKSUM 2
#define ERR_PKT_TYPE 3
#define ERR_PKT_UNHANDLED 4

#define PKT_POSE_TYPE 42
#define PKT_POSE_SZ (NUM_SERVO*2)

#define BAUD_RATE 38400
#define INP_BUF_SZ PKT_POSE_SZ


//
// Poses
//

int foldedPose[] = {0, -60, -40, 0, -60, -40, 0, -60, -40, 0, -60, -40, 0, -60, -40, 0, -60, -40};
int zeroPose[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

int* initialPose = zeroPose;

void setServo(uint8_t servo, uint16_t val) {
  uint8_t mp = mapping[servo];
  pwm[mp / NUM_SERVO_PER_PWM].setPWM(mp % NUM_SERVO_PER_PWM, 0, val);
}

inline void servoAngle(uint8_t servo, float angle) {
  setServo(servo, angle + initval[servo]);
}

inline void servoAngleSet(int perturb []) {
  for(uint8_t i = 0; i < NUM_SERVO; ++i)
    servoAngle(i, perturb[i]);
}

//
// Main logic
//

void getBytes(byte* pBuf, uint8_t len) {
  while(len) {
    int b = Serial.read();
    if (b >= 0) {
      *(pBuf++) = lowByte(b);
      len--;
    }
  }
}

byte inp_buf[INP_BUF_SZ];
int readPose(int* cP){
  // Get the checksum:
  if(!Serial.available())
    return ERR_NO_INPUT;
  
  uint8_t pkt_type, chksum;
  getBytes(&pkt_type, 1);
  getBytes(&chksum, 1);

  chksum ^= pkt_type;
  
  uint8_t payload_size;
  
  // Get the expected payload size:
  switch (pkt_type) {
    case PKT_POSE_TYPE:
      payload_size = PKT_POSE_SZ;
      break;
    default:
      while(Serial.available())
        Serial.read();
      return ERR_PKT_TYPE;
  }
    
  /*byte* inp_buf_p = inp_buf;
  // Fill the buffer with the payload.
  /*
  while(payload_size){
    int b = Serial.read();
    if (b >= 0) {
      *inp_buf_p = lowByte(b);
      chksum ^= *inp_buf_p;
      inp_buf_p++;
      payload_size--;
    }
  }
  */
  getBytes(inp_buf, payload_size);
  while(payload_size) {
    chksum ^= inp_buf[--payload_size];
  }
  
  if (chksum)
    return ERR_CHKSUM;
    
  // Handle the packet type:
  switch (pkt_type) {
    case PKT_POSE_TYPE:
      // We copy the pose directly
      // It's already in order and little-endian.
      memcpy((byte *) cP, inp_buf, PKT_POSE_SZ);
      break;
    default:
      return ERR_PKT_UNHANDLED;
  }
  
  return 0;
}

int currPose[NUM_SERVO];

void setup() {
  Serial.begin(BAUD_RATE);

  for (int i = 0; i < NUM_PWM; i++) {
    pwm[i].begin();
    pwm[i].setPWMFreq(50);
  }
  
  servoAngleSet(initialPose);
}

void loop() {
  int rv = readPose(currPose);
  if (rv == ERR_NO_INPUT) {
    return;
  } else if(rv) {
    Serial.print("Update failed with error ");
    Serial.print(rv);
    Serial.println();
  } else {
    servoAngleSet(currPose);
  }
}



