/* rover.ino
 * Author: Paul Sampson   Date: June 7, 2023
 * Description: Implements a combined linear velocity and angular position control system
 * on the Arduino Uno using the motor driver shield.
 */
#include <Wire.h>
#include "controller.h"
#include "constants.h"

//********* Global Variables *******************
// Positional variables
static struct loc {
  volatile float velocCounts[2] = {0,0};
  volatile long posCounts[2] = {0,0};
  float velocRads[2] = {0,0};
  float posRads[2] = {0,0};
  float xy[2] = {0,0};
  float phi = 0;
} loc;

// Time variables
static struct timing {
  const unsigned long MS_PER_DESIRED = 25;  // desired sample time in milliseconds
  unsigned long msTimeLast;
  unsigned long msTimeStart;
  float sTimeCurrent;
} timing;

// I2C communication variables
static struct i2c {
  volatile uint8_t offset = 0;
  volatile uint8_t msgLength = 0;
  volatile uint8_t requestLength = 0;
  volatile uint8_t messageIn[32];
  uint8_t reply[8];
} i2c;

// Semi-human-readable enumerations
// I2C registers as instructions
enum instruction {
  angle=0,relturn=1,forward=2,angveloc=3,pong=4,datarequest=0xff,curve=0xfe
};

// Refers to the length in bytes of i2c packets
enum requestLength {
  lenByte=1,lenWord=4,lenDouble=8,leni2cMax=32
};


enum requestSelection {
  phi=0,xy=1
};

// ************ Objects and Functions **************

// Control system instance
static Controller myController;

// Encoder ISRs
void enc1ISR();
void enc2ISR();

// Encoder & Motor functions
void updateEnc(uint8_t encoder);
void updateLoc();
void setMotors(float voltages[2]);

// I2C ISRs
void request(); 
void receive();

// I2C functions
void preparei2cOut();
void printReceived();

// Fxn pointer array for indexing
void (*encISRptr[2])() = { enc1ISR, enc2ISR };

void setup() {
  // Setup pin configurations
  pinMode(nD2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(nSF, INPUT);
  pinMode(M1FB, INPUT);
  pinMode(M2FB, INPUT);
  pinMode(PONG,OUTPUT);

  // Initialize I2C
  Wire.begin(I2C_ADDR);

  // Set callbacks for I2C interrupts
  Wire.onRequest(request);
  Wire.onReceive(receive);

  Serial.begin(115200);

  // Configure encoder interrupts
  for (uint8_t i = 0; i < 2; i++) {
    for (uint8_t j = 0; j < 2; j++)
      pinMode(ENC[i][j], INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC[i][0]),
     (*encISRptr[i]), FALLING);
  }
  
  digitalWrite(nD2, HIGH); // Enable the motor drivers
  digitalWrite(PONG,HIGH);

  timing.msTimeLast = millis();  // set up sample time variables
  timing.msTimeStart = timing.msTimeLast;
}

void loop() {

  printStats();
  // If there is data on the buffer, read it
  if(i2c.msgLength > 0) {
    if(!i2c.reply[0]) {
      i2c.reply[0] = COMPLETE;
    }
    printReceived();
    switch(i2c.offset) {
      case angle:
        myController.setPhiDesired(toFloat(i2c.messageIn));
        break;
      case relturn:
        myController.addPhiDesired(toFloat(i2c.messageIn));
        break;
      case forward:
        myController.setRhoDotDesired(toFloat(i2c.messageIn));
        break;
      case angveloc:
        myController.setPhiDotDesired(toFloat(i2c.messageIn));
        break;
      case pong:
        myController.setRhoDotDesired(toFloat(0));
        myController.setPhiDotDesired(toFloat(0));
        digitalWrite(PONG,LOW);
        delay(500);
        digitalWrite(PONG,HIGH);
        break;
      case datarequest:
        preparei2cOut();
        break;
      case curve:
        myController.setRhoDotDesired(toFloat(i2c.messageIn));
        myController.setPhiDesired(toFloat(&(i2c.messageIn[4])));
        break;
      default:
        break;
    }
    i2c.msgLength = 0;
  }

  // print out timestamp in seconds
  timing.sTimeCurrent = 
    (float)(timing.msTimeLast - timing.msTimeStart) / 1000;

  updateLoc();

  float voltages[2] = {0, 0}; // A place to store the voltages set by the controller
  // The last two arguments are passed by reference and modified:
  myController.calcState(loc.velocRads,float(timing.MS_PER_DESIRED)/1000,loc.xy, voltages);

  // Set the motor drivers to the specified voltages:
  setMotors(voltages);

  while (millis() < timing.msTimeLast + timing.MS_PER_DESIRED) {  //wait until desired time passes
  }
  timing.msTimeLast = millis();
}

// Convert word into float using memcpy
float toFloat(uint8_t input[4]) {
  float output;
  float *outPtr = &output;
  memcpy(outPtr,input,4);
  return output;
}

// ISR function for encoder 1
void enc1ISR() {
  updateEnc(0);
}

// ISR function for encoder 2
void enc2ISR() {
  updateEnc(1);
}


// Update encoder position with current encoder data
void updateEnc(uint8_t encoder) {
  long *pos = &(loc.posCounts)[encoder];
  if (digitalRead(ENC[encoder][1])) *pos = *pos + 4*pow(-1, encoder);
  else *pos = *pos - 4*pow(-1, encoder);
}

// Update dead-reckoned absolute location of the rover
void updateLoc() {
  for(uint8_t i = 0; i < 2; i++) {
    static long lastPos[2] = {0,0};
    long *pos = &(loc.posCounts)[i];
    loc.velocCounts[i] = (*pos-lastPos[i])/((float)timing.MS_PER_DESIRED/1000);
    loc.velocRads[i] = 2*PI*loc.velocCounts[i]/3200;
    loc.posRads[i] = 2*PI*(*pos)/3200;
    loc.phi = myController.getPhi();
    lastPos[i] = *pos;
  }
}

// Adjust motors based on approximate voltage
void setMotors(float voltages[2]) {
  int pwm[2];
  for(int i = 0; i < 2; i++) {
    pwm[i] = voltages[i] * float(255/VMAX);
    if(pwm[i] < 0) digitalWrite(DIR[i], HIGH);
    else digitalWrite(DIR[i],LOW);
    analogWrite(PWM[i],abs(pwm[i]));
  }
}

// Helps with debugging control system
void printStats() {
  Serial.print(myController.getRhoDot());
  Serial.print('\t');
  Serial.print(myController.getRhoDotDesired());
  Serial.print('\t');
  Serial.print(myController.getPhi());
  Serial.print('\t');
  Serial.print(myController.getPhiDesired());
  Serial.print('\t');
  Serial.print(myController.getPhiDot());
  Serial.print('\t');
  Serial.print(myController.getPhiDotDesired());
  Serial.print('\t');
  Serial.print(myController.getIntPhiErr());
  for(int i = 0; i < 2; i++) {
    Serial.print(digitalRead(DIR[i]));
    Serial.print('\t');
  }
  Serial.println("");
}

// Helps with debugging I2C reception
void printReceived() {
  // Print on serial console
  Serial.print("Offset received: ");
  Serial.println(i2c.offset);
  Serial.print("Data received: ");
  for(uint8_t i = 0; i < i2c.msgLength; i++) {
    Serial.print(i2c.messageIn[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

// Copy data from some pointer to the reply buffer
void writeReply(const void * src, uint8_t n) {
  uint8_t unused = i2c.requestLength - n;
  Serial.print("n = ");
  Serial.println(n);
  if(unused > 0) {
    for(uint8_t i = 0; i < lenDouble; i++) i2c.reply[0] = 0;
  }
  i2c.requestLength = n;
  memcpy(i2c.reply,src,n);
  printReply();
}

// Helps with debugging I2C transmission
void printReply() {
  Serial.print("x: ");
  Serial.print(loc.xy[0],10);
  Serial.print(" y: ");
  Serial.println(loc.xy[1],10);
  Serial.print("Peripheral Reply: ");
  for(uint8_t i = 0; i < i2c.requestLength; i++) {
    Serial.print(i2c.reply[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

// Based on the request type from messageIn[0], send the apropriate information back:
void preparei2cOut() {
  switch(i2c.messageIn[0]) {
    case xy:
      writeReply(loc.xy,lenDouble);
      break;
    case phi:
    default:
      writeReply(&loc.phi,lenWord);
      break;
  }
}

// I2C request ISR -- Handles the sending of the contents of i2c.reply to the controller
void request() {
  // According to the Wire source code, we must call write() within the requesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line 238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  switch(i2c.requestLength) {
    case lenDouble:
    case lenWord:
      Wire.write(i2c.reply,i2c.requestLength);
      i2c.reply[0] = COMPLETE;
      break;
    case lenByte:
    default:
      Wire.write(i2c.reply[0]);
      i2c.reply[0] = 0;
      break;
  }
  i2c.requestLength = lenByte;
}

// I2C receive ISR -- Handles the reception of data from the controller to i2c.messageIn
// with zero-indexed message length in i2c.msgLength
void receive() {
  // Set the offset, this will always be the first byte.
  i2c.offset = Wire.read();

  // Each successive byte available is appended to the messageIn buffer
  i2c.msgLength = 0;
  while(Wire.available()) {
    // Write to the buffer and append msgLength
    i2c.messageIn[i2c.msgLength++] = Wire.read();
  }
}
