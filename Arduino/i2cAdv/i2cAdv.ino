#include <Wire.h>
#include <Encoder.h>
#include <DualMC33926MotorShield.h>

#define MY_ADDR 4

// Signals that we can send back to the leader,
// as seen on an ASCII character table:
#define ACK 0x6     // To indicate success
#define NACK 0x15   // To indicate failure

#define debug 1

DualMC33926MotorShield motors;
Encoder encoderRight(2,8); 
Encoder encoderLeft(3,9);

typedef struct byte4 {
  uint8_t data[4];
} byte4;

long encoderPos = 1000;
byte4 replyBuffer;
volatile uint8_t status = 0;
uint8_t state = 0;

volatile uint8_t messageIn[32];
volatile uint8_t msgLength = 0;
volatile uint8_t offset = 0;

volatile bool isDataReceive = 0;
volatile uint8_t dataRequested = 0;

void request();
void receive();

uint8_t toggleLED(uint8_t command);
uint8_t blinkLED(uint16_t period, uint8_t dutyCycle);

void printMsg();
uint8_t actionSelector();

void setup() {
  Serial.begin(115200);

  // We want to control the built-in LED (pin 13)
  pinMode(LED_BUILTIN,OUTPUT);

  // Initialize I2C
  Wire.begin(MY_ADDR);

  // Initialize Motor Driver Board
  motors.init();

  // Set callbacks for I2C interrupts
  Wire.onRequest(request);
  Wire.onReceive(receive);
}

void loop() {

  // If there is data on the buffer, read it
  if(msgLength) {
    // If debuggging, print what we receive
    #if debug
    printMsg();
    #endif
    status = actionSelector();
    msgLength = 0;
  }
}

void printMsg() {
  Serial.print("Data received: ");
  for(uint8_t i = 0; i < msgLength; i++) {
    Serial.print(messageIn[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

uint8_t actionSelector() {
  #if debug
    Serial.print("Entered Action Selector: ");
    Serial.println(offset);
  #endif
  switch(offset) {
    case 4:
      motors.setM2Speed(messageIn[0]);
      return ACK;
    case 3:
      motors.setM1Speed(messageIn[0]);
      return ACK;
    case 2:
      return setReply(messageIn[0]);
    case 0:
      return toggleLED(messageIn[0]);
    case 1:
      uint16_t period = messageIn[0] + (uint16_t(messageIn[1]) << 8);
      uint8_t dutyCycle = messageIn[3];
      return blinkLED(period, dutyCycle);
    default:
      return NACK;
  }
}

uint8_t setReply(uint8_t command) {
  encoderPos = encoderRight.read();
  replyBuffer = replySelector(messageIn[0]);
  dataRequested = 2;
  return ACK;
}

byte4 replySelector(uint8_t command) {
  switch(command) {
    case 0:
      return convertToBytes(encoderLeft.read());
    case 1:
      return convertToBytes(encoderRight.read());
  }
}

uint8_t toggleLED(uint8_t command) {
  // If the data is equal to 1, flip the LED
  if (command == 1) {
      digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
      return ACK;
    }

  // Otherwise, we don't care! It is invalid.
  else return NACK;
}

uint8_t blinkLED(uint16_t period, uint8_t dutyCycle) {
  
  // Reject invalid values
  if (dutyCycle > 100 || period == 0)
    return NACK;

  // How long to delay for LED behaviors:
  uint32_t timeOn = period * (float(dutyCycle) / 100),
    timeOff = period * (1-(float(dutyCycle)/100));
  
  // If the LED is already on
  if(digitalRead(LED_BUILTIN) == HIGH) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(timeOff);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(timeOn);
  }

  // If the LED is off
  else {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(timeOn);
    digitalWrite(LED_BUILTIN, LOW);
    delay(timeOff);
  }

  // We have made it to the end of the function without any
  // errors, as far as we know; return ACK
  return ACK;
}

byte4 convertToBytes(long input) {
  byte4 output;
  #if debug
  Serial.print("Value:");
  Serial.println(input);
  Serial.print("Value (bytes): ");
  #endif
  for (int i = 0; i < 4; i++) {
    output.data[i] = input >> (i*8);
    #if debug
      Serial.print(output.data[i]);
      Serial.print(" ");
    #endif
    }
  #if debug
    Serial.println("");
  #endif
  return output;
}

// We want to do as little as possible within the interrupts
// so that we're not stopping other interrupts.

void request() {
  // According to the Wire source code, we must call write()
  // within the requesting ISR. Otherwise, the timing does not work out.
  // See line 238: https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  switch(dataRequested) {
    case 2:
      Wire.write(replyBuffer.data,4);
      status = ACK;
      break;
    default:
      Wire.write(status);      
  }
  dataRequested = 0;
}

void receive() {
  // Set the offset
  offset = Wire.read();
  msgLength = 0;
  while(Wire.available()) {
    messageIn[msgLength++] = Wire.read();
  }
}
