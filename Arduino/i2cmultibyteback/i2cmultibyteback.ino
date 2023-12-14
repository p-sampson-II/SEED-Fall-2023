#include <Wire.h>
#include <Encoder.h>

#define MY_ADDR 4

// Signals we can send back to leader, as seen on ASCII character tables
#define COMPLETE 0x7     // To indicate success
#define ERROR 0x1   // To indicate failure

Encoder enc1(2,8);
Encoder enc2(3,9);

// Global variables to be used for I2C communication
volatile uint8_t offset = 0;
volatile uint8_t instruction = 0;
volatile uint8_t msgLength = 0;
volatile uint8_t requestType = 0;
volatile uint8_t messageIn[32];
uint8_t reply[4];

void request(); 
void receive();

void prepareEncOut();
void printReceived();

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin(MY_ADDR);

  // Set callbacks for I2C interrupts
  Wire.onRequest(request);
  Wire.onReceive(receive);
}

void loop() {
  // If there is data on the buffer, read it
  if(msgLength > 0) {
    printReceived();
    switch(offset) {
      case 2:
        prepareEncOut();
    }
    msgLength = 0;
  }
}

void printReceived() {
  // Print on serial console
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Data received: ");
  for(uint8_t i = 0; i < msgLength; i++) {
    Serial.print(messageIn[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

void prepareEncOut() {
  // Holds the value we will send to leader
  long encValue;

  // Choose which encoder based on instruction
  switch(messageIn[0]) {
    case 1:
      encValue = enc2.read();
    case 0:
    default:
      encValue = enc1.read();
  }

  // Convert long to byte array
  for (int i = 0; i < 4; i++){
    reply[i] = encValue >> (i*8);
  }
  
  // Let the request ISR know we will send 4 bytes
  requestType = 2;
}

void request() {
  // According to the Wire source code, we must call write() within the requesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line 238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  switch(requestType) {
    case 2:
      Wire.write(reply,4);
      reply[0] = COMPLETE;
      break;
    default:
      Wire.write(reply[0]);
      reply[0] = 0;
  }
  requestType = 0;
}

void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();

  // Each successive byte available is appended to the messageIn buffer
  msgLength = 0;
  while(Wire.available()) {
    // Write to the buffer and append msgLength
    messageIn[msgLength++] = Wire.read();
  }
}
