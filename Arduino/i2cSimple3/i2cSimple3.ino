#include <Wire.h>

#define MY_ADDR 4

// Signals we can send back to leader, as seen on ASCII character tables
#define COMPLETE 0x7     // To indicate success
#define ERROR 0x1   // To indicate failure

// Global variables to be used for I2C communication
volatile uint8_t reply = 0, offset = 0;
volatile uint8_t messageIn[32];
volatile uint8_t msgLength = 0;

// I2C functions, which are called when an I2C interrupt event happens
void request(); 
void receive();

// This function calls when the offset 0 is received from the leader
uint8_t toggleLED();
uint8_t blinkLED();

// printReceived helps us see what data we are getting from the leader
void printReceived();

void setup() {
  Serial.begin(115200);

  // We want to control the built-in LED (pin 13)
  pinMode(LED_BUILTIN,OUTPUT);

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
      case 0:
        reply = toggleLED();
        break;
      case 1:
        reply = blinkLED();
        break;
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

uint8_t toggleLED() {
  // If the instruction is not 1, return an error
  if(instruction != 1) return ERROR;

  // Read the LED and take the compliment
  bool newState = !digitalRead(LED_BUILTIN);

  // Toggle LED and report success
  digitalWrite(LED_BUILTIN,newState);
  return COMPLETE;
}


uint8_t blinkLED() {

  // Stack the first two bytes of messageIn to get a 16-bit integer
  uint16_t period = messageIn[0] 
    + (uint16_t(messageIn[1]) << 8);

  // Next byte is the duty cycle
  uint8_t dutyCycle = messageIn[2];

  // Reject invalid values
  if (dutyCycle > 100 || period == 0)
    return ERROR;

  // How long to delay for LED behaviors:
  uint16_t timeOn = period * (float(dutyCycle) / 100),
    timeOff = period * (1-(float(dutyCycle)/100));

  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(timeOn);
  digitalWrite(LED_BUILTIN,LOW);
  delay(timeOff);
  return COMPLETE;
}

void request() {
  // According to the Wire source code, we must call write() within the requesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line 238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  Wire.write(reply);      
  reply = 0;
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
