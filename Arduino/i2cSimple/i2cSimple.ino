#include <Wire.h>

#define MY_ADDR 4

// Signals we can send back to leader, as seen on ASCII character tables
#define COMPLETE 0x7     // To indicate success
#define ERROR 0x1   // To indicate failure

// Global variables to be used for I2C communication
volatile uint8_t reply = 0, offset = 0;
volatile uint8_t instruction = 0;
volatile uint8_t msgLength = 0;

// I2C functions, which are called when an I2C interrupt event happens
void request();
void receive();

// This function calls when the offset 0 is received from the leader
uint8_t toggleLED();

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
    reply = toggleLED();
    msgLength = 0;
  }
}

void printReceived() {
  // Print on serial console
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Instruction received: ");
  Serial.println(instruction);
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

  // If there is information after the offset, it is telling us more about the command.
  if(Wire.available()) {
    msgLength = 1;
    instruction = Wire.read();
  }
}
