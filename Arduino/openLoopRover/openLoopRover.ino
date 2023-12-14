#include <arduino.h>
// Pin Assignments

// Motor Driver
#define nD2 4
#define M1DIR 7
#define M2DIR 8
#define M1PWM 9
#define M2PWM 10
#define nSF 12
#define M1FB A0
#define M2FB A1

// 0->rhoDot, 1->phiDot
#define TEST 0

// Dimensions in inches
#define WHEEL_L_RADIUS 5.904/2
#define WHEEL_R_RADIUS 5.891/2

#define WHEELS_DIST (11+(13/16) - 0.988)

#define START 1000
#define VOLT_ESTIM 8
#define M_TEST_PWM 100

// Encoders
void enc1ISR();
void enc2ISR();
void (*encISRptr[2])() = { enc1ISR, enc2ISR };

const uint8_t ENC[2][2] = {{ 2, 5 }, { 3, 11 }};
const float RADII[2] = {WHEEL_R_RADIUS,WHEEL_L_RADIUS};

volatile float velocCounts[2] = {0,0};
volatile long posCounts[2] = {0,0};
float velocRads[2] = {0,0};

float posRads[2] = {0,0};
float voltage = 0;

unsigned long desired_Ts_ms = 25;  // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time;

void setup() {
  pinMode(nD2, OUTPUT);
  pinMode(M1DIR, OUTPUT);
  pinMode(M2DIR, OUTPUT);
  pinMode(M1PWM, OUTPUT);
  pinMode(M2PWM, OUTPUT);
  pinMode(nSF, INPUT);
  pinMode(M1FB, INPUT);
  pinMode(M2FB, INPUT);

  Serial.begin(115200);

  for (uint8_t i = 0; i < 2; i++) {
    for (uint8_t j = 0; j < 2; j++)
      pinMode(ENC[i][j], INPUT);
    attachInterrupt(digitalPinToInterrupt(ENC[i][0]),
     (*encISRptr[i]), FALLING);
  }

  digitalWrite(nD2, HIGH);
  digitalWrite(M2DIR,HIGH);
  #if TEST == 0
  digitalWrite(M1DIR,HIGH);
  #endif

  last_time_ms = millis();  // set up sample time variable s
  start_time_ms = last_time_ms;
}

void loop() {
  // print out timestamp in seconds
  current_time = (float)(last_time_ms - start_time_ms) / 1000;
  Serial.print(current_time);
  Serial.print("\t");
  Serial.print(voltage);

  float testVar = 0;

  for(uint8_t i = 0; i < 2; i++) {
    updateVeloc(i);
    velocRads[i] = 2*PI*velocCounts[i]/3200;

    #if TEST == 1
    testVar += pow((-1),i)*(RADII[i]*velocRads[i])/WHEELS_DIST;
    #else
    testVar += (RADII[i]*velocRads[i])/2;
    #endif

    Serial.print("\t");
    Serial.print(velocRads[i]);
  }
  Serial.print("\t");
  Serial.println(testVar);
  //Serial.println("");
  static bool step = false;

  if(millis() > START && !step) {
    analogWrite(M1PWM,M_TEST_PWM);
    analogWrite(M2PWM,M_TEST_PWM);
    voltage = VOLT_ESTIM*M_TEST_PWM/255;
    step = true;
  }

  while (millis() < last_time_ms + desired_Ts_ms) {  //wait until desired time passes to
  }
  last_time_ms = millis();
}

void enc1ISR() {
  updateEnc(0);
}

void enc2ISR() {
  updateEnc(1);
}

void updateVeloc(uint8_t encoder) {
  static long lastPos[2] = {0,0};
  long *pos = &posCounts[encoder];
  velocCounts[encoder] = (*pos-lastPos[encoder])/((float)desired_Ts_ms/1000);
  lastPos[encoder] = *pos;
}

void updateEnc(uint8_t encoder) {
  long *pos = &posCounts[encoder];
  if (digitalRead(ENC[encoder][1])) *pos = *pos + 4*pow(-1, encoder);
  else *pos = *pos - 4*pow(-1, encoder);
}
