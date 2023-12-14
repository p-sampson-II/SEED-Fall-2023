/* constants.h
 * Author: Paul Sampson   Date: June 7, 2023
 * Description: Constants associated with the functioning of the rover.
 */

#ifndef CONST_H
#define CONST_H

#define I2C_ADDR 8

// Signals we can send back to leader, as seen on ASCII character tables
#define COMPLETE 0x7     // To indicate success
#define ERROR 0x1   // To indicate failure

//********** Dimensional Constants ******************
// Dimensions in inches
#define WHEEL_L_RADIUS 5.904/2
#define WHEEL_R_RADIUS 5.891/2

#define WHEELS_DIST 10.75

// In an array for indexing ("to avoid code dupes")
const float RADII[2] = {WHEEL_R_RADIUS,WHEEL_L_RADIUS};

//********** Control System Constants ****************

#define VMAX 8
#define VSAT 3.5

#define KP_RHODOT 2
#define KI_RHODOT 1
#define KP_PHIDOT 8
#define KI_PHIDOT 0.5
#define KP_PHI 4
#define KI_PHI 0

// Error thresholds
#define PHI_THRESH 0.05
#define RHODOT_THRESH 0.5

#define PHIDOT_MAX 2.0
#define EMP_VELOC_MAX 12 //empirical velocity max

//*********** Pin Assignments *****************

// Motor Driver Pins
#define nD2 4
#define M1DIR 7
#define M2DIR 8
#define M1PWM 9
#define M2PWM 10
#define nSF 12
#define M1FB A0
#define M2FB A1

// For emitting a ping pong ball. Update accordingly.
#define PONG A3

// Arrays of pins to avoid code dupes
const int FB[2] = {M1FB,M2FB};
const int DIR[2] = {M1DIR,M2DIR};
const int PWM[2] = {M1PWM,M2PWM};
const uint8_t ENC[2][2] = {{ 2, 5 }, { 3, 11 }};

#endif // CONST_H
