/* controller.cpp
 * Author: Paul Sampson   Date: June 7, 2023
 * Description: Implements the control system for the rover.
 */


#include <Arduino.h>
#include "controller.h"
#include "constants.h"

Controller::Controller() {
  init(defaultArgs);
}

// Class initialization
void Controller::init(float args[8]) {
  KpRhoDot = args[0];
  KiRhoDot = args[1];
  KpPhiDot = args[2];
  KiPhiDot = args[3];
  KpPhi = args[4];
  KiPhi = args[5];
  voltageSaturation = args[6];
  voltageMaximum = args[7];
}

// Set desired magnitude; reset error sum
void Controller::setRhoDotDesired(float rDD) {
  rhoDotDesired = rDD;
  integRhoDotErr = 0;
}

// Allow access of desired magnitude for debugging
float Controller::getRhoDotDesired() {
  return rhoDotDesired;
}

// Same with angular velocity
float Controller::getPhiDotDesired() {
  return phiDotDesired;
}

// Set desired angle; reset error sum
void Controller::setPhiDesired(float pD) {
  while(abs(pD) >= 2*PI)
    pD = (abs(pD)-2*PI)*pD/abs(pD);
  phiDesired = pD;
  isPhiCtrl = true;
  integPhiDotErr = 0;
}

// Set desired angular velocity; switch to velocity mode
void Controller::setPhiDotDesired(float pDD) {
  if(abs(pDD) > 0) {
    phiDotDesired = pDD;
    isPhiCtrl = false;
  }
  else {
    phiDesired = phiActual;
    isPhiCtrl = true;
  }
  integPhiDotErr = 0;
}

// Increment phi, aka set a relative phi
void Controller::addPhiDesired(float dpD) {
  phiDesired = phiActual + dpD;
  isPhiCtrl = true;
}

// Get desired angle for debugging
float Controller::getPhiDesired() {
  return phiDesired;
}

// Answers the question: Should we use the integrator term, based on the discrepancy between
// the present velocity and the present voltage? (Should we be trying harder?)
bool Controller::isIntegrator(float velocities[2], float voltages[2]) {
  bool output = false;
  double avgNormVolt = 0;
  double avgNormVeloc = 0;
  for(int i = 0; i < 2; i++) {
    avgNormVolt += abs(voltages[i]/VMAX)/2;
    avgNormVeloc += abs(velocities[i]/EMP_VELOC_MAX);
  }
  
  output = (avgNormVolt - avgNormVeloc > 0.02);
  return output;
}

// Update absolute location given current velocity of wheels
void Controller::updateLoc(float velocities[2],float deltaTime, float(&xy)[2]) {
  phiDotActual = 0;
  rhoDotActual = 0;

  for(int i = 0; i < 2; i++) {
   phiDotActual -= pow((-1),i)*(RADII[i]*velocities[i])/WHEELS_DIST;
   rhoDotActual += (RADII[i]*velocities[i])/2;
  }

  phiActual += phiDotActual * deltaTime;
  rhoActual += rhoDotActual * deltaTime;
  float xyDot[2] = {rhoDotActual * cos(phiActual),rhoDotActual * sin(phiActual)};
  for(int i = 0; i < 2; i++)
    xy[i] += xyDot[i] * deltaTime;
  
  while(abs(phiActual) >= 2*PI)
    phiActual = (abs(phiActual)-2*PI)*phiActual/abs(phiActual);
}

// Main control system code
void Controller::control(float velocities[2], float deltaTime, float (&voltages)[2]) {
  float rhoDotError = rhoDotDesired-rhoDotActual;
  integRhoDotErr = integRhoDotErr+rhoDotError*deltaTime;

  if(isPhiCtrl) {
    float phiError = (phiDesired - phiActual);

    if(isIntegrator(velocities,voltages))
      integPhiErr = integPhiErr + phiError*deltaTime;
    else
      integPhiErr = 0;

    phiDotDesired = KpPhi * phiError + KiPhi * integPhiErr;
    if(abs(phiDotDesired) > PHIDOT_MAX) phiDotDesired = (phiDotDesired/abs(phiDotDesired))*PHIDOT_MAX;
  }
  
  integPhiDotErr = integPhiDotErr+(phiDotActual-phiDotDesired)*deltaTime;

  for(int i = 0; i < 2; i++) {
    float vRhoDot = (KpRhoDot*rhoDotError+KiRhoDot*integRhoDotErr)/2;
    float vPhiDot = pow((-1),i)*(KpPhiDot*(phiDotActual-phiDotDesired)+KiPhiDot*integPhiDotErr)/2;//pow((-1),i)*(KpPhiDot*(phiDotActual-phiDotDesired)+

    voltages[i] = vRhoDot + vPhiDot;
    if(abs(voltages[i])*2 > VSAT) voltages[i] = VSAT * voltages[i]/abs(voltages[i]);
  }
}

// Public function which determines the next controller state
void Controller::calcState(float velocities[2], float deltaTime, float(&xy)[2], float (&voltages)[2]) {
  updateLoc(velocities, deltaTime, xy);
  control(velocities, deltaTime, voltages);
}

float Controller::getRhoDot(){
  return rhoDotActual;
}

float Controller::getPhiDot(){
  return phiDotActual;
}

float Controller::getPhi(){
  return phiActual;
}

float Controller::getRho(){
  return rhoActual;
}

float Controller::getIntPhiErr(){
  return integPhiErr;
}

bool Controller::isWithinThreshold() {
  return (abs(phiActual-phiDesired) < PHI_THRESH) && (abs(rhoDotActual-rhoDotDesired) < RHODOT_THRESH);
}
