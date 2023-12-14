/* controller.h
 * Author: Paul Sampson   Date: June 7, 2023
 * Description: Implements the control system for the rover.
 */


#ifndef CONT_H
#define CONT_H

#include "constants.h"

class Controller {
  protected:
  const float defaultArgs[8] = {KP_RHODOT,KI_RHODOT,KP_PHIDOT,KI_PHIDOT,KP_PHI,KI_PHI,VSAT,VMAX};

  float KpRhoDot;
  float KiRhoDot;
  float KpPhiDot;
  float KiPhiDot;
  float KpPhi;
  float KiPhi;
  float voltageSaturation;
  float voltageMaximum;

  float rhoDotDesired;
  float phiDotDesired;
  float phiDesired;

  float integPhiErr;
  float integPhiDotErr;
  float integRhoDotErr;

  float rhoDotActual;
  float phiDotActual;
  float phiActual;
  float rhoActual;

  bool isPhiCtrl;

  void control(float velocities[2], float deltaTime, float (&voltages)[2]);
  void updateLoc(float velocities[2],float deltaTime, float(&xy)[2]);

  bool isIntegrator(float velocities[2],float voltages[2]);

  public:
  Controller();
  Controller(float KpR, float KiR, float KpPD, float KiPD, float KpP, float KiP, float vSat, float vMax);

  void setRhoDotDesired(float rDD);
  void setPhiDotDesired(float pDD);
  void setPhiDesired(float phi);
  void addPhiDesired(float dpD);

  void init(float args[7]);
  void calcState(float velocities[2], float deltaTime,  float(&xy)[2], float (&voltages)[2]);

  float getRhoDot();
  float getPhiDot();
  float getRhoDotDesired();
  float getPhiDesired();
  float getPhiDotDesired();
  float getPhi();
  float getRho();
  float getIntPhiErr();

  bool isWithinThreshold();
};

#endif
