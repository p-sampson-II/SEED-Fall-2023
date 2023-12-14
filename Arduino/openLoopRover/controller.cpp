#include "controller.h"

Controller::Controller() {
  init(defaultArgs);
}

void Controller::init(float args[4]) {
  KpRhoDot = args[0];
  KpPhiDot = args[1];
  voltageSaturation = args[2];
  voltageMaximum = args[3];
}

void Controller::setDesired(float rDD, float pDD) {
  rhoDotDesired = rDD;
  phiDotDesired = pDD;
}

void Controller::control(const float &RvelocActual,const float &LvelocActual, float &voltageL, float &voltageR) {
  
}