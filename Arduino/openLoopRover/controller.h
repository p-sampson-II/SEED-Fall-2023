#ifndef CONT_H
#define CONT_H

class Controller {
  protected:
  const float defaultArgs[4] = {7,10,7.5,8};

  float KpRhoDot;
  float KpPhiDot;
  float voltageSaturation;
  float voltageMaximum;


  float rhoDotDesired;
  float phiDotDesired;
  public:
  Controller();
  Controller(float KpR, float KpP, float vSat, float vMax);

  void setDesired(float rDD, float pDD);

  void init(float args[4]);
  void control(const float &RvelocActual,const float &LvelocActual, float &voltageL, float &voltageR);
};

#endif