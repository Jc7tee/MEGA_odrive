#ifndef DEDRIFT_H
#define DEDRIFT_H

class deDrift{

  private:
    float myGyroVal;
    unsigned long lastTime;
    float drift_compensate(float*,float*);
    void setSampleTime(int);

  public:
    void setGyroVal(float);
    float *gyroInput;
};

#endif
