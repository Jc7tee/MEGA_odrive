#ifndef GYRO_H
#define GYRO_H
#include <I2Cdev.h>
#include <Arduino.h>

// #ifndef _MPU6050_6AXIS_MOTIONAPPS20_H_
// #define _MPU6050_6AXIS_MOTIONAPPS20_H_
// #include "MPU6050_6Axis_MotionApps20.h"
// #endif
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
// #define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL
// #define PRINTMODE
// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, use OUTPUT_READABLE_WORLDACCEL instead.
// #define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
// #define OUTPUT_READABLE_WORLDACCEL

class gyro{

  public:
    gyro(Stream& serial);
    // virtual ~gyro();
    float yaw;
    float yaw_stable;
    float pitch;
    float roll;
    float accel_x;
    float accel_y;
    float accel_z;
    float acc_total_vector;
    float angle_pitch_acc;
    float angle_roll_acc;
    void setup(/* arguments */);
    void process();
    // float filter(int mode, const int numOfReadings);
    // float filter(int mode, const int numOfReadings,float rawSenseVal);

  private:
    static void dmpDataReady();
    void printTimer();
    int period;
    unsigned long time_now;
    int doneCalibrateCount;
    Stream& serial_;
    int privMode;
    int privNumOfReadings;
    int readings[100];
    int readIndex;              // the index of the current reading
    int total;                  // the running total
    int average;                // the average
    // MPU control/status vars

};
#endif
