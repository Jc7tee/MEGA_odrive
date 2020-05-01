#include "complimentFilter.h"
#include "gyro.h"

float deDrift:: drift_compensate(float* gyroPitch, float* gyroRoll){

  //Gyro angle calculations
  //0.0000382 = 1 / 200Hz / 131
  // angle_pitch += gyro_x * 0.0000382;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  // angle_roll += gyro_y * 0.0000382;                                    //Calculate the traveled roll angle and add this to the angle_roll variable
  //
  // //0.000000666 = 0.0000382 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  // angle_pitch += angle_roll * sin(gyro_z * 0.000000666);               //If the IMU has yawed transfer the roll angle to the pitch angel
  // angle_roll -= angle_pitch * sin(gyro_z * 0.000000666);               //If the IMU has yawed transfer the pitch angle to the roll angel
  // //Accelerometer angle calculations
  // acc_total_vector = sqrt((aaReal.x*aaReal.x)+(aaReal.y*aaReal.y)+(aaReal.z*aaReal.z));//Calculate the total accelerometer vector
  // //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  // angle_pitch_acc = asin((float)aaReal.y/acc_total_vector)* 57.296; //Calculate the pitch angle
  // angle_roll_acc = asin((float)aaReal.x/acc_total_vector)* -57.296;//Calculate the roll angle
  //
  // yaw_compensate = (ypr[0] * 180/M_PI) * 0.9996 + angle_pitch_acc * 0.0004;
  // yaw = ypr[0] * 180/M_PI;
  // pitch = ypr[1] * 180/M_PI;
  // roll = ypr[2] * 180/M_PI;

}
