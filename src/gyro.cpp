#include "gyro.h"
#include <MPU6050_6Axis_MotionApps20.h>
MPU6050 mpu;

gyro::gyro(Stream& serial): serial_(serial){//Constructor
  yaw = 1, pitch = 2, roll = 3;
  accel_x=0, accel_y=0, accel_z=0;
  // period = 1000;
  doneCalibrateCount = 20000;
  time_now = 0;
  readIndex = 0;              // the index of the current reading
  total = 0;                  // the running total
  average = 0;                // the average
}
bool blinkState = false;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void gyro::dmpDataReady() {
    mpuInterrupt = true;
}
// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void gyro::setup(){
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

    // load and configure the DMP
  serial_.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(88);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(-13);
  mpu.setZAccelOffset(1518);
  // mpu.setXAccelOffset(160);//158
  // mpu.setYAccelOffset(-138);//141

  // time_now = millis();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    //turn on the DMP, now that it's ready
    serial_.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    serial_.print(F("Enabling interrupt detection (Arduino external interrupt "));
    serial_.print(digitalPinToInterrupt(INTERRUPT_PIN));
    serial_.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    serial_.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }else{
    serial_.print(F("DMP Initialization failed (code "));
    serial_.print(devStatus);
    serial_.println(F(")"));
  }
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

void gyro::process(){
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
      #ifdef OUTPUT_READABLE_EULER
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetEuler(euler, &q);
          serial_.print("euler\t");
          serial_.print(euler[0] * 180/M_PI);
          serial_.print("\t");
          serial_.print(euler[1] * 180/M_PI);
          serial_.print("\t");
          serial_.println(euler[2] * 180/M_PI);
      #endif

      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          yaw = ypr[0] * 180/M_PI;
          pitch = ypr[1] * 180/M_PI;
          roll = ypr[2] * 180/M_PI;
          // yaw_compensate+= 0.003847;
          //Start compensating once timer reach 22secs
          // if(millis() > (time_now + doneCalibrateCount) ){
            // yaw_stable = filter(privMode, privNumOfReadings,yaw);
          // }else{
          //   yaw_stable=0.77;
          // }
          // yaw_stable = yaw + yaw_compensate;
          #ifdef PRINTMODE
            serial_.print("ypr\t");
            serial_.print(yaw);
            serial_.print("\t");
            serial_.print(pitch);
            serial_.print("\t");
            serial_.print(roll);
            serial_.print("\t");
            serial_.println(yaw_stable);
          #endif

      #endif

      #ifdef OUTPUT_READABLE_REALACCEL
          // display real acceleration, adjusted to remove gravity
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          accel_x = aaReal.x;
          accel_y = aaReal.y;
          accel_z = aaReal.z;
          #ifdef PRINTMODE
          // if(millis() > (time_now + period) ){
            time_now = millis();
            serial_.print("areal\t");
            serial_.print(accel_x);
            serial_.print("\t");
            serial_.print(accel_y);
            serial_.print("\t");
            serial_.print(accel_z);
            serial_.print("\t");
            serial_.println(mpu.getDHPFMode());
          #endif
          // }
      #endif

      #ifdef OUTPUT_READABLE_WORLDACCEL
          // display initial world-frame acceleration, adjusted to remove gravity
          // and rotated based on known orientation from quaternion
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetAccel(&aa, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
          mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
          serial_.print("aworld\t");
          serial_.print(aaWorld.x);
          serial_.print("\t");
          serial_.print(aaWorld.y);
          serial_.print("\t");
          serial_.println(aaWorld.z);
      #endif

  }
}

// float gyro::filter(int mode, const int numOfReadings){ //used with internal sense val (contained in gyro.cpp)
//   privMode = mode;
//   privNumOfReadings = numOfReadings;
//   int readings[numOfReadings];
// return 0;
// }
//
//
// float gyro::filter(int mode, const int numOfReadings,float rawSenseVal){//for use with external sense value
//
//   if(mode==1){//moving average
//         // subtract the last reading:
//     total = total - readings[readIndex];
//     // read from the sensor:
//     readings[readIndex] = rawSenseVal;
//     // add the reading to the total:
//     total = total + readings[readIndex];
//     // advance to the next position in the array:
//     readIndex = readIndex + 1;
//
//     // if we're at the end of the array...
//     if (readIndex >= numOfReadings) {
//       // ...wrap around to the beginning:
//       readIndex = 0;
//     }
//     // calculate the average:
//     average = (total / numOfReadings);
//     delay(1);        // delay in between reads for stability
//   }else if(mode==2){ //future filter
//
//   }
//   return average;
// }
