#include <SoftwareSerial.h>
#include "ODriveArduino.h"
#include <PID_v1.h>
#include "gyro.h"
#include "MePS2_Short.h"
// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// ODrive object
ODriveArduino odrive(Serial1);
#define R0_MOTOR 0
#define L1_MOTOR 1
#define Sprintln(a) (Serial.println(a))
#define Sprint(a) (Serial.print(a))
#define MOVE_AVE 1
#define NUM_DATA_FILTER 20
#define GPIO3_M0 12
#define GPIO4_M1 13
#define VEL_LIMIT 50000
#define CURRENT_LIM 10.0f
#define SEMI_AUTO_MODE 122
#define MANUAL_MODE 211
gyro gyro(Serial);
#define OUTPUT_READABLE_YAWPITCHROLL
#define PRINTMODE
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters (1.92s for complete oscillation when Kp = 10)
double Kp=1200, Ki=0, Kd=0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// Variable used for millis debounce
unsigned long StartMills=0;
unsigned long downTime=0;
unsigned long TimeOfLastDebounce = 0;  // holds the last time the switch was pressed
unsigned long DelayofDebounce = 100;  // amount of time that needs to be experied between presses

struct motor_Params left_params = {
  1, 3.0f/10000.0f, 40.0f, 0.0f/10000.0f
};

struct motor_Params right_params = {
  /*axis*/0,/*vel_gain*/ 3.0f/10000.0f, /*pos_gain*/40.0f, /*vel_int_gain*/0.00f/10000.0f
};
float setupFinishedTime;
int requested_state;
void odriveSetupParams();
void setup() {
  // ODrive uses 115200 baud
  Serial1.begin(115200);
  MePS2.begin(115200);//Serial2
  // Serial to PC 57600
  Serial.begin(38400);
  myPID.SetOutputLimits(-VEL_LIMIT,VEL_LIMIT);
  // while (!Serial) ; // wait for Arduino Serial Monitor to open
  // while (!Serial1);
  // while (!MePS2);

  // Serial1 << "sr" << '\n';
  // delay(1000);
  Serial1 << "r axis" << L1_MOTOR << ".encoder.is_ready\n";
  bool L_done=odrive.readInt();
  Serial1 << "r axis" << R0_MOTOR << ".encoder.is_ready\n";
  bool R_done=odrive.readInt();
  requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  while(L_done==0){
    odrive.run_state(L1_MOTOR, requested_state, true);
    delay(50);
    Serial1 << "r axis" << L1_MOTOR << ".encoder.is_ready\n";
    delay(50);
    L_done=odrive.readInt();
  }
  while(R_done==0){
    odrive.run_state(R0_MOTOR, requested_state, true);
    delay(50);
    Serial1 << "r axis" << R0_MOTOR << ".encoder.is_ready\n";
    delay(50);
    R_done=odrive.readInt();
  }
  odriveSetupParams();
  gyro.setup();
  // gyro.filter(MOVE_AVE, NUM_DATA_FILTER);
  setupFinishedTime = millis();
}

int motorIdled = 0;
bool finishCal = 0;
int dirButtonpressed=0;
int prevButton, buttonPressed, longPressed;
int mode_status=0;

void pure_Serial();
void manualMode();
void semiAutoMode();
void loop() {

  gyro.process();
  StartMills = millis();
  // int leftAnalog = map(MePS2.ButtonPressed(MeJOYSTICK_LY),0,255,0,10000);
  // analogWrite(GPIO3_M0,200);
  // analogWrite(GPIO4_M1,200);

  /*Semi-autonomous mode- keep robot going up straight
    0. Enable Semi-autonomous mode ( when robot on the wall, set the robot upright using a spirit level)
    1. home the current gyroYaw readings(use button/command) and zero the value of encoder
    2. filter the gyro readings as robot travels
    3. Robot continues to travel until user stop it
    4. Starts again as user commanded while automatically travel straight
    5. User press a button to finish the run (endpoint)
    6. The robot descends automatically until it reach its starting point
    3.1. Enable or disable Set position functions

  */
  // Serial1 << "r vbus_voltage\n";
  // float voltage = odrive.readFloat();
  // if(!finishCal && (voltage > 2)){
  //   if(((StartMills-setupFinishedTime)>21000)){
  //     Sprintln("Gyro ready!");
  //     finishCal=1;
  //   }
  // }else if(voltage<2){
  //   setupFinishedTime=StartMills;
  //   Sprintln("No POWER!!");
  // }
  if (MePS2.available()) {
    MePS2.loop();
    if( (BTN_L2 && BTN_R2) && (prevButton != SEMI_AUTO_MODE)&& ((StartMills-setupFinishedTime)>21000)){
      if ((StartMills - TimeOfLastDebounce) > 500) {
        prevButton= SEMI_AUTO_MODE;
        downTime=millis();
        Sprintln("SEMI-AUTO MODE");
        mode_status=1;
        myPID.SetMode(AUTOMATIC);
        Setpoint = gyro.yaw_stable;//Initial orientation captured
        Sprintln(Setpoint);
        // Serial1 << "w axis" << L1_MOTOR << ".encoder.set_linear_count(0)\n";
        // Serial1 << "w axis" << R0_MOTOR << ".encoder.set_linear_count(0)\n";
        TimeOfLastDebounce = StartMills;
        Sprintln("stuck 5");
      }
    }
    else if ( (BTN_L2 && BTN_R2) && (prevButton != SEMI_AUTO_MODE) && ((StartMills-setupFinishedTime)<21000)){
      Sprint("gyro is calibrating. ready in: ");
      Sprintln( 21-((StartMills-setupFinishedTime)/1000));
    }

    if((BTN_R1 && BTN_L1) && (prevButton != MANUAL_MODE)){
      prevButton= MANUAL_MODE;
      downTime=millis();
      if ((StartMills - TimeOfLastDebounce) > 500) {
        Sprintln("MANUAL MODE");
        mode_status=0;
        myPID.SetMode(MANUAL);
        TimeOfLastDebounce = StartMills;
        Sprintln("stuck 6");
      }
    }
    if(BTN_START){//Restart
      Serial1 << "sr" << '\n';
      delay(1000);
      odriveSetupParams();
    }
    // if(!mode_status){
    //   manualMode();
    // }else{
    //   semiAutoMode();
    // }
  }else{//if signal lost
    // requested_state = ODriveArduino::AXIS_STATE_IDLE;
    // odrive.run_state(R0_MOTOR, requested_state, false);
    // odrive.run_state(L1_MOTOR, requested_state, false);
  }
  pure_Serial();
  if(mode_status == 0){
    manualMode();
  }else if(mode_status == 2){
    semiAutoMode();
  }

}

void semiAutoMode(){

  Input = gyro.yaw_stable;
  myPID.Compute();
  //motor needs to go idle before switching to manualMode
  //turn right
  odrive.SetVelocity(R0_MOTOR, Output);
  odrive.SetVelocity(L1_MOTOR, Output);
  // Sprintln(Output);
  //turn left
}
void manualMode(){
//TODO: Crawler orientation mapping on joystick
      if (BTN_TRIANGLE && (prevButton!=MeJOYSTICK_TRIANGLE))//&& (prevButton!=MeJOYSTICK_TRIANGLE)
      {
        if ((StartMills - TimeOfLastDebounce) > 300) {
          Serial1 << "r axis" << L1_MOTOR << ".encoder.is_ready\n";
          bool L_done=odrive.readInt();
          Serial1 << "r axis" << R0_MOTOR << ".encoder.is_ready\n";
          bool R_done=odrive.readInt();
          if(!L_done && !R_done){//If not calibrated yet
            odrive.SetParam(left_params);
            odrive.SetParam(right_params);
            requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
            while(L_done==0){
              odrive.run_state(L1_MOTOR, requested_state, true);
              delay(50);
              Serial1 << "r axis" << L1_MOTOR << ".encoder.is_ready\n";
              delay(50);
              L_done=odrive.readInt();
              Sprintln("stuck 2");
            }
            while(R_done==0){
              odrive.run_state(R0_MOTOR, requested_state, true);
              delay(50);
              Serial1 << "r axis" << R0_MOTOR << ".encoder.is_ready\n";
              delay(50);
              R_done=odrive.readInt();
              Sprintln("stuck 1");
            }

          }
            requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
            odrive.run_state(R0_MOTOR, requested_state, false); // don't wait
            delay(50);
            odrive.run_state(L1_MOTOR, requested_state, false); // don't wait
            delay(50);
            Sprintln("stuck 3");
          TimeOfLastDebounce = StartMills;
        }
        prevButton=MeJOYSTICK_TRIANGLE;
      }
      if (BTN_X)//Disable motors
      {
        requested_state = ODriveArduino::AXIS_STATE_IDLE;
        odrive.run_state(R0_MOTOR, requested_state, false);
        odrive.run_state(L1_MOTOR, requested_state, false);
        prevButton=MeJOYSTICK_XSHAPED;
        Sprintln("stuck 4");
      }

      if (BTN_SQUARE)//reset encoder
      {
        Serial1 << "w axis" << L1_MOTOR << ".encoder.set_linear_count(0)\n";
        Serial1 << "w axis" << R0_MOTOR << ".encoder.set_linear_count(0)\n";
      }

      if((MePS2.MeAnalog(MeJOYSTICK_LY)) && (requested_state == ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL)){//directional motion
        // if ((StartMills - TimeOfLastDebounce) > DelayofDebounce) {
          //Forward
          long leftAnalogY = map(MePS2.MeAnalog(MeJOYSTICK_LY),0,255,1000,VEL_LIMIT);
          odrive.SetVelocity(R0_MOTOR, -leftAnalogY);
          odrive.SetVelocity(L1_MOTOR, leftAnalogY);
          Sprint("R_speed= ");
          // Sprintln(odrive.GetVelocity(R0_MOTOR));
          Sprint("L_speed= ");
          // Sprintln(odrive.GetVelocity(L1_MOTOR));
          // TimeOfLastDebounce = StartMills;
          // Sprintln("debounced++++++++++++++++++++++");
        // }
        dirButtonpressed=1;

      }else if(BTN_DOWN){
        // if ((StartMills - TimeOfLastDebounce) > DelayofDebounce) {
          odrive.SetVelocity(R0_MOTOR, 10000);
          odrive.SetVelocity(L1_MOTOR, -10000);
          TimeOfLastDebounce = StartMills;
        // }
        dirButtonpressed=1;
      }else if(MePS2.MeAnalog(MeJOYSTICK_LX) && (requested_state == ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL)){
        if ((StartMills - TimeOfLastDebounce) > DelayofDebounce) {
          long leftAnalogX = map(MePS2.MeAnalog(MeJOYSTICK_LX),0,255,1000,VEL_LIMIT);
          odrive.SetVelocity(R0_MOTOR, -leftAnalogX);
          odrive.SetVelocity(L1_MOTOR, -leftAnalogX);
          TimeOfLastDebounce = StartMills;
        }
        dirButtonpressed=1;
      }else if(BTN_RIGHT){
        if ((StartMills - TimeOfLastDebounce) > DelayofDebounce) {
          odrive.SetVelocity(R0_MOTOR, 10000);
          odrive.SetVelocity(L1_MOTOR, 10000);
          TimeOfLastDebounce = StartMills;
        }
        dirButtonpressed=1;
      }else{//motors stop
        odrive.SetVelocity(R0_MOTOR, 0);
        odrive.SetVelocity(L1_MOTOR, 0);
        dirButtonpressed=0;
    }

}
bool pure_Serial_Start=0;
void pure_Serial(){

  // if (Serial1.available()) {
    char c = Serial.read();
    int requested_state;
    if(!pure_Serial_Start|| c== 'h'){
      Serial.println("ODriveArduino");
      Serial.println("Setting parameters...");
      Serial.println("Ready!");
      Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
      Serial.println("Send the character 'm' to restart");
      Serial.println("Send the character '2' to go to closed loop control");
      Serial.println("Send the character '4' to idle");
      Serial.println("Send the character 'x' to Start stabilisation mode. Please wait 21s for gyro cal.");
      Serial.println("Send the character 'y' to Exit stabilisation mode");
      Serial.println("Send the character '3' to fetch data");
      Serial.println("Send the character 's' to exectue test move");
      Serial.println("Send the character 'b' to read bus voltage");
      Serial.println("Send the character 'p' to read motor positions in a 10s loop");
      pure_Serial_Start=1;
    }
    // Run calibration sequence
    if (c == 'm') {
      Serial1 << "sr" << '\n';
      delay(1000);
      // Serial1 << "w axis" << 0 << ".controller.config.pos_gain " << 20.0f << '\n';
      odriveSetupParams();
      Serial1 << "w axis" << L1_MOTOR << ".controller.config.control_mode"<< 2 << '\n';
      Serial1 << "w axis" << R0_MOTOR << ".controller.config.control_mode"<< 2 << '\n';
      Serial1 << "w axis" << R0_MOTOR << ".controller.config.vel_limit_tolerance "<<2<<'\n';
      Serial.println("restarted");
    }
    if (c == '0' || c == '1') {

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(R0_MOTOR, requested_state, true);
      delay(50);
      odrive.run_state(L1_MOTOR, requested_state, true);
      delay(50);
      setupFinishedTime = millis();
      Serial1 << "r axis" << L1_MOTOR << ".encoder.is_ready\n";
      Serial.println(odrive.readInt());
      Serial1 << "r axis" << c << ".contoroller.config.vel_gain\n";
      Serial.println(odrive.readFloat());
//      Serial << "r axis" << c << ": Requesting state'\n' ";
    }

    if (c == '2') {//LOOP
      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(R0_MOTOR, requested_state, false);
      odrive.run_state(L1_MOTOR, requested_state, false);
    }
    if (c == 'x') {
      if((StartMills-setupFinishedTime)>21000){
        myPID.SetMode(AUTOMATIC);
        mode_status=2;
        Setpoint = gyro.yaw_stable;//Initial orientation captured
        Sprint("AuotCorrectionMode entered. Setpoint: ");
        Sprintln(Setpoint);
      }else{
        Sprint("gyro is calibrating. ready in: ");
        Sprintln( 21-((StartMills-setupFinishedTime)/1000));
      }

    }
    if (c == 'y') {//exit auto correction
      mode_status=10;
      myPID.SetMode(MANUAL);
      Sprint("Exit AuotCorrectionMode");
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_IDLE;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(R0_MOTOR, requested_state, false);
      odrive.run_state(L1_MOTOR, requested_state, false);
      motorIdled = 1;
    }
    if (c == '3') {//fetch state
      Serial1 << "r axis" << R0_MOTOR << ".current_state\n";
      Serial.print("Current state of M0: ");
      Serial.println(odrive.readInt());

      Serial1 << "r axis" << L1_MOTOR << ".current_state\n";
      Serial.print("Current stateof M1: ");
      Serial.println(odrive.readInt());

      Serial1 << "r axis" << R0_MOTOR << ".controller.config.vel_limit_tolerance\n";
      Serial.print("Tolerance0: ");
      Serial.println(odrive.readFloat());

      Serial1 << "r axis" << R0_MOTOR << ".controller.config.pos_gain\n";
      Serial.print("pos_gain0: ");
      Serial.println(odrive.readFloat());

      Serial1 << "r axis" << R0_MOTOR << ".controller.config.vel_gain\n";
      Serial.print("vel_gain0: ");
      Serial.println(odrive.readFloat(),8);
    }
    if (c == '4') {//IDLE
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_IDLE;
      Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(R0_MOTOR, requested_state, false);
      odrive.run_state(L1_MOTOR, requested_state, false);
      motorIdled = 1;
    }

    // Sinusoidal test move
    if (c == 's') {

      if(motorIdled==1){
        requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
        Serial << "Axis" << c << ": Requesting state " << requested_state << '\n';
        odrive.run_state(R0_MOTOR, requested_state, false); // don't wait
        odrive.run_state(L1_MOTOR, requested_state, false); // don't wait
      }
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = 20000.0f * cos(ph);
        float pos_m1 = 20000.0f * sin(ph);
        odrive.SetPosition(R0_MOTOR, pos_m0);
        odrive.SetPosition(L1_MOTOR, pos_m1);
        delay(5);
      }
    }

    // Read bus voltage
    if (c == 'b') {
      Serial1 << "r vbus_voltage\n";
      Serial << "Vbus voltage: " << odrive.readFloat() << '\n';
    }

    // print motor positions in a 10s loop
    if (c == 'p') {
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while(millis() - start < duration) {
        for (int motor = 0; motor < 2; ++motor) {
          Serial1 << "r axis" << motor << ".encoder.pos_estimate\n";
          Serial << odrive.readFloat() << '\t';
        }
        Serial << '\n';
      }
    }

  // }else{
    Sprintln( "BOLLOX");
  // }
}

void odriveSetupParams(){
  odrive.SetParam(left_params);
  odrive.SetParam(right_params);
  for (int axis = 0; axis < 2; ++axis) {
    Serial1 << "w axis" << axis << ".controller.config.vel_limit " << VEL_LIMIT << '\n';
    Serial1 << "w axis" << axis << ".motor.config.current_lim " << CURRENT_LIM << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }
  delay(50);
}
