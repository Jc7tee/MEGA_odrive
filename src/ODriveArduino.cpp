
#include "ODriveArduino.h"

// static const int kMotorOffsetFloat = 2;
// static const int kMotorStrideFloat = 28;
// static const int kMotorOffsetInt32 = 0;
// static const int kMotorStrideInt32 = 4;
// static const int kMotorOffsetBool = 0;
// static const int kMotorStrideBool = 4;
// static const int kMotorOffsetUint16 = 0;
// static const int kMotorStrideUint16 = 2;

// Print with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

ODriveArduino::ODriveArduino(HardwareSerial& serial)
: serial_(serial) {}

void ODriveArduino::SetParam(struct motor_Params M){
  serial_ << "w axis" << M.axis << ".controller.config.pos_gain " << M.pos_Gain << '\n';
  serial_ << "w axis" << M.axis << ".controller.config.vel_gain " << M.vel_Gain << '\n';
  serial_ << "w axis" << M.axis << ".controller.config.vel_integrator_gain " << M.vel_Int_Gain << '\n';
}

void ODriveArduino::TravelStraight(int masterSpeed, int slaveSpeed, int masterPin, int slavePin){
  //   //The powers we give to both motors. masterPower will remain constant while slavePower will change so that
  // //the right wheel keeps the same speed as the left wheel.
  // int masterPower = 30;
  // int slavePower = 30;
  //
  // //Essentially the difference between the master encoder and the slave encoder. Negative if slave has
  // //to slow down, positive if it has to speed up. If the motors moved at exactly the same speed, this
  // //value would be 0.
  // int error = 0;
  //
  // //'Constant of proportionality' which the error is divided by. Usually this is a number between 1 and 0 the
  // //error is multiplied by, but we cannot use floating point numbers. Basically, it lets us choose how much
  // //the difference in encoder values effects the final power change to the motor.
  // int kp = 5;
  //
  // //Reset the encoders.
  // SensorValue[leftEncoder] = 0;
  // SensorValue[rightEncoder] = 0;
  //
  // //Repeat ten times a second.
  // while(true)
  // {
  //   //Set the motor powers to their respective variables.
  //   motor[leftServo] = masterPower;
  //   motor[rightServo] = slavePower;
  //
  //   //This is where the magic happens. The error value is set as a scaled value representing the amount the slave
  //   //motor power needs to change. For example, if the left motor is moving faster than the right, then this will come
  //   //out as a positive number, meaning the right motor has to speed up.
  //   error = SensorValue[leftEncoder] - SensorValue[rightEncoder];
  //
  //   //This adds the error to slavePower, divided by kp. The '+=' operator literally means that this expression really says
  //   //"slavePower = slavepower + error / kp", effectively adding on the value after the operator.
  //   //Dividing by kp means that the error is scaled accordingly so that the motor value does not change too much or too
  //   //little. You should 'tune' kp to get the best value. For us, this turned out to be around 5.
  //   slavePower += error / kp;
  //
  //   //Reset the encoders every loop so we have a fresh value to use to calculate the error.
  //   SensorValue[leftEncoder] = 0;
  //   SensorValue[rightEncoder] = 0;
  //
  //   //Makes the loop repeat ten times a second. If it repeats too much we lose accuracy due to the fact that we don't have
  //   //access to floating point math, however if it repeats to little the proportional algorithm will not be as effective.
  //   //Keep in mind that if this value is changed, kp must change accordingly.
  //   wait1Msec(100);
// }
}
void ODriveArduino::SetPosition(int motor_number, float position) {
    SetPosition(motor_number, position, 0.0f, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward) {
    SetPosition(motor_number, position, velocity_feedforward, 0.0f);
}

void ODriveArduino::SetPosition(int motor_number, float position, float velocity_feedforward, float current_feedforward) {
    serial_ << "p " << motor_number  << " " << position << " " << velocity_feedforward << " " << current_feedforward << "\n";
}

void ODriveArduino::SetVelocity(int motor_number, float velocity) {
    SetVelocity(motor_number, velocity, 0.0f);
}

void ODriveArduino::SetVelocity(int motor_number, float velocity, float current_feedforward) {
    serial_ << "v " << motor_number  << " " << velocity << " " << current_feedforward << "\n";
}

void ODriveArduino::SetCurrent(int motor_number, float current) {
    serial_ << "c " << motor_number << " " << current << "\n";
}

void ODriveArduino::TrapezoidalMove(int motor_number, float position){
    serial_ << "t " << motor_number << " " << position << "\n";
}

float ODriveArduino::readFloat() {
    return readString().toFloat();
}

float ODriveArduino::GetVelocity(int motor_number){
	serial_<< "r axis" << motor_number << ".encoder.vel_estimate\n";
	return ODriveArduino::readFloat();
}

int32_t ODriveArduino::readInt() {
    return readString().toInt();
}

bool ODriveArduino::run_state(int axis, int requested_state, bool wait) {
    int timeout_ctr = 100;
    serial_ << "w axis" << axis << ".requested_state " << requested_state << '\n';
    if (wait) {
        do {
            delay(100);
            serial_ << "r axis" << axis << ".current_state\n";
        } while (readInt() != AXIS_STATE_IDLE && --timeout_ctr > 0);
    }

    return timeout_ctr > 0;
}

String ODriveArduino::readString() {
    String str = "";
    static const unsigned long timeout = 1000;
    unsigned long timeout_start = millis();
    for (;;) {
        while (!serial_.available()) {
            if (millis() - timeout_start >= timeout) {
                return str;
            }
        }
        char c = serial_.read();
        if (c == '\n')
            break;
        str += c;
    }
    return str;
}
