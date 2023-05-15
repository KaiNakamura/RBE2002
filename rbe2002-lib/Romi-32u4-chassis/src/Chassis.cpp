#include <Arduino.h>
#include <Chassis.h>
#include <LSM6.h>
#include <Pose.h>
#include <Romi32U4Motors.h>
#include <Wire.h>  // I2C library

// We'll declare motors as global to make the ISRs happier, but we'll put them in Chassis.cpp
// to keep things organized

LeftMotor leftMotor;
RightMotor rightMotor;

LSM6 imu;

Pose pose;

Chassis::Chassis(void) {}

void Chassis::init(void) {

  noInterrupts();  //disable interupts while we mess with the Timer4 registers

  //sets up timer 4
  TCCR4A = 0x00;  //disable some functionality -- no need to worry about this
  TCCR4B = 0x0A;  //sets the prescaler -- look in the handout for values
  TCCR4C = 0x04;  //toggles pin 6 at the timer frequency
  TCCR4D = 0x00;  //normal mode

  /**
     * Here we do a little trick to allow full 10-bit register access. 
     * We have 2 bytes in TC4H that we can use to add capacity to TOP. 
     * In the end, this sets TOP = 2 * 256 + 112 = 624
    */
  TC4H = 2;
  OCR4C = 112;

  TIMSK4 = 0x04;  //enable overflow interrupt

  interrupts();  //re-enable interrupts

  // init the motors
  Romi32U4Motor::init();

  // pinMode(6, OUTPUT); //COMMENT THIS OUT TO SHUT UP THE PIEZO!!!

  Wire.begin();

  if (!imu.init()) {
    // Failed to detect the LSM6.
    while (1) {
      Serial.println(
          F("Failed to detect the LSM6. Just smash that reset button."));
      delay(100);
    }
  }

  imu.setGyroDataOutputRate(LSM6::ODR416);
  imu.setAccDataOutputRate(LSM6::ODR416);

  reset();
}

void Chassis::reset() {
  pose.x = INITIAL_POSE_X;
  pose.y = INITIAL_POSE_Y;
  pose.heading = INITIAL_POSE_HEADING;
  estimatedYawAngle = INITIAL_POSE_HEADING * 180 / PI;
}

bool Chassis::loop(void) {
  bool retVal = false;
  if (readyToPID) {
    if (readyToPID > 1) {
      Serial.println("Missed update in Chassis::loop()");
    }

    update();
    readyToPID = 0;
    retVal = true;
  }

  return retVal;
}

void Chassis::update(void) {
  leftMotor.update();
  rightMotor.update();
  updatePose();
#ifdef __MOTOR_DEBUG__
  Serial.print('\n');
#endif
}

void Chassis::setMotorTargetSpeeds(float leftTicksPerInterval,
                                   float rightTicksPerInterval) {
  leftMotor.setTargetSpeed(leftTicksPerInterval);
  rightMotor.setTargetSpeed(rightTicksPerInterval);
}

void Chassis::setWheelTargetSpeeds(float leftCmPerSecond,
                                   float rightCmPerSecond) {
  setMotorTargetSpeeds((leftCmPerSecond / cmPerEncoderTick) * updateInterval,
                       (rightCmPerSecond / cmPerEncoderTick) * updateInterval);
}

void Chassis::stop() {
  setMotorTargetSpeeds(0, 0);
}

void Chassis::resetEncoders() {
  leftMotor.getAndResetCount();
  rightMotor.getAndResetCount();
}

int16_t Chassis::getLeftEncoderCount() {
  return leftMotor.getCount();
}

int16_t Chassis::getRightEncoderCount() {
  return rightMotor.getCount();
}

double Chassis::getLeftPosition() {
  return getLeftEncoderCount() * cmPerEncoderTick;
}

double Chassis::getRightPosition() {
  return getRightEncoderCount() * cmPerEncoderTick;
}

double Chassis::getAveragePosition() {
  return (getLeftPosition() + getRightPosition()) / 2.0;
}

Pose Chassis::getPose() {
  return pose;
}

double Chassis::getHeadingInDegrees() {
  return pose.heading * 180 / PI;
}

bool Chassis::checkForNewIMUData() {
  return hasNewAccelerometerData() && hasNewGyroscopeData();
}

bool Chassis::hasNewAccelerometerData() {
  return imu.getStatus() & 0x01;
}

bool Chassis::hasNewGyroscopeData() {
  return imu.getStatus() & 0x02;
}

double Chassis::updatePitch() {
  if (hasNewAccelerometerData()) {
    imu.readAcc();
  }

  if (hasNewGyroscopeData()) {
    imu.readGyro();

    float prediction =
        estimatedPitchAngle + imu.rawGyroToDegrees(imu.g.y - pitchBias);
    float observation = atan2(-imu.a.x, imu.a.z) * 180 / M_PI;
    float correction = prediction + PITCH_KAPPA * (observation - prediction);

    estimatedPitchAngle = correction;
    pitchBias -=
        PITCH_BIAS_EPSILON * imu.degreesToRawGyro(correction - prediction);
  }

  return estimatedPitchAngle;
}

double Chassis::updateYaw() {
  if (hasNewGyroscopeData()) {
    imu.readGyro();

#ifdef __DEBUG_YAW_BIAS__
    Serial.println(imu.g.z);
#endif

    estimatedYawAngle += imu.rawGyroToDegrees(imu.g.z - YAW_BIAS);
  }

  return estimatedYawAngle;
}

void Chassis::updatePose() {
  float dLeft = leftMotor.speed * cmPerEncoderTick;
  float dRight = rightMotor.speed * cmPerEncoderTick;
  float d = (dLeft + dRight) / 2.0;

  if (dLeft == dRight) {  // Going straight
    pose.x += d * cos(pose.heading);
    pose.y += d * sin(pose.heading);
  } else {  // Not going straight
    float dTheta = (dRight - dLeft) / wheel_track;
    float r = robotRadius * (dLeft + dRight) / (dRight - dLeft);
    pose.x += r * (sin(pose.heading + dTheta) - sin(pose.heading));
    pose.y += r * (cos(pose.heading) - cos(pose.heading + dTheta));
    pose.heading += dTheta;
  }

#ifdef __DEBUG_POSE__
  Serial.print(pose.x * 10);
  Serial.print(", ");
  Serial.print(pose.y * 10);
  Serial.print(", ");
  Serial.println(pose.heading * 180 / PI);
#endif
}

/*
 * ISR for timing. On overflow, it takes a 'snapshot' of the encoder counts and raises a flag to let
 * the main program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect) {
  //Capture a "snapshot" of the encoder counts for later processing
  leftMotor.calcEncoderDelta();
  rightMotor.calcEncoderDelta();

  chassis.readyToPID++;
}
