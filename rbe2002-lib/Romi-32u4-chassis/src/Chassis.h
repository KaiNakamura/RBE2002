#pragma once

#include <Arduino.h>
#include <LSM6.h>
#include <Pose.h>
#include <Romi32U4Motors.h>

class Chassis {

 protected:
  constexpr static double PITCH_KAPPA = 0.05;
  constexpr static double PITCH_BIAS_EPSILON = 0.05;

#if __ROBOT_NUMBER__ == 1
  constexpr static double YAW_BIAS = -643.6640238;
#elif __ROBOT_NUMBER__ == 2
  constexpr static double YAW_BIAS = -514.0178218;
#elif __ROBOT_NUMBER__ == 3
  constexpr static double YAW_BIAS = -253.4910891;
#else
  constexpr static double YAW_BIAS = 0;
#endif

#if __FIELD_NUMBER__ == 1
  constexpr static double INITIAL_POSE_X = 21.9;  // cm
  constexpr static double INITIAL_POSE_Y = 22.0;  // cm
#elif __FIELD_NUMBER__ == 2
  constexpr static double INITIAL_POSE_X = 22.3;  // cm
  constexpr static double INITIAL_POSE_Y = 20.0;  // cm
#else
  constexpr static double INITIAL_POSE_X = 0;  // cm
  constexpr static double INITIAL_POSE_Y = 0;  // cm
#endif

  constexpr static double INITIAL_POSE_HEADING = PI / 2.0;  // radians

  // Kinematic parameters
  float wheel_track = 14.174;       // cm
  float wheel_diam = 7.293;         // cm
  float ticks_per_rotation = 1440;  // from the datasheet
  float cmPerEncoderTick = PI * wheel_diam / ticks_per_rotation;
  float robotRadius = wheel_track / 2.0;
  float updateInterval = 20.0 / 1000.0;  // s

  float estimatedPitchAngle, estimatedYawAngle;
  float pitchBias;

 public:
  uint8_t readyToPID = 0;

  Chassis(void);

  void init(void);
  bool loop(void);
  void update(void);

  void setMotorEfforts(int16_t left, int16_t right) {
    leftMotor.setMotorEffort(left);
    rightMotor.setMotorEffort(right);
  }

  void setMotorTargetSpeeds(float leftTicksPerInterval,
                            float rightTicksPerInterval);
  void setWheelTargetSpeeds(float leftTicksPerSecond,
                            float rightTicksPerSecond);

  void stop();

  void reset();
  bool checkForNewIMUData();
  bool hasNewAccelerometerData();
  bool hasNewGyroscopeData();
  double updatePitch();
  double updateYaw();
  void updatePose();

  void resetEncoders();
  int16_t getLeftEncoderCount();
  int16_t getRightEncoderCount();
  double getLeftPosition();
  double getRightPosition();
  double getAveragePosition();
  Pose getPose();
  double getHeadingInDegrees();
};

extern Chassis chassis;