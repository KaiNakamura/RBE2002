/**
 * @file robot.h
 * @brief Contains the constants and function signatures for the robot.
 * @version 1.0
 * @date 2023-05-01
 * 
 */

#pragma once

#include <Arduino.h>
#include <Chassis.h>
#include <openmv.h>

enum RobotState {
  IDLE,
  LINE_FOLLOWING,
  CENTERING,
  TURNING_LEFT,
  TURNING_RIGHT,
  TURNING_180,
  CLIMBING_RAMP,
  APPROACHING_END_OF_RAMP,
  APPROACHING_APRIL_TAG,
  READING_APRIL_TAG,
};

enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

constexpr double LINE_FOLLOWING_SPEED = 20;        // cm / s
constexpr double TURNING_SPEED = 12;               // cm / s
constexpr double CLIMBING_RAMP_SPEED = 10;         // cm / s
constexpr double APPROACHING_APRIL_TAG_SPEED = 5;  // cm / s
constexpr double K_P_LINE_FOLLOWING = 9;
constexpr double LINE_THRESHOLD = 0.35;
constexpr double RAMP_CLIMBING_PITCH_THRESHOLD = -8.75;         // degrees
constexpr double APPROACHING_END_OF_RAMP_PITCH_THRESHOLD = -1;  // degrees
constexpr double MIN_DISTANCE_BEFORE_PITCH_COMPARISON = 15;     // cm
constexpr double CENTERING_DISTANCE = 10;                       // cm
constexpr double APPROACHING_APRIL_TAG_DISTANCE = 10;           // cm
constexpr double PUSHING_BUTTON_DISTANCE = 7.5;                 // cm
constexpr double MIN_WALL_DISTANCE = 20;                        // cm
const int LED_PIN = 13;
const int SEND_READY_REFRESH_RATE = 1000;  // ms
const int SEND_TAG_REFRESH_RATE = 250;     // ms

void initialize(void);

void update(void);

void setState(RobotState state);
String stateToString(RobotState state);
void handleKeyCode(int16_t keyCode);

void checkForAprilTag(void);
void handleAprilTag(AprilTagDatum tag);
void checkDistanceSensor(void);
void handleNewDistanceReading(float distanceReading);
void followLine(double speed);
bool isFinishedCentering(void);
bool isFinishedTurningLeft(void);
bool isFinishedTurningRight(void);
bool isFinishedTurning180(void);
bool isAtIntersection(void);
bool isClimbingRamp(void);
bool isApproachingEndOfRamp(void);
bool isFinishedApproachingAprilTag(void);

Direction getDirectionToLeft(Direction direction);
Direction getDirectionToRight(Direction direction);
Direction getDirection180(Direction direction);
String directionToString(Direction direction);
void changeDirectionToLeft(void);
void changeDirectionToRight(void);
void changeDirection180(void);
void sendDirection(void);

void incrementCoordinates(void);
void sendCoordinates(void);
void sendHeading(void);
void sendPose(void);

void sendMessage(const String& topic, const String& message, int robotNumber);
void sendMessage(const String& topic, const String& message);
