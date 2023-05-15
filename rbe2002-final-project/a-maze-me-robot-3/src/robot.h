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

enum RobotState {
  IDLE,
  WAITING_FOR_TAG_ID,
  WAITING_FOR_TAG_ROTATION,
  WAITING_FOR_CODE,
  LINE_FOLLOWING,
  CENTERING,
  TURNING_LEFT,
  TURNING_RIGHT,
  TURNING_180,
  EMITTING_CODE
};

enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };
enum TurningDirection { NO_TURN, TURN_LEFT, TURN_RIGHT, TURN_180 };

constexpr double LINE_FOLLOWING_SPEED = 20;  // cm / s
constexpr double TURNING_SPEED = 12;         // cm / s
constexpr double K_P_LINE_FOLLOWING = 9;
constexpr double LINE_THRESHOLD = 0.35;
constexpr double CENTERING_DISTANCE = 7.75;             // cm
constexpr double MIN_WALL_DISTANCE = 20;                // cm
const unsigned int CHECK_SERIAL_1_REFRESH_RATE = 1000;  // ms
const unsigned int EMIT_CODE_REFRESH_RATE = 1000;       // ms

void initialize(void);

void update(void);

void setState(RobotState state);
String stateToString(RobotState state);
void handleKeyCode(int16_t keyCode);

void checkDistanceSensor(void);
void handleNewDistanceReading(float distanceReading);
bool hasNewSerial1Message(String& message);
void checkSerial1(void);
void handleNewSerial1Reading(String message);
String getStringFromMessage(String message);
int getIntFromMessage(String message);
long getLongFromMessage(String message);
Direction getDirectionFromRotation(int rotation);
void followLine(double speed);
bool isFinishedWaitingForTagId(void);
bool isFinishedWaitingForTagRotation(void);
bool isFinishedWaitingForCode(void);
bool isFinishedCentering(void);
bool isFinishedTurningLeft(void);
bool isFinishedTurningRight(void);
bool isFinishedTurning180(void);
bool isAtIntersection(void);
bool isValidDoorCell(int row, int col);
void setExitCell(void);
bool isInDoorCell(void);
bool isInExitCell(void);
bool isFinishedEmittingCode(void);

Direction getDirectionToLeft(Direction direction);
Direction getDirectionToRight(Direction direction);
Direction getDirection180(Direction direction);
String directionToString(Direction direction);
TurningDirection getTurningDirection(void);
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
