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
  LINE_FOLLOWING,
  CENTERING,
  TURNING_LEFT,
  TURNING_RIGHT,
  TURNING_180,
  APPROACHING_BUTTON,
  WAITING_FOR_ROBOT_1,
  GETTING_CLOSE_TO_BUTTON,
  PUSHING_BUTTON,
};

enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

constexpr double LINE_FOLLOWING_SPEED = 20;  // cm / s
constexpr double TURNING_SPEED = 12;         // cm / s
constexpr double PUSHING_BUTTON_SPEED = 8;   // cm / s
constexpr double K_P_LINE_FOLLOWING = 9;
constexpr double LINE_THRESHOLD = 0.35;
constexpr double CENTERING_DISTANCE = 7.75;         // cm
constexpr double APPROACHING_BUTTON_DISTANCE = 20;  // cm
constexpr double PUSHING_BUTTON_DISTANCE = 6;       // cm
constexpr double PUSHING_BUTTON_TIME = 1000;        // ms
constexpr double MIN_WALL_DISTANCE = 20;            // cm
constexpr int LED_PIN = 13;
const unsigned int IR_FINDER_REFRESH_RATE = 50;         // ms
const unsigned int CHECK_SERIAL_1_REFRESH_RATE = 1000;  // ms
const unsigned int SEND_CODE_REFRESH_RATE = 1000;       // ms

void initialize(void);

void update(void);

void setState(RobotState state);
String stateToString(RobotState state);
void handleKeyCode(int16_t keyCode);

void checkIRFinder(void);
void handleNewIRReading(int x, int y);
void checkDistanceSensor(void);
void handleNewDistanceReading(float distanceReading);
bool hasNewSerial1Message(String& message);
void checkSerial1(void);
void handleNewSerial1Reading(String message);
void followLine(double speed);
bool isFinishedCentering(void);
bool isFinishedTurningLeft(void);
bool isFinishedTurningRight(void);
bool isFinishedTurning180(void);
bool isAtIntersection(void);
bool isFinishedPushingButton(void);

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
