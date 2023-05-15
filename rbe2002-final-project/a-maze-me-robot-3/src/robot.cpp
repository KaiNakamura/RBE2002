/**
 * @file robot.cpp
 * @brief Contains the code for robot 3 which includes updating sensors,
 * checking for events, and changing the current robot state.
 * @version 1.0
 * @date 2023-05-01
 * 
 */

#include <Chassis.h>
#include <HC-SR04.h>
#include <IRDirectionFinder.h>
#include <IREmitter.h>
#include <LineSensor.h>
#include <Pose.h>
#include <Romi32U4Buttons.h>
#include <StatusLED.h>
#include <ir_codes.h>
#include <robot.h>
#include <string.h>

HC_SR04 hc_sr04(17, 12);  // Echo, Trig
void ISR_HC_SR04(void) {
  hc_sr04.ISR_echo();
}

LineSensor lineSensor;
IREmitter irEmitter;
StatusLED statusLed;

RobotState robotState = IDLE;
Chassis chassis;

bool running = true;
Direction direction = NORTH;
int row = 0, col = 0;
bool hasTagId, hasTagRotation, hasCode;
Direction doorDirection;
int doorRow, doorCol;
int exitRow, exitCol;
long code;

double lastHeading;

unsigned long lastSerial1Check, lastEmittingCode;

/**
 * @brief Initialize the serial, chassis, ultrasonic, line sensor, and LED.
 * 
 */
void initialize(void) {
  Serial1.begin(115200);
  chassis.init();
  hc_sr04.init(ISR_HC_SR04);
  lineSensor.setup();
}

/**
 * @brief This function handles all the common achievement goals between states.
 * Rather than duplicating lines of code for multiple different transitions,
 * it's better to add shared achievement goals in this function. For example,
 * whenever the CENTERING state is entered, we always want to turn on the LED
 * and whenever we leave the CENTERING state we want to turn off the LED.
 * By using the setState function, we can create this behavior without needing
 * to add digitalWrite(LED_PIN, HIGH) and digitalWrite(LED_PIN, LOW) to every
 * transition into and out of CENTERING.
 * 
 * @param state The state to change to.
 */
void setState(RobotState state) {
  // Achievement goals for entering a new state.
  // Not all achievement goals are present, only ones shared across all
  // transitions entering the given new state.
  switch (state) {
    case IDLE:
      Serial.println("Idle");
      chassis.stop();
      break;
    case WAITING_FOR_TAG_ID:
      Serial.println("Waiting for tag id");
      chassis.stop();
      statusLed.setMode(LED_ON);
      hasTagId = false;
      break;
    case WAITING_FOR_TAG_ROTATION:
      Serial.println("Waiting for tag rotation");
      chassis.stop();
      statusLed.setMode(LED_SLOW_BLINK);
      hasTagRotation = false;
      break;
    case WAITING_FOR_CODE:
      Serial.println("Waiting for code");
      chassis.stop();
      statusLed.setMode(LED_FAST_BLINK);
      hasCode = false;
      break;
    case LINE_FOLLOWING:
      Serial.println("Line following");
      break;
    case CENTERING:
      Serial.println("Centering");
      statusLed.setMode(LED_ON);
      chassis.resetEncoders();
      break;
    case TURNING_LEFT:
      Serial.println("Turning left");
      chassis.setWheelTargetSpeeds(-TURNING_SPEED, TURNING_SPEED);
      lastHeading = chassis.getHeadingInDegrees();
      break;
    case TURNING_RIGHT:
      Serial.println("Turning right");
      chassis.setWheelTargetSpeeds(TURNING_SPEED, -TURNING_SPEED);
      lastHeading = chassis.getHeadingInDegrees();
      break;
    case TURNING_180:
      Serial.println("Turning 180");
      chassis.setWheelTargetSpeeds(-TURNING_SPEED, TURNING_SPEED);
      lastHeading = chassis.getHeadingInDegrees();
      break;
    case EMITTING_CODE:
      Serial.println("Emitting code");

      // Stop motors and force chassis update.
      // This needs to be done because irEmitter.emitCode() is halting.
      chassis.stop();
      while (!chassis.loop())
        ;

      irEmitter.emitCode(code);
      lastEmittingCode = millis();
      break;
    default:
      break;
  }

  // Achievement goals for leaving an old state.
  // Not all achievement goals are present, only ones shared across all
  // transitions leaving the old state.
  switch (robotState) {
    case IDLE:
      sendCoordinates();
      sendDirection();
      break;
    case WAITING_FOR_TAG_ROTATION:
      setExitCell();
      break;
    case WAITING_FOR_CODE:
      statusLed.setMode(LED_OFF);
      break;
    case LINE_FOLLOWING:
      incrementCoordinates();
      break;
    case CENTERING:
      statusLed.setMode(LED_OFF);
      sendPose();
      break;
    case TURNING_LEFT:
      changeDirectionToLeft();
      sendHeading();
      break;
    case TURNING_RIGHT:
      changeDirectionToRight();
      sendHeading();
      break;
    case TURNING_180:
      changeDirection180();
      sendHeading();
      break;
    default:
      break;
  }

  robotState = state;
  sendMessage("state", stateToString(robotState));
}

/**
 * @brief Update the chassis, sensors, and robot state.
 * 
 */
void update(void) {
  /**
    * Chassis::loop() returns true when the motor control loop fires. We can use that timer to trigger
    * any number of processes that we want to run on the same schedule, for example, the line following
    * controller.
    */
  chassis.loop();

  statusLed.update();

  // If not running, stop the chassis
  if (!running) {
    chassis.stop();
    return;
  }

  lineSensor.update();

  checkDistanceSensor();

  switch (robotState) {
    case WAITING_FOR_TAG_ID:
      checkSerial1();
      if (isFinishedWaitingForTagId()) {
        setState(WAITING_FOR_TAG_ROTATION);
      }
      break;
    case WAITING_FOR_TAG_ROTATION:
      checkSerial1();
      if (isFinishedWaitingForTagRotation()) {
        setState(WAITING_FOR_CODE);
      }
      break;
    case WAITING_FOR_CODE:
      checkSerial1();
      if (isFinishedWaitingForCode()) {
        if (doorDirection == SOUTH || doorDirection == EAST) {
          setState(LINE_FOLLOWING);
        } else {
          setState(TURNING_RIGHT);
        }
      }
      break;
    case LINE_FOLLOWING:
      followLine(LINE_FOLLOWING_SPEED);
      if (isAtIntersection()) {
        setState(CENTERING);
      }
      break;
    case CENTERING:
      followLine(LINE_FOLLOWING_SPEED);
      if (isFinishedCentering()) {
        if (isInDoorCell()) {  // Is in door cell
          switch (getTurningDirection()) {
            case TURN_LEFT:
              setState(TURNING_LEFT);
              break;
            case TURN_RIGHT:
              setState(TURNING_RIGHT);
              break;
            case TURN_180:
              setState(TURNING_180);
              break;
            default:
              setState(EMITTING_CODE);
              break;
          }
        } else if (isInExitCell()) {  // Is in exit cell
          setState(IDLE);
        } else if (row == doorRow &&
                   col != doorCol) {  // Correct row, but not col
          setState(TURNING_RIGHT);
        } else if (col == doorCol &&
                   row != doorRow) {  // Correct col, but not row
          setState(TURNING_LEFT);
        } else {
          setState(LINE_FOLLOWING);
        }
      }
      break;
    case TURNING_LEFT:
      if (isFinishedTurningLeft()) {
        if (isInDoorCell()) {
          setState(EMITTING_CODE);
        } else {
          setState(LINE_FOLLOWING);
        }
      }
      break;
    case TURNING_RIGHT:
      if (isFinishedTurningRight()) {
        if (isInDoorCell()) {
          setState(EMITTING_CODE);
        } else {
          setState(LINE_FOLLOWING);
        }
      }
      break;
    case TURNING_180:
      if (isFinishedTurning180()) {
        if (isInDoorCell()) {
          setState(EMITTING_CODE);
        } else {
          setState(LINE_FOLLOWING);
        }
      }
      break;
    case EMITTING_CODE:
      checkSerial1();
      break;
    default:
      break;
  }
}

/**
 * @brief Handle inputs from the IR remote.
 * 
 * @param keyCode The keycode of the IR remote.
 */
void handleKeyCode(int16_t keyCode) {
  Serial.println(keyCode);

  switch (keyCode) {
    case NUM_3:
      chassis.reset();
      setState(WAITING_FOR_TAG_ID);
      break;
    case PLAY_PAUSE:
      running = !running;
      break;
    default:
      break;
  }
}

/**
 * @brief Check the ultrasonic sensor for new readings and handle new readings.
 * 
 */
void checkDistanceSensor(void) {
  float distanceReading = 0;
  bool hasNewReading = hc_sr04.getDistance(distanceReading);
  if (hasNewReading) {
    handleNewDistanceReading(distanceReading);
  }
}

/**
 * @brief Handle new distance reading events depending on the current robot
 * state.
 * 
 * @param distanceReading The distance reading to handle.
 */
void handleNewDistanceReading(float distanceReading) {
#ifdef __DEBUG_RANGEFINDER__
  Serial.println(distanceReading);
#endif

  switch (robotState) {
    case EMITTING_CODE:
      if (isFinishedEmittingCode()) {
        if (distanceReading > MIN_WALL_DISTANCE) {
          setState(LINE_FOLLOWING);
        } else {
          setState(EMITTING_CODE);
        }
      }
      break;
    default:
      break;
  }
}

/**
 * @brief Returns whether a new MQTT messge was received and stores the message
 * in a variable passed by reference.
 * 
 * @param message The variable to store the MQTT message.
 * @return true A new MQTT message was received.
 * @return false A new MQTT message was not received.
 */
bool hasNewSerial1Message(String& message) {
  while (Serial1.available()) {
    char c = Serial1.read();
    message += c;

    if (c == '\n') {
      return true;
    }
  }

  return false;
}

/**
 * @brief Checks the MQTT broker for new messages and handles them.
 * 
 */
void checkSerial1(void) {
  if (millis() - lastSerial1Check > CHECK_SERIAL_1_REFRESH_RATE) {
    String message = "";
    if (hasNewSerial1Message(message)) {
      handleNewSerial1Reading(message);
    }
    lastSerial1Check = millis();
  }
}

/**
 * @brief Handles new MQTT messages depending on the current robot state.
 * 
 * @param message The MQTT message to handle.
 */
void handleNewSerial1Reading(String message) {
#ifdef __DEBUG_SERIAL_1__
  Serial.println(message);
#endif

  switch (robotState) {
    case WAITING_FOR_TAG_ID:
      if (message.indexOf("robot1/tag/id") >= 0) {
        int id = getIntFromMessage(message);
        int col = id / 10;
        int row = id % 10;

        if (isValidDoorCell(row, col)) {
          hasTagId = true;
          doorCol = col;
          doorRow = row;
          sendMessage("door/col", String(doorCol));
          sendMessage("door/row", String(doorRow));
        }
      }
      break;
    case WAITING_FOR_TAG_ROTATION:
      if (message.indexOf("robot1/tag/rotation") >= 0) {
        int rotation = getIntFromMessage(message);
        hasTagRotation = true;
        doorDirection = getDirectionFromRotation(rotation);
        sendMessage("door/direction", directionToString(doorDirection));
      }
      break;
    case WAITING_FOR_CODE:
      if (message.indexOf("robot2/code") >= 0) {
        code = getLongFromMessage(message);
        if (IREmitter::isValidCode(code)) {
          hasCode = true;
          sendMessage("code", String(code));
        }
      }
      break;
    default:
      break;
  }
}

/**
 * @brief Get the string from an MQTT message.
 * 
 * @param message The MQTT message.
 * @return String A string from the MQTT message.
 */
String getStringFromMessage(String message) {
  return message.substring(message.indexOf(":") + 1, message.length());
}

/**
 * @brief Get an int value from an MQTT message.
 * 
 * @param message The MQTT message.
 * @return int An int value from the MQTT message.
 */
int getIntFromMessage(String message) {
  return atoi(getStringFromMessage(message).c_str());
}

/**
 * @brief Get an long value from an MQTT message.
 * 
 * @param message The MQTT message.
 * @return long An long value from the MQTT message.
 */
long getLongFromMessage(String message) {
  return atol(getStringFromMessage(message).c_str());
}

/**
 * @brief Convert an AprilTag rotation to a field direction.
 * 
 * @param rotation The rotation of the AprilTag.
 * @return Direction The direction of exit door from the door cell.
 */
Direction getDirectionFromRotation(int rotation) {
  if (rotation >= 45 && rotation < 135) {
    return WEST;
  } else if (rotation >= 135 && rotation < 225) {
    return SOUTH;
  } else if (rotation >= 225 && rotation < 315) {
    return EAST;
  } else {
    return NORTH;
  }
}

/**
 * @brief Follow the field lines using proportional control.
 * 
 * @param speed The speed in cm/s to follow the line at.
 */
void followLine(double speed) {
  lineSensor.update();
  double output = K_P_LINE_FOLLOWING * lineSensor.getValue();
  chassis.setWheelTargetSpeeds(speed - output, speed + output);
}

/**
 * @brief Is the robot finished waiting for an AprilTag ID.
 * 
 * @return true The robot is finished waiting for an AprilTag ID.
 * @return false The robot is not finished waiting for an AprilTag ID.
 */
bool isFinishedWaitingForTagId(void) {
  return hasTagId;
}

/**
 * @brief Is the robot finished waiting for an AprilTag rotation.
 * 
 * @return true The robot is finished waiting for an AprilTag rotation.
 * @return false The robot is not finished waiting for an AprilTag rotation.
 */

bool isFinishedWaitingForTagRotation(void) {
  return hasTagRotation;
}

/**
 * @brief Is the robot finished waiting for an IR code.
 * 
 * @return true The robot is finished waiting for an IR code.
 * @return false The robot is not finished waiting for an IR code.
 */
bool isFinishedWaitingForCode(void) {
  return hasCode;
}

/**
 * @brief Is the robot finished centering.
 * 
 * @return true The robot is finished centering.
 * @return false The robot is not finished centering.
 */
bool isFinishedCentering() {
  static bool previousIsFinishedCentering = false;
  bool returnValue = false;
  bool isFinishedCentering = chassis.getAveragePosition() > CENTERING_DISTANCE;
  if (!previousIsFinishedCentering && isFinishedCentering) {
    returnValue = true;
  }
  previousIsFinishedCentering = isFinishedCentering;
  return returnValue;
}

/**
 * @brief Is the robot finished turning left.
 * 
 * @return true The robot is finished turning left.
 * @return false The robot is not finished turning left.
 */
bool isFinishedTurningLeft() {
  static bool previousIsFinishedTurningLeft = false;
  bool returnValue = false;
  bool isFinishedTurningLeft = chassis.getHeadingInDegrees() - lastHeading > 90;
  if (!previousIsFinishedTurningLeft && isFinishedTurningLeft) {
    returnValue = true;
  }
  previousIsFinishedTurningLeft = isFinishedTurningLeft;
  return returnValue;
}

/**
 * @brief Is the robot finished turning right.
 * 
 * @return true The robot is finished turning right.
 * @return false The robot is not finished turning right.
 */
bool isFinishedTurningRight() {
  static bool previousIsFinishedTurningRight = false;
  bool returnValue = false;
  bool isFinishedTurningRight =
      chassis.getHeadingInDegrees() - lastHeading < -90;
  if (!previousIsFinishedTurningRight && isFinishedTurningRight) {
    returnValue = true;
  }
  previousIsFinishedTurningRight = isFinishedTurningRight;
  return returnValue;
}

/**
 * @brief Is the robot finished turning 180 degrees.
 * 
 * @return true The robot is finished turning 180 degrees.
 * @return false The robot is not finished turning 180 degrees.
 */
bool isFinishedTurning180() {
  static bool previousIsFinishedTurning180 = false;
  bool returnValue = false;
  bool isFinishedTurning180 =
      chassis.getHeadingInDegrees() - lastHeading < -180;
  if (!previousIsFinishedTurning180 && isFinishedTurning180) {
    returnValue = true;
  }
  previousIsFinishedTurning180 = isFinishedTurning180;
  return returnValue;
}

/**
 * @brief Returns whether the robot is at an intersection by checking whether
 * the left and right line sensor values are below a certain threshold.
 * 
 * @return true The robot is at an intersection.
 * @return false The robot is not at an intersection.
 */
bool isAtIntersection() {
  static bool previousIsAtIntersection = false;
  bool returnValue = false;
  bool isAtIntersection = lineSensor.getLeftValue() < LINE_THRESHOLD &&
                          lineSensor.getRightValue() < LINE_THRESHOLD;
  if (!previousIsAtIntersection && isAtIntersection) {
    returnValue = true;
  }
  previousIsAtIntersection = isAtIntersection;
  return returnValue;
}

/**
 * @brief Returns whether the given row and col is a valid door cell location
 * (i.e. Not the starting cell, the row is between 0 and 5, and the col is
 * between 0 and 2).
 * 
 * @param row 
 * @param col 
 * @return true 
 * @return false 
 */
bool isValidDoorCell(int row, int col) {
  return (row != 0 || col != 0) && row >= 0 && row <= 5 && col >= 0 && col <= 2;
}

/**
 * @brief Set the exit cell based on the door row and col and the door
 * direction.
 * 
 */
void setExitCell() {
  switch (doorDirection) {
    case NORTH:
      exitRow = doorRow + 1;
      exitCol = doorCol;
      break;
    case WEST:
      exitRow = doorRow;
      exitCol = doorCol - 1;
      break;
    case SOUTH:
      exitRow = doorRow - 1;
      exitCol = doorCol;
      break;
    case EAST:
      exitRow = doorRow;
      exitCol = doorCol + 1;
    default:
      break;
  }
}

/**
 * @brief Returns whether the robot is currently in the door cell.
 * 
 * @return true The robot is in the door cell.
 * @return false The robot is not in the door cell.
 */
bool isInDoorCell() {
  return row == doorRow && col == doorCol;
}

/**
 * @brief Returns whether the robot is currently in the exit cell.
 * 
 * @return true The robot is in the exit cell.
 * @return false The robot is not in the exit cell.
 */
bool isInExitCell() {
  return row == exitRow && col == exitCol;
}

/**
 * @brief Returns whether the robot is finished emitting the IR code.
 * 
 * @return true The robot is finished emitting the IR code.
 * @return false The robot is not finished emitting the IR code.
 */
bool isFinishedEmittingCode() {
  static bool previousIsFinishedEmittingCode = false;
  bool returnValue = false;
  bool isFinishedEmittingCode =
      millis() - lastEmittingCode > EMIT_CODE_REFRESH_RATE;
  if (!previousIsFinishedEmittingCode && isFinishedEmittingCode) {
    returnValue = true;
  }
  previousIsFinishedEmittingCode = isFinishedEmittingCode;
  return returnValue;
}

/**
 * @brief Convert a robot state to string. Used for MQTT messaging.
 * 
 * @param state The robot state.
 * @return String The robot state as a String.
 */
String stateToString(RobotState state) {
  switch (state) {
    case IDLE:
      return "IDLE";
    case LINE_FOLLOWING:
      return "LINE_FOLLOWING";
    case CENTERING:
      return "CENTERING";
    case TURNING_LEFT:
      return "TURNING_LEFT";
    case TURNING_RIGHT:
      return "TURNING_RIGHT";
    case TURNING_180:
      return "TURNING_180";
    case APPROACHING_BUTTON:
      return "APPROACHING_BUTTON";
    case WAITING_FOR_ROBOT_1:
      return "WAITING_FOR_ROBOT_1";
    case GETTING_CLOSE_TO_BUTTON:
      return "GETTING_CLOSE_TO_BUTTON";
    case PUSHING_BUTTON:
      return "PUSHING_BUTTON";
    default:
      return "UNKNOWN_STATE";
  }
}

/**
 * @brief Get the direction to left of the provided direction.
 * 
 * @param direction The current direction.
 * @return direction The direction to the left of the current direction.
 */
Direction getDirectionToLeft(Direction direction) {
  switch (direction) {
    case NORTH:
      return WEST;
    case WEST:
      return SOUTH;
    case SOUTH:
      return EAST;
    default:
      return NORTH;
  }
}

/**
 * @brief Get the direction to right of the provided direction.
 * 
 * @param direction The current direction.
 * @return direction The direction to the right of the current direction.
 */
Direction getDirectionToRight(Direction direction) {
  switch (direction) {
    case NORTH:
      return EAST;
    case EAST:
      return SOUTH;
    case SOUTH:
      return WEST;
    default:
      return NORTH;
  }
}

/**
 * @brief Get the direction 180 degrees of the provided direction.
 * 
 * @param direction The current direction.
 * @return direction The direction 180 degrees of the current direction.
 */
Direction getDirection180(Direction direction) {
  switch (direction) {
    case NORTH:
      return SOUTH;
    case WEST:
      return EAST;
    case SOUTH:
      return NORTH;
    default:
      return WEST;
  }
}

/**
 * @brief Convert a direction to string. Used for MQTT messaging.
 * 
 * @param direction The direction.
 * @return String The direction as a String.
 */
String directionToString(Direction direction) {
  switch (direction) {
    case NORTH:
      return "NORTH";
    case EAST:
      return "EAST";
    case SOUTH:
      return "SOUTH";
    case WEST:
      return "WEST";
  }

  return "UNKNOWN";
}

/**
 * @brief Get the direction the robot should turn based on the robot's current
 * direction and the direction of the exit door.
 * 
 * @return TurningDirection The turning direction of the robot.
 */
TurningDirection getTurningDirection() {
  switch (direction) {
    case NORTH:
      switch (doorDirection) {
        case NORTH:
          return NO_TURN;
        case WEST:
          return TURN_LEFT;
        case SOUTH:
          return TURN_180;
        case EAST:
          return TURN_RIGHT;
      }
      break;
    case WEST:
      switch (doorDirection) {
        case NORTH:
          return TURN_RIGHT;
        case WEST:
          return NO_TURN;
        case SOUTH:
          return TURN_LEFT;
        case EAST:
          return TURN_180;
      }
      break;
    case SOUTH:
      switch (doorDirection) {
        case NORTH:
          return TURN_180;
        case WEST:
          return TURN_RIGHT;
        case SOUTH:
          return NO_TURN;
        case EAST:
          return TURN_LEFT;
      }
      break;
    case EAST:
      switch (doorDirection) {
        case NORTH:
          return TURN_LEFT;
        case WEST:
          return TURN_180;
        case SOUTH:
          return TURN_RIGHT;
        case EAST:
          return NO_TURN;
      }
      break;
  }

  return NO_TURN;
}

/**
 * @brief Change the current direction to the left and send over MQTT.
 * 
 */
void changeDirectionToLeft(void) {
  direction = getDirectionToLeft(direction);
  sendDirection();
}

/**
 * @brief Change the current direction to the right and send over MQTT.
 * 
 */
void changeDirectionToRight(void) {
  direction = getDirectionToRight(direction);
  sendDirection();
}

/**
 * @brief Change the current direction by 180 degrees and send over MQTT.
 * 
 */
void changeDirection180(void) {
  direction = getDirection180(direction);
  sendDirection();
}

/**
 * @brief Send the current direction over MQTT.
 * 
 */
void sendDirection(void) {
  sendMessage("direction", directionToString(direction));
}

/**
 * @brief Increment the robot's current row and col coordinates based on its
 * current direction and send over MQTT.
 * 
 */
void incrementCoordinates(void) {
  switch (direction) {
    case NORTH:
      row++;
      break;
    case EAST:
      col++;
      break;
    case SOUTH:
      row--;
      break;
    case WEST:
      col--;
      break;
    default:
      break;
  }

  sendCoordinates();
}

/**
 * @brief Send the robot's current row and col coordinates over MQTT.
 * 
 */
void sendCoordinates(void) {
  sendMessage("cell/row", String(row));
  sendMessage("cell/col", String(col));
}

/**
 * @brief Send the robot's current heading over MQTT.
 * 
 */
void sendHeading(void) {
  // Convert heading to degrees
  sendMessage("pose/heading", String(chassis.getPose().heading * 180 / PI));
}

/**
 * @brief Send the robot's current pose over MQTT.
 * 
 */
void sendPose(void) {
  Pose pose = chassis.getPose();

  // Convert position to millimeters
  sendMessage("pose/x", String(pose.x * 10.0));
  sendMessage("pose/y", String(pose.y * 10.0));

  sendHeading();
}

/**
 * @brief Send a message over MQTT with a topic, message, and robot number.
 * 
 * @param topic The topic of the MQTT message.
 * @param message The message of the MQTT message.
 * @param robotNumber The robot number to send this message as. Can be changed
 * to simulate receiving an MQTT message from a different robot for debugging
 * purposes.
 */
void sendMessage(const String& topic, const String& message, int robotNumber) {
  Serial1.println("robot" + String(robotNumber) + "/" + topic + String(':') +
                  message);
}

/**
 * @brief Send a message over MQTT with a topic and message.
 * 
 * @param topic The topic of the MQTT message.
 * @param message The message of the MQTT message.
 */
void sendMessage(const String& topic, const String& message) {
  sendMessage(topic, message, __ROBOT_NUMBER__);
}
