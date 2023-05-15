/**
 * @file robot.cpp
 * @brief Contains the code for robot 2 which includes updating sensors,
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
#include <ir_codes.h>
#include <robot.h>
#include <string.h>

HC_SR04 hc_sr04(17, 12);  // Echo, Trig
void ISR_HC_SR04(void) {
  hc_sr04.ISR_echo();
}

LineSensor lineSensor;

IRDirectionFinder irFinder;

RobotState robotState = IDLE;
Chassis chassis;

bool running = true;
Direction direction = NORTH;
int row = 0, col = 0;
double lastHeading;
bool hasLookedLeft;

unsigned long lastSerial1Check, lastPushingButton, lastSendCode;

/**
 * @brief Initialize the serial, chassis, ultrasonic, line sensor, and LED.
 * 
 */
void initialize(void) {
  Serial1.begin(115200);
  chassis.init();
  hc_sr04.init(ISR_HC_SR04);
  lineSensor.setup();
  irFinder.begin();
  pinMode(LED_PIN, OUTPUT);
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
      hasLookedLeft = false;
      break;
    case LINE_FOLLOWING:
      Serial.println("Line following");
      break;
    case CENTERING:
      Serial.println("Centering");
      digitalWrite(LED_PIN, HIGH);
      chassis.resetEncoders();
      hasLookedLeft = false;
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
      chassis.setWheelTargetSpeeds(TURNING_SPEED, -TURNING_SPEED);
      lastHeading = chassis.getHeadingInDegrees();
      break;
    case APPROACHING_BUTTON:
      Serial.println("Approaching button");
      break;
    case WAITING_FOR_ROBOT_1:
      Serial.println("Waiting for Robot 1");
      chassis.stop();
      break;
    case GETTING_CLOSE_TO_BUTTON:
      Serial.println("Getting close to button");
      sendMessage("code", String(IREmitter::digitsToCode('0' + col, '0' + row,
                                                         '0' + direction)));
      break;
    case PUSHING_BUTTON:
      Serial.println("Pushing button");
      lastPushingButton = millis();
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
    case LINE_FOLLOWING:
      incrementCoordinates();
      break;
    case CENTERING:
      digitalWrite(LED_PIN, LOW);
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

  // If not running, stop the chassis
  if (!running) {
    chassis.stop();
    return;
  }

  lineSensor.update();

  checkIRFinder();
  checkDistanceSensor();

  switch (robotState) {
    case IDLE:
      if (row != 0 || col != 0) {
        if (millis() - lastSendCode > SEND_CODE_REFRESH_RATE) {
          sendMessage("code", String(IREmitter::digitsToCode(
                                  '0' + col, '0' + row, '0' + direction)));
          lastSendCode = millis();
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
      break;
    case APPROACHING_BUTTON:
      followLine(LINE_FOLLOWING_SPEED);
      break;
    case WAITING_FOR_ROBOT_1:
      checkSerial1();
      break;
    case GETTING_CLOSE_TO_BUTTON:
      followLine(PUSHING_BUTTON_SPEED);
      break;
    case PUSHING_BUTTON:
      followLine(PUSHING_BUTTON_SPEED);
      if (isFinishedPushingButton()) {
        setState(IDLE);
      }
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
  switch (keyCode) {
    case NUM_2:
      chassis.reset();
      setState(LINE_FOLLOWING);
      break;
    case PLAY_PAUSE:
      running = !running;
      break;
    default:
      break;
  }
}

/**
 * @brief Check for IR readings from the IR finder and handle them.
 * 
 */
void checkIRFinder(void) {
  static uint32_t lastIRread = 0;
  if (millis() - lastIRread > IR_FINDER_REFRESH_RATE) {
    irFinder.requestPosition();
    lastIRread = millis();
  }

  if (irFinder.available()) {
    Point point = irFinder.ReadPoint(0);
    handleNewIRReading(point.x, point.y);
  }
}

/**
 * @brief Handle IR readings from the IR finder depending on the current robot
 * state.
 * 
 * @param x The x coordinate of the IR reading
 * @param y The y coordinate of the IR reading
 */
void handleNewIRReading(int x, int y) {
#ifdef __DEBUG_IR_FINDER__
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
#endif

  switch (robotState) {
    case TURNING_LEFT:
      if (isFinishedTurningLeft()) {
        if (x != 1023 || y != 1023) {
          sendMessage("ir/x", String(x));
          sendMessage("ir/y", String(y));
          setState(APPROACHING_BUTTON);
        } else {
          if (hasLookedLeft) {
            setState(LINE_FOLLOWING);
          } else {
            setState(TURNING_180);
            hasLookedLeft = true;
          }
        }
      }
      break;
    case TURNING_RIGHT:
      if (isFinishedTurningRight()) {
        if (x != 1023 || y != 1023) {
          sendMessage("ir/x", String(x));
          sendMessage("ir/y", String(y));
          setState(APPROACHING_BUTTON);
        }
      }
      break;
    case TURNING_180:
      if (isFinishedTurning180()) {
        if (x != 1023 || y != 1023) {
          sendMessage("ir/x", String(x));
          sendMessage("ir/y", String(y));
          setState(APPROACHING_BUTTON);
        } else {
          setState(TURNING_LEFT);
        }
      }
      break;
    case CENTERING:
      if (isFinishedCentering()) {
        if (x != 1023 || y != 1023) {
          sendMessage("ir/x", String(x));
          sendMessage("ir/y", String(y));
          setState(APPROACHING_BUTTON);
        }
      }
      break;
    case LINE_FOLLOWING:
      if (x != 1023 || y != 1023) {
        sendMessage("ir/x", String(x));
        sendMessage("ir/y", String(y));
        setState(APPROACHING_BUTTON);
      }
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
    case TURNING_RIGHT:
      if (isFinishedTurningRight()) {
        if (distanceReading < MIN_WALL_DISTANCE) {
          setState(TURNING_RIGHT);
        } else {
          setState(LINE_FOLLOWING);
        }
      }
      break;
    case CENTERING:
      if (isFinishedCentering()) {
        if (distanceReading < MIN_WALL_DISTANCE) {
          setState(TURNING_RIGHT);
        } else {
          if (col == 0) {
            setState(LINE_FOLLOWING);
          } else {
            setState(TURNING_LEFT);
          }
        }
      }
      break;
    case APPROACHING_BUTTON:
      if (distanceReading < APPROACHING_BUTTON_DISTANCE) {
        setState(WAITING_FOR_ROBOT_1);
      }
      break;
    case GETTING_CLOSE_TO_BUTTON:
      if (distanceReading < PUSHING_BUTTON_DISTANCE) {
        setState(PUSHING_BUTTON);
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

  if (robotState == WAITING_FOR_ROBOT_1) {
    if (message.indexOf("robot1/ready:1") >= 0) {
      setState(GETTING_CLOSE_TO_BUTTON);
    }
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
 * @brief Returns whether the robot is finished pushing the button by checking
 * whether the robot has driven forward for a given amount of time. We are
 * using time based control here because the ultrasonic sensor gives false
 * readings when too close the the wall and the wheels can sometimes get stuck
 * if the robot is right against a wall.
 * 
 * @return true The robot is finished pushing the button.
 * @return false The robot is not finished pushing the button.
 */
bool isFinishedPushingButton() {
  static bool previousIsPushingButton = false;
  bool returnValue = false;
  bool isPushingButton = millis() - lastPushingButton > PUSHING_BUTTON_TIME;
  if (!previousIsPushingButton && isPushingButton) {
    returnValue = true;
  }
  previousIsPushingButton = isPushingButton;
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
