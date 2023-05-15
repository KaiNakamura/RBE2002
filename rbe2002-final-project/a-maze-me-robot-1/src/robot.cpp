/**
 * @file robot.cpp
 * @brief Contains the code for robot 1 which includes updating sensors,
 * checking for events, and changing the current robot state.
 * @version 1.0
 * @date 2023-05-01
 * 
 */

#include <Chassis.h>
#include <HC-SR04.h>
#include <LineSensor.h>
#include <ir_codes.h>
#include <robot.h>

HC_SR04 hc_sr04(17, 12);  // Echo, Trig
void ISR_HC_SR04(void) {
  hc_sr04.ISR_echo();
}

LineSensor lineSensor;

OpenMV camera;

RobotState robotState = IDLE;
Chassis chassis;

bool running = true;
Direction direction = NORTH;
int row = 0, col = 0;
bool hasCheckedCol1, isOnRamp;
double lastHeading;

unsigned long lastSendReady, lastSendTag;

/**
 * @brief Initialize the serial, chassis, ultrasonic, line sensor, and LED.
 * 
 */
void initialize(void) {
  Serial1.begin(115200);
  chassis.init();
  hc_sr04.init(ISR_HC_SR04);
  lineSensor.setup();
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
      hasCheckedCol1 = false;
      isOnRamp = false;
      break;
    case LINE_FOLLOWING:
      Serial.println("Line following");
      break;
    case CENTERING:
      Serial.println("Centering");
      digitalWrite(LED_PIN, HIGH);
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
      chassis.setWheelTargetSpeeds(TURNING_SPEED, -TURNING_SPEED);
      lastHeading = chassis.getHeadingInDegrees();
      break;
    case CLIMBING_RAMP:
      Serial.println("Climbing ramp");
      digitalWrite(LED_PIN, HIGH);
      chassis.resetEncoders();
      isOnRamp = true;
      break;
    case APPROACHING_END_OF_RAMP:
      Serial.println("Approaching end of ramp");
      break;
    case APPROACHING_APRIL_TAG:
      Serial.println("Approaching April Tag");
      chassis.setWheelTargetSpeeds(APPROACHING_APRIL_TAG_SPEED,
                                   APPROACHING_APRIL_TAG_SPEED);
      chassis.resetEncoders();
      break;
    case READING_APRIL_TAG:
      Serial.println("Reading April Tag");
      chassis.stop();
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
    case CLIMBING_RAMP:
      digitalWrite(LED_PIN, LOW);
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

  // Only check distance sensors if the robot is not on the ramp
  // This is to prevent ultrasonic interferance between robot 1 and 2
  if (!isOnRamp) {
    checkDistanceSensor();
  }

  switch (robotState) {
    case LINE_FOLLOWING:
      followLine(LINE_FOLLOWING_SPEED);
      if (isAtIntersection()) {
        setState(CENTERING);
      } else if (isClimbingRamp()) {
        setState(CLIMBING_RAMP);
      }
      break;
    case CENTERING:
      followLine(LINE_FOLLOWING_SPEED);
      if (isFinishedCentering()) {
        switch (col) {
          case 0:
            setState(TURNING_RIGHT);
            break;
          case 1:
            if (hasCheckedCol1) {
              setState(TURNING_RIGHT);
            } else if (direction == EAST) {
              setState(TURNING_LEFT);
            }
            break;
          case 2:
            if (direction == EAST) {
              setState(TURNING_LEFT);
            } else {
              setState(LINE_FOLLOWING);
            }
            break;
        }
      }
      break;
    case TURNING_180:
      if (isFinishedTurning180()) {
        if (hasCheckedCol1) {
          setState(TURNING_RIGHT);
        } else {
          setState(LINE_FOLLOWING);
        }
      }
      break;
    case CLIMBING_RAMP:
      followLine(CLIMBING_RAMP_SPEED);
      if (isApproachingEndOfRamp()) {
        setState(APPROACHING_END_OF_RAMP);
      }
      break;
    case APPROACHING_END_OF_RAMP:
      followLine(APPROACHING_APRIL_TAG_SPEED);
      if (isAtIntersection()) {
        setState(APPROACHING_APRIL_TAG);
      }
      break;
    case APPROACHING_APRIL_TAG:
      if (isFinishedApproachingAprilTag()) {
        setState(READING_APRIL_TAG);
      }
      break;
    case READING_APRIL_TAG:
      checkForAprilTag();
      if (millis() - lastSendReady > SEND_READY_REFRESH_RATE) {
        sendMessage("ready", String(true));
        lastSendReady = millis();
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
    case NUM_1:
      setState(TURNING_RIGHT);
      break;
    case PLAY_PAUSE:
      running = !running;
      break;
    default:
      break;
  }
}

/**
 * @brief Check for visible AprilTags and then handle them.
 * 
 */
void checkForAprilTag(void) {
  AprilTagDatum tag;
  if (camera.checkUART(tag)) {
#ifdef D__DEBUG_APRIL_TAG__
    Serial.print(F("Tag [cx="));
    Serial.print(tag.cx);
    Serial.print(F(", cy="));
    Serial.print(tag.cy);
    Serial.print(F(", w="));
    Serial.print(tag.w);
    Serial.print(F(", h="));
    Serial.print(tag.h);
    Serial.print(F(", id="));
    Serial.print(tag.id);
    Serial.print(F(", rot="));
    Serial.print(tag.rot);
    Serial.println(F("]"));
#endif

    handleAprilTag(tag);
  }
}

/**
 * @brief Handle AprilTags by sending them over MQTT.
 * 
 * @param tag The AprilTag to handle.
 */
void handleAprilTag(AprilTagDatum tag) {
  if (millis() - lastSendTag > SEND_TAG_REFRESH_RATE) {
    sendMessage("tag/id", String(tag.id));
    sendMessage("tag/rotation", String(tag.rot));
    lastSendTag = millis();
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
    case TURNING_LEFT:
      if (isFinishedTurningLeft()) {
        if (distanceReading < MIN_WALL_DISTANCE) {
          setState(TURNING_180);
        } else {
          setState(LINE_FOLLOWING);
        }
      }
      break;
    case TURNING_RIGHT:
      if (isFinishedTurningRight()) {
        if (distanceReading < MIN_WALL_DISTANCE) {
          setState(TURNING_LEFT);
        } else {
          setState(LINE_FOLLOWING);
        }
      }
      break;
    case CENTERING:
      if (isFinishedCentering() && col == 1) {
        if (direction == NORTH) {
          if (distanceReading < MIN_WALL_DISTANCE) {
            setState(TURNING_180);
          } else {
            setState(LINE_FOLLOWING);
          }
        } else if (direction == SOUTH) {
          if (distanceReading < MIN_WALL_DISTANCE) {
            setState(TURNING_180);
            hasCheckedCol1 = true;
          } else {
            setState(LINE_FOLLOWING);
          }
        }
      }
      break;
    default:
      break;
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
 * @brief Returns whether the robot is climbing the ramp by checking whether
 * the pitch of the IMU is below a certain threshold (negative values are
 * pitched up). The function also makes sure the robot has traveled a minimum
 * distance to prevent false readings while the robot is braking hard or
 * accelerating from a standstill.
 * 
 * @return true The robot is climbing the ramp.
 * @return false The robot is not climbing the ramp.
 */
bool isClimbingRamp() {
  static bool previousIsClimbingRamp = false;
  bool returnValue = false;
  bool isClimbingRamp =
      chassis.getAveragePosition() > MIN_DISTANCE_BEFORE_PITCH_COMPARISON &&
      chassis.updatePitch() < RAMP_CLIMBING_PITCH_THRESHOLD;
  if (!previousIsClimbingRamp && isClimbingRamp) {
    returnValue = true;
  }
  previousIsClimbingRamp = isClimbingRamp;
  return returnValue;
}

/**
 * @brief Returns whether the robot is approaching the end of the ramp by
 * checking whether the pitch of the IMU is above a certain threshold (positive
 * values are pitched down). The function also makes sure the robot has traveled
 * a minimum distance to prevent false readings while the robot is braking hard
 * or accelerating from a standstill.
 * 
 * @return true The robot is approaching the end of the ramp.
 * @return false The robot is not approaching the end of the ramp.
 */
bool isApproachingEndOfRamp() {
  static bool previousIsApproachingEndOfRamp = false;
  bool returnValue = false;
  bool isApproachingEndOfRamp =
      chassis.getAveragePosition() > MIN_DISTANCE_BEFORE_PITCH_COMPARISON &&
      chassis.updatePitch() > APPROACHING_END_OF_RAMP_PITCH_THRESHOLD;
  if (!previousIsApproachingEndOfRamp && isApproachingEndOfRamp) {
    returnValue = true;
  }
  previousIsApproachingEndOfRamp = isApproachingEndOfRamp;
  return returnValue;
}

/**
 * @brief Returns whether the robot is finished approaching the AprilTag by
 * checking whether the robot has traveled a certain distance.
 * 
 * @return true The robot is finished approaching the AprilTag.
 * @return false The robot is not finished approaching the AprilTag.
 */
bool isFinishedApproachingAprilTag() {
  static bool previousIsApproachingAprilTag = false;
  bool returnValue = false;
  bool isApproachingAprilTag =
      chassis.getAveragePosition() > APPROACHING_APRIL_TAG_DISTANCE;
  if (!previousIsApproachingAprilTag && isApproachingAprilTag) {
    returnValue = true;
  }
  previousIsApproachingAprilTag = isApproachingAprilTag;
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
    case CLIMBING_RAMP:
      return "CLIMBING_RAMP";
    case APPROACHING_END_OF_RAMP:
      return "APPROACHING_END_OF_RAMP";
    case APPROACHING_APRIL_TAG:
      return "APPROACHING_APRIL_TAG";
    case READING_APRIL_TAG:
      return "READING_APRIL_TAG";
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
