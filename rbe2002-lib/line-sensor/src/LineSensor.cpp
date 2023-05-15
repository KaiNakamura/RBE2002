#include "LineSensor.h"

LineSensor::LineSensor() {}

void LineSensor::setup() {
  pinMode(LEFT_PIN, INPUT);
  pinMode(RIGHT_PIN, INPUT);
  reset();
}

void LineSensor::update() {
  // Read sensor values
  leftRaw += analogRead(LEFT_PIN);
  rightRaw += analogRead(RIGHT_PIN);
  numReads++;

  // If we've read enough values, take averages
  if (numReads >= MAX_NUM_READS) {

// For calibration
#ifdef __LINE_DEBUG__
    Serial.print(leftRaw / numReads);
    Serial.print("\t");
    Serial.println(rightRaw / numReads);
#endif

    leftValue = lerp(leftRaw / numReads, LEFT_WHITE, LEFT_BLACK);
    rightValue = lerp(rightRaw / numReads, RIGHT_WHITE, RIGHT_BLACK);
    reset();
  }
}

void LineSensor::reset() {
  lastTime = millis();
  numReads = 0;
  leftRaw = 0;
  rightRaw = 0;
}

double LineSensor::lerp(double value, double min, double max) {
  return (value - min) / (max - min);
}

/**
 * Returns raw integer value
 */
uint16_t LineSensor::getLeftRaw() {
  return leftRaw;
}

/**
 * Returns raw integer value
 */
uint16_t LineSensor::getRightRaw() {
  return rightRaw;
}

/**
 * Returns value from 0 to 1
 */
double LineSensor::getLeftValue() {
  return leftValue;
}

/**
 * Returns value from 0 to 1
 */
double LineSensor::getRightValue() {
  return rightValue;
}

/**
 * Returns negative values if to the left and positive values if to the right
 */
double LineSensor::getValue() {
  return rightValue - leftValue;
}
