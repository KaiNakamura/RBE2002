#include <Arduino.h>
#include <LineSensor.h>
#include <Romi32U4Buttons.h>

LineSensor lineSensor;

Romi32U4ButtonA buttonA;

bool isCalibrating = false;

uint16_t leftWhite, leftBlack, rightWhite, rightBlack;

void resetCalibration() {
  leftWhite = UINT16_MAX;
  leftBlack = 0;
  rightWhite = UINT16_MAX;
  rightBlack = 0;
}

void printCalibration() {
  Serial.print("  static const uint16_t LEFT_WHITE = ");
  Serial.print(leftWhite);
  Serial.println(";");
  Serial.print("  static const uint16_t LEFT_BLACK = ");
  Serial.print(leftBlack);
  Serial.println(";");
  Serial.print("  static const uint16_t RIGHT_WHITE = ");
  Serial.print(rightWhite);
  Serial.println(";");
  Serial.print("  static const uint16_t RIGHT_BLACK = ");
  Serial.print(rightBlack);
  Serial.println(";");
}

void setup() {
  Serial.begin(115200);

  Serial.println("setup()");

  lineSensor.setup();

  Serial.println("/setup()");
}

void loop() {
  if (isCalibrating) {
    if (buttonA.getSingleDebouncedPress()) {
      isCalibrating = false;
      printCalibration();
    }

    lineSensor.update();
    uint16_t leftRaw = lineSensor.getLeftRaw();
    uint16_t rightRaw = lineSensor.getRightRaw();

    if (leftRaw > 0 && leftRaw < leftWhite) {
      leftWhite = leftRaw;
    }
    if (rightRaw > 0 && rightRaw < rightWhite) {
      rightWhite = rightRaw;
    }

    // TODO: fix
    leftBlack = leftRaw;
    rightBlack = rightRaw;
  } else {
    if (buttonA.getSingleDebouncedPress()) {
      isCalibrating = true;
      resetCalibration();
      Serial.println("Calibrating...");
    }
  }
}