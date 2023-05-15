#pragma once

#include <Arduino.h>

class LineSensor {
 public:
  LineSensor();
  void setup();
  void update();
  void reset();
  uint16_t getLeftRaw();
  uint16_t getRightRaw();
  double getLeftValue();
  double getRightValue();
  double getValue();

 private:
  static double lerp(double value, double min, double max);

#if __ROBOT_NUMBER__ == 1
  static const uint16_t LEFT_WHITE = 783;
  static const uint16_t LEFT_BLACK = 965;
  static const uint16_t RIGHT_WHITE = 545;
  static const uint16_t RIGHT_BLACK = 935;
#elif __ROBOT_NUMBER__ == 2
  static const uint16_t LEFT_WHITE = 741;
  static const uint16_t LEFT_BLACK = 942;
  static const uint16_t RIGHT_WHITE = 619;
  static const uint16_t RIGHT_BLACK = 905;
#elif __ROBOT_NUMBER__ == 3
  static const uint16_t LEFT_WHITE = 748;
  static const uint16_t LEFT_BLACK = 970;
  static const uint16_t RIGHT_WHITE = 570;
  static const uint16_t RIGHT_BLACK = 940;
#else
  static const uint16_t LEFT_WHITE = 0;
  static const uint16_t LEFT_BLACK = 0;
  static const uint16_t RIGHT_WHITE = 0;
  static const uint16_t RIGHT_BLACK = 0;
#endif

  static const uint8_t LEFT_PIN = A4;
  static const uint8_t RIGHT_PIN = A3;
  static const uint16_t MAX_NUM_READS = 10;
  uint16_t numReads;
  long lastTime;
  double leftRaw, rightRaw, leftValue, rightValue;
};