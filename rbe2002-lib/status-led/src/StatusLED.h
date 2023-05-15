#pragma once

#include <Arduino.h>

enum LEDMode { LED_OFF, LED_ON, LED_SLOW_BLINK, LED_FAST_BLINK };

class StatusLED {
 public:
  StatusLED();
  void setMode(LEDMode mode);
  void update(void);

 private:
  const int LED_PIN = 13;
  const unsigned int SLOW_BLINK_TIME = 500;  // ms
  const unsigned int FAST_BLINK_TIME = 20;   // ms
  LEDMode mode = LED_OFF;
  unsigned long lastLEDBlink;
  bool isOn;
};