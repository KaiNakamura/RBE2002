#include "StatusLED.h"

StatusLED::StatusLED() {
  pinMode(LED_PIN, OUTPUT);
}

void StatusLED::setMode(LEDMode mode) {
  this->mode = mode;
}

void StatusLED::update() {
  switch (mode) {
    case LED_OFF:
      isOn = false;
      break;
    case LED_ON:
      isOn = true;
      break;
    case LED_SLOW_BLINK:
      if (millis() - lastLEDBlink > SLOW_BLINK_TIME) {
        lastLEDBlink = millis();
        isOn = !isOn;
      }
      break;
    case LED_FAST_BLINK:
      if (millis() - lastLEDBlink > FAST_BLINK_TIME) {
        lastLEDBlink = millis();
        isOn = !isOn;
      }
      break;
  }

  digitalWrite(LED_PIN, isOn ? HIGH : LOW);
}
