#include "MaxBotix.h"
#include <PCint.h>

void MaxBotix::init(void) {
  Serial.println("MaxBotix::init()");

  if (triggerPin !=
      -1)  // Arduino will ignore if it's -1, but good practice to check
  {
    //control pin for commanding pings
    pinMode(triggerPin, OUTPUT);
  }
  Serial.println("/MaxBotix::init()");
}

void MaxBotix::commandPing(void) {
  if (triggerPin != -1) {
    // ... toggle the trigger
    digitalWrite(triggerPin,
                 HIGH);  //commands a ping; leave high for the duration
    delayMicroseconds(
        25);  //datasheet says hold HIGH for >20us; we'll use 25 to be 'safe'
    digitalWrite(triggerPin, LOW);  //unclear if pin has to stay HIGH

    lastPing = millis();
  }
}

void MaxBotixPulse::init(void (*isr)(void)) {
  Serial.println("MaxBotixPulse::init()");
  MaxBotix::init();

  pinMode(echoPin, INPUT);

  if (digitalPinToInterrupt(echoPin) != NOT_AN_INTERRUPT) {
    Serial.println(F("Attaching ISR"));
    attachInterrupt(digitalPinToInterrupt(echoPin), isr, CHANGE);
  } else if (digitalPinToPCInterrupt(echoPin) != NOT_AN_INTERRUPT) {
    Serial.print(F("Attaching PC_ISR to PCINT"));
    Serial.println(digitalPinToPCInterrupt(echoPin));
    attachPCInt(digitalPinToPCInterrupt(echoPin), isr);
  } else {
    Serial.println(F("Not an interrupt pin!"));
  }

  Serial.println("/MaxBotixPulse::init()");
}

bool MaxBotixPulse::getDistance(float& distance) {
  bool newReading = false;

  if (state & ECHO_RECD) {
    cli();
    state &= ~ECHO_RECD;

    uint16_t echoLength = pulseEnd - pulseStart;
    sei();

    distance = ((echoLength / 1000.0) - 0.353) / 0.124;

    newReading = true;
  }

  return newReading;
}

/**
 * This is grossly too long. Edit once you know how fast you want to read the sensor
*/
const uint32_t ADC_INTERVAL = 250;  // TODO: Edit to a reasonable time

bool MaxBotixAnalog::getDistance(float& distance) {
  bool newReading = false;
  uint32_t currTime = millis();
  if (currTime - lastPing >= ADC_INTERVAL) {
    lastPing = currTime;
    uint16_t adcResult = analogRead(adcPin);

    distance = (adcResult - 9.67) / 1.35;

    newReading = true;
  }

  return newReading;
}

bool MaxBotixSerial::getDistance(float& distance) {
  bool newReading = false;
  while (Serial1.available()) {
    char c = Serial1.read();

    if (c != 'R') {
      serialString += c;
    }
    if (c == 0xD) {
      uint16_t result = serialString.toInt();
      serialString = "";
      distance = result;

      newReading = true;
    }
  }

  return newReading;
}