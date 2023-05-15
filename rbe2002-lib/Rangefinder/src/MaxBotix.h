#pragma once

/*
 * Datasheet: https://www.maxbotix.com/documents/LV-MaxSonar-EZ_Datasheet.pdf
 */

#include <Arduino.h>
#include <Rangefinder.h>

#define PING_SENT 0x01
#define ECHO_RECD 0x02

class MaxBotix : public Rangefinder {
 protected:
  uint32_t lastPing = 0;
  int triggerPin = -1;

 public:
  MaxBotix(uint8_t trig = -1) : triggerPin(trig) {}

  virtual void init(void);
  virtual void commandPing(void);
};

class MaxBotixPulse : public MaxBotix {
 private:
  uint8_t echoPin;
  volatile uint8_t state = 0;

  volatile uint32_t pulseStart = 0;
  volatile uint32_t pulseEnd = 0;

 public:
  MaxBotixPulse(uint8_t echo, uint8_t trig = -1) : MaxBotix(trig) {
    echoPin = echo;
  }
  virtual void init(void (*isr)(void));

  virtual bool getDistance(float& distance);

  inline void mbISR(void) {
    if (digitalRead(echoPin))  //transitioned to HIGH
    {
      pulseStart = micros();
      state |= PING_SENT;
    }

    else  //transitioned to LOW
    {
      pulseEnd = micros();
      state |= ECHO_RECD;
    }
  }
};

class MaxBotixAnalog : public MaxBotix {
 private:
  uint8_t adcPin = -1;

 public:
  MaxBotixAnalog(uint8_t adc, uint8_t trig = -1) : MaxBotix(trig) {
    adcPin = adc;
  }
  virtual void init(void) {
    MaxBotix::init();
    pinMode(adcPin, INPUT);
  }

  bool getDistance(float& distance);
};

class MaxBotixSerial : public MaxBotix {
 private:
  String serialString;

 public:
  MaxBotixSerial(uint8_t trig = -1) : MaxBotix(trig) {}
  void init(void) {
    MaxBotix::init();
    Serial1.begin(9600);
  }

  bool getDistance(float& distance);
};
