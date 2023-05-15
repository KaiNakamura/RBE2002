#pragma once

#include <Arduino.h>

class IREmitter {
 public:
  IREmitter();
  void emitCode(char digit1, char digit2, char digit3);
  void emitCode(long code);
  static long digitsToCode(char digit1, char digit2, char digit3);
  static bool isValidCode(long code);

 private:
  void setHigh();
  void setLow();
  void emitStart();
  void emitStop();
  void emitZero();
  void emitOne();
};