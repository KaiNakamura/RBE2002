#include "IREmitter.h"

IREmitter::IREmitter() {
  setLow();
}

/**
 * @brief Turn on the 38 kHz signal by setting COM1C1 on TCCR1A to 1.
 * 
 */
void IREmitter::setHigh() {
  TCCR1A |= (1 << COM1C1);
}

/**
 * @brief Turn off the 38 kHz signal by setting COM1C1 on TCCR1A to 0.
 * 
 */
void IREmitter::setLow() {
  TCCR1A &= ~(1 << COM1C1);
}

/**
 * @brief Emit a 9ms leading pulse burst (16 times the pulse burst length used
 * for a logical data bit) followed by a 4.5ms space
 */
void IREmitter::emitStart() {
  setHigh();
  delayMicroseconds(9000);
  setLow();
  delayMicroseconds(4500);
}

/**
 * @brief Emit a 562.5 microsecond pulse burst to signify the end of message
 * transmission.
 */
void IREmitter::emitStop() {
  setHigh();
  delayMicroseconds(562);
  setLow();
}

/**
 * @brief Emit a logical '0' – a 562.5 microsecond pulse burst followed by a
 * 562.5 microsecond space, with a total transmit time of 1.125ms.
 */
void IREmitter::emitZero() {
  setHigh();
  delayMicroseconds(562);
  setLow();
  delayMicroseconds(562);
}

/**
 * @brief Emit a logical '1' – a 562.5 microsecond pulse burst followed by a
 * 1.6875ms space, with a total transmit time of 2.25ms.
 */
void IREmitter::emitOne() {
  setHigh();
  delayMicroseconds(562);
  setLow();
  delayMicroseconds(1687);
}

/**
 * @brief Emit a three digit code and checksum.
 * 
 * @param digit1 The first digit.
 * @param digit2 The second digit.
 * @param digit3 The third digit.
 */
void IREmitter::emitCode(char digit1, char digit2, char digit3) {
  emitCode(digitsToCode(digit1, digit2, digit3));
}

/**
 * @brief Emit a 32-bit code.
 * 
 * @param code The 32-bit code to emit.
 */
void IREmitter::emitCode(long code) {
  Serial.print("Emitting code: ");
  Serial.println(code);

  long mask = 1;

  emitStart();

  for (int i = 0; i < 32; i++) {
    if (code & mask) {
      emitOne();
    } else {
      emitZero();
    }

    mask = mask << 1;
  }

  emitStop();
}

/**
 * @brief Convert a three digit code to a 32-bit code.
 * 
 * @param digit1 The first digit.
 * @param digit2 The second digit.
 * @param digit3 The third digit.
 * @return long The 32-bit code.
 */
long IREmitter::digitsToCode(char digit1, char digit2, char digit3) {
  long a = digit1;
  long b = digit2;
  long c = digit3;
  long code = a ^ b ^ c;
  code |= c << 8;
  code |= b << 16;
  code |= a << 24;
  return code;
}

/**
 * @brief Returns whether a 32-bit code is a valid code depending on the
 * checksum.
 * 
 * @param code The 32-bit code.
 * @return true The 32-bit code is a valid code.
 * @return false The 32-bit code is not a valid code.
 */
bool IREmitter::isValidCode(long code) {
  long mask1 = (long)0xFF << 24;
  long mask2 = (long)0xFF << 16;
  long mask3 = (long)0xFF << 8;
  long digit1 = (code & mask1) >> 24;
  long digit2 = (code & mask2) >> 16;
  long digit3 = (code & mask3) >> 8;
  long checksum = code & 0xFF;
  bool isValidButtonCell = digit1 >= '1' && digit1 <= '2' && digit2 >= '0' &&
                           digit2 <= '5' && digit3 >= '0' && digit3 <= '3';
  return isValidButtonCell && ((digit1 ^ digit2 ^ digit3) == checksum);
}