/**
 * @file main.cpp
 * @brief Contains the main code for the program that runs the robot and IR
 * decoder.
 * @version 1.0
 * @date 2023-05-01
 * 
 */

#include <Arduino.h>
#include <robot.h>

/**
 * Most of the accessories, we put in robot.h/.cpp. We put the IR remote here because it's 
 * responsible for overall command and control -- it's not something that the robot uses directly
 * for control
 */
#include <IRdecoder.h>
#include <ir_codes.h>

#define IR_PIN 14
IRDecoder decoder(IR_PIN);

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("setup()");

  decoder.init();

  initialize();

  Serial.println("/setup()");
}

void loop() {
  /**
   * But we can also process asynchronous events, such as IR remote presses or distance sensor readings.
   */
  int16_t keyCode = decoder.getKeyCode();
  if (keyCode != -1) {
    handleKeyCode(keyCode);
  }

  update();
}