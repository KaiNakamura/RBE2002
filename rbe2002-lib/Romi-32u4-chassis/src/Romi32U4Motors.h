// Adapted from a library by Pololu Corporation.  For more information, see http://www.pololu.com/

/*! \file Romi32U4Motors.h */

#pragma once

#include <Arduino.h>
#include <stdint.h>

/** \class Romi32U4Motor
 * Controls motor effort and direction on the Romi 32U4.
 *
 * This library uses Timer 1, so it will conflict with any other libraries using
 * that timer. 
 * 
 * Also reads counts from the encoders on the Romi 32U4.
 *
 * This class allows you to read counts from the encoders on the Romi 32U4,
 * which lets you tell how much each motor has turned and in what direction.
 *
 * The encoders are monitored in the background using interrupts, so your code
 * can perform other tasks without missing encoder counts.
 */
class Romi32U4Motor {
 protected:
  // Used to control the motors in different ways
  enum CTRL_MODE : uint8_t { CTRL_DIRECT, CTRL_SPEED };
  volatile CTRL_MODE ctrlMode = CTRL_DIRECT;

  // this is the 'speed' of the motor, in "encoder counts / 16 ms interval"
  volatile int16_t speed = 0;

  // used to keep track of the target speed
  float targetSpeed = 0;

  // tracking integral
  int32_t sumError = 0;
  int32_t maxSumError = 150;

  // used to manage PID coefficients
  float Kp = 7;
  float Ki = 2;
  float Kd = 0;

  // Maximum effort (to protect the gear boxes). Can be changed by setting turbo mode
  int16_t maxEffort = 300;

  // keeps track of encoder changes
  volatile int16_t prevCount = 0;
  volatile int16_t count = 0;
  volatile int16_t lastA = 0;
  volatile int16_t lastB = 0;

  // Make Chassis a friend so we don't have to break out all the member data
  friend class Chassis;

 public:
  Romi32U4Motor(void) {}

  /**
    * Must be called near the beginning of the program [usually in Chassis::init()]
    * */
  static void init() {
    initMotors();
    initEncoders();
  }

  /**
    * A function that is called to update and control speeds
   
   */
  void update(void);

 protected:
  // Used to set up motors and encoders. Do not call directly; they are called from init(), which
  // is called from Chassis::init()
  static void initMotors();
  static void initEncoders();

  /** \brief Sets the effort for the motor directly. Overloaded for the left and right motors.
   * Use Chassis::setEfforts() from the user code to control motor efforts directly.
   *
   * \param effort A number from -300 to 300 representing the effort and
   * direction of the left motor. Values of -300 or less result in full effort
   * reverse, and values of 300 or more result in full effort forward. */
  virtual void setEffort(int16_t effort) = 0;

  /*! Returns the number of counts that have been detected from the
   * encoder. These counts start at 0. Positive counts correspond to forward
   * movement of the left side of the Romi, while negative counts correspond
   * to backwards movement.
   *
   * The count is returned as a signed 16-bit integer. When the count goes
   * over 32767, it will overflow down to -32768.  When the count goes below
   * -32768, it will underflow up to 32767. */
  int16_t getCount(void);
  int16_t getAndResetCount(void);

 public:
  /** \brief Turns turbo mode on or off.
     *
     * By default turbo mode is off.  When turbo mode is on, the range of speeds
     * accepted by the other functions in this library becomes -400 to 400
     * (instead of -300 to 300).  Turning turbo mode on allows the Romi to move
     * faster but could decrease the lifetime of the motors.
     *
     * This function does not have any immediate effect on the speed of the
     * motors; it just changes the behavior of the other functions in this
     * library.
     *
     * \param turbo If true, turns turbo mode on.
     *   If false, turns turbo mode off. */
  void allowTurbo(bool turbo);

  /**
    * Sets the target speed to be maintained by the PID algorithm.
    * \param target In encoder ticks per update interval
   */
  void setTargetSpeed(float target);

  void calcEncoderDelta(void);

  void setPIDCoeffs(float p, float i, float d) {
    Kp = p;
    Ki = i;
    sumError = 0;
  }

  /** 
    * Service function for the ISR
    * */
  inline void handleISR(bool newA, bool newB);
};

/**
 * \class LeftMotor
 * 
 * A derived class for the left motor.
 * */
class LeftMotor : public Romi32U4Motor {
 protected:
  void setEffort(int16_t effort);

 public:
  /** Used to set the motor effort directly. 
    * 
    * Calling this function will switch the control mode, if needed. 
    * */
  void setMotorEffort(int16_t effort) {
    ctrlMode = CTRL_DIRECT;
    setEffort(effort);
  }
};

/**
 * \class RightMotor
 * 
 * A derived class for the right motor.
 * */
class RightMotor : public Romi32U4Motor {
 protected:
  void setEffort(int16_t effort);

 public:
  /** Used to set the motor effort directly. 
    * 
    * Calling this function will switch the control mode, if needed. 
    * */
  void setMotorEffort(int16_t effort) {
    ctrlMode = CTRL_DIRECT;
    setEffort(effort);
  }
};

extern LeftMotor leftMotor;
extern RightMotor rightMotor;
