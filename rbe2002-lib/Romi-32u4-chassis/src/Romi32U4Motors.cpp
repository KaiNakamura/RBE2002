// Adapted from a library by Pololu Corporation.  For more information, see http://www.pololu.com/

#include <FastGPIO.h>
#include <Romi32U4Motors.h>
#include <avr/io.h>

// define the motor pins here
#define PWM_L 10
#define PWM_R 9
#define DIR_L 16
#define DIR_R 15

/**
 * initMotors() should be called near the beginning of the program (usually in Chassis::init()).
 * It sets up Timer4 to run at 38 kHz, which is used to both drive the PWM signal for the motors
 * and (tangentially) allow for a 38 kHz signal on pin 11, which can be used, say, to drive an
 * IR LED at a common rate.
 * 
 * Timer 1 has the following configuration:
 *  prescaler of 1
 *  outputs enabled on channels A (pin 9), B (pin 10) and C (pin 11)
 *  fast PWM mode
 *  top of 420, which will be the max speed
 *  frequency is then: 16 MHz / [1 (prescaler) / (420 + 1)] = 38.005 kHz
 * */
void Romi32U4Motor::initMotors() {
  Serial.println("initMotors()");

  FastGPIO::Pin<PWM_L>::setOutputLow();
  FastGPIO::Pin<PWM_R>::setOutputLow();
  FastGPIO::Pin<DIR_L>::setOutputLow();
  FastGPIO::Pin<DIR_R>::setOutputLow();

  noInterrupts();  //disable interupts while we set Timer1 registers

  TCCR1A = 0xAA;  //0b10101010; //Fast PWM + outputs enabled
  TCCR1B = 0x19;  //0b00011001; //Fast PWM
  ICR1 =
      420;  //runs at 38kHz; lowers speed for given effort by 5% from Pololu version

  //set all three outputs to zero
  OCR1A = 0;
  OCR1B = 0;
  OCR1C = 200;  //can be used to create 38 kHz signal on pin 11

  interrupts();  //re-enable interrupts

  pinMode(11, OUTPUT);

  Serial.println("/initMotors()");
}

/**
 * This is where you'll put the guts of your motor code
*/
void Romi32U4Motor::update(void) {
  if (ctrlMode == CTRL_SPEED) {
    // Calculate the error in speed
    int16_t error = targetSpeed - speed;

    // Sum the error
    sumError += error;

    // Clamp sum error
    if (sumError > maxSumError) {
      sumError = maxSumError;
    } else if (sumError < -maxSumError) {
      sumError = -maxSumError;
    }

    // Calculate the effort from the PID gains
    int16_t effort = Kp * error + Ki * sumError;

    // Set the effort for the motor
    setEffort(effort);

#ifdef __MOTOR_DEBUG__
    Serial.print(targetSpeed);
    Serial.print('\t');
    Serial.print(speed);
    Serial.print('\t');
    Serial.print(effort / 10.0);
    Serial.print('\t');
#endif
  }
}

/**
 * Sets the target speed in "encoder ticks / interval"
 * */
void Romi32U4Motor::setTargetSpeed(float target) {
  targetSpeed = target;

  if (ctrlMode != CTRL_SPEED) {
    // Reset the error integral if we are switching from another mode
    // Otherwise, the robot may jump due to residual integral
    sumError = 0;

    //Also reset prevCount to avoid jumps
    prevCount = count;
  }

  ctrlMode = CTRL_SPEED;
}

/**
 * Because the Pololu library is based on the FastGPIO library, we don't/can't use analogWrite.
 * Instead, we set the duty cycle directly at the register level.
 * 
 * We also have to have separate classes for the left and right motors to avoid the complex
 * mapping of speeds to registers.
 * */
void LeftMotor::setEffort(int16_t effort) {
  bool reverse = 0;

  if (effort < 0) {
    effort = -effort;  // Make speed a positive quantity.
    reverse = 1;       // Preserve the direction.
  }
  if (effort > maxEffort) {
    effort = maxEffort;
  }

  OCR1B = effort;
  FastGPIO::Pin<DIR_L>::setOutput(reverse);
}

void RightMotor::setEffort(int16_t effort) {
  bool reverse = 0;

  if (effort < 0) {
    effort = -effort;  // Make speed a positive quantity.
    reverse = 1;       // Preserve the direction.
  }
  if (effort > maxEffort) {
    effort = maxEffort;
  }

  OCR1A = effort;
  FastGPIO::Pin<DIR_R>::setOutput(reverse);
}

/**
 * Top speed is limited to 300/420 by default. This allow you to go faster. Be careful.
 * */
void Romi32U4Motor::allowTurbo(bool turbo) {
  maxEffort = turbo ? 420 : 300;
}