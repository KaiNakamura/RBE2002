/*!
 * @file IRDirectionFinder.cpp
 * @brief DFRobot's Positioning ir camera
 * @n CPP file for DFRobot's Positioning ir camera
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [Angelo](Angelo.qiao@dfrobot.com)
 * @version  V1.0
 * @date  2016-02-17
 */

#include "IRDirectionFinder.h"

IRDirectionFinder::IRDirectionFinder() {}

IRDirectionFinder::~IRDirectionFinder() {}

// Instead of "writing two bytes", let's use proper I2C protocol and write to a register
void IRDirectionFinder::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IRAddress);
  Wire.write(reg);    //tell it what register we want to write to
  Wire.write(value);  //then set the contents of the register
  Wire.endTransmission();
}

void IRDirectionFinder::begin() {
  Wire.begin();
  writeRegister(0x30, 0x01);
  delay(10);
  writeRegister(
      0x30,
      0x08);  //this makes no sense because now we're clobbering the previous write
  delay(10);
  writeRegister(0x06, 0x90);
  delay(10);
  writeRegister(0x08, 0xC0);
  delay(10);
  writeRegister(0x1A, 0x40);
  delay(10);
  writeRegister(0x33, 0x33);
  delay(10);

  delay(100);
}

void IRDirectionFinder::requestPosition() {
  Wire.beginTransmission(IRAddress);
  Wire.write(0x36);  //write address 0x36 to initiate a reading
  Wire.endTransmission();
  Wire.requestFrom(IRAddress, 16);
}

/**
 * As written, this will not play nicely with other I2C devices.
*/
bool IRDirectionFinder::available() {
  if (Wire.available() == 16) {  //read only the data length fits.
    for (int i = 0; i < 16; i++) {
      positionData.receivedBuffer[i] = Wire.read();
    }

    for (int i = 0; i < 4; i++) {
      points[i].x =
          (uint16_t)(positionData.positionFrame.rawPosition[i].xLowByte) +
          ((uint16_t)(positionData.positionFrame.rawPosition[i].xyHighByte &
                      0x30U)
           << 4);

      points[i].y =
          (uint16_t)(positionData.positionFrame.rawPosition[i].yLowByte) +
          ((uint16_t)(positionData.positionFrame.rawPosition[i].xyHighByte &
                      0xC0U)
           << 2);
    }
    return true;
  }

  else {  //otherwise skip them.
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }
}
