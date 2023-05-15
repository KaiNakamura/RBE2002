/*!
 * @file IRDirectionFinder.h
 * @brief DFRobot's Positioning ir camera
 * @n Header file for DFRobot's Positioning ir camera
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2016
 * @copyright	GNU Lesser General Public License
 *
 * @author [Angelo](Angelo.qiao@dfrobot.com)
 * @version  V1.0
 * @date  2016-02-01
 */

// https://www.dfrobot.com/product-1088.html

#include "Arduino.h"
#include "Wire.h"

#ifndef IRDirectionFinder_cpp
#define IRDirectionFinder_cpp

struct Point {
  int16_t x, y;
};

class IRDirectionFinder {
  //just use the actual address
  const int IRAddress = 0x58;

  /*!
   *  @brief position data structure from IIC sensor
   */
  union PositionData {
    uint8_t receivedBuffer[16];  ///< received buffer for IIC read
    struct {
      uint8_t header;
      struct {
        uint8_t xLowByte;    ///< position x low byte.
        uint8_t yLowByte;    ///< position y low byte.
        uint8_t xyHighByte;  ///< position x and y high byte.
      } __attribute__((packed)) rawPosition[4];  ///< 4 raw positions.
    } __attribute__((packed)) positionFrame;
  } __attribute__((packed)) positionData;

  Point points[4];

  /*!
   *  @brief write to a register.
   *
   *  @param reg  the register address
   *  @param value the value
   */
  void writeRegister(uint8_t reg, uint8_t value);

 public:
  /*!
   *  @brief Constructor
   */
  IRDirectionFinder();

  /*!
   *  @brief Destructor
   */
  ~IRDirectionFinder();

  /*!
   *  @brief initialize the sensor.
   */
  void begin();

  /*!
   *  @brief request the position, IIC will block the progress until all the data is transmitted.
   */
  void requestPosition();

  /*!
   *  @brief After requesting the position, and the data read from the sensor is ready, True will be returned.
   *
   *  @return Whether data reading is ready.
   *  @retval true Is ready
   *  @retval false Is not ready
   */
  bool available();

  /*!
   *  @brief Reads a Point from the list of points that were found
   *
   *  @param i  index of the point
   */
  Point ReadPoint(int i) { return points[i]; }
};

#endif
