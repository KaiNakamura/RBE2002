#ifndef __CAMERA_H
#define __CAMERA_H

#include <Arduino.h>
#include <Wire.h>

#define CAMERA_I2C_ADDRESS 0x12

#include <apriltagdatum.h>

class Camera {};

class OpenMV : public Camera {
 protected:
  uint8_t mvArray[sizeof(
      AprilTagDatum)];  //array for receiving data from the OpenMV cam
  uint8_t mvIndex = 0;  //for counting bytes

 public:
  uint8_t getTagCount(void);
  bool readTag(AprilTagDatum& tag);

  bool checkUART(AprilTagDatum& tag);
  bool handleUART(uint8_t b);
};

#endif
