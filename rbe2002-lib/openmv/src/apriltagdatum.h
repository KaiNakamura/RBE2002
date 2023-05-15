#ifndef __APRILTAGDATUM_H
#define __APRILTAGDATUM_H

/*
 * AprilTagDatum is the data received from the camera 
*/
struct AprilTagDatum {
  uint16_t header, checksum, id = 0, cx, cy, w, h, rot;
};

#endif
