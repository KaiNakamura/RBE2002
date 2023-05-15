#pragma once

#include <Arduino.h>

/** \class Rangefinder 
 * \brief Virtual base class to manage ultrasonic rangefinders. 
 * 
 * */
class Rangefinder {
 protected:
 public:
  Rangefinder(void) {}

  // must define init(); call init() in your setup() function
  // void init(void);

  /** \brief Returns true when there is a new echo and false otherwise.
     * The last recorded distance will be stored in distance.
     * */
  virtual bool getDistance(float& distance) = 0;
};
