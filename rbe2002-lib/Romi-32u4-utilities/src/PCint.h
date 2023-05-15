#pragma once

#include <Arduino.h>

/** \brief Attaches a function to a pin change interrupt. Currently only for
 * ATmega32U4 (e.g., Leonardo or Pololu's A-star 32U4 boards like the Romi).
 * Other chips will compile a dummy function that should not be called.
 *
 * \param pcInt The specific pin change interrupt, e.g., "PCINT3"
 * \param pcisr A pointer to the ISR function, of type void fxn(void).
 * 
 *  */
void attachPCInt(uint8_t pcInt, void (*pcisr)(void));

/** \brief Translates Arduino pin number to a PCINT.
 * 
 * Currently only for ATmega32U4.
 * 
 * */
int8_t digitalPinToPCInterrupt(uint8_t pin);