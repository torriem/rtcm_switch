#ifndef __WHICHTEENSY_H__
#define __WHICHTEENSY_H__

#ifdef ARDUINO_TEENSY40
#define TEENSY 1
#endif
#ifdef ARDUINO_TEENSY41
#define TEENSY 1
#endif
#ifdef ARDUINO_TEENSY36
#error "Teensy 3.6 not supported at this time."
#define TEENSY 1
#endif

#endif
