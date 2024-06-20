//comment all below for random colour picked every reset
// #define PINK
// #define YELLOW
// #define BLUE
// #define CUSTOM_COLOUR //set custom colour at cols.h

#if defined(PINK) || defined(YELLOW) || defined(BLUE) || defined(CUSTOM_COLOUR)
#define FIXED_COLOUR
#endif

//Inverts the blinking patterns
//#define INVERT_BLINK

// #define USE_INTERRUPTS

uint8_t ledMode = 4;  //starts at 0
uint8_t ledpwr = 1;   //1 being on
#define Random_Period 1000

#if !defined(ARDUINO_AVR_DIGISPARKPRO)
#define USE_EEPROM  //this is to retain current mode even after power cycle
#endif
//#define COLOUR_WIPE_TEST //Will have individual RGB wipe test every startup
#define COLOUR_WIPE_STARTUP //Will wipe into colour every startup
// #define LEGACY_CODES //Older codes; More flash memory, lesser dynamic memory [deprecated]

#define EXTRA_LED  //Extra LED to be attached
#define IntLEDF 5
#define IntLEDB 8
#define IntLEDT (IntLEDF + IntLEDB)  //internal led numbers

#ifndef EXTRA_LED
#define ExtLED 0  //external led numbers
#else
#define ExtLED 44  //external led numbers
#endif

#define COLOUR_WIPE_DELAYS 5
#define LED_BRIGHTNESS (0.15 * 255)

