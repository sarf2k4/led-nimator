//comment all below for random colour picked every reset
//#define PINK
//#define YELLOW
//#define BLUE
// #define CUSTOM_COLOUR //set custom colour at vars.h

//only works if all above are commented
//#define SEIZURE_INDUCER

#if defined(PINK) || defined(YELLOW) || defined(BLUE) || defined(CUSTOM_COLOUR)
#define SINGLE_COLOUR_ONLY
#endif

//Inverts the blinking patterns
//#define INVERT_BLINK

#define SOFT_LED //LED ON/OFF switch
// #define INTERRUPT_MODE_CHANGE
// #define INTERRUPT_LED_SWITCH
uint8_t ledMode = 1;  //starts at 1
uint8_t ledpwr = 1;   //1 being on

#if !defined(ARDUINO_AVR_DIGISPARKPRO)
// #define USE_EEPROM  //this is to retain current mode even after power cycle
#endif
//#define COLOUR_WIPE_TEST //Will have individual RGB wipe test every startup
//#define COLOUR_WIPE_STARTUP //Will wipe into colour every startup
// #define LEGACY_CODES //Older codes; More flash memory, lesser dynamic memory

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
#define LED_BRIGHTNESS (0.25 * 255)

#define EXTRA_MODES
//#define BOUNCING_BALL


#define COOLING 50
#define SPARKING 120
#define FIREDELAY 5

#define METEORSIZE 3
#define METEORTRAIL 6
#define METEORDELAY 10
#define METEOR_RAINBOW false

#define BALL_NUM 1
#define BALLDELAY 0
#define BALL_RAINBOW true
