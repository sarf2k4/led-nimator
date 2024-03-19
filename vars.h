#if defined(ARDUINO_AVR_DIGISPARKPRO) || !defined(EXTRA_MODES)
uint8_t modes = 8;
#else
uint8_t modes = 10;
#endif

#define EndLED (int)IntLEDT + (int)ExtLED

int brightness = LED_BRIGHTNESS;

#define TrimFloor 0
#define TrimCeil 1023

#if defined(PINK)
byte RD = 255;
byte GR = 0;
byte BL = 62;
#elif defined(YELLOW)
byte RD = 255;
byte GR = 213;
byte BL = 0;
#elif defined(BLUE)
byte RD = 125;
byte GR = 249;
byte BL = 255;
#elif defined(CUSTOM_COLOUR)
byte RD = 125;
byte GR = 128;
byte BL = 255;
#else
byte RD = 0;
byte GR = 0;
byte BL = 0;
#endif
/*
  LED Object Initialisation
*/
#if defined(ARDUINO_AVR_DIGISPARKPRO)
Adafruit_NeoPixel Lled = Adafruit_NeoPixel(IntLEDT + ExtLED, Lpin, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel Rled = Adafruit_NeoPixel(IntLEDT + ExtLED, Rpin, NEO_GRBW + NEO_KHZ800);
#else
npNeoPixel Lled = npNeoPixel(IntLEDT + ExtLED, Lpin, NEO_GRBW + NEO_KHZ800);
npNeoPixel Rled = npNeoPixel(IntLEDT + ExtLED, Rpin, NEO_GRBW + NEO_KHZ800);

//npVirtualNeo VIntLled = npVirtualNeo(&Lled, 0, IntLEDT);
//npVirtualNeo VIntRled = npVirtualNeo(&Lled, 0, IntLEDT);

#ifdef EXTRA_MODES
//ExtraLED Direction To Middle
//npVirtualNeo TMLled1 = npVirtualNeo(&Lled, IntLEDT, IntLEDT + (ExtLED / 2) - 1);
//npVirtualNeo TMLled2 = npVirtualNeo(&Lled, EndLED, IntLEDT + (ExtLED / 2));
//npVirtualNeo TMRled1 = npVirtualNeo(&Rled, IntLEDT, IntLEDT + (ExtLED / 2) - 1);
//npVirtualNeo TMRled2 = npVirtualNeo(&Rled, EndLED, IntLEDT + (ExtLED / 2));

//ExtraLED Direction From Middle
//npVirtualNeo FMLled1 = npVirtualNeo(&Lled, IntLEDT + (ExtLED / 2) - 1, IntLEDT);
//npVirtualNeo FMLled2 = npVirtualNeo(&Lled, IntLEDT + (ExtLED / 2), EndLED);
//npVirtualNeo FMRled1 = npVirtualNeo(&Rled, IntLEDT + (ExtLED / 2) - 1, IntLEDT);
//npVirtualNeo FMRled2 = npVirtualNeo(&Rled, IntLEDT + (ExtLED / 2), EndLED);


#ifdef BOUNCING_BALL
//npBouncingBall Lball111 = npBouncingBall(BALLDELAY, TMLled1);
npBouncingBall Lball11 = npBouncingBall(BALLDELAY, npVirtualNeo(&Lled, IntLEDT, IntLEDT + (ExtLED / 2) - 1));
npBouncingBall Lball21 = npBouncingBall(BALLDELAY, npVirtualNeo(&Lled, EndLED, IntLEDT + (ExtLED / 2)));
npBouncingBall Rball11 = npBouncingBall(BALLDELAY, npVirtualNeo(&Rled, IntLEDT, IntLEDT + (ExtLED / 2) - 1));
npBouncingBall Rball21 = npBouncingBall(BALLDELAY, npVirtualNeo(&Rled, EndLED, IntLEDT + (ExtLED / 2)));
#endif

npFire LFire1(COOLING, SPARKING, FIREDELAY, npVirtualNeo(&Lled, IntLEDT, IntLEDT + (ExtLED / 2) - 1));
npFire LFire2(COOLING, SPARKING, FIREDELAY, npVirtualNeo(&Lled, EndLED, IntLEDT + (ExtLED / 2)));
npFire RFire1(COOLING, SPARKING, FIREDELAY, npVirtualNeo(&Rled, IntLEDT, IntLEDT + (ExtLED / 2) - 1));
npFire RFire2(COOLING, SPARKING, FIREDELAY, npVirtualNeo(&Rled, EndLED, IntLEDT + (ExtLED / 2)));

//npMeteor LMeteor1(METEORSIZE, METEORTRAIL, true, METEORDELAY, FMLled1, false);
//npMeteor LMeteor2(METEORSIZE, METEORTRAIL, true, METEORDELAY, FMLled2, false);
//npMeteor RMeteor1(METEORSIZE, METEORTRAIL, true, METEORDELAY, FMRled1, false);
//npMeteor RMeteor2(METEORSIZE, METEORTRAIL, true, METEORDELAY, FMRled2, false);

npMeteor LMeteor1(METEORSIZE, METEORTRAIL, true, METEORDELAY, npVirtualNeo(&Lled, IntLEDT + (ExtLED / 2) - 1, IntLEDT), false);
npMeteor LMeteor2(METEORSIZE, METEORTRAIL, true, METEORDELAY, npVirtualNeo(&Lled, IntLEDT + (ExtLED / 2), EndLED), false);
npMeteor RMeteor1(METEORSIZE, METEORTRAIL, true, METEORDELAY, npVirtualNeo(&Rled, IntLEDT + (ExtLED / 2) - 1, IntLEDT), false);
npMeteor RMeteor2(METEORSIZE, METEORTRAIL, true, METEORDELAY, npVirtualNeo(&Rled, IntLEDT + (ExtLED / 2), EndLED), false);
#endif
#endif

int rainbowCycles = 0;

#ifdef LEGACY_CODES
#define SAMPLE_WINDOW 10  // Sample window for average level
//music blink sensitivity on mode 4
#define MUSINPUT_FLOOR 25     //Lower range of analogRead input
#define MUSINPUT_CEILING 375  //Max range of analogRead input, the lower the value the more sensitive (1023 = max)
#define boost 1               //will boost the analog signal (signal * boost) lower = sensitive

//music vu mode 5
#define MUSINPUT_FLOOR2 25     //Lower range of analogRead input
#define MUSINPUT_CEILING2 375  //Max range of analogRead input, the lower the value the more sensitive (1023 = max)
#define boost2 1               //will boost the analog signal (signal * boost)

// Mic sensitivity mode 6 and 7
#define MICINPUT_FLOOR 100    //Lower range of analogRead input
#define MICINPUT_CEILING 500  //Max range of analogRead input, the lower the value the more sensitive (1023 = max)

#else
#include <VolAnalyzer.h>
VolAnalyzer LanalyzeL(leftCh);
VolAnalyzer RanalyzeR(rightCh);
VolAnalyzer ManalyzeM(micCh);
#include <Bounce2.h>
Bounce mdchange = Bounce();
Bounce ledonoff = Bounce();
#endif

//#define Trimpot true //true uses trimpot to adjust music sensitivity(signal Boosts)

#define Trim_Sense_Ceil 10.0
float Trimboost = 1.0;  //Extra signal boost for trimpot/lower source volume [1 = no boost]



//universal variables
#define maxPwm 255
#define PEAK_HANG 24  //Time of pause before peak dot falls
#define PEAK_FALL 4   //Rate of falling peak dot
byte
  Lpeak = 16,
  LBpeak = 16,
  ELpeak = 16,
  Rpeak = 16,
  RBpeak = 16,
  ERpeak = 16,
  Mpeak = 16;

int Lvu = 0, Rvu = 0;

byte
  LdotCount = 0,
  LBdotCount = 0,
  ELdotCount = 0,
  RdotCount = 0,
  RBdotCount = 0,
  ERdotCount = 0,
  
  LdotHangCount = 0,
  LBdotHangCount = 0,
  ELdotHangCount = 0,
  RdotHangCount = 0,
  RBdotHangCount = 0,
  ERdotHangCount = 0;
//universal variables end

#ifdef LEGACY_CODES
unsigned int lSample, rSample, msample;

int
  buttonState = 0,
  lastButtonState = 1;  //mode change
#endif
//
//Fade variables
//
//using mili instead of delay variable
#define UP 0
#define DOWN 1

const int minPWM = 0;

//state variable
byte fadeDirection = UP;

//global fade
int fadeValue = 0;     //for pulse 1
// int fadeValue2 = 1;    //for pulse 2
// int revFade = maxPwm;  //for pulse 2

//fade smoothing
byte fadeIncrement = 5;

//timing variable
unsigned long previousFadeMillis;

//fade interval
unsigned long fadeInterval = 15;  //interval / (256 / fadeIncrement);
//end of milli variables
