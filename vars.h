OneButton bpd2(BPD2, true);
#ifdef BPD3
OneButton bpd3(BPD3, true);
#endif

#if defined(ARDUINO_AVR_DIGISPARKPRO) || !defined(EXTRA_MODES)
uint8_t modes = 7;
#else
uint8_t modes = 10;
#endif

#define EndLED (int)IntLEDT + (int)ExtLED

int brightness = LED_BRIGHTNESS;

#define TrimFloor 0
#define TrimCeil 1023

#define eprUsed 5  //Number of EEPROM slots used

byte Colour[3] = { RD, GR, BL };
/*
  LED Object Initialisation
*/

Adafruit_NeoPixel Lled = Adafruit_NeoPixel(IntLEDT + ExtLED, Lpin, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel Rled = Adafruit_NeoPixel(IntLEDT + ExtLED, Rpin, NEO_GRBW + NEO_KHZ800);

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
VolAnalyzer LanalyzeL(leftCh);
VolAnalyzer RanalyzeR(rightCh);
VolAnalyzer ManalyzeM(micCh);
const int vuVolK[2] = { 25, 16 };
const int blVolK = 25;
#endif

//#define Trimpot true //true uses trimpot to adjust music sensitivity(signal Boosts)

#define Trim_Sense_Ceil 10.0
float Trimboost = 1.0;  //Extra signal boost for trimpot/lower source volume [1 = no boost]

//universal variables
int AltMode = 0;
const int vu_offset = 0;
#define maxPwm 255

int PeakHangs[2] = { 36, 12 };  //Time of pause before peak dot falls; higher = longer [24]
int PeakFalls[2] = { 5, 2 };    //Rate of falling peak dot; higher = slower [4]

int PEAK_HANG = PeakHangs[AltMode];
int PEAK_FALL = PeakFalls[AltMode];

int Lvu = 0, Rvu = 0;

byte
  Lpeak = 16,
  LBpeak = 16,
  ELpeak = 16,
  Rpeak = 16,
  RBpeak = 16,
  ERpeak = 16,
  Mpeak = 16,

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
//
//Fade variables
//
//using mili instead of delay variable
#define UP 0
#define DOWN 1

const int minPWM = 0;

//state variable
byte fadeDirection = UP;
#endif

//global fade
int fadeValue = 0;  //for pulse 1
// int fadeValue2 = 1;    //for pulse 2
// int revFade = maxPwm;  //for pulse 2

//timing variable
unsigned long previousFadeMillis, prevMillis;

//fade interval
unsigned long fadeInterval = 15;  //interval / (256 / fadeIncrement);
unsigned long wipeInterval = 100;
//end of milli variables
//rainbow settings
int rainbowInt = 10;
long runningHue = 0, firstHue = 0;
long const finalHue = 65536L;
long const startHueF = 2978, endHueF = 30247;
long const startHueB = 23125, endHueB = 65536L;

// nodelay parameters
noDelay fadeDelay(fadeInterval);