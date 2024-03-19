#include <Adafruit_NeoPixel.h>
/*
  The code accepts 3 input and interchangeable mode by button press
  left led is at pin 0, right led at pin 1
  left channel pin 5, right channel pin 4
  mic or sound sensor pin 2
  mode button pin 3 and ground
*/
;
#define PINK
//#define YELLOW

#define IntLED 4 //internal led numbers

#define EXTRA_LED
#ifndef EXTRA_LED
#define ExtLED 0 //external led numbers
#else
#define ExtLED 18 //external led numbers
#endif

//#define INTERRUPT_MODE_CHANGE

#if defined(ARDUINO_AVR_DIGISPARKPRO) // Using Digispark Pro with Attiny167
#define leftCh A12 //L audio input [pin 12]
#define rightCh A11 //R audio input [pin 11]
#define MDButton 8 //Button mode [pin 8]
#define micPin A10 //Mic input Analog [pin 10]
#define micDigipin 10 //Mic input Digital [pin 10]
#define Lpin 0 //left LED data [pin 1]
#define Rpin 1 //right LED data [pin 0]
//#define LTpin 2 //left ext LED data [pin 2]
//#define RTpin 3 //right ext LED data [pin 3]
#define Bright A5 //Brightness trimpot [pin 5]
#define Sense A9 //LED music sensitivity [pin 9]
#define ENtrimpots 2 //Jumper to enable trimpots [pin 2]

#elif defined(ARDUINO_AVR_ATMEGA328BB) || defined(ARDUINO_AVR_UNO) // Using ATMEGA328P Internal Oscilator
#define leftCh A0 //L audio input
#define rightCh A1 //R audio input
#define micPin A2 //Mic input Analog
#define Sense A4 //LED music sensitivity trimpot
#define Bright A5 //Brightness trimpot

#define MDButton 2 //Button mode pin as an interrupt
#define ENsensesTrim 4 //Jumper to ground, enables manual sensitivity boost
#define ENbrightTrim 5 //Jumper to ground, enable brightness adjustment trimpots [pin 5]
#define EXled 7 //External led sense pulled to ground [1:NC,0:Connected]
#define Lpin 8 //left LED data pin
#define Rpin 9 //right LED data pin
//#define LTpin 8 //left ext LED data pin
//#define RTpin 7 //right ext LED data pin

#endif

Adafruit_NeoPixel Lled = Adafruit_NeoPixel(IntLED + ExtLED, Lpin, NEO_GRBW + NEO_KHZ800);
Adafruit_NeoPixel Rled = Adafruit_NeoPixel(IntLED + ExtLED, Rpin, NEO_GRBW + NEO_KHZ800);

int EndLED = IntLED + ExtLED;
int Tled = 0;

float Mbright = 0.05;
int brightness = 255 * Mbright;

int TrimBrightFloor = 0;
int TrimBrightCeil = 1023;

#ifdef PINK
byte RD = 255;
byte GR = 0;
byte BL = 62;
#elif YELLOW
byte RD = 255;
byte GR = 213;
byte BL = 0;
#endif
uint32_t NEOCOOL = Lled.Color(RD, GR, BL);

unsigned long pixelsInterval = 50; // the time we need to wait
unsigned long colorWipePreviousMillis = 0;
unsigned long theaterChasePreviousMillis = 0;

int rainbowCycleCycles = 0;
int rainbowCycles = 0;

#define SAMPLE_WINDOW   10  // Sample window for average level

//music blink sensitivity on mode 4
#define MUSINPUT_FLOOR 256 //Lower range of analogRead input
#define MUSINPUT_CEILING 1024 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)
float boost = 6.5; //will boost the analog signal (signal * boost) lower = sensitive

//music vu mode 5
#define MUSINPUT_FLOOR2 256 //Lower range of analogRead input
#define MUSINPUT_CEILING2 1024 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)
float boost2 = 4.0; //will boost the analog signal (signal * boost)

// Mic sensitivity mode 6 and 7
#define MICINPUT_FLOOR 100 //Lower range of analogRead input
#define MICINPUT_CEILING 500 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)

//#define Trimpot true //true uses trimpot to adjust music sensitivity(signal Boosts)

float Trimboost = 1.0; //Extra signal boost for trimpot/lower source volume [1 = no boost]

#define maxPwm 255


#define PEAK_HANG 24 //Time of pause before peak dot falls
#define PEAK_FALL 4 //Rate of falling peak dot
byte
Lpeak = 16,
ELpeak = 16,
Rpeak = 16,
ERpeak = 16,
Mpeak = 16;

unsigned int lSample, rSample, msample;
int Lvu = 0, Rvu = 0;

byte
LdotCount = 0,
ELdotCount = 0,
RdotCount = 0,
ERdotCount = 0,
MdotCount = 0, //Frame counter for peak dot
LdotHangCount = 0,
ELdotHangCount = 0,
RdotHangCount = 0,
ERdotHangCount = 0,
MdotHangCount = 0; //Frame counter for holding peak dot


int left,
    right,
    i,
    leftLarge,
    rightLarge,
    leftMap,
    rightMap; //led control variables

int
buttonState = 0,
lastButtonState = 1,
buttonPushCounter = 0; //mode change

int
mic,
micMap; //microphone or voice reactive

int
ledMode = 0,
modes = 8; //total number of modes

int micDigi;

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
int fadeValue = 1; //for pulse 1
int fadeValue2 = 1; //for pulse 2
int revFade = maxPwm; //for pulse 2

//fade smoothing
byte fadeIncrement = 5;

//timing variable
unsigned long previousFadeMillis;

//fade interval
int interval = 250;
unsigned long fadeInterval = 10; //interval / (256 / fadeIncrement);
//end of milli variables


int ExtSense = 1; //External LED Checks
int Trimsense = 0;

void setup()
{
  Serial.begin(9600);

  pinMode(ENsensesTrim, INPUT_PULLUP);
  pinMode(ENbrightTrim, INPUT_PULLUP);
  pinMode (EXled, INPUT_PULLUP);
  pinMode(MDButton, INPUT_PULLUP);
#ifdef INTERRUPT_MODE_CHANGE
  attachInterrupt(digitalPinToInterrupt(MDButton), intmodeChange, FALLING);
#endif

  Lled.setBrightness(brightness);
  Rled.setBrightness(brightness);

  Lled.begin();
  Rled.begin();

  Lled.show();
  Rled.show();

  ledMode = 9;
}

/*
  ledOn(); //colour
  ledPulse1(); //symmetrical pulse colour
  ledPulse2(); //assymmetrical pulse colour
  ledPulse3(); //symmetrical square pulse colour
  ledPulse4(); //experimental rainbow pulse
  ledVU(); //music blink colour
  ledvu2(1);//music vu meter
  ledvu2(2); //mic vu meter
  ledMIca(); //analog mic blink colour
  ledMIcd(); //digital mic blink colour
  rainbow(); //gradient rainbow
  rainbowCycle(); // all rainbow cycle
  rainbowCycle2();// only extras rainbow cycle
*/

void loop()
{
  unsigned long currentMillis = millis();
  ExtSense = digitalRead(EXled);

  TrimBright();
  ExtraSense();
  switch (ledMode)
  {
    case 0:
      ledOn();
      break;
    case 1:
      ledPulse1();
      break;
    case 2:
      ledPulse2();
      break;
    case 3:
      ledVU();
      break;
    case 4:
      ledVU2(1); //LEDVU mode
      break;
    case 5:
      ledMica();
      break;
    case 6:
      ledVU2(2); //MicVU mode
      break;
    case 7:
      rainbowCycle();
      break;
  }
  //Serial.println(ledMode);

#ifndef INTERRUPT_MODE_CHANGE
  modeChange(); //comment for usage of interrupt
#endif
  Serial.println();

}

void intmodeChange() //interrupt mode
{
  ledMode++;
  ledMode %= modes;
  //delay(500); //This debounce is not necessary, the flashing LED's will function as debounce

  //LED's off
  ledOff();
  delay (250);

  //Flash LED's
  for (int i = 0; i <= ledMode; i++)
  {
    rainbow(255);
    delay (100);
    ledOff();
    delay (300);
  }

}

void modeChange()
{
  buttonState = digitalRead(MDButton);
  if ((buttonState != lastButtonState) && (buttonState == HIGH))
  {
    ledMode++;
    ledMode %= modes;
    //delay(500); //This debounce is not necessary, the flashing LED's will function as debounce

    //LED's off
    ledOff();
    delay (250);

    //Flash LED's
    for (int i = 0; i <= ledMode; i++)
    {
      rainbow(255);
      delay (100);
      ledOff();
      delay (300);
    }
  }
  lastButtonState = buttonState;
}

//only on colour
void ledOn()
{
  for (i = 0; i < Lled.numPixels(); i++)
  {
    Lled.setPixelColor(i, NEOCOOL);
    Rled.setPixelColor(i, NEOCOOL);
  }
  LedShow(0);
}

//all pulse colour
void ledPulse1()
{
  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    if (fadeDirection == UP) {
      for (uint8_t i = 0; i < Lled.numPixels(); i++) {
        Lled.setPixelColor(i, RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
        Rled.setPixelColor(i, RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
      }
      fadeValue++;
    }
    else if (fadeDirection == DOWN) {
      for (uint8_t i = 0; i < Lled.numPixels(); i++) {
        Lled.setPixelColor(i, RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
        Rled.setPixelColor(i, RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
      }
      fadeValue--;
    }
    LedShow(0);
  }

  if (fadeValue >= maxPwm && fadeDirection == UP)
  {
    fadeDirection = DOWN;
  } else if (fadeValue <= minPWM && fadeDirection == DOWN)
  {
    fadeDirection = UP;
  }
}

//Alternate pulse colour
void ledPulse2()
{

  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    if (fadeDirection == UP) {
      for (uint8_t i = 0; i < Lled.numPixels(); i++) {
        Lled.setPixelColor(i, RD * fadeValue2 / 255, GR * fadeValue2 / 255, BL * fadeValue2 / 255);
      }
      for (uint8_t i = 0; i < Rled.numPixels(); i++) {
        Rled.setPixelColor(i, RD * revFade / 255, GR * revFade / 255, BL * revFade / 255);
      }
      fadeValue2++;
      revFade--;
    }
    else if (fadeDirection == DOWN) {
      for (uint8_t i = 0; i < Lled.numPixels(); i++) {
        Lled.setPixelColor(i, RD * fadeValue2 / 255, GR * fadeValue2 / 255, BL * fadeValue2 / 255);
      }
      for (uint8_t i = 0; i < Rled.numPixels(); i++) {
        Rled.setPixelColor(i, RD * revFade / 255, GR * revFade / 255, BL * revFade / 255);
      }
      fadeValue2--;
      revFade++;

    }
    LedShow(0);
  }

  if (fadeValue2 >= maxPwm && fadeDirection == UP)
  {
    fadeDirection = DOWN;
  } else if (fadeValue2 <= minPWM && fadeDirection == DOWN)
  {
    fadeDirection = UP;
  }
}

//only extras pulsing
void ledPulse3()
{

  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    for (i = 0; i < IntLED; i++)
    {
      Lled.setPixelColor(i, NEOCOOL);
      Rled.setPixelColor(i, NEOCOOL);
    }
    if (fadeDirection == UP) {
      for (uint8_t i = 0; i < ExtLED; i++) {
        Tled = map(i, 0, ExtLED, IntLED, IntLED + ExtLED);
        Lled.setPixelColor(Tled, RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
        Rled.setPixelColor(Tled, RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
      }
      fadeValue++;
    }
    else if (fadeDirection == DOWN) {
      for (uint8_t i = 0; i < ExtLED; i++) {
        Tled = map(i, 0, ExtLED, IntLED, IntLED + ExtLED);
        Lled.setPixelColor(Tled, RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
        Rled.setPixelColor(Tled, RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
      }
      fadeValue--;
    }
    LedShow(0);
  }

  if (fadeValue >= maxPwm && fadeDirection == UP)
  {
    fadeDirection = DOWN;
  } else if (fadeValue <= minPWM && fadeDirection == DOWN)
  {
    fadeDirection = UP;
  }
}

//rainbow pulsing
void ledPulse4()
{

  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    if (fadeDirection == UP) {
      //      for (uint8_t i = 0; i < IntLED + ExtLED; i++) {
      rainbow(fadeValue);
      //      }
      fadeValue++;
    }
    else if (fadeDirection == DOWN) {
      //      for (uint8_t i = 0; i < IntLED + ExtLED; i++) {
      rainbow(fadeValue);
      //      }
      fadeValue--;
    }
    LedShow(0);
  }

  if (fadeValue >= maxPwm && fadeDirection == UP)
  {
    fadeDirection = DOWN;
  } else if (fadeValue <= minPWM && fadeDirection == DOWN)
  {
    fadeDirection = UP;
  }
}

//music blinking single colour
void ledVU()
{
  unsigned long startMillis = millis(); // Start of sample window

  unsigned int L, R;
  float lpeakToPeak = 0, rpeakToPeak = 0;
  unsigned int lsignalMax = 0, rsignalMax = 0;
  unsigned int lsignalMin = 1023, rsignalMin = 1023;

  while (millis() - startMillis < SAMPLE_WINDOW)
  {

    lSample = (analogRead(leftCh) * Trimboost) * log(analogRead(leftCh) + 1);
    rSample = (analogRead(rightCh) * Trimboost) * log(analogRead(leftCh) + 1);
    if (lSample < 1024)  // toss out spurious readings
    {
      if (lSample > lsignalMax)
      {
        lsignalMax = lSample;  // save just the max levels
      }
      else if (lSample < lsignalMin)
      {
        lsignalMin = lSample;  // save just the min levels
      }
    }
    if (rSample < 1024) // toss out spurious readings
    {
      if (rSample > rsignalMax)
      {
        rsignalMax = rSample;  // save just the max levels
      }
      else if (rSample < rsignalMin)
      {
        rsignalMin = rSample;  // save just the min levels
      }
    }
  }
  lpeakToPeak = lsignalMax - lsignalMin;  // max - min = peak-peak amplitude
  rpeakToPeak = rsignalMax - rsignalMin;  // max - min = peak-peak amplitude

  L = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, maxPwm, 0, lpeakToPeak, 2);
  R = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, maxPwm, 0, rpeakToPeak, 2);

  Lvu = map(L, 0, 255, 255, 0);
  Rvu = map(R, 0, 255, 255, 0);

  for (uint8_t i = 0; i < Lled.numPixels(); i++) {
    Lled.setPixelColor(i, RD * Lvu / 255, GR * Lvu / 255, BL * Lvu / 255);
    Rled.setPixelColor(i, RD * Rvu / 255, GR * Rvu / 255, BL * Rvu / 255);
  }
  LedShow(0);
}

//music vu rainbow
void ledVU2(int Md)
{
  unsigned long startMillis = millis(); // Start of sample window

  unsigned int L, R, EL, ER;
  float lpeakToPeak = 0, rpeakToPeak = 0;
  unsigned int lsignalMax = 0, rsignalMax = 0;
  unsigned int lsignalMin = 1023, rsignalMin = 1023;

  unsigned int c, Ly, Ry, ELy, ERy;


  // collect data for length of sample window (in mS)
  while (millis() - startMillis < SAMPLE_WINDOW)
  {
    if (Md == 1)
    {
      lSample = (analogRead(leftCh) * Trimboost) * boost2;
      rSample = (analogRead(rightCh) * Trimboost) * boost2;
    }
    else if (Md == 2)
    {
      lSample = analogRead(micPin);
      rSample = analogRead(micPin);
    }
    if (lSample < 1024)  // toss out spurious readings
    {
      if (lSample > lsignalMax)
      {
        lsignalMax = lSample;  // save just the max levels
      }
      else if (lSample < lsignalMin)
      {
        lsignalMin = lSample;  // save just the min levels
      }
    }
    if (rSample < 1024) // toss out spurious readings
    {
      if (rSample > rsignalMax)
      {
        rsignalMax = rSample;  // save just the max levels
      }
      else if (rSample < rsignalMin)
      {
        rsignalMin = rSample;  // save just the min levels
      }
    }
  }
  lpeakToPeak = lsignalMax - lsignalMin;  // max - min = peak-peak amplitude
  rpeakToPeak = rsignalMax - rsignalMin;  // max - min = peak-peak amplitude
  // Serial.println(peakToPeak);

  //Scale the input logarithmically instead of linearly
  if (Md == 1)
  {
    L = fscale(MUSINPUT_FLOOR2, MUSINPUT_CEILING2, IntLED, 0, lpeakToPeak, 2);
    EL = fscale(MUSINPUT_FLOOR2, MUSINPUT_CEILING2, (ExtLED / 2), 0, lpeakToPeak, 2);
    R = fscale(MUSINPUT_FLOOR2, MUSINPUT_CEILING2, IntLED, 0, rpeakToPeak, 2);
    ER = fscale(MUSINPUT_FLOOR2, MUSINPUT_CEILING2, (ExtLED / 2), 0, rpeakToPeak, 2);
    //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
  }
  else if (Md == 2)
  {
    L = fscale(MICINPUT_FLOOR, MICINPUT_CEILING, IntLED, 0, lpeakToPeak, 2);
    EL = fscale(MICINPUT_FLOOR, MICINPUT_CEILING, (ExtLED / 2), 0, lpeakToPeak, 2);
    R = fscale(MICINPUT_FLOOR, MICINPUT_CEILING, IntLED, 0, rpeakToPeak, 2);
    ER = fscale(MICINPUT_FLOOR, MICINPUT_CEILING, (ExtLED / 2), 0, rpeakToPeak, 2);
    //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
  }
  //Fill the strip with rainbow gradient
  for (int i = 0; i < EndLED; i++)
  {
    if (i < IntLED)
    {
      Lled.setPixelColor(i, WheelVU(map(i, 0, IntLED - 1, 30, 150)));
      Rled.setPixelColor(i, WheelVU(map(i, 0, IntLED - 1, 30, 150)));
    }
    if ((i >= IntLED) && (i < IntLED + (ExtLED / 2)))
    {
      Lled.setPixelColor(i, WheelVU(map(i, IntLED, (IntLED + (ExtLED / 2)) - 1, 30, 150)));
      Rled.setPixelColor(i, WheelVU(map(i, IntLED, (IntLED + (ExtLED / 2)) - 1, 30, 150)));

    }
    if ((i >= IntLED + (ExtLED / 2)) && (i < EndLED))
    {
      Lled.setPixelColor(i, WheelVU(map(i, EndLED, EndLED - (ExtLED / 2), 30, 150)));
      Rled.setPixelColor(i, WheelVU(map(i, EndLED, EndLED - (ExtLED / 2), 30, 150)));
    }
  }

  if (L < Lpeak) {
    Lpeak = L;        // Keep dot on top
    LdotHangCount = 0;    // make the dot hang before falling
  }
  if (EL < ELpeak) {
    ELpeak = EL;        // Keep dot on top
    ELdotHangCount = 0;    // make the dot hang before falling
  }
  if (L <= IntLED) { // Fill partial column with off pixels
    drawLine(IntLED, IntLED - L, Lled.Color(0, 0, 0), 0);
  }
  if (EL <= ExtLED / 2) { // Fill partial column with off pixels
    drawLine(IntLED + (ExtLED / 2), (IntLED + (ExtLED / 2)) - EL, Lled.Color(0, 0, 0), 0);
    drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + EL, Lled.Color(0, 0, 0), 0);
  }

  if (R < Rpeak) {
    Rpeak = R;        // Keep dot on top
    RdotHangCount = 0;    // make the dot hang before falling
  }
  if (ER < ERpeak) {
    ERpeak = ER;        // Keep dot on top
    ERdotHangCount = 0;    // make the dot hang before falling
  }
  if (R <= IntLED) { // Fill partial column with off pixels
    drawLine(IntLED, IntLED - R, Rled.Color(0, 0, 0), 1);
  }
  if (ER <= ExtLED / 2) { // Fill partial column with off pixels
    drawLine(IntLED + (ExtLED / 2), (IntLED + (ExtLED / 2)) - ER, Rled.Color(0, 0, 0), 1);
    drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + ER, Rled.Color(0, 0, 0), 1);
  }
  // Set the peak dot to match the rainbow gradient
  Ly = IntLED - Lpeak;
  ELy = (ExtLED / 2) - ELpeak;
  Ry = IntLED - Rpeak;
  ERy = (ExtLED / 2) - ERpeak;

  Lled.setPixelColor(Ly - 1, WheelVU(map(Ly, 0, IntLED - 1, 30, 150)));
  Lled.setPixelColor(IntLED + ELy - 1, WheelVU(map(ELy, 0, (ExtLED / 2) , 30, 150)));
  Lled.setPixelColor(EndLED - ELy, WheelVU(map(ELy, 0, (ExtLED / 2) , 30, 150)));

  if (ELy == 0)
  {
    Lled.setPixelColor(IntLED + ELy - 1, 0 , 0, 0);
    Lled.setPixelColor(EndLED - ELy, 0 , 0, 0);

  }

  Rled.setPixelColor(Ry - 1, WheelVU(map(Ry, 0, IntLED - 1, 30, 150)));
  Rled.setPixelColor(IntLED + ERy - 1, WheelVU(map(ERy, 0, (ExtLED / 2) , 30, 150)));
  Rled.setPixelColor(EndLED - ERy, WheelVU(map(ERy, 0, (ExtLED / 2) , 30, 150)));

  if (ERy == 0)
  {
    Rled.setPixelColor(IntLED + ERy - 1, 0 , 0, 0);
    Rled.setPixelColor(EndLED - ERy, 0 , 0, 0);

  }
  LedShow(0);

  // Frame based peak dot animation
  if (LdotHangCount > PEAK_HANG) { //Peak pause length
    if (++LdotCount >= PEAK_FALL) { //Fall rate
      Lpeak++;
      LdotCount = 0;
    }
  }
  else {
    LdotHangCount++;
  }

  if (RdotHangCount > PEAK_HANG) { //Peak pause length
    if (++RdotCount >= PEAK_FALL) { //Fall rate
      Rpeak++;
      RdotCount = 0;
    }
  }
  else {
    RdotHangCount++;
  }

  if (ELdotHangCount > PEAK_HANG) { //Peak pause length
    if (++ELdotCount >= PEAK_FALL) { //Fall rate
      ELpeak++;
      ELdotCount = 0;
    }
  }
  else {
    ELdotHangCount++;
  }

  if (ERdotHangCount > PEAK_HANG) { //Peak pause length
    if (++ERdotCount >= PEAK_FALL) { //Fall rate
      ERpeak++;
      ERdotCount = 0;
    }
  }
  else {
    ERdotHangCount++;
  }
  int sensitivityValue, sensitivityFactor, leftValue;
  int minValue = 10, maxValue = 700, maxSensitivity = 4 * 255;

  sensitivityValue = 512;
  sensitivityValue = map(sensitivityValue, 0, 1023, 0, 255);
  sensitivityFactor = ((float) sensitivityValue / 255 * (float) maxSensitivity / 255);
  leftValue = map(analogRead(leftCh) * sensitivityFactor, minValue, maxValue, 0, EndLED);

  //  Serial.print(analogRead(leftCh));
  //  Serial.print("\t");
  //  Serial.print(log(analogRead(leftCh) + 1));
  //  Serial.print("\t");
  //  Serial.print(lSample);
  //  Serial.print("\t");
  //  Serial.print(log(lSample + 1));
  //  Serial.print("\t");
  //  Serial.print(Ly);
  //  Serial.print("\t");
  //  Serial.print(ELy);
  //  Serial.print("\t");
}

//Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c , int T) {
  uint8_t fromTemp;
  if (from > to) {
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  if (T == 0)
  {
    for (int i = from; i <= to; i++) {
      Lled.setPixelColor(i, c);
    }
  }
  else if (T == 1)
  {
    for (int i = from; i <= to; i++) {
      Rled.setPixelColor(i, c);
    }
  }
}

//mic blink
void ledMica()
{
  unsigned long startMillis = millis(); // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y, Mvu;


  // collect data for length of sample window (in mS)
  while (millis() - startMillis < SAMPLE_WINDOW)
  {
    msample = analogRead(micPin);
    if (msample < 1024)  // toss out spurious readings
    {
      if (msample > signalMax)
      {
        signalMax = msample;  // save just the max levels
      }
      else if (msample < signalMin)
      {
        signalMin = msample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude

  //Scale the input logarithmically instead of linearly
  c = fscale(MICINPUT_FLOOR, MICINPUT_CEILING, maxPwm, 0, peakToPeak, 2);
  Mvu = map(c, 0, 255, 255, 0);
  for (uint8_t i = 0; i < EndLED; i++) {
    Lled.setPixelColor(i, RD * Mvu / 255, GR * Mvu / 255, BL * Mvu / 255);
    Rled.setPixelColor(i, RD * Mvu / 255, GR * Mvu / 255, BL * Mvu / 255);
  }
  LedShow(0);
}

//void ledMicd()
//{
//  micDigi = digitalRead (micDigipin);
//  if (micDigi != 1)
//    ledOn();
//  else
//    ledOff();
//  Serial.println(micDigi);
//}

void ledOff()
{

  for (i = 0; i < IntLED + ExtLED; i++)
  {
    Lled.setPixelColor(i, 0, 0, 0, 0);
    Rled.setPixelColor(i, 0, 0, 0, 0);
  }
  LedShow(0);
}
void maxVol()
{
  if ((unsigned long)(millis() - previousFadeMillis) >= 500) {
    previousFadeMillis = millis();
    leftLarge = 0;
    rightLarge = 0;
  }
}

void LedShow(int ledtype)
{
  if (ledtype == 0)
  {
    Lled.show();
    Rled.show();
  }
}

//dynamic volume

float fscale( float originalMin, float originalMax, float newBegin, float
              newEnd, float inputValue, float curve) {

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
    Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution
    Serial.println();
  */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin) {
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}

//rainbow
void rainbow(int clr)
{
  for (int i = 0; i < EndLED ; i++)
  {
    if (i < IntLED)
    {
      Lled.setPixelColor(i, WheelVU(map(i, 0, IntLED - 1, 30, 150)));
      Rled.setPixelColor(i, WheelVU(map(i, 0, IntLED - 1, 30, 150)));
    }
    if (i >= IntLED)
    {
      Lled.setPixelColor(i, WheelVU(map(i, IntLED, EndLED - 1, 30, 150)));
      Rled.setPixelColor(i, WheelVU(map(i, IntLED, EndLED - 1, 30, 150)));
    }
  }
  LedShow(0);
}

//all running rainbow
void rainbowCycle() {
  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    uint16_t i;
    for (i = 0; i < EndLED; i++) {
      if (i < IntLED)
      {
        Lled.setPixelColor(i, Wheel(((i * 256 / IntLED) + rainbowCycleCycles) & 255));
        Rled.setPixelColor(i, Wheel(((i * 256 / IntLED) + rainbowCycleCycles) & 255));
      }
      if (i >= IntLED)
      {
        Lled.setPixelColor(i, Wheel(((i * 256 / ExtLED) + rainbowCycleCycles) & 255));
        Rled.setPixelColor(i, Wheel(((i * 256 / ExtLED) + rainbowCycleCycles) & 255));
      }
    }
    LedShow(0);

    rainbowCycleCycles++;
    if (rainbowCycleCycles >= 256 * 5) rainbowCycleCycles = 0;
  }
}

//only extras running rainbow
void rainbowCycle2() {
  if (millis() - previousFadeMillis >= fadeInterval) {
    uint16_t i;
    previousFadeMillis = millis();

    for (i = 0; i < IntLED; i++)
    {
      Lled.setPixelColor(i, NEOCOOL);
      Rled.setPixelColor(i, NEOCOOL);
    }
    for (i = 0; i < ExtLED; i++) {
      Tled = map(i, 0, ExtLED, IntLED, IntLED + ExtLED);
      Lled.setPixelColor(Tled, Wheel(((i * 256 / Lled.numPixels()) + rainbowCycleCycles) & 255));
      Rled.setPixelColor(Tled, Wheel(((i * 256 / Lled.numPixels()) + rainbowCycleCycles) & 255));
    }
    LedShow(0);

    rainbowCycleCycles++;
    if (rainbowCycleCycles >= 256 * 5) rainbowCycleCycles = 0;
  }
}

void TrimBright()
{
  Serial.print(Trimsense);
  Serial.print("\t");
  Serial.print(brightness);
  Serial.print("\t");
  Serial.print(analogRead(Bright));
  Serial.print("\t");
  int ENBright = digitalRead(ENbrightTrim);
  if (ENBright)
    brightness = 255 * Mbright;
  else
    brightness = map(analogRead(Bright), TrimBrightFloor, TrimBrightCeil, 10, 255);
  //  brightness = constrain(brightness, 13, 255);
  Lled.setBrightness(brightness);
  Rled.setBrightness(brightness);
}
void ExtraSense()
{
  float Tempsense = 0;
  int ENTrimsenses = digitalRead(ENsensesTrim);
  if (!ENTrimsenses)
    Tempsense = analogRead(Sense);
  Trimboost = mapped(Tempsense, 0.0, 1023.0, 1.0, 5.0);
  Trimboost = constrain(Trimboost, 1.0, 10.0);
}

uint32_t WheelVU(byte WheelPos) {
  if (WheelPos < 85) {
    return Lled.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  else if (WheelPos < 170) {
    WheelPos -= 85;
    return Lled.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  else {
    WheelPos -= 170;
    return Lled.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if (WheelPos < 85) {
    return Lled.Color(255 - WheelPos * 3, 0, WheelPos * 3, 0);
  }
  if (WheelPos < 170) {
    WheelPos -= 85;
    return Lled.Color(0, WheelPos * 3, 255 - WheelPos * 3, 0);
  }
  WheelPos -= 170;
  return Lled.Color(WheelPos * 3, 255 - WheelPos * 3, 0, 0);
}
uint8_t red(uint32_t c) {
  return (c >> 16);
}
uint8_t green(uint32_t c) {
  return (c >> 8);
}
uint8_t blue(uint32_t c) {
  return (c);
}

//for floating point map
float mapped(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void debugging()
{
  if (ExtSense)  //true/high being not connected
  {
    digitalWrite(6, LOW);
    Serial.print("internal");
    Serial.print(ExtSense);
  }
  else
  {
    digitalWrite(6, HIGH);
    Serial.print ("external");
    Serial.print(ExtSense);
  }
}
