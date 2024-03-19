;
#include "configs.h"
#include "pinouts.h"

#ifdef USE_EEPROM
#include <EEPROM.h>
#endif

#ifndef ARDUINO_AVR_DIGISPARKPRO
#include <npBouncingBall.h>
#include <npColorWipe.h>
#include <npFire.h>
#include <npMeteor.h>
#include <npBase.h>
#include <npVirtualNeo.h>
#include <npNeoPixel.h>
#endif

#ifndef Adafruit_NeoPixel.h
#include <Adafruit_NeoPixel.h>
#endif

#include "vars.h"

// uint32_t Wheel(byte, int, int);

void setup() {
  // Serial.begin(9600);

#ifndef LEGACY_CODES
  LanalyzeL.setAmpliDt(10);
  LanalyzeL.setTrsh(20);
  LanalyzeL.setAmpliK(31);
  LanalyzeL.setWindow(10);
  LanalyzeL.setVolK(22);
  LanalyzeL.setVolMin(0);
  LanalyzeL.setVolMax(100);
  LanalyzeL.setPulseMax(90);

  RanalyzeR.setAmpliDt(10);
  RanalyzeR.setTrsh(20);
  RanalyzeR.setAmpliK(31);
  RanalyzeR.setWindow(10);
  RanalyzeR.setVolK(22);
  RanalyzeR.setVolMin(0);
  RanalyzeR.setVolMax(100);
  RanalyzeR.setPulseMax(90);

  ManalyzeM.setAmpliDt(10);
  ManalyzeM.setAmpliK(31);
  ManalyzeM.setWindow(10);
  ManalyzeM.setVolK(22);
  ManalyzeM.setVolMin(0);
  ManalyzeM.setVolMax(100);
  ManalyzeM.setPulseMax(90);

  mdchange.attach(MDButton, INPUT_PULLUP);
  mdchange.interval(10);
#if !defined(ARDUINO_AVR_DIGISPARKPRO) && defined(SOFT_LED)
  ledonoff.attach(LEDbtn, INPUT_PULLUP);
  ledonoff.interval(10);
#endif
#else
  pinMode(MDButton, INPUT_PULLUP);
#if !defined(ARDUINO_AVR_DIGISPARKPRO) && defined(SOFT_LED)
  pinMode(LEDbtn, INPUT_PULLUP);
#endif

#endif

#ifndef SINGLE_COLOUR_ONLY
#ifndef SEIZURE_INDUCER
  RandColour();
#endif
#endif


#if !defined(ARDUINO_AVR_DIGISPARKPRO)
#ifdef ENTP1
  pinMode(ENTP1, INPUT_PULLUP);
#endif
#ifdef ENTP2
  pinMode(ENTP2, INPUT_PULLUP);
#endif
#ifdef INTERRUPT_MODE_CHANGE && !defined(ARDUINO_AVR_DIGISPARKPRO)
  // attachInterrupt(digitalPinToInterrupt(MDButton), ImodeChange, LOW);
#endif
#ifdef INTERRUPT_LED_SWITCH && !defined(ARDUINO_AVR_DIGISPARKPRO) && defined(SOFT_LED)
  // attachInterrupt(digitalPinToInterrupt(LEDbtn), ILDio, LOW);
#endif
#endif



  Lled.setBrightness(brightness);
  Rled.setBrightness(brightness);

  Lled.begin();
  Rled.begin();

  Lled.clear();
  Rled.clear();

  LedShow(0);
#ifdef COLOUR_WIPE_TEST
  ColorTest(COLOUR_WIPE_DELAYS);
#endif
#ifdef COLOUR_WIPE_STARTUP
  ColorWipe(RD, GR, BL, COLOUR_WIPE_DELAYS);
#endif
#if defined(EXTRA_MODES) && !defined(ARDUINO_AVR_DIGISPARKPRO)
  LMeteor1.changeColor(RD, GR, BL);
  LMeteor2.changeColor(RD, GR, BL);
#endif

#ifdef USE_EEPROM
  EEPROM.get(0, ledMode);
  EEPROM.get(1, ledpwr);
#endif
}

/* mode list
  ledOn(); //colour
  Pulses(0); //Symmetriical Pulse
  Pulses(1); // Assymmetrical Pulse
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

void loop() {


#ifndef LEGACY_CODES
  mdchange.update();
  ledonoff.update();
#endif

#ifdef SEIZURE_INDUCER
  RandColour();
#endif

#ifdef DEBUGS
  debugging();
#endif

#if !defined(ARDUINO_AVR_DIGISPARKPRO)
  // TrimBright();
#ifndef LEGACY_CODES
#ifdef ENTP1
  entp1();
#endif
#ifdef ENTP2
  entp2();
#endif

#endif
#endif
#ifdef SOFT_LED
  if (ledpwr) {
#endif
    switch (ledMode) {
      case 1:
        ledOn();
        break;
      case 2:
        Pulses(0);
        break;
      case 3:
        Pulses(1);
        break;
      case 4:
        blinkVU(1);
        break;
      case 5:
        rainbowVU(1);  //LEDVU mode
        break;
      case 6:
        blinkVU(2);
        break;
      case 7:
        rainbowVU(2);  //MicVU mode
        break;
      case 8:
        rainbowCycle();
        break;
#if !defined(ARDUINO_AVR_DIGISPARKPRO) && defined(EXTRA_MODES)
      case 9:
        FireShows();
        break;
      case 10:
        MeteorShows(METEOR_RAINBOW);
        break;
#ifdef BOUNCING_BALL
      case 11:
        BallBounces(BALL_RAINBOW);
        break;
#endif
#endif
    }
#ifdef SOFT_LED
  } else if (ledpwr == 0) {
    ledOff();
  }
#endif
  //Serial.println(ledMode);

#if !defined(INTERRUPT_MODE_CHANGE) || defined(ARDUINO_AVR_DIGISPARKPRO)
  modeChange();  //comment for usage of interrupt
#endif
#if (!defined(INTERRUPT_LED_SWITCH) || defined(ARDUINO_AVR_DIGISPARKPRO)) && defined(SOFT_LED)
  LDio();  //comment for usage of interrupt
#endif
  //  Serial.print(RD);
  //  Serial.print(", ");
  //  Serial.print(GR);
  //  Serial.print(", ");
  //  Serial.print(BL);
  //  Serial.println();
}
#ifdef SOFT_LED
void ILDio() {
  detachInterrupt(digitalPinToInterrupt(LEDbtn));
  LDio();
  attachInterrupt(digitalPinToInterrupt(LEDbtn), ILDio, LOW);
}
#endif

#ifdef SOFT_LED
void LDio() {
#ifndef INTERRUPT_LED_SWITCH
#if defined(LEGACY_CODES)
  buttonState = digitalRead(LEDbtn);
  if ((buttonState != lastButtonState) && (buttonState == HIGH))
#else
  if (ledonoff.fell())
#endif
  {
#endif
    if (ledpwr) {
      ledpwr = 0;
      // ledOff();
    } else {
      ledpwr = 1;
      RandColour();
    }  //LED's off
       // ledOff();

#ifndef INTERRUPT_LED_SWITCH
  }
#ifdef LEGACY_CODES
  lastButtonState = buttonState;
#endif
#endif

#ifdef USE_EEPROM
  EEPROM.put(1, ledpwr);
#endif
}
#endif

#ifdef INTERRUPT_MODE_CHANGE
void ImodeChange() {
  detachInterrupt(digitalPinToInterrupt(MDButton));
  modeChange();
  attachInterrupt(digitalPinToInterrupt(MDButton), ImodeChange, LOW);
}
#endif

void modeChange() {
#ifdef SOFT_LED
  if (ledpwr) {
#endif
#ifndef INTERRUPT_MODE_CHANGE
#if defined(LEGACY_CODES)
    buttonState = digitalRead(MDButton);
    if ((buttonState != lastButtonState) && (buttonState == HIGH))
#else
    if (mdchange.fell())
#endif
    {
#endif
      fadeValue = 0;

      ledMode++;
      // ledMode %= modes;
      if (ledMode > modes)
        ledMode = 1;

      //LED's off
      ledOff();
      delay(250);

      //Flash LED's
      for (int i = 1; i <= ledMode; i++) {
        rainbow(255);
        delay(100);
        ledOff();
        delay(300);

        //   Rled.rainbow(0,1,255,Rled.sine8(j),true);
      }
#ifndef INTERRUPT_MODE_CHANGE
    }
#ifdef LEGACY_CODES
    lastButtonState = buttonState;
#endif
#endif
#ifdef USE_EEPROM
    EEPROM.put(0, ledMode);
#endif
#ifdef SOFT_LED
  }
#endif
}

void RandColour() {
  randomSeed(analogRead(A7) + analogRead(A6));
  RD = random(0, 255);
  GR = random(0, 255);
  BL = random(0, 255);
}

//only on colour
void ledOn() {
  ;
  //  for (uint8_t i = 0; i <EndLED; i++)
  //  {
  Lled.fill(Lled.gamma32(Lled.Color(RD, GR, BL)));
  Rled.fill(Rled.gamma32(Rled.Color(RD, GR, BL)));
  //    Lled.setPixelColor(i, RD, GR, BL);
  //    Rled.setPixelColor(i, RD, GR, BL);
  //  }
  LedShow(0);
}

void Pulses(int Pmd) {

#ifdef LEGACY_CODES
  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    if (Pmd == 0)  // All Pulses
    {
      if (fadeDirection == UP) {
        for (uint8_t i = 0; i < EndLED; i++) {
          Lled.setPixelColor(i, Lled.gamma32(Lled.Color(RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255)));
          Rled.setPixelColor(i, Rled.gamma32(Rled.Color(RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255)));
        }
        fadeValue++;
      } else if (fadeDirection == DOWN) {
        for (uint8_t i = 0; i < EndLED; i++) {
          Lled.setPixelColor(i, Lled.gamma32(Lled.Color(RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255)));
          Rled.setPixelColor(i, Rled.gamma32(Rled.Color(RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255)));
        }
        fadeValue--;
      }
    }
    if (Pmd == 1)  // Alternate Pulses
    {
      if (fadeDirection == UP) {
        for (uint8_t i = 0; i < EndLED; i++) {
          Lled.setPixelColor(i, RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
        }
        for (uint8_t i = 0; i < EndLED; i++) {
          Rled.setPixelColor(i, RD * map(fadeValue, 0, 255, 255, 0) / 255, GR * map(fadeValue, 0, 255, 255, 0) / 255, BL * map(fadeValue, 0, 255, 255, 0) / 255);
        }
        fadeValue++;
      } else if (fadeDirection == DOWN) {
        for (uint8_t i = 0; i < EndLED; i++) {
          Lled.setPixelColor(i, RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
        }
        for (uint8_t i = 0; i < EndLED; i++) {
          Rled.setPixelColor(i, RD * map(fadeValue, 0, 255, 255, 0) / 255, GR * map(fadeValue, 0, 255, 255, 0) / 255, BL * map(fadeValue, 0, 255, 255, 0) / 255);
        }
        fadeValue--;
      }
    }
    LedShow(0);

    if (fadeValue >= maxPwm && fadeDirection == UP) {
      fadeDirection = DOWN;
    } else if (fadeValue <= minPWM && fadeDirection == DOWN) {
      fadeDirection = UP;
    }
  }
#else
  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    if (Pmd == 0)  // All Pulses
    {
      Lled.fill(Lled.gamma32(Lled.Color(RD * Lled.sine8(fadeValue) / 255, GR * Lled.sine8(fadeValue) / 255, BL * Lled.sine8(fadeValue) / 255)));
      Rled.fill(Rled.gamma32(Rled.Color(RD * Rled.sine8(fadeValue) / 255, GR * Rled.sine8(fadeValue) / 255, BL * Rled.sine8(fadeValue) / 255)));
    } else if (Pmd == 1)  //Alternate pulses
    {
      Lled.fill(Lled.gamma32(Lled.Color(RD * Lled.sine8(fadeValue) / 255, GR * Lled.sine8(fadeValue) / 255, BL * Lled.sine8(fadeValue) / 255)));
      Rled.fill(Rled.gamma32(Rled.Color(RD * Rled.sine8(map(fadeValue, 0, 255, 255, 0)) / 255, GR * Rled.sine8(map(fadeValue, 0, 255, 255, 0)) / 255, BL * Rled.sine8(map(fadeValue, 0, 255, 255, 0)) / 255)));
    }
    fadeValue++;
    LedShow(0);
  }
  fadeValue %= 256;
#endif
}

// void ledPulse1() {
//   if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
//     previousFadeMillis = millis();
//     if (fadeDirection == UP) {
//       for (uint8_t i = 0; i < EndLED; i++) {
//         Lled.setPixelColor(i, Lled.gamma32(Lled.Color(RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255)));
//         Rled.setPixelColor(i, Rled.gamma32(Rled.Color(RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255)));
//       }
//       fadeValue++;
//     } else if (fadeDirection == DOWN) {
//       for (uint8_t i = 0; i < EndLED; i++) {
//         Lled.setPixelColor(i, Lled.gamma32(Lled.Color(RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255)));
//         Rled.setPixelColor(i, Rled.gamma32(Rled.Color(RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255)));
//       }
//       fadeValue--;
//     }
//     LedShow(0);
//   }
//   if (fadeValue >= maxPwm && fadeDirection == UP) {
//     fadeDirection = DOWN;
//   } else if (fadeValue <= minPWM && fadeDirection == DOWN) {
//     fadeDirection = UP;
//   }
// }

//Alternate pulse colour
// void ledPulse2() {

//   if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
//     previousFadeMillis = millis();
//     if (fadeDirection == UP) {
//       for (uint8_t i = 0; i < EndLED; i++) {
//         Lled.setPixelColor(i, RD * fadeValue2 / 255, GR * fadeValue2 / 255, BL * fadeValue2 / 255);
//       }
//       for (uint8_t i = 0; i < EndLED; i++) {
//         Rled.setPixelColor(i, RD * revFade / 255, GR * revFade / 255, BL * revFade / 255);
//       }
//       fadeValue2++;
//       revFade--;
//     } else if (fadeDirection == DOWN) {
//       for (uint8_t i = 0; i < EndLED; i++) {
//         Lled.setPixelColor(i, RD * fadeValue2 / 255, GR * fadeValue2 / 255, BL * fadeValue2 / 255);
//       }
//       for (uint8_t i = 0; i < EndLED; i++) {
//         Rled.setPixelColor(i, RD * revFade / 255, GR * revFade / 255, BL * revFade / 255);
//       }
//       fadeValue2--;
//       revFade++;
//     }
//     LedShow(0);
//   }

//   if (fadeValue2 >= maxPwm && fadeDirection == UP) {
//     fadeDirection = DOWN;
//   } else if (fadeValue2 <= minPWM && fadeDirection == DOWN) {
//     fadeDirection = UP;
//   }
// }

#if !defined(ARDUINO_AVR_DIGISPARKPRO)

//only extras pulsing
void ledPulse3() {

  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    for (uint8_t i = 0; i < IntLEDT; i++) {
      Lled.setPixelColor(i, RD, GR, BL);
      Rled.setPixelColor(i, RD, GR, BL);
    }
    if (fadeDirection == UP) {
      for (uint8_t i = 0; i < ExtLED; i++) {
        Lled.setPixelColor(map(i, 0, ExtLED, IntLEDT, IntLEDT + ExtLED), RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
        Rled.setPixelColor(map(i, 0, ExtLED, IntLEDT, IntLEDT + ExtLED), RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
      }
      fadeValue++;
    } else if (fadeDirection == DOWN) {
      for (uint8_t i = 0; i < ExtLED; i++) {
        Lled.setPixelColor(map(i, 0, ExtLED, IntLEDT, IntLEDT + ExtLED), RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
        Rled.setPixelColor(map(i, 0, ExtLED, IntLEDT, IntLEDT + ExtLED), RD * fadeValue / 255, GR * fadeValue / 255, BL * fadeValue / 255);
      }
      fadeValue--;
    }
    LedShow(0);
  }

  if (fadeValue >= maxPwm && fadeDirection == UP) {
    fadeDirection = DOWN;
  } else if (fadeValue <= minPWM && fadeDirection == DOWN) {
    fadeDirection = UP;
  }
}

//rainbow pulsing
void ledPulse4() {

  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    if (fadeDirection == UP) {
      //      for (uint8_t i = 0; i < IntLEDT + ExtLED; i++) {
      rainbow(fadeValue);
      //      }
      fadeValue++;
    } else if (fadeDirection == DOWN) {
      //      for (uint8_t i = 0; i < IntLEDT + ExtLED; i++) {
      rainbow(fadeValue);
      //      }
      fadeValue--;
    }
    LedShow(0);
  }

  if (fadeValue >= maxPwm && fadeDirection == UP) {
    fadeDirection = DOWN;
  } else if (fadeValue <= minPWM && fadeDirection == DOWN) {
    fadeDirection = UP;
  }
}
#endif


//music blinking single colour
void blinkVU(int Md)

//legacy codes
#if defined(LEGACY_CODES)
{
  unsigned long startMillis = millis();  // Start of sample window

  unsigned int L, R;
  float lpeakToPeak = 0, rpeakToPeak = 0;
  unsigned int lsignalMax = 0, rsignalMax = 0;
  unsigned int lsignalMin = 1023, rsignalMin = 1023;

  while (millis() - startMillis < SAMPLE_WINDOW) {
    if (Md == 1) {
      lSample = analogRead(leftCh);
      rSample = analogRead(rightCh);
    } else if (Md == 2) {
      lSample = analogRead(micCh);
      rSample = analogRead(micCh);
    }
    if (lSample < 1024)  // toss out spurious readings
    {
      if (lSample > lsignalMax) {
        lsignalMax = lSample;  // save just the max levels
      } else if (lSample < lsignalMin) {
        lsignalMin = lSample;  // save just the min levels
      }
    }
    if (rSample < 1024)  // toss out spurious readings
    {
      if (rSample > rsignalMax) {
        rsignalMax = rSample;  // save just the max levels
      } else if (rSample < rsignalMin) {
        rsignalMin = rSample;  // save just the min levels
      }
    }
  }
  lpeakToPeak = lsignalMax - lsignalMin;  // max - min = peak-peak amplitude
  rpeakToPeak = rsignalMax - rsignalMin;  // max - min = peak-peak amplitude

  L = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, maxPwm, 0, lpeakToPeak, 2);
  R = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, maxPwm, 0, rpeakToPeak, 2);

#ifdef INVERT_BLINK
  Lvu = map(L, 0, 255, 255, 0);
  Rvu = map(R, 0, 255, 255, 0);
#else
  Lvu = map(L, 0, 255, 0, 255);
  Rvu = map(R, 0, 255, 0, 255);
#endif
  for (uint8_t i = 0; i < EndLED; i++) {
    Lled.setPixelColor(i, Lled.gamma32(RD * Lvu / 255, GR * Lvu / 255, BL * Lvu / 255));
    Rled.setPixelColor(i, Rled.gamma32(RD * Rvu / 255, GR * Rvu / 255, BL * Rvu / 255));
  }
  LedShow(0);
}

#else
{
  unsigned int L, R;

  if (LanalyzeL.tick() || RanalyzeR.tick() || ManalyzeM.tick()) {
    if (Md == 1) {
      L = LanalyzeL.getVol();
      R = RanalyzeR.getVol();
    } else if (Md == 2) {
      L = ManalyzeM.getVol();
      R = ManalyzeM.getVol();
    }

#ifdef INVERT_BLINK
    Lvu = map(L, 0, 100, maxPwm, 0);
    Rvu = map(R, 0, 100, maxPwm, 0);
#else
    Lvu = map(L, 0, 100, 0, maxPwm);
    Rvu = map(R, 0, 100, 0, maxPwm);
#endif
    for (uint8_t i = 0; i < EndLED; i++) {
      Lled.setPixelColor(i, Lled.gamma32(Lled.Color(RD * Rvu / 255, GR * Rvu / 255, BL * Rvu / 255)));
      Rled.setPixelColor(i, Rled.gamma32(Lled.Color(RD * Rvu / 255, GR * Rvu / 255, BL * Rvu / 255)));
    }
    LedShow(0);
  }
}
#endif

//music vu rainbow

//music vu rainbow
void rainbowVU(int Md)

//legacy rainbow vu
#if defined(LEGACY_CODES)
{
  unsigned long startMillis = millis();  // Start of sample window

  unsigned int L, R, EL, ER;
  float lpeakToPeak = 0, rpeakToPeak = 0;
  unsigned int lsignalMax = 0, rsignalMax = 0;
  unsigned int lsignalMin = 1023, rsignalMin = 1023;

  unsigned int c, Ly, Ry, ELy, ERy;


  // collect data for length of sample window (in mS)
  while (millis() - startMillis < SAMPLE_WINDOW) {
    if (Md == 1) {
      lSample = analogRead(leftCh);
      rSample = analogRead(rightCh);
    } else if (Md == 2) {
      lSample = analogRead(micCh);
      rSample = analogRead(micCh);
    }
    if (lSample < 1024)  // toss out spurious readings
    {
      if (lSample > lsignalMax) {
        lsignalMax = lSample;  // save just the max levels
      } else if (lSample < lsignalMin) {
        lsignalMin = lSample;  // save just the min levels
      }
    }
    if (rSample < 1024)  // toss out spurious readings
    {
      if (rSample > rsignalMax) {
        rsignalMax = rSample;  // save just the max levels
      } else if (rSample < rsignalMin) {
        rsignalMin = rSample;  // save just the min levels
      }
    }
  }
  lpeakToPeak = lsignalMax - lsignalMin;  // max - min = peak-peak amplitude
  rpeakToPeak = rsignalMax - rsignalMin;  // max - min = peak-peak amplitude

  //Scale the input logarithmically instead of linearly
  if (Md == 1) {
    L = fscale(MUSINPUT_FLOOR2, MUSINPUT_CEILING2, IntLEDT, 0, lpeakToPeak, 2);
    EL = fscale(MUSINPUT_FLOOR2, MUSINPUT_CEILING2, (ExtLED / 2), 0, lpeakToPeak, 2);
    R = fscale(MUSINPUT_FLOOR2, MUSINPUT_CEILING2, IntLEDT, 0, lpeakToPeak, 2);
    ER = fscale(MUSINPUT_FLOOR2, MUSINPUT_CEILING2, (ExtLED / 2), 0, rpeakToPeak, 2);
    //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
  } else if (Md == 2) {
    L = fscale(MICINPUT_FLOOR, MICINPUT_CEILING, IntLEDT, 0, lpeakToPeak, 2);
    EL = fscale(MICINPUT_FLOOR, MICINPUT_CEILING, (ExtLED / 2), 0, lpeakToPeak, 2);
    R = fscale(MICINPUT_FLOOR, MICINPUT_CEILING, IntLEDT, 0, rpeakToPeak, 2);
    ER = fscale(MICINPUT_FLOOR, MICINPUT_CEILING, (ExtLED / 2), 0, rpeakToPeak, 2);
    //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
  }
  //Fill the strip with rainbow gradient
  for (uint8_t i = 0; i < EndLED; i++) {
    if (i < IntLEDT) {
      Lled.setPixelColor(i, WheelVU(map(i, 0, IntLEDT - 1, 30, 150), 0));
      Rled.setPixelColor(i, WheelVU(map(i, 0, IntLEDT - 1, 30, 150), 1));
    }
    if ((i >= IntLEDT) && (i < IntLEDT + (ExtLED / 2))) {
      Lled.setPixelColor(i, WheelVU(map(i, IntLEDT, (IntLEDT + (ExtLED / 2)) - 1, 30, 150), 0));
      Rled.setPixelColor(i, WheelVU(map(i, IntLEDT, (IntLEDT + (ExtLED / 2)) - 1, 30, 150), 1));
    }
    if ((i >= IntLEDT + (ExtLED / 2)) && (i < EndLED)) {
      Lled.setPixelColor(i, WheelVU(map(i, EndLED, EndLED - (ExtLED / 2), 30, 150), 0));
      Rled.setPixelColor(i, WheelVU(map(i, EndLED, EndLED - (ExtLED / 2), 30, 150), 1));
    }
  }

  if (L < Lpeak) {
    Lpeak = L;          // Keep dot on top
    LdotHangCount = 0;  // make the dot hang before falling
  }
  if (EL < ELpeak) {
    ELpeak = EL;         // Keep dot on top
    ELdotHangCount = 0;  // make the dot hang before falling
  }
  if (L <= IntLEDT) {  // Fill partial column with off pixels
    drawLine(IntLEDT, IntLEDT - L, Lled.Color(0, 0, 0), 0);
  }
  if (EL <= ExtLED / 2) {  // Fill partial column with off pixels
    drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - EL, Lled.Color(0, 0, 0), 0);
    drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + EL, Lled.Color(0, 0, 0), 0);
  }

  if (R < Rpeak) {
    Rpeak = R;          // Keep dot on top
    RdotHangCount = 0;  // make the dot hang before falling
  }
  if (ER < ERpeak) {
    ERpeak = ER;         // Keep dot on top
    ERdotHangCount = 0;  // make the dot hang before falling
  }
  if (R <= IntLEDT) {  // Fill partial column with off pixels
    drawLine(IntLEDT, IntLEDT - R, Rled.Color(0, 0, 0), 1);
  }
  if (ER <= ExtLED / 2) {  // Fill partial column with off pixels
    drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - ER, Rled.Color(0, 0, 0), 1);
    drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + ER, Rled.Color(0, 0, 0), 1);
  }
  // Set the peak dot to match the rainbow gradient
  Ly = IntLEDT - Lpeak;
  ELy = (ExtLED / 2) - ELpeak;
  Ry = IntLEDT - Rpeak;
  ERy = (ExtLED / 2) - ERpeak;

  Lled.setPixelColor(Ly - 1, WheelVU(map(Ly, 0, IntLEDT - 1, 30, 150), 0));
  Lled.setPixelColor(IntLEDT + ELy - 1, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));
  Lled.setPixelColor(EndLED - ELy, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));

  if (ELy == 0) {
    Lled.setPixelColor(IntLEDT + ELy - 1, 0, 0, 0);
    Lled.setPixelColor(EndLED - ELy, 0, 0, 0);
  }

  Rled.setPixelColor(Ry - 1, WheelVU(map(Ry, 0, IntLEDT - 1, 30, 150), 1));
  Rled.setPixelColor(IntLEDT + ERy - 1, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));
  Rled.setPixelColor(EndLED - ERy, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));

  if (ERy == 0) {
    Rled.setPixelColor(IntLEDT + ERy - 1, 0, 0, 0);
    Rled.setPixelColor(EndLED - ERy, 0, 0, 0);
  }
  LedShow(0);

  // Frame based peak dot animation
  if (LdotHangCount > PEAK_HANG) {   //Peak pause length
    if (++LdotCount >= PEAK_FALL) {  //Fall rate
      Lpeak++;
      LdotCount = 0;
    }
  } else {
    LdotHangCount++;
  }

  if (RdotHangCount > PEAK_HANG) {   //Peak pause length
    if (++RdotCount >= PEAK_FALL) {  //Fall rate
      Rpeak++;
      RdotCount = 0;
    }
  } else {
    RdotHangCount++;
  }

  if (ELdotHangCount > PEAK_HANG) {   //Peak pause length
    if (++ELdotCount >= PEAK_FALL) {  //Fall rate
      ELpeak++;
      ELdotCount = 0;
    }
  } else {
    ELdotHangCount++;
  }

  if (ERdotHangCount > PEAK_HANG) {   //Peak pause length
    if (++ERdotCount >= PEAK_FALL) {  //Fall rate
      ERpeak++;
      ERdotCount = 0;
    }
  } else {
    ERdotHangCount++;
  }
  int sensitivityValue, sensitivityFactor, leftValue;
  int minValue = 10, maxValue = 700, maxSensitivity = 4 * 255;

  sensitivityValue = 512;
  sensitivityValue = map(sensitivityValue, 0, 1023, 0, 255);
  sensitivityFactor = ((float)sensitivityValue / 255 * (float)maxSensitivity / 255);
  leftValue = map(analogRead(leftCh) * sensitivityFactor, minValue, maxValue, 0, EndLED);
}

#else
{
  unsigned int L, R, EL, ER, LB, RB;
  unsigned int c, Ly, Ry, ELy, ERy, LBy, RBy;

  // collect data for length of sample window (in mS)
  if (LanalyzeL.tick() || RanalyzeR.tick() || ManalyzeM.tick()) {
    //    Serial.print(LanalyzeL.getVol());
    //    Serial.print(" ");
    //    Serial. print (75);
    //    Serial.print(" ");
    //    Serial.print(LanalyzeL.pulse());
    //    Serial.println();
    //Scale the input logarithmically instead of linearly
    if (Md == 1) {
      L = map(LanalyzeL.getVol(), 0, 100, IntLEDT, 0);
      EL = map(LanalyzeL.getVol(), 0, 100, (ExtLED / 2), 0);
      R = map(RanalyzeR.getVol(), 0, 100, IntLEDT, 0);
      ER = map(RanalyzeR.getVol(), 0, 100, (ExtLED / 2), 0);
      //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    } else if (Md == 2) {
      L = map(ManalyzeM.getVol(), 0, 100, IntLEDT, 0);
      EL = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
      R = map(ManalyzeM.getVol(), 0, 100, IntLEDT, 0);
      ER = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
      //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    } else if (Md == 3) { //Parallel VU
      L = map(LanalyzeL.getVol(), 0, 100, IntLEDF, 0);
      LB = map(LanalyzeL.getVol(), 0, 100, IntLEDB, 0);
      EL = map(LanalyzeL.getVol(), 0, 100, (ExtLED / 2), 0);
      R = map(RanalyzeR.getVol(), 0, 100, IntLEDF, 0);
      RB = map(RanalyzeR.getVol(), 0, 100, IntLEDB, 0);
      ER = map(RanalyzeR.getVol(), 0, 100, (ExtLED / 2), 0);
      //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    } else if (Md == 4) {
      L = map(ManalyzeM.getVol(), 0, 100, IntLEDF, 0);
      LB = map(ManalyzeM.getVol(), 0, 100, IntLEDB, 0);
      EL = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
      R = map(ManalyzeM.getVol(), 0, 100, IntLEDF, 0);
      RB = map(ManalyzeM.getVol(), 0, 100, IntLEDB, 0);
      ER = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
      //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    }
    //Fill the strip with rainbow gradient
    for (uint8_t i = 0; i < EndLED; i++) {
      if (i < IntLEDT) {
        if (Md == 3 && i < IntLEDF) {
          Lled.setPixelColor(i, WheelVU(map(i, 0, IntLEDF - 1, 30, 150), 0));
          Rled.setPixelColor(i, WheelVU(map(i, 0, IntLEDF - 1, 30, 150), 1));
        }
        if (Md == 3 && i >= IntLEDF && i < IntLEDT) {
          Lled.setPixelColor(i, WheelVU(map(i, IntLEDF, IntLEDB - 1, 30, 150), 0));
          Rled.setPixelColor(i, WheelVU(map(i, IntLEDF, IntLEDB - 1, 30, 150), 1));
        }
        if (Md != 3) {
          Lled.setPixelColor(i, WheelVU(map(i, 0, IntLEDT - 1, 30, 150), 0));
          Rled.setPixelColor(i, WheelVU(map(i, 0, IntLEDT - 1, 30, 150), 1));
        }
      }
      if ((i >= IntLEDT) && (i < IntLEDT + (ExtLED / 2))) {
        Lled.setPixelColor(i, WheelVU(map(i, IntLEDT, (IntLEDT + (ExtLED / 2)) - 1, 30, 150), 0));
        Rled.setPixelColor(i, WheelVU(map(i, IntLEDT, (IntLEDT + (ExtLED / 2)) - 1, 30, 150), 1));
      }
      if ((i >= IntLEDT + (ExtLED / 2)) && (i < EndLED)) {
        Lled.setPixelColor(i, WheelVU(map(i, EndLED, EndLED - (ExtLED / 2), 30, 150), 0));
        Rled.setPixelColor(i, WheelVU(map(i, EndLED, EndLED - (ExtLED / 2), 30, 150), 1));
      }
    }
    if (Md == 1 || Md == 2) {
      if (L < Lpeak) {
        Lpeak = L;          // Keep dot on top
        LdotHangCount = 0;  // make the dot hang before falling
      }
      if (EL < ELpeak) {
        ELpeak = EL;         // Keep dot on top
        ELdotHangCount = 0;  // make the dot hang before falling
      }
      if (L <= IntLEDT) {  // Fill partial column with off pixels
        drawLine(IntLEDT, IntLEDT - L, Lled.Color(0, 0, 0), 0);
      }
      if (EL <= ExtLED / 2) {  // Fill partial column with off pixels
        drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - EL, Lled.Color(0, 0, 0), 0);
        drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + EL, Lled.Color(0, 0, 0), 0);
      }

      if (R < Rpeak) {
        Rpeak = R;          // Keep dot on top
        RdotHangCount = 0;  // make the dot hang before falling
      }
      if (ER < ERpeak) {
        ERpeak = ER;         // Keep dot on top
        ERdotHangCount = 0;  // make the dot hang before falling
      }
      if (R <= IntLEDT) {  // Fill partial column with off pixels
        drawLine(IntLEDT, IntLEDT - R, Rled.Color(0, 0, 0), 1);
      }
      if (ER <= ExtLED / 2) {  // Fill partial column with off pixels
        drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - ER, Rled.Color(0, 0, 0), 1);
        drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + ER, Rled.Color(0, 0, 0), 1);
      }
      // Set the peak dot to match the rainbow gradient
      Ly = IntLEDT - Lpeak;
      ELy = (ExtLED / 2) - ELpeak;
      Ry = IntLEDT - Rpeak;
      ERy = (ExtLED / 2) - ERpeak;

      Lled.setPixelColor(Ly - 1, WheelVU(map(Ly, 0, IntLEDT - 1, 30, 150), 0));
      Lled.setPixelColor(IntLEDT + ELy - 1, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));
      Lled.setPixelColor(EndLED - ELy, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));

      if (ELy == 0) {
        Lled.setPixelColor(IntLEDT + ELy - 1, 0, 0, 0);
        Lled.setPixelColor(EndLED - ELy, 0, 0, 0);
      }

      Rled.setPixelColor(Ry - 1, WheelVU(map(Ry, 0, IntLEDT - 1, 30, 150), 1));
      Rled.setPixelColor(IntLEDT + ERy - 1, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));
      Rled.setPixelColor(EndLED - ERy, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));

      if (ERy == 0) {
        Rled.setPixelColor(IntLEDT + ERy - 1, 0, 0, 0);
        Rled.setPixelColor(EndLED - ERy, 0, 0, 0);
      }
      LedShow(0);

      // Frame based peak dot animation
      if (LdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++LdotCount >= PEAK_FALL) {  //Fall rate
          Lpeak++;
          LdotCount = 0;
        }
      } else {
        LdotHangCount++;
      }

      if (RdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++RdotCount >= PEAK_FALL) {  //Fall rate
          Rpeak++;
          RdotCount = 0;
        }
      } else {
        RdotHangCount++;
      }

      if (ELdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++ELdotCount >= PEAK_FALL) {  //Fall rate
          ELpeak++;
          ELdotCount = 0;
        }
      } else {
        ELdotHangCount++;
      }

      if (ERdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++ERdotCount >= PEAK_FALL) {  //Fall rate
          ERpeak++;
          ERdotCount = 0;
        }
      } else {
        ERdotHangCount++;
      }
    }
    if (Md == 3 || Md == 4) {
      if (L < Lpeak) {
        Lpeak = L;          // Keep dot on top
        LdotHangCount = 0;  // make the dot hang before falling
      }
      if (LB < LBpeak) {
        LBpeak = LB;         // Keep dot on top
        LBdotHangCount = 0;  // make the dot hang before falling
      }
      if (EL < ELpeak) {
        ELpeak = EL;         // Keep dot on top
        ELdotHangCount = 0;  // make the dot hang before falling
      }
      if (L <= IntLEDF) {  // Fill partial column with off pixels
        drawLine(IntLEDF, IntLEDF - L, Lled.Color(0, 0, 0), 0);
      }
      if (LB <= IntLEDB) {  // Fill partial column with off pixels
        drawLine(IntLEDB, IntLEDB - LB, Lled.Color(0, 0, 0), 0);
      }
      if (EL <= ExtLED / 2) {  // Fill partial column with off pixels
        drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - EL, Lled.Color(0, 0, 0), 0);
        drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + EL, Lled.Color(0, 0, 0), 0);
      }
      if (R < Rpeak) {
        Rpeak = R;          // Keep dot on top
        RdotHangCount = 0;  // make the dot hang before falling
      }
      if (RB < RBpeak) {
        RBpeak = RB;         // Keep dot on top
        RBdotHangCount = 0;  // make the dot hang before falling
      }
      if (ER < ERpeak) {
        ERpeak = ER;         // Keep dot on top
        ERdotHangCount = 0;  // make the dot hang before falling
      }
      if (R <= IntLEDT) {  // Fill partial column with off pixels
        drawLine(IntLEDT, IntLEDT - R, Rled.Color(0, 0, 0), 1);
      }
      if (RB <= IntLEDB) {  // Fill partial column with off pixels
        drawLine(IntLEDF, IntLEDF - RB, Rled.Color(0, 0, 0), 1);
      }
      if (ER <= ExtLED / 2) {  // Fill partial column with off pixels
        drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - ER, Rled.Color(0, 0, 0), 1);
        drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + ER, Rled.Color(0, 0, 0), 1);
      }
      // Set the peak dot to match the rainbow gradient
      Ly = IntLEDF - Lpeak;
      LBy = IntLEDB - LBpeak;
      ELy = (ExtLED / 2) - ELpeak;
      Ry = IntLEDF - Rpeak;
      RBy = IntLEDB - RBpeak;
      ERy = (ExtLED / 2) - ERpeak;

      Lled.setPixelColor(Ly - 1, WheelVU(map(Ly, 0, IntLEDF - 1, 30, 150), 0));
      Lled.setPixelColor(LBy - 1, WheelVU(map(LBy, 0, IntLEDB - 1, 30, 150), 0));
      Lled.setPixelColor(IntLEDT + ELy - 1, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));
      Lled.setPixelColor(EndLED - ELy, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));

      if (ELy == 0) {  //turns off the external if there's no Peak
        Lled.setPixelColor(IntLEDT + ELy - 1, 0, 0, 0);
        Lled.setPixelColor(EndLED - ELy, 0, 0, 0);
      }

      Rled.setPixelColor(Ry - 1, WheelVU(map(Ry, 0, IntLEDF - 1, 30, 150), 1));
      Rled.setPixelColor(RBy - 1, WheelVU(map(RBy, 0, IntLEDB - 1, 30, 150), 1));
      Rled.setPixelColor(IntLEDT + ERy - 1, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));
      Rled.setPixelColor(EndLED - ERy, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));

      if (ERy == 0) {  //turns off the external if there's no Peak
        Rled.setPixelColor(IntLEDT + ERy - 1, 0, 0, 0);
        Rled.setPixelColor(EndLED - ERy, 0, 0, 0);
      }
      LedShow(0);

      // Frame based peak dot animation
      if (LdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++LdotCount >= PEAK_FALL) {  //Fall rate
          Lpeak++;
          LdotCount = 0;
        }
      } else {
        LdotHangCount++;
      }
      if (LBdotHangCount > PEAK_HANG) {  //Peak pause length
        if (++LdotCount >= PEAK_FALL) {  //Fall rate
          LBpeak++;
          LBdotCount = 0;
        }
      } else {
        LBdotHangCount++;
      }

      if (RdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++RdotCount >= PEAK_FALL) {  //Fall rate
          Rpeak++;
          RdotCount = 0;
        }
      } else {
        RdotHangCount++;
      }
      if (RBdotHangCount > PEAK_HANG) {  //Peak pause length
        if (++RdotCount >= PEAK_FALL) {  //Fall rate
          RBpeak++;
          RBdotCount = 0;
        }
      } else {
        RBdotHangCount++;
      }

      if (ELdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++ELdotCount >= PEAK_FALL) {  //Fall rate
          ELpeak++;
          ELdotCount = 0;
        }
      } else {
        ELdotHangCount++;
      }

      if (ERdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++ERdotCount >= PEAK_FALL) {  //Fall rate
          ERpeak++;
          ERdotCount = 0;
        }
      } else {
        ERdotHangCount++;
      }
    }
  }
}
#endif

void rainbowpvu(int Md)
{
  unsigned int L, R, EL, ER, LB, RB;
  unsigned int c, Ly, Ry, ELy, ERy, LBy, RBy;

  // collect data for length of sample window (in mS)
  if (LanalyzeL.tick() || RanalyzeR.tick() || ManalyzeM.tick()) {
    //    Serial.print(LanalyzeL.getVol());
    //    Serial.print(" ");
    //    Serial. print (75);
    //    Serial.print(" ");
    //    Serial.print(LanalyzeL.pulse());
    //    Serial.println();
    //Scale the input logarithmically instead of linearly
    if (Md == 1) {
      L = map(LanalyzeL.getVol(), 0, 100, IntLEDT, 0);
      EL = map(LanalyzeL.getVol(), 0, 100, (ExtLED / 2), 0);
      R = map(RanalyzeR.getVol(), 0, 100, IntLEDT, 0);
      ER = map(RanalyzeR.getVol(), 0, 100, (ExtLED / 2), 0);
      //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    } else if (Md == 2) {
      L = map(ManalyzeM.getVol(), 0, 100, IntLEDT, 0);
      EL = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
      R = map(ManalyzeM.getVol(), 0, 100, IntLEDT, 0);
      ER = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
      //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    } else if (Md == 3) { //Parallel VU
      L = map(LanalyzeL.getVol(), 0, 100, IntLEDF, 0);
      LB = map(LanalyzeL.getVol(), 0, 100, IntLEDB, 0);
      EL = map(LanalyzeL.getVol(), 0, 100, (ExtLED / 2), 0);
      R = map(RanalyzeR.getVol(), 0, 100, IntLEDF, 0);
      RB = map(RanalyzeR.getVol(), 0, 100, IntLEDB, 0);
      ER = map(RanalyzeR.getVol(), 0, 100, (ExtLED / 2), 0);
      //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    } else if (Md == 4) {
      L = map(ManalyzeM.getVol(), 0, 100, IntLEDF, 0);
      LB = map(ManalyzeM.getVol(), 0, 100, IntLEDB, 0);
      EL = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
      R = map(ManalyzeM.getVol(), 0, 100, IntLEDF, 0);
      RB = map(ManalyzeM.getVol(), 0, 100, IntLEDB, 0);
      ER = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
      //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    }
    //Fill the strip with rainbow gradient
    for (uint8_t i = 0; i < EndLED; i++) {
      if (i < IntLEDT) {
        if (Md == 3 && i < IntLEDF) {
          Lled.setPixelColor(i, WheelVU(map(i, 0, IntLEDF - 1, 30, 150), 0));
          Rled.setPixelColor(i, WheelVU(map(i, 0, IntLEDF - 1, 30, 150), 1));
        }
        if (Md == 3 && i >= IntLEDF && i < IntLEDT) {
          Lled.setPixelColor(i, WheelVU(map(i, IntLEDF, IntLEDB - 1, 30, 150), 0));
          Rled.setPixelColor(i, WheelVU(map(i, IntLEDF, IntLEDB - 1, 30, 150), 1));
        }
        if (Md != 3) {
          Lled.setPixelColor(i, WheelVU(map(i, 0, IntLEDT - 1, 30, 150), 0));
          Rled.setPixelColor(i, WheelVU(map(i, 0, IntLEDT - 1, 30, 150), 1));
        }
      }
      if ((i >= IntLEDT) && (i < IntLEDT + (ExtLED / 2))) {
        Lled.setPixelColor(i, WheelVU(map(i, IntLEDT, (IntLEDT + (ExtLED / 2)) - 1, 30, 150), 0));
        Rled.setPixelColor(i, WheelVU(map(i, IntLEDT, (IntLEDT + (ExtLED / 2)) - 1, 30, 150), 1));
      }
      if ((i >= IntLEDT + (ExtLED / 2)) && (i < EndLED)) {
        Lled.setPixelColor(i, WheelVU(map(i, EndLED, EndLED - (ExtLED / 2), 30, 150), 0));
        Rled.setPixelColor(i, WheelVU(map(i, EndLED, EndLED - (ExtLED / 2), 30, 150), 1));
      }
    }
    if (Md == 1 || Md == 2) {
      if (L < Lpeak) {
        Lpeak = L;          // Keep dot on top
        LdotHangCount = 0;  // make the dot hang before falling
      }
      if (EL < ELpeak) {
        ELpeak = EL;         // Keep dot on top
        ELdotHangCount = 0;  // make the dot hang before falling
      }
      if (L <= IntLEDT) {  // Fill partial column with off pixels
        drawLine(IntLEDT, IntLEDT - L, Lled.Color(0, 0, 0), 0);
      }
      if (EL <= ExtLED / 2) {  // Fill partial column with off pixels
        drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - EL, Lled.Color(0, 0, 0), 0);
        drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + EL, Lled.Color(0, 0, 0), 0);
      }

      if (R < Rpeak) {
        Rpeak = R;          // Keep dot on top
        RdotHangCount = 0;  // make the dot hang before falling
      }
      if (ER < ERpeak) {
        ERpeak = ER;         // Keep dot on top
        ERdotHangCount = 0;  // make the dot hang before falling
      }
      if (R <= IntLEDT) {  // Fill partial column with off pixels
        drawLine(IntLEDT, IntLEDT - R, Rled.Color(0, 0, 0), 1);
      }
      if (ER <= ExtLED / 2) {  // Fill partial column with off pixels
        drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - ER, Rled.Color(0, 0, 0), 1);
        drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + ER, Rled.Color(0, 0, 0), 1);
      }
      // Set the peak dot to match the rainbow gradient
      Ly = IntLEDT - Lpeak;
      ELy = (ExtLED / 2) - ELpeak;
      Ry = IntLEDT - Rpeak;
      ERy = (ExtLED / 2) - ERpeak;

      Lled.setPixelColor(Ly - 1, WheelVU(map(Ly, 0, IntLEDT - 1, 30, 150), 0));
      Lled.setPixelColor(IntLEDT + ELy - 1, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));
      Lled.setPixelColor(EndLED - ELy, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));

      if (ELy == 0) {
        Lled.setPixelColor(IntLEDT + ELy - 1, 0, 0, 0);
        Lled.setPixelColor(EndLED - ELy, 0, 0, 0);
      }

      Rled.setPixelColor(Ry - 1, WheelVU(map(Ry, 0, IntLEDT - 1, 30, 150), 1));
      Rled.setPixelColor(IntLEDT + ERy - 1, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));
      Rled.setPixelColor(EndLED - ERy, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));

      if (ERy == 0) {
        Rled.setPixelColor(IntLEDT + ERy - 1, 0, 0, 0);
        Rled.setPixelColor(EndLED - ERy, 0, 0, 0);
      }
      LedShow(0);

      // Frame based peak dot animation
      if (LdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++LdotCount >= PEAK_FALL) {  //Fall rate
          Lpeak++;
          LdotCount = 0;
        }
      } else {
        LdotHangCount++;
      }

      if (RdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++RdotCount >= PEAK_FALL) {  //Fall rate
          Rpeak++;
          RdotCount = 0;
        }
      } else {
        RdotHangCount++;
      }

      if (ELdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++ELdotCount >= PEAK_FALL) {  //Fall rate
          ELpeak++;
          ELdotCount = 0;
        }
      } else {
        ELdotHangCount++;
      }

      if (ERdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++ERdotCount >= PEAK_FALL) {  //Fall rate
          ERpeak++;
          ERdotCount = 0;
        }
      } else {
        ERdotHangCount++;
      }
    }
    if (Md == 3 || Md == 4) {
      if (L < Lpeak) {
        Lpeak = L;          // Keep dot on top
        LdotHangCount = 0;  // make the dot hang before falling
      }
      if (LB < LBpeak) {
        LBpeak = LB;         // Keep dot on top
        LBdotHangCount = 0;  // make the dot hang before falling
      }
      if (EL < ELpeak) {
        ELpeak = EL;         // Keep dot on top
        ELdotHangCount = 0;  // make the dot hang before falling
      }
      if (L <= IntLEDF) {  // Fill partial column with off pixels
        drawLine(IntLEDF, IntLEDF - L, Lled.Color(0, 0, 0), 0);
      }
      if (LB <= IntLEDB) {  // Fill partial column with off pixels
        drawLine(IntLEDB, IntLEDB - LB, Lled.Color(0, 0, 0), 0);
      }
      if (EL <= ExtLED / 2) {  // Fill partial column with off pixels
        drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - EL, Lled.Color(0, 0, 0), 0);
        drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + EL, Lled.Color(0, 0, 0), 0);
      }
      if (R < Rpeak) {
        Rpeak = R;          // Keep dot on top
        RdotHangCount = 0;  // make the dot hang before falling
      }
      if (RB < RBpeak) {
        RBpeak = RB;         // Keep dot on top
        RBdotHangCount = 0;  // make the dot hang before falling
      }
      if (ER < ERpeak) {
        ERpeak = ER;         // Keep dot on top
        ERdotHangCount = 0;  // make the dot hang before falling
      }
      if (R <= IntLEDT) {  // Fill partial column with off pixels
        drawLine(IntLEDT, IntLEDT - R, Rled.Color(0, 0, 0), 1);
      }
      if (RB <= IntLEDB) {  // Fill partial column with off pixels
        drawLine(IntLEDF, IntLEDF - RB, Rled.Color(0, 0, 0), 1);
      }
      if (ER <= ExtLED / 2) {  // Fill partial column with off pixels
        drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - ER, Rled.Color(0, 0, 0), 1);
        drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2)) + ER, Rled.Color(0, 0, 0), 1);
      }
      // Set the peak dot to match the rainbow gradient
      Ly = IntLEDF - Lpeak;
      LBy = IntLEDB - LBpeak;
      ELy = (ExtLED / 2) - ELpeak;
      Ry = IntLEDF - Rpeak;
      RBy = IntLEDB - RBpeak;
      ERy = (ExtLED / 2) - ERpeak;

      Lled.setPixelColor(Ly - 1, WheelVU(map(Ly, 0, IntLEDF - 1, 30, 150), 0));
      Lled.setPixelColor(LBy - 1, WheelVU(map(LBy, 0, IntLEDB - 1, 30, 150), 0));
      Lled.setPixelColor(IntLEDT + ELy - 1, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));
      Lled.setPixelColor(EndLED - ELy, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));

      if (ELy == 0) {  //turns off the external if there's no Peak
        Lled.setPixelColor(IntLEDT + ELy - 1, 0, 0, 0);
        Lled.setPixelColor(EndLED - ELy, 0, 0, 0);
      }

      Rled.setPixelColor(Ry - 1, WheelVU(map(Ry, 0, IntLEDF - 1, 30, 150), 1));
      Rled.setPixelColor(RBy - 1, WheelVU(map(RBy, 0, IntLEDB - 1, 30, 150), 1));
      Rled.setPixelColor(IntLEDT + ERy - 1, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));
      Rled.setPixelColor(EndLED - ERy, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));

      if (ERy == 0) {  //turns off the external if there's no Peak
        Rled.setPixelColor(IntLEDT + ERy - 1, 0, 0, 0);
        Rled.setPixelColor(EndLED - ERy, 0, 0, 0);
      }
      LedShow(0);

      // Frame based peak dot animation
      if (LdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++LdotCount >= PEAK_FALL) {  //Fall rate
          Lpeak++;
          LdotCount = 0;
        }
      } else {
        LdotHangCount++;
      }
      if (LBdotHangCount > PEAK_HANG) {  //Peak pause length
        if (++LdotCount >= PEAK_FALL) {  //Fall rate
          LBpeak++;
          LBdotCount = 0;
        }
      } else {
        LBdotHangCount++;
      }

      if (RdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++RdotCount >= PEAK_FALL) {  //Fall rate
          Rpeak++;
          RdotCount = 0;
        }
      } else {
        RdotHangCount++;
      }
      if (RBdotHangCount > PEAK_HANG) {  //Peak pause length
        if (++RdotCount >= PEAK_FALL) {  //Fall rate
          RBpeak++;
          RBdotCount = 0;
        }
      } else {
        RBdotHangCount++;
      }

      if (ELdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++ELdotCount >= PEAK_FALL) {  //Fall rate
          ELpeak++;
          ELdotCount = 0;
        }
      } else {
        ELdotHangCount++;
      }

      if (ERdotHangCount > PEAK_HANG) {   //Peak pause length
        if (++ERdotCount >= PEAK_FALL) {  //Fall rate
          ERpeak++;
          ERdotCount = 0;
        }
      } else {
        ERdotHangCount++;
      }
    }
  }
}

//Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c, int T) {
  uint8_t fromTemp;
  if (from > to) {
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  if (T == 0) {
    for (int i = from; i <= to; i++) {
      Lled.setPixelColor(i, Lled.gamma32(c));
    }
  } else if (T == 1) {
    for (int i = from; i <= to; i++) {
      Rled.setPixelColor(i, Rled.gamma32(c));
    }
  }
}

void ledOff() {

  // for (uint8_t i = 0; i < IntLEDT + ExtLED; i++) {
  Lled.clear();
  Rled.clear();
  // }
  LedShow(0);
}

void LedShow(int ledtype) {
  if (ledtype == 0) {
    Lled.show();
    Rled.show();
  }
}

#ifdef LEGACY_CODES
//dynamic volume
float fscale(float originalMin, float originalMax, float newBegin, float newEnd, float inputValue, float curve) {

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

  curve = (curve * -.1);   // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output
  curve = pow(10, curve);  // convert linear scale into lograthimic exponent for other pow function

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
  } else {
    NewRange = newBegin - newEnd;
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal = zeroRefCurVal / OriginalRange;  // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine
  if (originalMin > originalMax) {
    return 0;
  }

  if (invFlag == 0) {
    rangedValue = (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  } else  // invert the ranges
  {
    rangedValue = newBegin - (pow(normalizedCurVal, curve) * NewRange);
  }

  return rangedValue;
}
#endif
uint32_t wheel(byte, int, int);

//rainbow
void rainbow(int clr) {
  for (uint8_t i = 0; i < EndLED; i++) {
    if (i < IntLEDT) {
      Lled.setPixelColor(i, WheelVU(map(i, 0, IntLEDT - 1, 30, 150), 0));
      Rled.setPixelColor(i, WheelVU(map(i, 0, IntLEDT - 1, 30, 150), 1));
    }
    if (i >= IntLEDT) {
      Lled.setPixelColor(i, WheelVU(map(i, IntLEDT, EndLED - 1, 30, 150), 0));
      Rled.setPixelColor(i, WheelVU(map(i, IntLEDT, EndLED - 1, 30, 150), 1));
    }
  }
  LedShow(0);
}

//all running rainbow
void rainbowCycle() {
  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();

    for (uint8_t i = 0; i < EndLED; i++) {
      if (i < IntLEDT) {
        Lled.setPixelColor(i, (WheelVU(((i * 256 / IntLEDT) + rainbowCycles) & 255, 0)));
        Rled.setPixelColor(i, (WheelVU(((i * 256 / IntLEDT) + rainbowCycles) & 255, 1)));
      }
      if (i >= IntLEDT) {
        Lled.setPixelColor(i, (WheelVU(((i * 256 / ExtLED) + rainbowCycles) & 255, 0)));
        Rled.setPixelColor(i, (WheelVU(((i * 256 / ExtLED) + rainbowCycles) & 255, 1)));
      }
    }
    LedShow(0);

    rainbowCycles++;
    if (rainbowCycles >= 256 * 5) rainbowCycles = 0;
  }
}

#if !defined(ARDUINO_AVR_DIGISPARKPRO)

//only extras running rainbow
void rainbowCycle2() {
  if (millis() - previousFadeMillis >= fadeInterval) {
    previousFadeMillis = millis();

    for (uint8_t i = 0; i < IntLEDT; i++) {
      Lled.setPixelColor(i, RD, GR, BL);
      Rled.setPixelColor(i, RD, GR, BL);
    }
    for (uint8_t i = 0; i < ExtLED; i++) {
      //      Tled = map(i, 0, ExtLED, IntLEDT, IntLEDT + ExtLED);
      //  Lled.setPixelColor(Tled, WheelVU(((i * 256 / EndLED) + rainbowCycles) & 255,0));
      //  Rled.setPixelColor(Tled, WheelVU(((i * 256 / EndLED) + rainbowCycles) & 255,1));
    }
    LedShow(0);

    rainbowCycles++;
    if (rainbowCycles >= 256 * 5) rainbowCycles = 0;
  }
}

#ifndef LEGACY_CODES
#ifdef ENTP1
void entp1() {
  int trsh = map(analogRead(ENTP1), 0, 1023, 0, 255);
  LanalyzeL.setTrsh(trsh);
  RanalyzeR.setTrsh(trsh);
  ManalyzeM.setTrsh(trsh);

  // unsigned long colour = map(analogRead(ENTP1), 0, 1023, 0, pow(2,32));
  // uint8_t rr = (uint8_t)(colour << 24);
  // uint8_t gg = (uint8_t)(colour << 16);
  // uint8_t bb = (uint8_t)(colour << 8);
  // RD = (byte) rr;
  // GR = (byte) gg;
  // BL = (byte) bb;
  // RD = (uint8_t)(colour & 0x00ff);
  // GR = (uint8_t)(colour & 0x0000ff);
  // BL = (uint8_t)(colour & 0x000000ff);
}
#endif
#ifdef ENTP2
void entp2() {
}
#endif
#endif
#endif

uint32_t WheelVU(byte WheelPos, int sd) {
  if (sd == 0) {
    if (WheelPos < 85) {
      return Lled.gamma32(Lled.Color(WheelPos * 3, 255 - WheelPos * 3, 0));
    } else if (WheelPos < 170) {
      WheelPos -= 85;
      return Lled.gamma32(Lled.Color(255 - WheelPos * 3, 0, WheelPos * 3));
    } else {
      WheelPos -= 170;
      return Lled.gamma32(Lled.Color(0, WheelPos * 3, 255 - WheelPos * 3));
    }
  } else if (sd == 1) {
    if (WheelPos < 85) {
      return Rled.gamma32(Rled.Color(WheelPos * 3, 255 - WheelPos * 3, 0));
    } else if (WheelPos < 170) {
      WheelPos -= 85;
      return Rled.gamma32(Rled.Color(255 - WheelPos * 3, 0, WheelPos * 3));
    } else {
      WheelPos -= 170;
      return Rled.gamma32(Rled.Color(0, WheelPos * 3, 255 - WheelPos * 3));
    }
  }
}

uint32_t Wheel(byte WheelPos, int sd, int ssin) {
  WheelPos = 255 - WheelPos;
  if (sd == 0) {
    if (sd == 0)
      if (WheelPos < 85) {
        return Lled.gamma32(Lled.Color(255 - WheelPos * 3, 0, WheelPos * 3, 0));
      }
    if (WheelPos < 170) {
      WheelPos -= 85;
      return Lled.gamma32(Lled.Color(0, WheelPos * 3, 255 - WheelPos * 3, 0));
    }
    WheelPos -= 170;
    return Lled.gamma32(Lled.Color(WheelPos * 3, 255 - WheelPos * 3, 0, 0));
  } else if (sd == 1) {
    if (WheelPos < 85) {
      return Rled.gamma32(Rled.Color(((255 - WheelPos * 3) * Rled.sine8(ssin)) / 255, 0, ((WheelPos * 3) * Rled.sine8(ssin)) / 255, 0));
    }
    if (WheelPos < 170) {
      WheelPos -= 85;
      return Rled.gamma32(Rled.Color(0, ((WheelPos * 3) * Rled.sine8(ssin)) / 255, ((255 - WheelPos * 3) * Rled.sine8(ssin)) / 255, 0));
    }
    WheelPos -= 170;
    return Rled.gamma32(Rled.Color(((WheelPos * 3) * Rled.sine8(ssin)) / 255, ((255 - WheelPos * 3) * Rled.sine8(ssin)) / 255, 0, 0));
  }
}

uint32_t Wheel2(byte WheelPos, int sd, int ssin = 128) {
  WheelPos = 255 - WheelPos;
  if (sd == 0) {
    if (sd == 0)
      if (WheelPos < 85) {
        return Lled.gamma32(Lled.Color(255 - WheelPos * 3, 0, WheelPos * 3, 0));
      }
    if (WheelPos < 170) {
      WheelPos -= 85;
      return Lled.gamma32(Lled.Color(0, WheelPos * 3, 255 - WheelPos * 3, 0));
    }
    WheelPos -= 170;
    return Lled.gamma32(Lled.Color(WheelPos * 3, 255 - WheelPos * 3, 0, 0));
  } else if (sd == 1) {
    if (WheelPos < 85) {
      return Rled.gamma32(Rled.Color(Rled.sine8(ssin) * (255 - WheelPos * 3), 0, Rled.sine8(ssin) * (WheelPos * 3), 0));
    }
    if (WheelPos < 170) {
      WheelPos -= 85;
      return Rled.gamma32(Rled.Color(0, Rled.sine8(ssin) * (WheelPos * 3), Rled.sine8(ssin) * (255 - WheelPos * 3), 0));
    }
    WheelPos -= 170;
    return Rled.gamma32(Rled.Color(Rled.sine8(ssin) * (WheelPos * 3), Rled.sine8(ssin) * (255 - WheelPos * 3), 0, 0));
  }
}

//for floating point map
float mapped(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#if !defined(ARDUINO_AVR_DIGISPARKPRO) && defined(EXTRA_MODES)

void MeteorShows(bool DoRainbow) {
  setInternals();
  if (DoRainbow) {
    LMeteor1.changeColor();
    LMeteor2.changeColor();
    RMeteor1.changeColor();
    RMeteor2.changeColor();
  } else {
    LMeteor1.changeColor(RD, GR, BL);
    LMeteor2.changeColor(RD, GR, BL);
    RMeteor1.changeColor(RD, GR, BL);
    RMeteor2.changeColor(RD, GR, BL);
  }

  LMeteor1.update();
  LMeteor2.update();

  RMeteor1.update();
  RMeteor2.update();

  if (LMeteor1.hasFinished()) LMeteor1.restart();
  if (LMeteor2.hasFinished()) LMeteor2.restart();

  if (RMeteor1.hasFinished()) RMeteor1.restart();
  if (RMeteor2.hasFinished()) RMeteor2.restart();
}

void FireShows() {
  setInternals();
  LFire1.update();
  LFire2.update();
  RFire1.update();
  RFire2.update();

  if (LFire1.hasFinished()) LFire1.restart();
  if (LFire2.hasFinished()) LFire2.restart();
  if (RFire1.hasFinished()) RFire1.restart();
  if (RFire2.hasFinished()) RFire2.restart();
}

#ifdef BOUNCING_BALL
void BallBounces(bool DoRainbow) {
  setInternals();
  Lball11.update();
  //  Lball12.update();
  Lball21.update();
  Rball11.update();
  Rball21.update();

  if (DoRainbow) {
    Lball11.changeColor();
    Lball21.changeColor();
    Rball11.changeColor();
    Rball21.changeColor();
  } else {
    Lball11.changeColor(RD, GR, BL);
    Lball21.changeColor(RD, GR, BL);
    Rball11.changeColor(RD, GR, BL);
    Rball21.changeColor(RD, GR, BL);
  }

  if (Lball11.hasFinished()) Lball11.restart();
  if (Lball21.hasFinished()) Lball21.restart();
  if (Rball11.hasFinished()) Rball11.restart();
  if (Rball21.hasFinished()) Rball21.restart();
}
#endif

void setInternals() {
  uint32_t colors = Lled.Color(RD, GR, BL);
  //  for (uint8_t i = 0; i < IntLEDT; i++)
  //  {
  Lled.fill(Lled.gamma32(colors), 0, IntLEDT);
  Rled.fill(Rled.gamma32(colors), 0, IntLEDT);
  //    Lled.setPixelColor(i, RD, GR, BL);
  //    Rled.setPixelColor(i, RD, GR, BL);
  //  }
}
#endif
void ColorWipe(byte red, byte green, byte blue, int LEDspeed) {
  for (uint8_t i = 0; i < EndLED; i++) {
    Lled.setPixelColor(i, Lled.gamma32(Lled.Color(red, green, blue)));
    Rled.setPixelColor(i, Rled.gamma32(Rled.Color(red, green, blue)));
    LedShow(0);
    delay(LEDspeed);
  }
}

void ColorTest(int LEDspeed) {
  ColorWipe(255, 0, 0, LEDspeed);
  ColorWipe(0, 0, 0, LEDspeed);
  ColorWipe(0, 255, 0, LEDspeed);
  ColorWipe(0, 0, 0, LEDspeed);
  ColorWipe(0, 0, 255, LEDspeed);
  ColorWipe(0, 0, 0, LEDspeed);
}

void debugging() {
}
