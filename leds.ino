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
void LDio() {
#if defined(LEGACY_CODES)
  buttonState = digitalRead(PD2);
  if ((buttonState != lastButtonState) && (buttonState == HIGH))
#endif
  {
    if (ledpwr) {
      ledpwr = 0;
#if !defined(ARDUINO_AVR_DIGISPARKPRO)
      wipeAnim(0, 0, 0, ledpwr, 0);
      if (RD >= 0 && GR >= 0 && BL >= 0) {
        Colour[0] = RD;
        Colour[1] = GR;
        Colour[2] = BL;
      }
#endif
      // ledOff();
      runningHue = 0;
    } else {
      ledpwr = 1;
#if !defined(ARDUINO_AVR_DIGISPARKPRO)
      if (AltMode) {
        wipeAnim(0, 0, 0, ledpwr, 5);
      } else {
        wipeAnim(Colour[0], Colour[1], Colour[2], 2, 5);
      }
      delay(250);
#endif
    }  //LED's off
       // ledOff();
  }
#ifdef LEGACY_CODES
  lastButtonState = buttonState;
#endif
#ifdef USE_EEPROM
  EEPROM.put(1, ledpwr);
#endif
}

void modeChange() {
  if (ledpwr) {
#if defined(LEGACY_CODES)
    buttonState = digitalRead(BPD2);
    if ((buttonState != lastButtonState) && (buttonState == HIGH))
#endif
    {
      if (ledMode == 20)
        EEPROM.get(0, ledMode);
      else {
        animRST();
        ledMode++;
        ledMode %= modes;
      }
      // if (ledMode > modes)
      //   ledMode = 1;

#if !defined(ARDUINO_AVR_DIGISPARKPRO)
      //LED's off
      wipeAnim(0, 0, 0, 2, 0);
      // delay(100);

      //Flash LED's
      for (int i = 1; i <= ledMode + 1; i++) {
        wipeAnim(0, 0, 0, 2, 5);
        delay(100);
        if (AltMode) {
          wipeAnim(0, 0, 0, 1, 5);
        } else {
          wipeAnim(Colour[0], Colour[1], Colour[2], 2, 5);
        }
        //   Rled.rainbow(0,1,255,Rled.sine8(j),true);
      }
      delay(100);
#else
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
#endif
    }
#ifdef LEGACY_CODES
    lastButtonState = buttonState;
#endif
#ifdef USE_EEPROM
    EEPROM.put(0, ledMode);
#endif
  }
}

void AltModes() {
  animRST();
  AltMode++;
  AltMode %= 2;
  wipeAnim(Colour[0], Colour[1], Colour[2], AltMode == 1 ? AltMode : 2, 10);
  setVol();
#ifdef USE_EEPROM
  EEPROM.put(2, AltMode);
#endif
}

void animRST() {
  fadeValue = 127;
  runningHue = 0;
}
void ledModes() {
  if (ledpwr) {
    switch (ledMode) {
      case 0:
        if (AltMode)
          rainbowCycle();
        else if (AltMode == 0)
          ledOn();
        break;
      case 1:
        Pulses(0);
        break;
      case 2:
        Pulses(1);
        break;
      case 3:
        blinkVU(1);
        break;
      case 4:
        rainbowVU(1);  //LEDVU mode
        // rainbowpvu();
        break;
      case 5:
        blinkVU(2);
        break;
      case 6:
        rainbowVU(2);  //MicVU mode
        break;
      case 20:
        // rainbowCycle();
        timedRandom();
        break;
#if !defined(ARDUINO_AVR_DIGISPARKPRO) && defined(EXTRA_MODES)
      case 8:
        FireShows();
        break;
      case 9:
        MeteorShows(METEOR_RAINBOW);
        break;
#ifdef BOUNCING_BALL
      case 10:
        BallBounces(BALL_RAINBOW);
        break;
#endif
#endif
    }
  } else if (ledpwr == 0) {
    ledOff();
  }
}

//only on colour
void ledOn() {
  Lled.fill(Lled.gamma32(Lled.Color(Colour[0], Colour[1], Colour[2])));
  Rled.fill(Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])));
  LedShow(0);
}

void Pulses(int Pmd) {
  int offset = Pmd ? 128 : 0;
#ifdef LEGACY_CODES
  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    if (Pmd == 0)  // All Pulses
    {
      if (fadeDirection == UP) {
        for (uint8_t i = 0; i < EndLED; i++) {
          Lled.setPixelColor(i, Lled.gamma32(Lled.Color(Colour[0] * fadeValue / maxPwm, Colour[1] * fadeValue / maxPwm, Colour[2] * fadeValue / maxPwm)));
          Rled.setPixelColor(i, Rled.gamma32(Rled.Color(Colour[0] * fadeValue / maxPwm, Colour[1] * fadeValue / maxPwm, Colour[2] * fadeValue / maxPwm)));
        }
        fadeValue++;
      } else if (fadeDirection == DOWN) {
        for (uint8_t i = 0; i < EndLED; i++) {
          Lled.setPixelColor(i, Lled.gamma32(Lled.Color(Colour[0] * fadeValue / maxPwm, Colour[1] * fadeValue / maxPwm, Colour[2] * fadeValue / maxPwm)));
          Rled.setPixelColor(i, Rled.gamma32(Rled.Color(Colour[0] * fadeValue / maxPwm, Colour[1] * fadeValue / maxPwm, Colour[2] * fadeValue / maxPwm)));
        }
        fadeValue--;
      }
    }
    if (Pmd == 1)  // Alternate Pulses
    {
      if (fadeDirection == UP) {
        for (uint8_t i = 0; i < EndLED; i++) {
          Lled.setPixelColor(i, Colour[0] * fadeValue / maxPwm, Colour[1] * fadeValue / maxPwm, Colour[2] * fadeValue / maxPwm);
        }
        for (uint8_t i = 0; i < EndLED; i++) {
          Rled.setPixelColor(i, Colour[0] * map(fadeValue, 0, 255, 255, 0) / maxPwm, Colour[1] * map(fadeValue, 0, 255, 255, 0) / maxPwm, Colour[2] * map(fadeValue, 0, 255, 255, 0) / maxPwm);
        }
        fadeValue++;
      } else if (fadeDirection == DOWN) {
        for (uint8_t i = 0; i < EndLED; i++) {
          Lled.setPixelColor(i, Colour[0] * fadeValue / maxPwm, Colour[1] * fadeValue / maxPwm, Colour[2] * fadeValue / maxPwm);
        }
        for (uint8_t i = 0; i < EndLED; i++) {
          Rled.setPixelColor(i, Colour[0] * map(fadeValue, 0, 255, 255, 0) / maxPwm, Colour[1] * map(fadeValue, 0, 255, 255, 0) / maxPwm, Colour[2] * map(fadeValue, 0, 255, 255, 0) / maxPwm);
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
    if (AltMode) {
      for (int i = 0; i < (ExtLED / 2); i++) {
        int pixelHue = runningHue + (i * 65536L / (ExtLED / 2));
        // Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Lled.gamma32(Lled.ColorHSV(pixelHue)));
        // Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Lled.gamma32(Lled.ColorHSV(pixelHue, 255, Lled.sine8(fadeValue))));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDT), Lled.gamma32(Lled.ColorHSV(pixelHue, 255, Lled.sine8(fadeValue))));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), Lled.gamma32(Lled.ColorHSV(pixelHue, 255, Lled.sine8(fadeValue))));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED - 1, IntLEDT + (ExtLED / 2) - 1), Lled.gamma32(Lled.ColorHSV(pixelHue, 255, Lled.sine8(fadeValue))));


        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Rled.gamma32(Rled.ColorHSV(pixelHue)));
        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Rled.gamma32(Rled.ColorHSV(pixelHue, 255, Rled.sine8(fadeValue))));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDT), Rled.gamma32(Rled.ColorHSV(pixelHue, 255, Rled.sine8(fadeValue + offset))));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), Rled.gamma32(Rled.ColorHSV(pixelHue, 255, Rled.sine8(fadeValue + offset))));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED - 1, IntLEDT + (ExtLED / 2) - 1), Rled.gamma32(Rled.ColorHSV(pixelHue, 255, Rled.sine8(fadeValue + offset))));
      }
    } else {
      Lled.fill(Lled.gamma32(Lled.Color(Colour[0] * Lled.sine8(fadeValue) / maxPwm, Colour[1] * Lled.sine8(fadeValue) / maxPwm, Colour[2] * Lled.sine8(fadeValue) / maxPwm)));
      Rled.fill(Rled.gamma32(Rled.Color(Colour[0] * Rled.sine8(fadeValue + offset) / maxPwm, Colour[1] * Rled.sine8(fadeValue + offset) / maxPwm, Colour[2] * Rled.sine8(fadeValue + offset) / maxPwm)));
    }
    fadeValue++;
    LedShow(0);
    runningHue += 768;
    if (runningHue >= 5 * 65536)
      runningHue = 0;
  }
  fadeValue %= 256;
#endif
}

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

  Lvu = map(L, 0, 255, 255, 0);
  Rvu = map(R, 0, 255, 255, 0);

  for (uint8_t i = 0; i < EndLED; i++) {
    Lled.setPixelColor(i, Lled.gamma32(Lled.Color(Colour[0] * Rvu / maxPwm, Colour[1] * Rvu / maxPwm, Colour[2] * Rvu / maxPwm)));
    Rled.setPixelColor(i, Rled.gamma32(Rled.Color(Colour[0] * Rvu / maxPwm, Colour[1] * Rvu / maxPwm, Colour[2] * Rvu / maxPwm)));
  }
  LedShow(0);
}

#else
{
  unsigned int L, R;

  if (LanalyzeL.tick() || RanalyzeR.tick() || ManalyzeM.tick()) {
    // if (Md == 1) {
    L = Md == 2 ? ManalyzeM.getVol() : LanalyzeL.getVol();
    R = Md == 2 ? ManalyzeM.getVol() : RanalyzeR.getVol();
    // } else if (Md == 2) {
    //   L = ManalyzeM.getVol();
    //   R = ManalyzeM.getVol();
    // }

    // if (AltMode == 0) {
    Lvu = map(L, 0, 100, 0, maxPwm);
    Rvu = map(R, 0, 100, 0, maxPwm);
    // } else if (AltMode == 1) {
    //   Lvu = map(L, 0, 100, maxPwm, 0);
    //   Rvu = map(R, 0, 100, maxPwm, 0);
    // }
    if (AltMode) {
      for (int i = 0; i < (ExtLED / 2); i++) {
        int pixelHue = runningHue + (i * 65536L / (ExtLED / 2));
        // Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Lled.gamma32(Lled.ColorHSV(pixelHue)));
        // Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Lled.gamma32(Lled.ColorHSV(pixelHue, 255, Lled.sine8(fadeValue))));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDT), Lled.gamma32(Lled.ColorHSV(pixelHue, 255, Lvu)));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), Lled.gamma32(Lled.ColorHSV(pixelHue, 255, Lvu)));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED - 1, IntLEDT + (ExtLED / 2) - 1), Lled.gamma32(Lled.ColorHSV(pixelHue, 255, Lvu)));


        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Rled.gamma32(Rled.ColorHSV(pixelHue)));
        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Rled.gamma32(Rled.ColorHSV(pixelHue, 255, Rled.sine8(fadeValue))));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDT), Rled.gamma32(Rled.ColorHSV(pixelHue, 255, Rvu)));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), Rled.gamma32(Rled.ColorHSV(pixelHue, 255, Rvu)));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED - 1, IntLEDT + (ExtLED / 2) - 1), Rled.gamma32(Rled.ColorHSV(pixelHue, 255, Rvu)));
      }
    } else if (AltMode == 0) {
      Lled.fill(Lled.gamma32(Lled.Color(Colour[0] * Rvu / maxPwm, Colour[1] * Rvu / maxPwm, Colour[2] * Rvu / maxPwm)));
      Rled.fill(Rled.gamma32(Rled.Color(Colour[0] * Rvu / maxPwm, Colour[1] * Rvu / maxPwm, Colour[2] * Rvu / maxPwm)));
    }
    LedShow(0);
    runningHue += 256;
    if (runningHue >= 5 * 65536)
      runningHue = 0;
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
  int minValue = 10, maxValue = 700, maxSensitivity = 4 * maxPwm;

  sensitivityValue = 512;
  sensitivityValue = map(sensitivityValue, 0, 1023, 0, maxPwm);
  sensitivityFactor = ((float)sensitivityValue / maxPwm * (float)maxSensitivity / maxPwm);
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
    L = map((Md == 1 || Md == 3) ? LanalyzeL.getVol() : ManalyzeM.getVol(), 0, 100, ((Md == 1 || Md == 2) ? IntLEDT : IntLEDF), 0);
    LB = map((Md == 3) ? LanalyzeL.getVol() : ManalyzeM.getVol(), 0, 100, IntLEDB, 0);
    EL = map((Md == 1 || Md == 3) ? LanalyzeL.getVol() : ManalyzeM.getVol(), 0, 100, ExtLED / 2, 0);

    R = map((Md == 1 || Md == 3) ? RanalyzeR.getVol() : ManalyzeM.getVol(), 0, 100, ((Md == 1 || Md == 2) ? IntLEDT : IntLEDF), 0);
    RB = map((Md == 3) ? RanalyzeR.getVol() : ManalyzeM.getVol(), 0, 100, IntLEDB, 0);
    ER = map((Md == 1 || Md == 3) ? RanalyzeR.getVol() : ManalyzeM.getVol(), 0, 100, ExtLED / 2, 0);

    // if (Md == 1) {
    // L = map(LanalyzeL.getVol(), 0, 100, IntLEDT, 0);
    // EL = map(LanalyzeL.getVol(), 0, 100, (ExtLED / 2), 0);
    // R = map(RanalyzeR.getVol(), 0, 100, IntLEDT, 0);
    // ER = map(RanalyzeR.getVol(), 0, 100, (ExtLED / 2), 0);
    //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    // } else if (Md == 2) {
    // L = map(ManalyzeM.getVol(), 0, 100, IntLEDT, 0);
    // EL = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
    // R = map(ManalyzeM.getVol(), 0, 100, IntLEDT, 0);
    // ER = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
    //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    // } else if (Md == 3) {  //Parallel VU
    // L = map(LanalyzeL.getVol(), 0, 100, IntLEDF, 0);
    // LB = map(LanalyzeL.getVol(), 0, 100, IntLEDB, 0);
    // EL = map(LanalyzeL.getVol(), 0, 100, (ExtLED / 2), 0);
    // R = map(RanalyzeR.getVol(), 0, 100, IntLEDF, 0);
    // RB = map(RanalyzeR.getVol(), 0, 100, IntLEDB, 0);
    // ER = map(RanalyzeR.getVol(), 0, 100, (ExtLED / 2), 0);
    //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    // } else if (Md == 4) {
    // L = map(ManalyzeM.getVol(), 0, 100, IntLEDF, 0);
    // LB = map(ManalyzeM.getVol(), 0, 100, IntLEDB, 0);
    // EL = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
    // R = map(ManalyzeM.getVol(), 0, 100, IntLEDF, 0);
    // RB = map(ManalyzeM.getVol(), 0, 100, IntLEDB, 0);
    // ER = map(ManalyzeM.getVol(), 0, 100, (ExtLED / 2), 0);
    //  c = fscale(MUSINPUT_FLOOR, MUSINPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
    // }
    //Fill the strip with rainbow gradient
    if (AltMode == 0) {
      Lled.fill(Lled.gamma32(Lled.Color(Colour[0], Colour[1], Colour[2])));
      Rled.fill(Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])));
    } else if (AltMode == 1) {
      for (uint8_t i = 0; i < EndLED; i++) {
        if (i < IntLEDT) {
          if (Md == 3 && i < IntLEDF) {
            Lled.setPixelColor(i, WheelVU(map(i, IntLEDF - 1, 0, 30, 150), 0));
            Rled.setPixelColor(i, WheelVU(map(i, IntLEDF - 1, 0, 30, 150), 1));
          }
        }
        if (Md == 3 && i > IntLEDF && i < IntLEDT) {
          Lled.setPixelColor(i, WheelVU(map(i, IntLEDF, IntLEDT - 1, 30, 150), 0));
          Rled.setPixelColor(i, WheelVU(map(i, IntLEDF, IntLEDT - 1, 30, 150), 1));
        }
        if (Md != 3) {
          Lled.setPixelColor(i, Lled.gamma32(Lled.ColorHSV(map(i, 0, IntLEDT - 1, 0, finalHue))));
          Rled.setPixelColor(i, Rled.gamma32(Rled.ColorHSV(map(i, 0, IntLEDT - 1, 0, finalHue))));
        }
        if ((i >= IntLEDT) && (i < IntLEDT + (ExtLED / 2))) {
          Lled.setPixelColor(i, Lled.gamma32(Lled.ColorHSV(map(i, IntLEDT, (IntLEDT + (ExtLED / 2)) - 1, 0, finalHue))));
          Rled.setPixelColor(i, Rled.gamma32(Rled.ColorHSV(map(i, IntLEDT, (IntLEDT + (ExtLED / 2)) - 1, 0, finalHue))));
        }
        if ((i >= IntLEDT + (ExtLED / 2)) && (i < EndLED)) {
          Lled.setPixelColor(i, Lled.gamma32(Lled.ColorHSV(map(i, EndLED - 1, EndLED - (ExtLED / 2), 0, finalHue))));
          Rled.setPixelColor(i, Rled.gamma32(Rled.ColorHSV(map(i, EndLED - 1, EndLED - (ExtLED / 2), 0, finalHue))));
        }
        //         int pixelHue = runningHue + (i * 65536L / EndLED);
        //         if (i < IntLEDT) {
        //           if (Md == 3 && i < IntLEDF) {
        //             Lled.setPixelColor(i, WheelVU(map(i, IntLEDF - 1, 0, 30, 150), 0));
        //             Rled.setPixelColor(i, WheelVU(map(i, IntLEDF - 1, 0, 30, 150), 1));
        //           }
        //         }
        //         if (Md == 3 && i > IntLEDF && i < IntLEDT) {
        //           Lled.setPixelColor(i, WheelVU(map(i, IntLEDF, IntLEDT - 1, 30, 150), 0));
        //           Rled.setPixelColor(i, WheelVU(map(i, IntLEDF, IntLEDT - 1, 30, 150), 1));
        //         }
        //         if (Md != 3) {
        //           Lled.setPixelColor(i, Lled.gamma32(Lled.ColorHSV(pixelHue)));
        //           Rled.setPixelColor(i, Rled.gamma32(Rled.ColorHSV(pixelHue)));
        //         }
        //         if ((i >= IntLEDT) && (i < IntLEDT + (ExtLED / 2))) {
        //           Lled.setPixelColor(i, Lled.gamma32(Lled.ColorHSV(pixelHue)));
        //           Rled.setPixelColor(i, Rled.gamma32(Rled.ColorHSV(pixelHue)));
        //         }
        //         if ((i >= IntLEDT + (ExtLED / 2)) && (i < EndLED)) {
        //           Lled.setPixelColor(i, Lled.gamma32(Lled.ColorHSV(pixelHue)));
        //           Rled.setPixelColor(i, Rled.gamma32(Rled.ColorHSV(pixelHue)));
        //         }
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
        drawLine(IntLEDT - 1, IntLEDT - L, Lled.Color(0, 0, 0), 0);
      }
      if (EL <= ExtLED / 2) {  // Fill partial column with off pixels
        drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - EL, Lled.Color(0, 0, 0), 0);
        drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2) - 1) + EL, Lled.Color(0, 0, 0), 0);
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
        drawLine(IntLEDT - 1, IntLEDT - R, Rled.Color(0, 0, 0), 1);
      }
      if (ER <= ExtLED / 2) {  // Fill partial column with off pixels
        drawLine(IntLEDT + (ExtLED / 2), (IntLEDT + (ExtLED / 2)) - ER, Rled.Color(0, 0, 0), 1);
        drawLine(EndLED - (ExtLED / 2), (EndLED - (ExtLED / 2) - 1) + ER, Rled.Color(0, 0, 0), 1);
      }
      // Set the peak dot to match the rainbow gradient
      Ly = IntLEDT - Lpeak;
      ELy = (ExtLED / 2) - ELpeak;
      Ry = IntLEDT - Rpeak;
      ERy = (ExtLED / 2) - ERpeak;

      // if (AltMode == 0) {
      //   Lled.setPixelColor(Ly + vu_offset, Lled.gamma32(Lled.Color(Colour[0], Colour[1], Colour[2])));
      //   Lled.setPixelColor(IntLEDT + ELy + vu_offset, Lled.gamma32(Lled.Color(Colour[0], Colour[1], Colour[2])));
      //   Lled.setPixelColor((EndLED - 1) - ELy, Lled.gamma32(Lled.Color(Colour[0], Colour[1], Colour[2])));
      // } else if (AltMode == 1) {
      Lled.setPixelColor(Ly + vu_offset, ((Ly == 0) ? Lled.Color(0, 0, 0) : ((AltMode == 1) ? Lled.gamma32(Lled.ColorHSV(map(Ly, 0, IntLEDT - 1, 0, finalHue))) : Lled.gamma32(Lled.Color(Colour[0], Colour[1], Colour[2])))));
      Lled.setPixelColor(IntLEDT + ELy + vu_offset, ((ELy == 0) ? Lled.Color(0, 0, 0) : ((AltMode == 1) ? Lled.gamma32(Lled.ColorHSV(map(ELy, 0, (ExtLED / 2), 0, finalHue))) : Lled.gamma32(Lled.Color(Colour[0], Colour[1], Colour[2])))));
      Lled.setPixelColor((EndLED - 1) - ELy, ((ELy == 0) ? Lled.Color(0, 0, 0) : ((AltMode == 1) ? Lled.gamma32(Lled.ColorHSV(map(ELy, 0, (ExtLED / 2), 0, finalHue))) : Lled.gamma32(Lled.Color(Colour[0], Colour[1], Colour[2])))));
      // }

      // if (Ly == 0)
      //   Lled.setPixelColor(Ly - 0, 0, 0, 0);
      // if (ELy == 0) {
      //   Lled.setPixelColor(IntLEDT + ELy, 0, 0, 0);
      //   Lled.setPixelColor((EndLED - 1) - ELy, 0, 0, 0);
      // }

      // if (AltMode == 0) {
      //   Rled.setPixelColor(Ry + vu_offset, Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])));
      //   Rled.setPixelColor(IntLEDT + ERy + vu_offset, Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])));
      //   Rled.setPixelColor((EndLED - 1) - ERy, Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])));
      // } else if (AltMode == 1) {
      Rled.setPixelColor(Ry + vu_offset, ((Ry == 0) ? Rled.Color(0, 0, 0) : ((AltMode == 1) ? Rled.gamma32(Rled.ColorHSV(map(Ry, 0, IntLEDT - 1, 0, finalHue))) : Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])))));
      Rled.setPixelColor(IntLEDT + ERy + vu_offset, ((ERy == 0) ? Rled.Color(0, 0, 0) : ((AltMode == 1) ? Rled.gamma32(Rled.ColorHSV(map(ERy, 0, (ExtLED / 2), 0, finalHue))) : Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])))));
      Rled.setPixelColor((EndLED - 1) - ERy, ((ERy == 0) ? Rled.Color(0, 0, 0) : ((AltMode == 1) ? Rled.gamma32(Rled.ColorHSV(map(ERy, 0, (ExtLED / 2), 0, finalHue))) : Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])))));
      // }
      // if (Ry == 0)
      //   Rled.setPixelColor(Ry - 0, 0, 0, 0);
      // if (ERy == 0) {
      //   Rled.setPixelColor(IntLEDT + ERy, 0, 0, 0);
      //   Rled.setPixelColor((EndLED - 1) - ERy, 0, 0, 0);
      // }
      LedShow(0);

      runningHue += 256;
      if (runningHue >= 5 * 65536)
        runningHue = 0;

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
        drawLine(0, R, Lled.Color(0, 0, 0), 1);
      }
      if (LB <= IntLEDB) {  // Fill partial column with off pixels
        drawLine(IntLEDT, IntLEDT - LB, Lled.Color(0, 0, 0), 1);
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
      if (R <= IntLEDF) {  // Fill partial column with off pixels
        drawLine(-1, R, Rled.Color(0, 0, 0), 1);
      }
      if (RB <= IntLEDB) {  // Fill partial column with off pixels
        drawLine(IntLEDT, IntLEDT - RB, Rled.Color(0, 0, 0), 1);
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

      Lled.setPixelColor(Ly + vu_offset, WheelVU(map(Ly, 0, IntLEDF - 1, 30, 150), 0));
      Lled.setPixelColor(IntLEDF + LBy + vu_offset, WheelVU(map(LBy, 0, IntLEDB - 1, 30, 150), 0));
      Lled.setPixelColor(IntLEDT + ELy + vu_offset, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));
      Lled.setPixelColor(EndLED - ELy, WheelVU(map(ELy, 0, (ExtLED / 2), 30, 150), 0));

      if (Ly == 0)
        Lled.setPixelColor(Ry + vu_offset, 0, 0, 0);
      if (LBy == 0)
        Lled.setPixelColor(IntLEDF + RBy + vu_offset, 0, 0, 0);
      if (ELy == 0) {  //turns off the external if there's no Peak
        Lled.setPixelColor(IntLEDT + ELy - 1, 0, 0, 0);
        Lled.setPixelColor(EndLED - ELy, 0, 0, 0);
      }

      if (AltMode == 0) {
        Rled.setPixelColor(IntLEDF - Ry + vu_offset, Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])));
        Rled.setPixelColor(IntLEDF + RBy + vu_offset, Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])));
        Rled.setPixelColor(IntLEDT + ERy + vu_offset, Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])));
        Rled.setPixelColor(EndLED - ERy, Rled.gamma32(Rled.Color(Colour[0], Colour[1], Colour[2])));
      } else if (AltMode == 1) {
        Rled.setPixelColor(IntLEDF - Ry - 1, WheelVU(map(Ry, 0, IntLEDF, 30, 150), 1));
        Rled.setPixelColor(IntLEDF + RBy + vu_offset, WheelVU(map(RBy, 0, IntLEDB - 1, 30, 150), 1));
        Rled.setPixelColor(IntLEDT + ERy + vu_offset, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));
        Rled.setPixelColor(EndLED - ERy, WheelVU(map(ERy, 0, (ExtLED / 2), 30, 150), 1));
      }
      if (Ry == 0)
        Rled.setPixelColor(Ry, 0, 0, 0);
      if (RBy == 0)
        Rled.setPixelColor(IntLEDF + RBy + vu_offset, 0, 0, 0);
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
  if ((unsigned long)(millis() - prevMillis) >= rainbowInt) {
    prevMillis = millis();
    // for (uint8_t i = 0; i < EndLED; i++) {
    //   if (i < IntLEDT) {
    //     Lled.setPixelColor(i, (WheelVU(((i * 256 / IntLEDT) + rainbowCycles) & 255, 0)));
    //     Rled.setPixelColor(i, (WheelVU(((i * 256 / IntLEDT) + rainbowCycles) & 255, 1)));
    //   }
    //   if (i >= IntLEDT) {
    //     Lled.setPixelColor(i, (WheelVU(((i * 256 / ExtLED) + rainbowCycles) & 255, 0)));
    //     Rled.setPixelColor(i, (WheelVU(((i * 256 / ExtLED) + rainbowCycles) & 255, 1)));
    // }
    // for (uint8_t i = 0; i < (ExtLED / 2); i++) {
    //   // if (i < IntLEDT)
    //   {
    //     Lled.setPixelColor(map(i, 0, ExtLED / 2, 0, IntLEDF - 1), (WheelVU(((i * 256 / (ExtLED / 2)) + rainbowCycles) & 255, 0)));
    //     Lled.setPixelColor(map(i, 0, ExtLED / 2, IntLEDT - 1, IntLEDF), (WheelVU(((i * 256 / (ExtLED / 2)) + rainbowCycles) & 255, 0)));

    //     Rled.setPixelColor(map(i, 0, ExtLED / 2, IntLEDF - 1, -1), (WheelVU(((i * 256 / (ExtLED / 2)) + rainbowCycles) & 255, 1)));
    //     Rled.setPixelColor(map(i, 0, ExtLED / 2, IntLEDF, IntLEDT), (WheelVU(((i * 256 / (ExtLED / 2)) + rainbowCycles) & 255, 1)));
    //   }
    //   // if (i >= IntLEDT)
    //   {
    //     Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), (WheelVU(((i * 256 / (ExtLED / 2)) + rainbowCycles) & 255, 0)));
    //     Lled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED - 1, IntLEDT + (ExtLED / 2) - 1), (WheelVU(((i * 256 / (ExtLED / 2)) + rainbowCycles) & 255, 0)));

    //     Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), (WheelVU(((i * 256 / (ExtLED / 2)) + rainbowCycles) & 255, 1)));
    //     Rled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED - 1, IntLEDT + (ExtLED / 2) - 1), (WheelVU(((i * 256 / (ExtLED / 2)) + rainbowCycles) & 255, 1)));
    //   }
    // }
    // for (long firstPixelHue = 0; firstPixelHue < 5 * 65536; firstPixelHue += 256) {
    for (int i = 0; i < (ExtLED / 2); i++) {
      int pixelHue = runningHue + (i * 65536L / (ExtLED / 2));
      // Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Lled.gamma32(Lled.ColorHSV(pixelHue)));
      // Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Lled.gamma32(Lled.ColorHSV(pixelHue)));
      Lled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDT), Lled.gamma32(Lled.ColorHSV(pixelHue)));
      Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), Lled.gamma32(Lled.ColorHSV(pixelHue)));
      Lled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED - 1, IntLEDT + (ExtLED / 2) - 1), Lled.gamma32(Lled.ColorHSV(pixelHue)));


      // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Rled.gamma32(Rled.ColorHSV(pixelHue)));
      // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Rled.gamma32(Rled.ColorHSV(pixelHue)));
      Rled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDT), Rled.gamma32(Rled.ColorHSV(pixelHue)));
      Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), Rled.gamma32(Rled.ColorHSV(pixelHue)));
      Rled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED - 1, IntLEDT + (ExtLED / 2) - 1), Rled.gamma32(Rled.ColorHSV(pixelHue)));
    }
    // }
    LedShow(0);
    runningHue += 256;
    if (runningHue >= 5 * 65536)
      runningHue = 0;

    // rainbowCycles++;
    // if (rainbowCycles >= 256 * 5)
    //   rainbowCycles = 0;
  }
}

void ledOff() {
  Lled.clear();
  Rled.clear();
  LedShow(0);
}

void LedShow(int ledtype) {
  if (ledtype == 0) {
    Lled.show();
    Rled.show();
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

void ColorWipe(byte red, byte green, byte blue, int LEDspeed) {
  // int i = 0;
  // while ((i < EndLED)) {
  //   Lled.setPixelColor(i, Lled.gamma32(Lled.Color(red, green, blue)));
  //   Rled.setPixelColor(i, Rled.gamma32(Rled.Color(red, green, blue)));
  //   i++;
  // }
  for (uint8_t i = 0; i < EndLED; i++) {
    // if ((unsigned long)(millis() - prevMillis) >= LEDspeed) {
    //   prevMillis = millis();
    Lled.setPixelColor(i, Lled.gamma32(Lled.Color(red, green, blue)));
    Rled.setPixelColor(i, Rled.gamma32(Rled.Color(red, green, blue)));
    LedShow(0);
    delay(LEDspeed);
  }
  // }
}

void ColorTest(int LEDspeed) {
  ColorWipe(255, 0, 0, LEDspeed);
  ColorWipe(0, 0, 0, LEDspeed);
  ColorWipe(0, 255, 0, LEDspeed);
  ColorWipe(0, 0, 0, LEDspeed);
  ColorWipe(0, 0, 255, LEDspeed);
  ColorWipe(0, 0, 0, LEDspeed);
}

#if !defined(ARDUINO_AVR_DIGISPARKPRO)
/*
tp = 0: colour goes down
tp = 1: rainbow goes up
tp = 2: colour goes up
tp = 3: rainbow goes down
*/
void wipeAnim(byte R, byte G, byte B, int tp, int dly) {
  // if ((unsigned long)(millis() - prevMillis) >= dly) {
  //   prevMillis = millis();
  int pixelHue = 0;
  for (uint8_t i = 0; i < (ExtLED / 2); i++) {
    switch (tp) {
      case 0:  // colour goes down
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDF), Lled.gamma32(Lled.Color(R, G, B)));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT - 1, IntLEDF - 1), Lled.gamma32(Lled.Color(R, G, B)));

        Rled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDF), Rled.gamma32(Rled.Color(R, G, B)));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT - 1, IntLEDF - 1), Rled.gamma32(Rled.Color(R, G, B)));

        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT + (ExtLED / 2) - 1, IntLEDT - 1), Lled.gamma32(Lled.Color(R, G, B)));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT + (ExtLED / 2), EndLED), Lled.gamma32(Lled.Color(R, G, B)));

        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT + (ExtLED / 2) - 1, IntLEDT - 1), Rled.gamma32(Rled.Color(R, G, B)));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT + (ExtLED / 2), EndLED), Rled.gamma32(Rled.Color(R, G, B)));
        break;
      case 1:  // rainbow goes up
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Lled.gamma32(Lled.ColorHSV(map(i, 0, (ExtLED / 2), endHueF, startHueF))));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Lled.gamma32(Lled.ColorHSV(map(i, 0, (ExtLED / 2), startHueB, endHueB))));

        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), 0, finalHue))));
        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), 0, finalHue))));

        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDT), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), 0, finalHue))));

        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF-1, -1), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), 30247, 2978))));//BEST
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), endHueF, startHueF))));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), startHueB, endHueB))));
        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), 23125, 65536)))); //BEST
        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF-1, IntLEDT), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), 18005, 65536)))); // closest
        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), 0, 65536))));

        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), Lled.gamma32(Lled.ColorHSV(map(i, 0, (ExtLED / 2), 0, finalHue))));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED - 1, IntLEDT + (ExtLED / 2) - 1), Lled.gamma32(Lled.ColorHSV(map(i, 0, (ExtLED / 2), 0, finalHue))));

        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), 0, finalHue))));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED - 1, IntLEDT + (ExtLED / 2) - 1), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), 0, finalHue))));
        break;
      case 2:  // Colour goes up
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Lled.gamma32(Lled.Color(R, G, B)));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Lled.gamma32(Lled.Color(R, G, B)));

        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF - 1, -1), Rled.gamma32(Rled.Color(R, G, B)));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDF, IntLEDT), Rled.gamma32(Rled.Color(R, G, B)));

        Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), Lled.gamma32(Lled.Color(R, G, B)));
        Lled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED, IntLEDT + (ExtLED / 2)) - 1, Lled.gamma32(Lled.Color(R, G, B)));

        Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT, IntLEDT + (ExtLED / 2)), Rled.gamma32(Rled.Color(R, G, B)));
        Rled.setPixelColor(map(i, 0, (ExtLED / 2), EndLED, IntLEDT + (ExtLED / 2)) - 1, Rled.gamma32(Rled.Color(R, G, B)));
        break;
      case 3:  // rainbow goes down

        // Rled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDT), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), 0, 65536))));

        //   Lled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDF), Lled.gamma32(Lled.ColorHSV(map(i, 0, (ExtLED / 2), finalHue, 0))));
        //   Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT - 1, IntLEDF), Lled.gamma32(Lled.ColorHSV(map(i, 0, (ExtLED / 2), finalHue, 0))));

        //   Rled.setPixelColor(map(i, 0, (ExtLED / 2), 0, IntLEDF), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), finalHue, 0))));
        //   Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT - 1, IntLEDF-1), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), finalHue, 0))));

        //   Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT + (ExtLED / 2) - 1, IntLEDT - 1), Lled.gamma32(Lled.ColorHSV(map(i, 0, (ExtLED / 2), finalHue, 0))));
        //   Lled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT + (ExtLED / 2), EndLED), Lled.gamma32(Lled.ColorHSV(map(i, 0, (ExtLED / 2), finalHue, 0))));
        //   //
        //   Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT + (ExtLED / 2), IntLEDT), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), finalHue, 0))));
        //   Rled.setPixelColor(map(i, 0, (ExtLED / 2), IntLEDT + (ExtLED / 2) - 1, EndLED - 1), Rled.gamma32(Rled.ColorHSV(map(i, 0, (ExtLED / 2), finalHue, 0))));
        break;
    }
    delay(dly);
    LedShow(0);
  }
  // }
}

void timedRandom() {
  if ((unsigned long)(millis() - prevMillis) >= Random_Period) {
    prevMillis = millis();
    RandColour();
    int Rn = random(3);
    int Sp = random(0, 20);
    // rgbTrans();
    wipeAnim(Colour[0], Colour[1], Colour[2], Rn, Sp);
    // delay(500);
    // wipeAnim(Colour[0], Colour[1], Colour[2], 1, 20);
    // delay(500);
    // wipeAnim(0, 0, 0, 3, 20);
  }
  // LedShow(0);
}

void rgbTrans() {
  byte red, blue, green;
  int dred, dblue, dgreen, mxd, mxp, dly;
  randomSeed(analogRead(A6) + analogRead(A7));
  red = random(0, 255);
  green = random(0, 255);
  blue = random(0, 255);

  dred = int(red - Colour[0]);
  dgreen = int(green - Colour[1]);
  dblue = int(blue - Colour[2]);

  mxd = (abs(dred) >= abs(dgreen) && abs(dred) >= abs(dblue)) ? dred : (abs(dgreen) >= abs(dred) && abs(dgreen) >= abs(dblue)) ? dgreen
                                                                                                                               : dblue;
  // mxd =  (abs(dred) >= abs(dgreen) && abs(dred) >= abs(dblue)) ? dred : dgreen;
  mxp = abs(mxd);
  dly = 1000 / mxp;

  for (int i = 0; i <= mxp; i++) {
    int mred = map(i, 0, mxp, 0, dred);
    int mgreen = map(i, 0, mxp, 0, dgreen);
    int mblue = map(i, 0, mxp, 0, dblue);

    Lled.fill(Lled.gamma32(Lled.Color(Colour[0] + mred, Colour[1] + mgreen, Colour[2] + mblue)));
    Rled.fill(Rled.gamma32(Rled.Color(Colour[0] + mred, Colour[1] + mgreen, Colour[2] + mblue)));
    LedShow(0);
    // delay(dly);
  }
  Colour[0] = red;
  Colour[1] = green;
  Colour[2] = blue;
  // delay(500);
}

EasyColor::HSVRGB HSVConverter;
void hsvTrans() {
  hsv conhsv1, conhsv2;
  rgb conrgb1, conrgb2;
  long hue1, hue2;
  byte sat1, sat2, val1, val2;
  int target, targetdly;
  int dsatp, dvalp;
  int dhue, dsat, dval;

  conrgb1.r = Colour[0];
  conrgb1.g = Colour[1];
  conrgb1.b = Colour[2];

  randomSeed(analogRead(A6) + analogRead(A7));
  conrgb2.r = random(0, 255);
  conrgb2.g = random(0, 255);
  conrgb2.b = random(0, 255);

  conhsv1 = HSVConverter.RGBtoHSV(conrgb1, conhsv1);
  conhsv2 = HSVConverter.RGBtoHSV(conrgb2, conhsv2);

  hue1 = map(conhsv1.h, 0, 360, 0, 65536L);
  sat1 = map(conhsv1.s, 0, 100, 0, 255);
  val1 = map(conhsv1.v, 0, 100, 0, 255);

  hue2 = map(conhsv2.h, 0, 360, 0, 65536L);
  sat2 = map(conhsv2.s, 0, 100, 0, 255);
  val2 = map(conhsv2.v, 0, 100, 0, 255);

  // double hh, hhr;
  long mhh, mhhr;

  dhue = int(conhsv2.h - conhsv1.h);
  // hhr = dhue >= 0 ? 360 : -360;
  // mhhr = dhue >= 0 ? 65536L : -65536L;
  mhh = map(dhue, 0, dhue >= 0 ? 360 : -360, 0, dhue >= 0 ? 65536L : -65536);

  // npx.fill(npx.gamma32(npx.ColorHSV(hue1, sat1, val1)));
  dhue = hue2 - hue1;
  dsat = sat2 - sat1;
  dval = val2 - val1;

  dsatp = abs(dsat);
  dvalp = abs(dval);
  target = max(dsatp, dvalp);
  targetdly = 1000 / target;

  for (int i = 0; i <= target; i++) {
    int msat, mval;
    long mhue;
    mhue = map(i, 0, target, 0, mhh);
    msat = map(i, 0, target, 0, dsat);
    mval = map(i, 0, target, 0, dval);
    Rled.fill(Rled.gamma32(Rled.ColorHSV(hue1 + mhue, sat1 + msat, val1 + mval)));

    LedShow(0);
    // delay(targetdly);
  }
  Colour[0] = conrgb2.r;
  Colour[1] = conrgb2.g;
  Colour[2] = conrgb2.b;
  // delay(500);
}
#endif