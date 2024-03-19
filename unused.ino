
#if !defined(ARDUINO_AVR_DIGISPARKPRO) && defined(LEGACY_CODES)

//only extras pulsing
void ledPulse3() {

  if ((unsigned long)(millis() - previousFadeMillis) >= fadeInterval) {
    previousFadeMillis = millis();
    for (uint8_t i = 0; i < IntLEDT; i++) {
      Lled.setPixelColor(i, Colour[0], Colour[1], Colour[2]);
      Rled.setPixelColor(i, Colour[0], Colour[1], Colour[2]);
    }
    if (fadeDirection == UP) {
      for (uint8_t i = 0; i < ExtLED; i++) {
        Lled.setPixelColor(map(i, 0, ExtLED, IntLEDT, IntLEDT + ExtLED), Colour[0] * fadeValue / 255, Colour[1] * fadeValue / 255, Colour[2] * fadeValue / 255);
        Rled.setPixelColor(map(i, 0, ExtLED, IntLEDT, IntLEDT + ExtLED), Colour[0] * fadeValue / 255, Colour[1] * fadeValue / 255, Colour[2] * fadeValue / 255);
      }
      fadeValue++;
    } else if (fadeDirection == DOWN) {
      for (uint8_t i = 0; i < ExtLED; i++) {
        Lled.setPixelColor(map(i, 0, ExtLED, IntLEDT, IntLEDT + ExtLED), Colour[0] * fadeValue / 255, Colour[1] * fadeValue / 255, Colour[2] * fadeValue / 255);
        Rled.setPixelColor(map(i, 0, ExtLED, IntLEDT, IntLEDT + ExtLED), Colour[0] * fadeValue / 255, Colour[1] * fadeValue / 255, Colour[2] * fadeValue / 255);
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

/*
uint32_t Wheel(byte WheelPos, int sd, int ssin) {
  WheelPos = 255 - WheelPos;
  if (sd == 0) {
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

#if !defined(ARDUINO_AVR_DIGISPARKPRO)

//only extras running rainbow
void rainbowCycle2() {
  if (millis() - previousFadeMillis >= fadeInterval) {
    previousFadeMillis = millis();

    for (uint8_t i = 0; i < IntLEDT; i++) {
      Lled.setPixelColor(i, Colour[0], Colour[1], Colour[2]);
      Rled.setPixelColor(i, Colour[0], Colour[1], Colour[2]);
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
#endif
*/