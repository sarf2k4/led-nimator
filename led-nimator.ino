;
#include "configs.h"
#include "pinouts.h"
#include "cols.h"
#include "vars.h"

#ifdef USE_EEPROM
#include <EEPROM.h>
#endif

void setup() {

#ifdef USE_EEPROM
  // eepromCheck();
  EEPROM.get(0, ledMode);
  EEPROM.get(1, ledpwr);
  EEPROM.get(2, AltMode);
#endif
// Serial.begin(9600);

// if (F_CPU == 16000000)
// clock_prescale_set(clock_div_16);

//onebutton
#ifdef BPD2
#if defined(USE_INTERRUPTS)
  attachInterrupt(digitalPinToInterrupt(BPD2), bpd2checkTicks, CHANGE);
#endif
  bpd2.attachClick(bpd2click);
  bpd2.attachDoubleClick(bpd2double);
  bpd2.attachMultiClick(bpd2multi);
  bpd2.attachLongPressStart(bpd2longstart);
  bpd2.setPressMs(2000);
#endif

#ifdef BPD3
#if defined(USE_INTERRUPTS)
  attachInterrupt(digitalPinToInterrupt(BPD3), bpd3checkTicks, CHANGE);
#endif
  bpd3.attachClick(bpd3click);
  bpd3.attachDoubleClick(bpd3double);
  bpd3.attachMultiClick(bpd3multi);
  bpd3.attachLongPressStart(bpd3longstart);
  bpd3.setPressMs(2000);
#endif
  //onebutton

#ifndef LEGACY_CODES
  //Volanalyzer
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
  ManalyzeM.setTrsh(20);
  ManalyzeM.setAmpliK(31);
  ManalyzeM.setWindow(10);
  ManalyzeM.setVolK(22);
  ManalyzeM.setVolMin(0);
  ManalyzeM.setVolMax(100);
  ManalyzeM.setPulseMax(90);
// Volanalyzer
#endif

  if (RD < 0 && GR < 0 && BL < 0)
    RandColour();

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
  wipeAnim(Colour[0], Colour[1], Colour[2], AltMode == 1 ? AltMode : 2, 10);
  if (!ledpwr)
    wipeAnim(Colour[0], Colour[1], Colour[2], ledpwr, 5);
#endif
#if defined(EXTRA_MODES) && !defined(ARDUINO_AVR_DIGISPARKPRO)
  LMeteor1.changeColor(Colour[0], Colour[1], Colour[2]);
  LMeteor2.changeColor(Colour[0], Colour[1], Colour[2]);
#endif

  setVol();
}

//main loop
void loop() {

  bpd2.tick();
#ifdef BPD3
  bpd3.tick();
#endif

#ifdef TP1
  tp1();
#endif

#ifdef TP2
  tp2();
#endif

  ledModes();

  //Serial.println(ledMode);

  //  Serial.print(Colour[0]);
  //  Serial.print(", ");
  //  Serial.print(Colour[1]);
  //  Serial.print(", ");
  //  Serial.print(Colour[2]);
  //  Serial.println();
}
//main loop

#if defined(USE_INTERRUPTS) && defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_ATmega328)
void bpd2checkTicks() {
  bpd2.tick();
}
#endif

void bpd2click() {
  modeChange();
  setVol();
}

void bpd2double() {
  LDio();
}

void bpd2multi() {
  int clicks = bpd2.getNumberClicks();
  MultiClicks(clicks);
}

void bpd2longstart() {
  longClick();
}

#ifdef BPD3
#ifdef USE_INTERRUPTS
void bpd3checkTicks() {
  bpd3.tick();
}
#endif

void bpd3click() {
  modeChange();
  setVol();
}

void bpd3double() {
  LDio();
}

void bpd3multi() {
  int clicks = bpd3.getNumberClicks();
  MultiClicks(clicks);
}

void bpd3longstart() {
  longClick();
}
#endif

void longClick() {
  ledMode = 20;
}

void MultiClicks(int clicks) {
  if (ledpwr) {
    switch (clicks) {
      case 3:  // Random colour
        if (AltMode == 0) {
          // rgbTrans();
          RandColour();
          wipeAnim(Colour[0], Colour[1], Colour[2], 2, 10);
        } else if (AltMode == 1) {
        }
        break;
      case 4:  // Alternate modes
        AltModes();
        break;
      case 5:  // Colour wipe test
        ColorTest(10);
        break;
      case 10:  // Seizure Inducer
        ledMode = 29;
        break;
    }
  }
}

void setVol() {
  switch (ledMode) {
    case 3:
    case 5:
      LanalyzeL.setVolK(blVolK);
      RanalyzeR.setVolK(blVolK);
      ManalyzeM.setVolK(blVolK);
      break;
    case 4:
    case 6:
      PEAK_HANG = PeakHangs[AltMode];
      PEAK_FALL = PeakFalls[AltMode];
      LanalyzeL.setVolK(vuVolK[AltMode]);
      RanalyzeR.setVolK(vuVolK[AltMode]);
      ManalyzeM.setVolK(vuVolK[AltMode]);
      break;
  }
}

void RandColour() {
  // #ifndef FIXED_COLOUR
  randomSeed(analogRead(A7) + analogRead(A6));
  Colour[0] = random(0, 255);
  Colour[1] = random(0, 255);
  Colour[2] = random(0, 255);
  // #endif
}

#ifdef TP1
void tp1() {
  int trsh = map(analogRead(TP1), 0, 1023, 0, 255);
  LanalyzeL.setTrsh(trsh);
  RanalyzeR.setTrsh(trsh);
  ManalyzeM.setTrsh(trsh);

  // unsigned long colour = map(analogRead(TP1), 0, 1023, 0, pow(2,32));
  // uint8_t rr = (uint8_t)(colour << 24);
  // uint8_t gg = (uint8_t)(colour << 16);
  // uint8_t bb = (uint8_t)(colour << 8);
  // Colour[0] = (byte) rr;
  // Colour[1] = (byte) gg;
  // Colour[2] = (byte) bb;
  // Colour[0] = (uint8_t)(colour & 0x00ff);
  // Colour[1] = (uint8_t)(colour & 0x0000ff);
  // Colour[2] = (uint8_t)(colour & 0x000000ff);
}
#endif

#ifdef TP2
void tp2() {
}
#endif

#ifdef USE_EEPROM
void eepromCheck() {
  int ep[eprUsed];
  for (int i = 0; i < eprUsed; i++) {
    EEPROM.get(0, ep[i]);
    if (isnan(ep[i])) {
      if (i == 0 || i == 1)
        EEPROM.put(i, 1);
      EEPROM.put(i, 0);
    }
  }
}
#endif