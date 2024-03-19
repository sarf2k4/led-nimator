// #define EXTRA_MODES
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

#ifndef ARDUINO_AVR_DIGISPARKPRO
// #include <npBouncingBall.h>
// #include <npColorWipe.h>
// #include <npFire.h>
// #include <npMeteor.h>
// #include <npBase.h>
// #include <npVirtualNeo.h>
// #include <npNeoPixel.h>
#endif
// npNeoPixel Lled = npNeoPixel(IntLEDT + ExtLED, Lpin, NEO_GRBW + NEO_KHZ800);
// npNeoPixel Rled = npNeoPixel(IntLEDT + ExtLED, Rpin, NEO_GRBW + NEO_KHZ800);

// npVirtualNeo VIntLled = npVirtualNeo(&Lled, 0, IntLEDT-1);
// npVirtualNeo VIntRled = npVirtualNeo(&Lled, 0, IntLEDT-1);

// npVirtualNeo VExtLled = npVirtualNeo(&Lled, 0, IntLEDT);

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
// #endif

#if !defined(ARDUINO_AVR_DIGISPARKPRO) && defined(EXTRA_MODES)
void MeteorShows(bool DoRainbow) {
  setInternals();
  if (DoRainbow) {
    LMeteor1.changeColor();
    LMeteor2.changeColor();
    RMeteor1.changeColor();
    RMeteor2.changeColor();
  } else {
    LMeteor1.changeColor(Colour[0], Colour[1], Colour[2]);
    LMeteor2.changeColor(Colour[0], Colour[1], Colour[2]);
    RMeteor1.changeColor(Colour[0], Colour[1], Colour[2]);
    RMeteor2.changeColor(Colour[0], Colour[1], Colour[2]);
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
    Lball11.changeColor(Colour[0], Colour[1], Colour[2]);
    Lball21.changeColor(Colour[0], Colour[1], Colour[2]);
    Rball11.changeColor(Colour[0], Colour[1], Colour[2]);
    Rball21.changeColor(Colour[0], Colour[1], Colour[2]);
  }

  if (Lball11.hasFinished()) Lball11.restart();
  if (Lball21.hasFinished()) Lball21.restart();
  if (Rball11.hasFinished()) Rball11.restart();
  if (Rball21.hasFinished()) Rball21.restart();
}
#endif

void setInternals() {
  uint32_t colors = Lled.Color(Colour[0], Colour[1], Colour[2]);
  //  for (uint8_t i = 0; i < IntLEDT; i++)
  //  {
  Lled.fill(Lled.gamma32(colors), 0, IntLEDT);
  Rled.fill(Rled.gamma32(colors), 0, IntLEDT);
  //    Lled.setPixelColor(i, Colour[0], Colour[1], Colour[2]);
  //    Rled.setPixelColor(i, Colour[0], Colour[1], Colour[2]);
  //  }
}
#endif