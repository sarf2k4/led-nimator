// #define AT328P_TO_DIGISPARK_PRO  //Atmega328 to digispark pro pinout

#if defined(ARDUINO_AVR_DIGISPARKPRO)  // Using Digispark Pro with Attiny167
#define leftCh A12                     //L audio input [pin 12]
#define rightCh A11                    //R audio input [pin 11]
#define micCh A10                      //Mic input Analog [pin 10]
#define MDButton 8                     //Button mode [pin 8]

#define Lpin 0  //left LED data [pin 1]
#define Rpin 1  //right LED data [pin 0]


#elif defined(AT328P_TO_DIGISPARK_PRO) && defined(ARDUINO_AVR_ATmega328)
#define leftCh A1   //L audio input
#define rightCh A0  //R audio input
#define micCh A2    //Mic input Analog
#define Senses A4   //Volume sensitivity
#define Bright A5   //Brightness trimpot

#define MDButton 2      //Button mode pin as an interrupt
#define ENsenseTrim 4   //jumper to ground, enable volume sensitityvy adjustment [pin4]
#define ENbrightTrim 5  //Jumper to ground, enable brightness adjustment trimpots [pin 5][1:NC,0:Connected]
#define Lpin 8          //left LED data pin
#define Rpin 9          //right LED data pin

#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_ATmega328) // Using ATMEGA328P Internal Oscilator
#define leftCh A0                                                   //L audio input
#define rightCh A1                                                  //R audio input
#define micCh A2                                                    //Mic input Analog
#define TP1 A4                                                   //Volume sensitivity
#define TP2 A5                                                   //Brightness trimpot

#define MDButton 2      //Button mode pin as an interrupt
#define LEDbtn 3        //Soft LED Button
#define ENTP1 4   //jumper to ground, enable volume sensitityvy adjustment [pin4]
#define ENTP2 5  //Jumper to ground, enable brightness adjustment trimpots [pin 5][1:NC,0:Connected]
#define Lpin 9          //left LED data pin
#define Rpin 8          //right LED data pin
//#define LTpin 8 //left ext LED data pin
//#define RTpin 7 //right ext LED data pin

#endif
