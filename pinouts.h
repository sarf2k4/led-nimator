// #define AT328P_TO_DIGISPARK_PRO  //Atmega328 to digispark pro pinout

#if defined(ARDUINO_AVR_DIGISPARKPRO)  // Using Digispark Pro with Attiny167
#define leftCh A12                     //L audio input [pin 12]
#define rightCh A11                    //R audio input [pin 11]
#define micCh A10                      //Mic input Analog [pin 10]
#define BPD2 8                         //Button mode [pin 8]

#define Lpin 0  //left LED data [pin 1]
#define Rpin 1  //right LED data [pin 0]


#elif defined(AT328P_TO_DIGISPARK_PRO) && defined(ARDUINO_AVR_ATmega328)
#define leftCh A0   //L audio input
#define rightCh A1  //R audio input
#define micCh A2    //Mic input Analog
// #define TP1 A4   //TP1 Trimpot
// #define TP2 A5   //TP2 Trimpot

#define BPD2 2  //Button mode pin as an interrupt

#define Lpin 8  //left LED data pin
#define Rpin 9  //right LED data pin

#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_ATmega328)  // Using ATMEGA328P Internal Oscilator [L:E2,H:D7,E:FD,LB:FF]
#define leftCh A0                                                 //L audio input
#define rightCh A1                                                //R audio input
#define micCh A2                                                  //Mic input Analog
// #define TP1 A4   //TP1 Trimpot
// #define TP2 A5   //TP2 Trimpot

#define BPD2 2  //BPD2 button (INT0)
#define BPD3 3   //BPD3 button (INT1)

#define Lpin 9  //left LED data pin
#define Rpin 8  //right LED data pin

#endif
