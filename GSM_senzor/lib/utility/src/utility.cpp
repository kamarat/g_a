#include "utility.h"
#include "Arduino.h"

const float KOREKCIA_VCC = 5180.0 / 5170.0; // napatie zdroja delene nameranym napatim
const uint32_t REF_VCC = 1125300 * KOREKCIA_VCC;     // vypocet referencneho napatia v mV - 1125300 = 1.1*1023*1000

uint16_t merajVcc( void )
{
  /* Nastavenie registrov pre meranie 1,1V proti referencnemu napatiu Vcc
   *   REFS[1:0]     Volba referencneho napatia
   *      01           AVcc s externym kondenzatorom na AREF pine
   *   MUX[3:0]      Volba vstupneho kanalu
   *     1110          1,1V (Vbg)
   */
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV( REFS0 ) | _BV( MUX3 ) | _BV( MUX2 ) | _BV( MUX1 ); // nastavenie pre ATmega 328p
  #endif

  delay( 2 );                                 // pauza pre ustalenie Vref
  ADCSRA |= _BV( ADSC );                      // spustenie konverzie
  while ( bit_is_set( ADCSRA,ADSC ));         // meranie
  uint8_t lowADC  = ADCL;                     // prvy citany register ADCL - uzamknutie registra ADCH
  uint8_t highADC = ADCH;                     // po precitani ADCH odomknutie oboch registrov
  uint16_t napatie = ( highADC<<8 ) | lowADC; // vytvorenie 16-bitoveho cisla z dvoch 8-bitovych
  napatie = REF_VCC / napatie;               // Vypocet Vcc (v mV)
  return napatie;                             // Vcc v mV
}
