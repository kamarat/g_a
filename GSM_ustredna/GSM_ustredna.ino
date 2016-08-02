/*== GSM Alarm - ustredna ==
 *==========================
 *
 * GA_ustredna.ino
 *
 * Hlavny riadiaci program GSM Alarmu urceny pre beh na Arduino Mega
 *
 * @author: mr.nobody
 * @date:   august 2016
 *
 * mr.nobody (cleft) 2016
 */

/*== ZAPOJENIE PINOV ==
 *=====================
 *
 * ARDUINO --- PERIFERIE
 * A  0 -
 *
 * D  2 - 
 * D  3 - 
 * D 11 - vystup RF modulu
 * D 13 - kontrolna LED
 * 
 * LCD displej
 * D 21 - RS
 * D 22 - E
 * D 23 - D4
 * D 25 - D5
 * D 26 - D6
 * D 27 - D7
 */

/*== KNIZNICE A SUBORY ==
 *=======================
 */
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include <RH_ASK.h>         // kniznica RadioHead pre RF modul s ASK modulaciou
#include <SPI.h>            // nie je pouzita, ale je potrebna pre kompilaciu s RadioHead

#include <LiquidCrystal.h>  // kniznica pre pracu s LCD displejom

/*== GLOBALNE PREMENNE ==
 *=======================
 * premenne popisujuce zapojene piny
 * premenne popisujuce nastavenie Arduina
 */

#define DEBUG 1               // definicia odladovania a vypisov
#define RF_ASK 1              // definicia po uziteho RF modulu
//#define __AVR_ATmega2560__ 1  // definicia Megy pre ucely merania napatia

/*== Deklaracia konstant ==
 */
//const uint8_t 

// analogove piny

// digitalne piny
//const uint8_t PIR = 3;  // vystup PIR pripojeny na port D3
const uint8_t LED = 13;

/*== Deklaracia premennych ==
 */ 

// Nastavenie premennych RF modulu prijimaca
#ifdef RF_ASK
  /* Constructor.
   * At present only one instance of RH_ASK per sketch is supported.
   *   param[in] speed The desired bit rate in bits per second
   *   param[in] rxPin The pin that is used to get data from the receiver
   *   param[in] txPin The pin that is used to send data to the transmitter
   *   param[in] pttPin The pin that is connected to the transmitter controller. It will be set HIGH to enable the transmitter (unless pttInverted is true)
   *   param[in] pttInverted true if you desire the pttin to be inverted so that LOW wil enable the transmitter
   * RH_ASK(uint16_t speed = 2000, uint8_t rxPin = 11, uint8_t txPin = 12, uint8_t pttPin = 10, bool pttInverted = false);
   */    
  RH_ASK driverASK;
  //const uint8_t RF_VSTUP = 12;  // vstup vysielaca pripojeny na pin D12
  const uint8_t RF_VYSTUP = 11;  // vystup prijimaca pripojeny na pin D12 (pin DATA vedla GND)
#endif

// Nastavenie premennych LCD modulu
const uint8_t LCD_RS = 22;
const uint8_t LCD_E = 23;
const uint8_t LCD_D4 = 24;
const uint8_t LCD_D5 = 25;
const uint8_t LCD_D6 = 26;
const uint8_t LCD_D7 = 27;
const uint8_t LCD_A = 28;   // podsvietenie displeja - odber cca 6,1 mA
LiquidCrystal lcd( LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7 ); 

/*== DEFINICIA FUNKCII ==
 *=======================
 */
  
void prijmiSpravu( void )
{
  digitalWrite( LED, HIGH );

/*  #if DEBUG == 1
    // Vypis na seriovu konzolu s naslednym cakanim na ukoncenie serioveho prenosu
    Serial.println( "Prijimam spravu" );
    Serial.flush();
  #endif
*/
  // Prijatie spravy o poplachu
  #ifdef RF_ASK
    uint8_t buf[10] = {0};
    uint8_t buflen = sizeof( buf );
    if ( driverASK.recv( buf, &buflen )) // Non-blocking
    {
      int i;
      // Message with a good checksum received, dump it.
      Serial.print( "Message: " );
      Serial.println(( char* ) buf );
      
      digitalWrite( LCD_A, HIGH );
      lcd.setCursor( 0, 2 );
      lcd.print(( char * ) buf );
      delay( 5000 );
    }
  #endif

/*  #if DEBUG == 1
    Serial.println( "Koncim" );
    Serial.flush();
  #endif
*/  
  digitalWrite( LED, LOW );
}

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
  napatie = 1125300L / napatie;               // Vypocet Vcc (v mV); 1125300 = 1.1*1023*1000
  return napatie;                             // Vcc v mV
}

/*== INICIALIZACIA ==
 *===================
 */
void setup()
{
  /* Casto akekelkovek zariadenie pripojene na vystupne piny moze spotrebuvat male mnozstvo energie aj ked nie je pouzivane.
   * Vstupne piny, ktore "plavaju" (nepouzivaju pullup alebo pulldown odpor), budu tiez spotrebuvat energiu.
   * Preto je vhodne vsetky piny okrem TX a RX nastavit na vstupny mod a aktivovat pullup odpory, pokial na piny nie je nic pripojene.
   */
  /*DDRD &= B00000011;       // nastavenie pinov 2 - 7 na vstupne, pinom 0 a 1 (RX a TX) ponechane nastavenia
  DDRB = B00000000;        // nastavenie pinov 8 - 13 na vstupne
  PORTD |= B11111100;      // aktivacia pullup na pinoch 2 - 7, piny 0 a 1 (RX a TX) ponechane bez
  PORTB |= B11111111;      // aktivacia pullup na pinoch 8 - 13
  */
  
  // Inicializacia pinov
  //pinMode( PIR, INPUT );
  pinMode( RF_VYSTUP, INPUT );
  pinMode( LED, OUTPUT );
  digitalWrite( LED, HIGH );
  pinMode( LCD_A, OUTPUT );
  digitalWrite( LCD_A, HIGH );

  lcd.begin( 20, 4);  // nastavenie poctu stlpcov a riadkov displeja
  lcd.setCursor( 0, 1);
  lcd.print( "Inicializacia !" );
  
  #if DEBUG == 1
    Serial.begin( 9600 ); // inicializacia serioveho vystupu
    if ( !driverASK.init() )
           Serial.println( "Inicializacia drivera ASK sa nepodarila!" );
    Serial.println( "Inicializacia ukoncena." );
  #endif
  delay( 5000 );

  lcd.clear();      // vymazanie displeja a nastavenie kurzora do laveho horneho rohu
}

/*== HLAVNY PROGRAM ==
 *====================
 */
void loop()
{  
  prijmiSpravu();

  lcd.clear();
  digitalWrite( LCD_A, LOW );
  #if DEBUG == 1
  #endif
}

