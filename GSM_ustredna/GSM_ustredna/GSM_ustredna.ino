/*== GSM Alarm - ustredna ==
 *==========================
 *
 * GA_ustredna.ino
 *
 * Hlavny riadiaci program GSM Alarmu urceny pre beh na Arduino Mega
 *
 * @author:   kamarat
 * @date:     august 2016
 * @version:  0.1beta
 *
 * kamarat (cleft) 2016
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
 * D 22 - RS
 * D 23 - E
 * D 24 - D4
 * D 25 - D5
 * D 26 - D6
 * D 27 - D7
 * D 28 - A
 *
 * GSM modul
 * D  6 - reset pin GSM
 * D  9 - softwarovy prepinac zapnutia/vypnutia GSM
 * D 16 - RX harwdare (pin 1)
 * D 17 - TX hardware (pin 0)
 */

/*== KNIZNICE A SUBORY ==
 *=======================
 */
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include <RH_ASK.h>         // kniznica RadioHead pre RF modul s ASK modulaciou
#include <SPI.h>            // nie je pouzita, ale je potrebna pre kompilaciu s RadioHead

#include <LiquidCrystal.h>  // kniznica pre pracu s LCD displejom

#include "Private.h"        // subor so sukromnymi udajmi
//#include "GSM_modul.h"      // vlastna kniznica GSM

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

// Nastavenie premennych GSM modulu
const uint8_t RST_GSM = 6;
const uint8_t POWER_GSM = 9;
char odpovedGSM[ 32 ] = {0};

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

      if( strncmp( ( char* ) buf, "Poplach", 7 ) == 0 ) {
        volajGSM();
      }

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

void vlozPinGSM( void )
{
  Serial2.print( "AT+CPIN=" );
  Serial2.println( PIN );
  delay( 100 );
  readGSM();
}

void volajGSM( void )
{
  //Serial.println( "AT" ); // Wake up GSM
  //readGSM();
  Serial2.print( "ATD" );      // sekvencia pre volanie
  Serial2.print( TEL_CISLO );
  Serial2.println( ";" );
  delay( 100 );
  readGSM();
  delay( 20000 );
  Serial2.println( "ATH" );    // ukoncenie volania
  readGSM();
}

void zapniGSM( void )
{
  digitalWrite( POWER_GSM,LOW );
  delay( 1000 );
  digitalWrite( POWER_GSM, HIGH );
  delay( 2000 ) ;
  digitalWrite( POWER_GSM, LOW );
  delay( 3000 );
}
void readGSM( void )
{
  if ( Serial2.available() ) {

    digitalWrite( LCD_A, HIGH );

    while( Serial2.available() > 0 ) {
      //char znak = Serial2.read();
      //Serial.print( znak );
      //lcd.print ( znak );
      lcd.write( Serial2.read() );
    }

    delay( 5000 );
    lcd.clear(); // vymazanie displeja a nastavenie kurzora do laveho horneho rohu
  }
}


/*
char * initGSM()
{
  char odpoved[ 100 ] = {0};
  uint8_t i = 0;

  //Serial3.begin( 19200 ); // inicializacia serioveho vystupu

  Serial2.print( "AT" );

  while ( Serial2.available() ) {
    delay( 10 );
    if ( Serial2.available() > 0 ) {
      char c = Serial2.read();
      odpoved[ i ] = c;
    }
  }

  return odpoved;
}*/

/*== INICIALIZACIA ==
 *===================
 */
void setup()
{
  // Inicializacia pinov
  pinMode( RF_VYSTUP, INPUT );
  pinMode( LED, OUTPUT );
  digitalWrite( LED, HIGH );
  pinMode( LCD_A, OUTPUT );
  digitalWrite( LCD_A, HIGH );
  //pinMode( RST_GSM, OUTPUT );
  //digitalWrite( RST_GSM, LOW );
  pinMode( POWER_GSM, OUTPUT );
  digitalWrite( POWER_GSM, LOW );

  // Inicializacia displeja
  lcd.begin( 20, 4);  // nastavenie poctu stlpcov a riadkov displeja
  lcd.setCursor( 0, 1);
  lcd.print( "Inicializacia !" );
  delay( 1000 );

  // Inicializacia GSM modulu
  zapniGSM();
  Serial2.begin( 19200 ); // inicializacia serioveho vystupu
  Serial2.println( "AT" );
  delay( 100 );
  readGSM();
  vlozPinGSM();
  readGSM();

  // Inicializacia RF modulu
  if ( !driverASK.init() ) {
    lcd.clear();
    lcd.print( "Inicializacia drivera ASK sa nepodarila!" );
  }

  #if DEBUG == 1
    Serial.begin( 19200 ); // inicializacia serioveho vystupu
    if ( !driverASK.init() )
           Serial.println( "Inicializacia drivera ASK sa nepodarila!" );
    Serial.println( "Inicializacia ukoncena." );
  #endif
  delay( 3000 );

  lcd.clear(); // vymazanie displeja a nastavenie kurzora do laveho horneho rohu
}

/*== HLAVNY PROGRAM ==
 *====================
 */
void loop()
{
  prijmiSpravu();



  lcd.clear();
  digitalWrite( LCD_A, LOW );
  delay( 2000 );
  #if DEBUG == 1
  #endif
}
