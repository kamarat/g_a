/*== GSM Alarm - senzor ==
 *========================
 *
 * GA_senzor.ino
 *
 * Program nacita stav PIR a odosle prostrednictvom radioveho signalu
 * na GSM ústrednu
 *
 * @author: kamarat
 * @date:   jul 2016
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
 * D  3 - vystup signalu z PIR (prerusenie)
 * D 12 - vstup RF modulu
 * D 13 - kontrolna LED
 *
 */

/*== KNIZNICE A SUBORY ==
 *=======================
 */
#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include <RH_ASK.h> // kniznica RadioHead pre RF modul s ASK modulaciou
#include <SPI.h>    // kniznica nie je pouzita, ale je potrebna pre kompilaciu s RadioHead

#include "utility.h"

/*== GLOBALNE PREMENNE ==
 *=======================
 * premenne popisujuce zapojene piny
 * premenne popisujuce nastavenie Arduina
 */
#define DEBUG 1   // definicia odladovania a vypisov
#define RF_ASK    // definica pouziteho RF modulu

/*== Deklaracia konstant ==
 */
// digitalne piny
const uint8_t PIR = 3;  // vystup PIR pripojeny na port D3
const uint8_t LED = 13;

// analogove piny

/*== Deklaracia premennych ==
 */
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
  const uint8_t RF_VSTUP = 12;  // vstup vysielaca pripojeny na pin D12
#endif

/*== DEKLARACIA FUNKCII ==
 *=======================
 */
void zaspiTeraz( void );
void posliPoplach( void );
void odosliSpravu( const char* sprava );

/*== INICIALIZACIA ==
 *===================
 */
void setup()
{
  /* Casto akekelkovek zariadenie pripojene na vystupne piny moze spotrebuvat male mnozstvo energie aj ked nie je pouzivane.
   * Vstupne piny, ktore "plavaju" (nepouzivaju pullup alebo pulldown odpor), budu tiez spotrebuvat energiu.
   * Preto je vhodne vsetky piny okrem TX a RX nastavit na vstupny mod a aktivovat pullup odpory, pokial na piny nie je nic pripojene.
   */
  DDRD &= B00000011;       // nastavenie pinov 2 - 7 na vstupne, pinom 0 a 1 (RX a TX) ponechane nastavenia
  DDRB = B00000000;        // nastavenie pinov 8 - 13 na vstupne
  PORTD |= B11111100;      // aktivacia pullup na pinoch 2 - 7, piny 0 a 1 (RX a TX) ponechane bez
  PORTB |= B11111111;      // aktivacia pullup na pinoch 8 - 13

  // Inicializacia pinov
  pinMode( PIR, INPUT );
  pinMode( RF_VSTUP, OUTPUT );
  pinMode( LED, OUTPUT );
  digitalWrite( LED, HIGH );

  #if DEBUG == 1
    Serial.begin( 9600 ); // inicializacia serioveho vystupu
  #endif

  if ( !driverASK.init() ) {
    #if DEBUG == 1
      Serial.println( "Inicializacia drivera ASK sa nepodarila!" );
    #endif

    while ( true ) {
      digitalWrite( LED, LOW );
      delay( 1000 );
      digitalWrite( LED, HIGH );
      delay( 1000 );
    }
  }

  #if DEBUG == 1
    Serial.println( "Inicializacia ukoncena." );
  #endif

  delay( 1000 );
}

/*== HLAVNY PROGRAM ==
 *====================
 */
void loop()
{
  // Arduino zostane prebudene sekundu, potom zaspi.
  // LED po zaspati zhasne a zasvieti pri prebudeni.
  //delay( 1000) ;
  zaspiTeraz();

  #if DEBUG == 1
  #endif
}

/*== DEFINICIA FUNKCII ==
 *=======================
 */

void zaspiTeraz( void )
{
  #if DEBUG == 1
    Serial.println( "Zaspavam." );
    Serial.flush();
  #endif

  /* Mod prerusenia definuje, kedy ma byt prerusenie spustene. Su preddefinovane 4 konstanty:
  *  LOW to trigger the interrupt whenever the pin is low,
  *  CHANGE to trigger the interrupt whenever the pin changes value
  *  RISING to trigger when the pin goes from low to high,
  *  FALLING for when the pin goes from high to low.
  */
  attachInterrupt( digitalPinToInterrupt( PIR ), posliPoplach, RISING );  // nastavenie prerusenia
  delay( 100 );

  /* Volba spiaceho rezimu
  * Existuje 5 modov pouzitelnych na standardnych 8-bitovych AVR:
  *    SLEEP_MODE_IDLE       – least power savings
  *    SLEEP_MODE_ADC
  *    SLEEP_MODE_PWR_SAVE
  *    SLEEP_MODE_STANDBY
  *    SLEEP_MODE_PWR_DOWN   – most power savings
  */
  set_sleep_mode( SLEEP_MODE_PWR_DOWN );
  //cli();                // zakazanie globalnych preruseni
  noInterrupts();         // zakazanie globalnych preruseni
  sleep_enable();         // nastavenie spiaceho bitu - sleep enable (SE)
  sleep_bod_disable();    // vypnutie the Brown Out Detector (BOD) pred uspatim
  //sei();                // povolenie preruseni
  interrupts();           // povolenie preruseni

  digitalWrite( LED, LOW );

  sleep_cpu();            // uvedenie zariadenia do rezimu spanku
  sleep_disable();        // po prebudeni program pokracuje v tomto bode
  //sei();
  //interrupts();
  //detachInterrupt( digitalPinToInterrupt( PIR ));
}

void posliPoplach( void )
{
  detachInterrupt( digitalPinToInterrupt( PIR ));
  digitalWrite( LED, HIGH );

  const char * SPRAVA = "Poplach";
  odosliSpravu( SPRAVA );
}

void odosliSpravu( const char* sprava )
{
  #if DEBUG == 1
    // Vypis na seriovu konzolu s naslednym cakanim na ukoncenie serioveho prenosu
    Serial.print( "Posielam spravu = " );
    Serial.println( sprava );
    Serial.print( "Napajacie napatie (mV) = " );
    Serial.println( merajVcc() );
    Serial.flush();
  #endif

  // Odoslanie spravy o poplachu
  #ifdef RF_ASK
    driverASK.send(( uint8_t* ) sprava, strlen( sprava ));
    //driverASK.waitPacketSent();
    delay(200);
  #endif

  #if DEBUG == 1
    Serial.println( "Koncim" );
    Serial.flush();
  #endif
}
