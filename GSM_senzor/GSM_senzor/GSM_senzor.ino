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
#include <avr/wdt.h>

#include <RH_ASK.h> // kniznica RadioHead pre RF modul s ASK modulaciou
#include <SPI.h>    // kniznica nie je pouzita, ale je potrebna pre kompilaciu s RadioHead

#include "utility.h"

/*== GLOBALNE PREMENNE ==
 *=======================
 * premenne popisujuce zapojene piny
 * premenne popisujuce nastavenie Arduina
 */
#define DEBUG 0   // definicia odladovania a vypisov
#define RF_ASK    // definicia pouziteho RF modulu
#define WATCHDOG

/*== Deklaracia konstant ==
 */
// digitalne piny
const uint8_t PIR = 3;  // vystup PIR pripojeny na port D3
const uint8_t LED = 13; // LED umiestnena na PCB

// analogove piny

// globalne konstanty
const float KOREKCIA_VCC = 5180.0 / 5000.0; // napatie zdroja delene nameranym napatim

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

/* Nastavenie premennych pre watchdog
 * priznak_WDT - ukladanie stavu, ci prisiel impulzu z watchdog timeru,
 *            - hodnota 1 simuluje impulz po zapnuti, aby sme necakali
 * pocitadlo_impulzov_WDT - ukladanie impulzov
 * IMPULZOV_WDT_PRE_SPUSTENIE - nastavenie pozadovaneho poctu impulzov
 *                            - v pripade nastavenia 2 je jeden impulz cca 8 s / 8192 ms
 *
 * Klucove slovo volatile hovori kompilatoru ako ma zaobchadzat s premennou,
 * aby ju nacital z pamate RAM a nie z pamatoveho regstra. Vzhladom k spankovemu
 * rezimu budu tieto hodnoty urcite presne.
 */
#ifdef WATCHDOG
  const uint16_t INTERVAL_MERANIA_VCC = 60000;  // cas udavany v ms
  volatile uint8_t priznak_WDT = 1;
  volatile uint16_t pocitadlo_impulzov_WDT = 0;
  const volatile uint8_t IMPULZOV_WDT_PRE_SPUSTENIE = INTERVAL_MERANIA_VCC / 8192;
#endif

volatile uint8_t priznak_IRQ = 1;              // ukladanie stavu, ci nastalo prerusenie

/*== DEKLARACIA FUNKCII ==
 *=======================
 */
void zaspiTeraz( void );
void nastaloPrerusenie( void );
void posliPoplach( void );
void posliVcc ( void );
void odosliSpravu( const char* sprava );

/*== INICIALIZACIA ==
 *===================
 */
void setup()
{
  #ifdef WATCHDOG
    // Nastavenie WDT
    MCUSR &= ~( 1<<WDRF ); // vycistenie priznaku reset WDT
    /* Pre pripad zmeny WDE alebo delica je potrebne nastavit WDCE,
     * to umozni update pre 4 hodinove cykly.
     */
    WDTCSR |= ( 1<<WDCE ) | ( 1<<WDE );
    WDTCSR = 1<<WDP0 | 1<<WDP3; // nastavenie hodnoty delica WDT - 8 sekund
    WDTCSR |= _BV( WDIE );      // povolenie WD prerusenia (ziadny reset)
  #endif

  /* Doplnkove nastavenie pre usporny rezim.
   * Casto akekelkovek zariadenie pripojene na vystupne piny moze spotrebuvat male mnozstvo energie aj ked nie je pouzivane.
   * Vstupne piny, ktore "plavaju" (nepouzivaju pullup alebo pulldown odpor), budu tiez spotrebuvat energiu.
   * Preto je vhodne vsetky piny okrem TX a RX nastavit na vstupny mod a aktivovat pullup odpory, pokial na piny nie je nic pripojene.
   */
  DDRD &= B00000011;       // nastavenie pinov 2 - 7 na vstupne, pinom 0 a 1 (RX a TX) ponechane nastavenia
  DDRB = B00000000;        // nastavenie pinov 8 - 13 na vstupne
  PORTD |= B11111100;      // aktivacia pullup na pinoch 2 - 7, piny 0 a 1 (RX a TX) ponechane bez
  PORTB |= B11111111;      // aktivacia pullup na pinoch 8 - 13

  // Vypnutie ADC prevodnika
  ADCSRA &= ( uint8_t )~( 1 << ADEN ) ;
  PRR |= ( uint8_t )( 1 << PRADC );

  // Vypnutie SPI
  PRR |= ( uint8_t )( 1 << PRSPI );

  // Vypnutie TWI
  PRR |= ( uint8_t )( 1 << PRTWI );

  // Inicializacia pinov
  pinMode( PIR, INPUT_PULLUP );
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

  delay( 15000 );
}

/*== HLAVNY PROGRAM ==
 *====================
 */
void loop()
{
  if ( priznak_IRQ == 1 ) {
    priznak_IRQ = 0;
    if ( pocitadlo_impulzov_WDT) {
      posliPoplach();
    }
  }

  #ifdef WATCHDOG
    if ( priznak_WDT == 1 ) {
      priznak_WDT = 0;
      pocitadlo_impulzov_WDT++;
      #if DEBUG == 1
        Serial.print( "Pocitadlo WDT = " );
        Serial.println( pocitadlo_impulzov_WDT );
      #endif

      if ( pocitadlo_impulzov_WDT >= IMPULZOV_WDT_PRE_SPUSTENIE ) {
        posliVcc();
        pocitadlo_impulzov_WDT = 0;
      }
    }
  #endif

  #if DEBUG == 1
    Serial.println( "Zaspavam." );
    Serial.flush();
  #endif
  zaspiTeraz();
  #if DEBUG == 1
    Serial.println( "Zobudil som sa." );
    Serial.flush();
  #endif
}

/*== DEFINICIA FUNKCII ==
 *=======================
 */
#ifdef WATCHDOG
ISR( WDT_vect )
{
  if ( priznak_WDT == 0 )
  {
    priznak_WDT = 1;
  }
}
#endif

void nastaloPrerusenie( void )
{
  if ( priznak_IRQ == 0 )
  {
    priznak_IRQ = 1;
  }
}

void zaspiTeraz( void )
{
  // Vypnutie ADC prevodnika
  ADCSRA &= ( uint8_t )~( 1 << ADEN) ;
  PRR |= ( uint8_t )( 1 << PRADC );

  //cli();                // zakazanie globalnych preruseni
  noInterrupts();         // zakazanie globalnych preruseni
  //EIFR = 1 << INTF0;      // vycistenie priznaku pre prerusenie 0

  digitalWrite( LED, LOW );

  /* Mod prerusenia definuje, kedy ma byt prerusenie spustene. Su preddefinovane 4 konstanty:
  *  LOW to trigger the interrupt whenever the pin is low,
  *  CHANGE to trigger the interrupt whenever the pin changes value
  *  RISING to trigger when the pin goes from low to high,
  *  FALLING for when the pin goes from high to low.
  */
  attachInterrupt( digitalPinToInterrupt( PIR ), nastaloPrerusenie, RISING );  // nastavenie prerusenia

  /* Volba spiaceho rezimu
  * Existuje 5 modov pouzitelnych na standardnych 8-bitovych AVR:
  *    SLEEP_MODE_IDLE       – least power savings
  *    SLEEP_MODE_ADC
  *    SLEEP_MODE_PWR_SAVE
  *    SLEEP_MODE_STANDBY
  *    SLEEP_MODE_PWR_DOWN   – most power savings
  */
  set_sleep_mode( SLEEP_MODE_PWR_DOWN );
  sleep_bod_disable();    // vypnutie the Brown Out Detector (BOD) pred uspatim
  sleep_enable();         // nastavenie spiaceho bitu - sleep enable (SE)

  //sei();
  interrupts();

  sleep_cpu();            // uvedenie zariadenia do rezimu spanku
  sleep_disable();        // po prebudeni program pokracuje v tomto bode

  detachInterrupt( digitalPinToInterrupt( PIR ));
  digitalWrite( LED, HIGH );

  // Zapnutie ADC prevodnika
  PRR &= ( uint8_t )~( 1 << PRADC );
  ADCSRA |= ( uint8_t )( 1 << ADEN );
}

void posliPoplach( void )
{
  //sleep_disable();
  //detachInterrupt( digitalPinToInterrupt( PIR ));
  //digitalWrite( LED, HIGH );

  const char * SPRAVA = "Poplach";
  odosliSpravu( SPRAVA );
}

void posliVcc( void )
{
  //sleep_disable();
  //detachInterrupt( digitalPinToInterrupt( PIR ));
  //digitalWrite( LED, HIGH );

  char sprava[ 5 ] = {0};
  uint16_t napatie = merajVcc( KOREKCIA_VCC );
  snprintf( sprava, sizeof( sprava ), "%u", napatie );
  odosliSpravu( sprava );
}

void odosliSpravu( const char* sprava )
{
  #if DEBUG == 1
    // Vypis na seriovu konzolu s naslednym cakanim na ukoncenie serioveho prenosu
    Serial.print( "Posielam spravu = " );
    Serial.println( sprava );
    //Serial.flush();
  #endif

  // Odoslanie spravy o poplachu
  #ifdef RF_ASK
    driverASK.send(( uint8_t* ) sprava, strlen( sprava ));
    driverASK.waitPacketSent();
    //delay(200);
  #endif

  #if DEBUG == 1
    Serial.println( "Koncim" );
    //Serial.flush();
  #endif
}
