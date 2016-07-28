/*== GSM Alarm - senzor ==
 *================================
 *
 * GA_senzor.ino
 *
 * Program nacita stav PIR a odosle prostrednictvom radioveho signalu
 * na GSM ústrednu
 *
 * @author: mr.nobody
 * @date:   jul 2016
 *
 * mr.nobody (cleft) 2016
 */

/*== ZAPOJENIE PINOV ==
 *=====================
 *
 * ARDUINO --- PERIFERIE
 * A   0 -
 *
 * D   2 - 
 * D   3 - vystup signalu z PIR (prerusenie)
 * D   3 - kontrolna LED
 *
 */

/*== KNIZNICE A SUBORY ==
 *=======================
 */
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

/*== GLOBALNE PREMENNE ==
 *=======================
 * premenne popisujuce zapojene piny
 * premenne popisujuce nastavenie Arduina
 */

// Definicia odladovania a vypisov
#define DEBUG 1

/*== Deklaracia konstant ==
 */
//const uint8_t 

// analogove piny

// digitalne piny
// pripojenie citaciek na digitalne piny
const uint8_t PIR = 3;
const uint8_t LED = 13;

/*== Deklaracia premennych ==
 */

/*== DEFINICIA FUNKCII ==
 *=======================
 */
  
void odosliSpravu( void )
{
  detachInterrupt( digitalPinToInterrupt( PIR ));
  Serial.println( "Posielam spravu" );
  digitalWrite( LED, HIGH );
  Serial.println( "Koncim" );
  #if DEBUG == 1
  #endif
  digitalWrite( LED, LOW );
  attachInterrupt( digitalPinToInterrupt( PIR ), odosliSpravu, RISING );
}

void zaspiTeraz( void )
{
  attachInterrupt( digitalPinToInterrupt( PIR ), odosliSpravu, RISING );
  delay( 100 );
  // Volba spiaceho rezimu
  set_sleep_mode( SLEEP_MODE_IDLE );

  // Nastavenie spiaceho bitu - sleep enable (SE)
  sleep_enable();

  // Uvedenie zariadenia do rezimu spanku
  sleep_mode();

  // Po prebudeni program pokracuje v tomto bode
  sleep_disable();

  digitalWrite( LED, HIGH );
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
  DDRD &= B00000011;       // nastavenie pinov 2 - 7 na vstupne, pinom 0 a 1 (RX a TX) ponechane nastavenia
  DDRB = B00000000;        // nastavenie pinov 8 - 13 na vstupne
  PORTD |= B11111100;      // aktivacia pullup na pinoch 2 - 7, piny 0 a 1 (RX a TX) ponechane bez
  PORTB |= B11111111;      // aktivacia pullup na pinoch 8 - 13
  
  // Inicializacia pinov
  pinMode( PIR, INPUT );
  pinMode( LED, OUTPUT );
  digitalWrite( LED, HIGH );

  // Nastavenie prerusenia
  attachInterrupt( digitalPinToInterrupt( PIR ), odosliSpravu, RISING );

  Serial.begin( 115200 ); // inicializacia serioveho vystupu
  Serial.println( "Inicializacia ukoncena." );
  delay( 500 );
}

/*== HLAVNY PROGRAM ==
 *====================
 */
void loop()
{
  // Arduino zostane prebudene sekundu, potom zaspi.
  // LED po zaspati zhasne a zasvieti pri prebudeni.
  delay( 1000) ;
  zaspiTeraz();
  
  #if DEBUG == 1
  #endif
}

