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
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

#include <RH_ASK.h> // kniznica RadioHead pre RF modul s ASK modulaciou
#include <SPI.h>    // nie je pouzita, ale je potrebna pre kompilaciu s RadioHead

/*== GLOBALNE PREMENNE ==
 *=======================
 * premenne popisujuce zapojene piny
 * premenne popisujuce nastavenie Arduina
 */

#define DEBUG 1   // definicia odladovania a vypisov
#define RF_ASK 1  // definica pouziteho RF modulu

/*== Deklaracia konstant ==
 */
//const uint8_t 

// analogove piny

// digitalne piny
const uint8_t PIR = 3;  // vystup PIR pripojeny na port D3
const uint8_t LED = 13;

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

/*== DEFINICIA FUNKCII ==
 *=======================
 */
  
void odosliSpravu( void )
{
  detachInterrupt( digitalPinToInterrupt( PIR ));
  digitalWrite( LED, HIGH );

  #if DEBUG == 1
    // Vypis na seriovu konzolu s naslednym cakanim na ukoncenie serioveho prenosu
    Serial.println( "Posielam spravu" );
    Serial.print( "Napajacie napatie (mV) = " );
    Serial.println( merajVcc() );
    Serial.flush();
  #endif
  
  /*while( digitalRead( PIR ))
    digitalWrite( LED, HIGH );
    */
  // Odoslanie spravy o poplachu
  #ifdef RF_ASK
    const char *SPRAVA = "Poplach";
    driverASK.send(( uint8_t * )SPRAVA, strlen( SPRAVA ));
    //driverASK.waitPacketSent();
    delay(200);
  #endif

  #if DEBUG == 1
    Serial.println( "Koncim" );
    Serial.flush();
  #endif
}

void zaspiTeraz( void )
{
  /* Mod prerusenia definuje, kedy ma byt prerusenie spustene. Su preddefinovane 4 konstanty: 
   *  LOW to trigger the interrupt whenever the pin is low,
   *  CHANGE to trigger the interrupt whenever the pin changes value
   *  RISING to trigger when the pin goes from low to high,
   *  FALLING for when the pin goes from high to low.
   */
  attachInterrupt( digitalPinToInterrupt( PIR ), odosliSpravu, RISING );  // nastavenie prerusenia
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
  noInterrupts();       // zakazanie globalnych preruseni
  sleep_enable();       // nastavenie spiaceho bitu - sleep enable (SE)
  sleep_bod_disable();  // vypnutie the Brown Out Detector (BOD) pred uspatim
  //sei();                // povolenie preruseni
  interrupts();         // povolenie preruseni
  
  digitalWrite( LED, LOW );
  #if DEBUG == 1
    Serial.println( "Zaspavam." );
    Serial.flush();
  #endif
  
  sleep_cpu();      // uvedenie zariadenia do rezimu spanku 
  sleep_disable();  // po prebudeni program pokracuje v tomto bode
  //sei();
  interrupts();
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
    if ( !driverASK.init() )
           Serial.println( "Inicializacia drivera ASK sa nepodarila!" );
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

