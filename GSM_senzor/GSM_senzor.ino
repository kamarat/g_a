/*== GSM Alarm - senzor ==
* ================================
*
* GA_snzor.ino
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
* =====================
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
* =======================
*/

/*== GLOBALNE PREMENNE ==
* =======================
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
* =======================
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

/*== INICIALIZACIA ==
* ===================
*/
void setup()
{
  // Inicializacia pinov
  pinMode( PIR, INPUT );
  pinMode( LED, OUTPUT );
  digitalWrite( LED, LOW );

  // Nastavenie prerusenia
  attachInterrupt( digitalPinToInterrupt( PIR ), odosliSpravu, RISING );

  Serial.begin( 115200 ); // inicializacia serioveho vystupu
  Serial.println( "Inicializacia ukoncena." );
  delay( 500 );
}


/*== HLAVNY PROGRAM ==
* ====================
*/
void loop()
{
  digitalWrite( LED, LOW );
  #if DEBUG == 1
  #endif
}

