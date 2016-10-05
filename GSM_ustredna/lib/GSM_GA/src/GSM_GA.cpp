

#include "GSM_GA.h"
#include "../../../GSM_ustredna/Private.h"

GSMGA::GSMGA( HardwareSerial* serial, const uint8_t* power )
{
  HWSerial_ = serial;
  POWER_GSM_ = power;
  HWSerial_->begin( 19200 );
}

GSMGA::GSMGA( HardwareSerial* serial, const uint8_t* rx, const uint8_t* tx )
{
  HWSerial_ = serial;
  GSM_RX_ = rx;
  GSM_TX_ = tx;
}

void GSMGA::zapniGSM( void )
{
  digitalWrite( *POWER_GSM_, LOW );
  delay( 1000 );
  digitalWrite( *POWER_GSM_, HIGH );
  delay( 2000 ) ;
  digitalWrite( *POWER_GSM_, LOW );
  delay( 3000 );
}

//char * initGSM()
void GSMGA::initGSM( void )
{
  char odpoved[ 100 ] = {0};
  uint8_t i = 0;

  //Serial3.begin( 19200 ); // inicializacia serioveho vystupu

  HWSerial_->print( "AT" );

  while ( HWSerial_->available() ) {
    delay( 10 );
    if ( HWSerial_->available() > 0 ) {
      char c = HWSerial_->read();
      odpoved[ i ] = c;
    }
  }

  //return odpoved;
}
