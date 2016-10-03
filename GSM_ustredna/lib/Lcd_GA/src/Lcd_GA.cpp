#include "Lcd_GA.h"

LcdGA::LcdGA( LiquidCrystal* lcd, uint8_t pocetRiadkov, uint8_t pocetStlpcov ) {
  prLcd = lcd;
  prPocetRiadkov = pocetRiadkov;
  prPocetStlpcov = pocetStlpcov;
}

void LcdGA::vymazRiadok( uint8_t riadok) {
  prLcd->setCursor( 0, riadok );
  for ( uint8_t i = 0; i < prPocetStlpcov; i++ ) {
    prLcd->print( " " );
  }
}
