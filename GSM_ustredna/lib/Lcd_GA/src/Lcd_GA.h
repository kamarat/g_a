#ifndef LCD_GA_H
#define LCD_GA_H

#include "Arduino.h"
#include <LiquidCrystal.h>

class LcdGA
{
public:
  LcdGA( LiquidCrystal* lcd, uint8_t pocetRiadkov, uint8_t pocetStlpcov );

  void vymazRiadok( uint8_t riadok );

private:
  LiquidCrystal* prLcd;
  uint8_t prPocetRiadkov;
  uint8_t prPocetStlpcov;
};

#endif
