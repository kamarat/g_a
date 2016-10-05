#ifndef GSM_GA_H
#define GSM_GA_H

#include "Arduino.h"
//#include <stdint.h>

class GSMGA
{
  public:
    GSMGA( HardwareSerial* serial, const uint8_t* power );
    GSMGA(HardwareSerial* serial, const uint8_t* rx, const uint8_t* tx );

    void zapniGSM( void );
    void initGSM( void );

  private:
    // Nastavenie premennych GSM modulu
    //const uint8_t _GSM_RX = 14;  // GSM RX pripojene na TX3
    //const uint8_t _GSM_TX = 15;  // GSM TX pripojene na RX3
    HardwareSerial* HWSerial_;
    const uint8_t* POWER_GSM_;
    const uint8_t* GSM_RX_;
    const uint8_t* GSM_TX_;

    //char * initGSM();
};

#endif
