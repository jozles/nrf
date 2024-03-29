#ifndef NRF24L01P
#define NRF24L01P

#include <Arduino.h>

#include "nRF24L01.h"

#define MAX_PAYLOAD_LENGTH 32
#define ADDR_LENGTH 5

class Nrfp
{
  public:
    Nrfp();
    void hardInit();
    void config();
    void regRead(uint8_t reg,byte* data,uint8_t len);
    void regWrite(uint8_t reg,byte* data,uint8_t len) ;
    void pwrUpRx();
    void pwrUpTx();
    bool transmitting();
    bool available();
    void flushRx();
    void flushTx();
    void Nrfp::dataRead(byte* data,uint8_t* pipe,uint8_t* pldLength);
    void dataWrite(byte* data,char na,uint8_t len);

    uint8_t  ce_pin;
    uint8_t  csn_pin;

    uint8_t channel;

    byte  mode;
    byte* r1_addr;
    byte* tr_addr;
    byte* br_addr;

};

#endif // NRF24L01P INCLUDED

