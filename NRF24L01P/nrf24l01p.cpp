#include <SPI.h>
#include "nrf24l01p.h"

// hardware is depending on environment
#define INIT_CE   pinMode(ce_pin,OUTPUT);
#define INIT_CSN  pinMode(csn_pin,OUTPUT);
#define CE_HIGH   digitalWrite(ce_pin,HIGH);
#define CE_LOW    digitalWrite(ce_pin,LOW);
#define CSN_HIGH  digitalWrite(csn_pin,HIGH);
#define CSN_LOW   digitalWrite(csn_pin,LOW);
#define INIT_SPI  SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));
#define START_SPI SPI.begin();


#define DEF_CHANNEL 1

#define RX 1
#define TX 0

#define CONFREG (0<<MASK_RX_DR) | (0<<MASK_TX_DS) | (0<<MASK_MAX_RT) | (1<<EN_CRC) | (0<<CRCO) | (0<<PWR_UP) | (0<<PRIM_RX)
#define RF_DR_LOW  5
#define RF_DR_HIGH 3
#define RFREG   (1<<RF_DR_HIGH) | (0<<RF_DR_LOW) | (0x03<<RF_PWR)

uint8_t regw,stat,fstat,conf;

Nrfp::Nrfp()    // constructeur
{
}


void Nrfp::hardInit()
{
  CE_LOW
  INIT_CE
  CSN_HIGH
  INIT_CSN


  INIT_SPI
  START_SPI
}


/*
 * Principe :             (one to one)
 *
 *  le bit PXR_UP et CE_LOW place le chip en Standby en maxi 4,5mS
 *
 *  pour transmettre :
 *    charger le FIFO, bit PRIM_RX_low puis CE high (delay 130uS)
 *    attendre la fin de la transmission (TX_DS set ou MAX_RT set)
 *    retour en standby avec CE low
 *      (pwrUpRx() et PwrUpTx() font CE_LOW)
 *      (dataWrite() fait flushTx() et termine avec CE_HIGH)
 *      (transmitting() termine avec pwrUpRx() si false)
 *
 *  pour recevoir :
 *    bit PRIM_RX_high, CE_HIGH (delay 130uS)
 *    attendre un paquet (RX_DR set ou RX_EMPTY clr)
 *    retour en standby avec CE low
 *       (available() fait CE_HIGH
 *                    et termine avec CE_LOW si true)
 *
 */

void Nrfp::config()           // power on minimal config
{
  regWrite(RX_ADDR_P0,tr_addr,ADDR_LENGTH);
  regWrite(RX_ADDR_P1,r1_addr,ADDR_LENGTH);
  if(mode=='C'){
    regWrite((EN_AA),ENAA_P5|ENAA_P4|ENAA_P3|ENAA_P2|ENAA_P1|ENAA_P0,1);
    regWrite((EN_RXADDR),ERX_P5|ERX_P4|ERX_P3|ERX_P2|ERX_P1|ERX_P0,1);
    for(uint8_t i=1;i<ADDR_LENGTH;i++){
      regWrite((RX_ADDR_P1+i),r1_addr[ADDR_LENGTH],1);
    }
  }
  regWrite(TX_ADDR,tr_addr,ADDR_LENGTH);

  if(channel==0){channel=DEF_CHANNEL;}
  regWrite(RF_CH,&channel,1);
  regw=MAX_PAYLOAD_LENGTH;
  regWrite(RX_PW_P0,&regw,1);
  regWrite(RX_PW_P1,&regw,1);
  regw=(1<<EN_DYN_ACK);                // no ack enable
  regWrite(FEATURE,&regw,1);
  regw=RFREG;
  regWrite(RF_SETUP,&regw,1);

  pwrUpRx();

  delay(5);

  flushRx();
  flushTx();

  regw=(1<<TX_DS) | (1<<MAX_RT); // clear bits TX_DS & MAX_RT
  regWrite(STATUS,&regw,1);
}


/* ********************************************************** */

void Nrfp::regRead(uint8_t reg,byte* data,uint8_t len)
{
    CSN_LOW
    SPI.transfer((R_REGISTER | (REGISTER_MASK & reg)));
    for(len;len>0;len--){data[len-1]=SPI.transfer(data[len-1]);}
    CSN_HIGH
}

void Nrfp::regWrite(uint8_t reg,byte* data,uint8_t len)
{
    CSN_LOW
    SPI.transfer((W_REGISTER | (REGISTER_MASK & reg)));
    for(len;len>0;len--){SPI.transfer(data[len-1]);}
    CSN_HIGH
}

void Nrfp::pwrUpRx()
{
    CE_LOW

      conf=CONFREG|(1<<PWR_UP)|(1<<PRIM_RX);       // powerUP, Rx
      regWrite(CONFIG,&conf,1);
}

void Nrfp::pwrUpTx()
{
    CE_LOW

      conf=CONFREG|(1<<PWR_UP)|(0<<PRIM_RX);       // powerUP, Tx
      regWrite(CONFIG,&conf,1);
}

bool Nrfp::available()      // keep CE high when false
{
    if(mode=='P'){
          regWrite(RX_ADDR_P0,br_addr,ADDR_LENGTH);
    }
    CE_HIGH
    regRead(FIFO_STATUS,&fstat,1);
    if((fstat & (1<<RX_EMPTY))!=0){

        regRead(STATUS,&stat,1);
        if((stat & (1 << RX_DR))==0){
            return false;
        }
    }
    CE_LOW
    return true;  // dataRead should be done now
}

void Nrfp::dataRead(byte* data,uint8_t* pipe,uint8_t* pldLength)
{
    regRead(STATUS,&stat,1);
    *pipe=(stat>>RX_P_NO)&0x03;

    regRead((RX_PW_P0+pipe),*pldLength,1);
    if(*pldLength>MAX_PAYLOAD_LENGTH){
        *pldLength=0;
        flushRx();
        memset(data,0x00,MAX_PAYLOAD_LENGTH);
    }
    else{
        CSN_LOW
        SPI.transfer(R_RX_PAYLOAD);
        SPI.transfer(data,pldLength);
        CSN_HIGH
    }
    stat=(1<<RX_DR);
    regWrite(STATUS,&stat,1);     // clear RX_DR bit
}

void Nrfp::dataWrite(byte* data,char na,uint8_t len)
{
    pwrUpTx();

    flushTx();

    stat=(1<<TX_DS) | (1<<MAX_RT);
    regWrite(STATUS,&stat,1);     // clear TX_DS & MAX_RT bits

    CSN_LOW
    if(na=='A'){SPI.transfer(W_TX_PAYLOAD);}
    else{       SPI.transfer(W_TX_PAYLOAD_NA);}
    for(uint8_t i=0;i<len;i++){SPI.transfer(data[i]);}
    CSN_HIGH

    CE_HIGH                       // transmit

// transmitting() should be checked now to wait end of paquet transmission
// before turning CE low
}

bool Nrfp::transmitting()         // if end -> true else turn to rxMode
{
      regRead(STATUS,&stat,1);
      if((stat & ((1 << TX_DS)  | (1 << MAX_RT)))){

        stat|=(1<<TX_DS)|(1 << MAX_RT); // clear TX_DS & MAX_RT bits
        regWrite(STATUS,&stat,1);
        pwrUpRx();
        return false;
      }
      return true;
}

void Nrfp::flushTx()
{
    CSN_LOW
    SPI.transfer(FLUSH_TX);
    CSN_HIGH
}

void Nrfp::flushRx()
{
    CSN_LOW
    SPI.transfer(FLUSH_RX);
    CSN_HIGH
}

