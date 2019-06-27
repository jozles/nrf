#include <SPI.h>
#include <nRF24L01.h>

#define SRV_NRF
//#define CLI_NRF

#define CE_PIN  9
#define CSN_PIN 10

#define CE_HIGH   digitalWrite(CE_PIN,HIGH);
#define CE_LOW    digitalWrite(CE_PIN,LOW);
#define CSN_HIGH  digitalWrite(CSN_PIN,HIGH);
#define CSN_LOW   digitalWrite(CSN_PIN,LOW);

#define MAX_PAYLOAD_LENGTH 32
#define ADDR_LENGTH 5
#define CHANNEL 1
#ifdef CLI_NRF
#define RE_ADDR "nrf01" // "serv1"
#define TR_ADDR "nrf02" // "peri1"
#endif CLI_NRF
#ifdef SRV_NRF
#define RE_ADDR "nrf02" // "peri1"
#define TR_ADDR "nrf01" // "serv1"
#endif SRV_NRF

#define RX 1
#define TX 0

#define CONFWORD (0<<MASK_RX_DR) | (0<<MASK_TX_DS) | (0<<MASK_MAX_RT) | (1<<EN_CRC) | (0<<CRCO) | (0<<PWR_UP) | (0<<PRIM_RX)

uint8_t txrxMode=99;
uint8_t regw,stat,statb,conf;

unsigned long cnt=0;
unsigned long cntko=0;
unsigned long time_message;
unsigned long time_beg;
unsigned long time_end;

unsigned long nrfTime0=0;
unsigned long nrfTime1=0;
unsigned long nrfTime2=0;
unsigned long nrfTime3=0;


byte message[MAX_PAYLOAD_LENGTH+1]={"ABCDEFGHIJKLMNOPQRSTUVWXYZ012345"};

/* prototypes */

void regRead(uint8_t reg,byte* data,uint8_t len);
void regWrite(uint8_t reg,byte* data,uint8_t len) ;
void pwrUpRx();
void pwrUpRx();
bool transmitting();
void flushRx();
void flushTx();
void dataRead(byte* data);
void dataWrite(byte* data);
void timeStat(unsigned long* timeS0,unsigned long* timeS,byte* stat,char* lib);
void hprint(byte* stat);
 
void setup() {
  
  Serial.begin(115200);

  pinMode(CE_PIN,OUTPUT);
  CE_LOW
  pinMode(CSN_PIN,OUTPUT);
  CSN_HIGH

  SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));
  SPI.begin();

/* inits nrf24L01+ */

/*
 * Principe : 
 *  le bit PXR_UP et CE_LOW place le chip en Standby en maxi 4,5mS
 *  pour transmettre charger le FIFO, bit PRIM_RX_low faire un pulse >10uS sur CE, delay 130uS
 *    retour auto en standby
 *  pour recevoir bit PRIM_RX_high, CE_HIGH, delay 130uS retour en standby avec CE_LOW
 *  
 */

  
  regWrite(RX_ADDR_P1,RE_ADDR,ADDR_LENGTH);
  regWrite(RX_ADDR_P0,TR_ADDR,ADDR_LENGTH);
  regWrite(TX_ADDR,TR_ADDR,ADDR_LENGTH);
  regw=CHANNEL;regWrite(RF_CH,&regw,1); // channel 1
  regw=MAX_PAYLOAD_LENGTH; 
  regWrite(RX_PW_P0,&regw,1);
  regWrite(RX_PW_P1,&regw,1);
  regw=0x0B; // crc 8 bits ; power Up ; RX
  regWrite(CONFIG,&regw,1);

  delay(5); 

  flushTx();
  regw=0x30; // clear bits TX_DS & MAX_RT
  regWrite(STATUS,&regw,1);
 // timeStat(&nrfTime0,&nrfTime0,&stat,"start");

Serial.println("start");

}
 
void loop() {

#ifdef CLI_NRF  

  cnt++;
  time_beg = micros();
  
  dataWrite(message);

 // Serial.print("transmitting...");Serial.println((char*)message);
  
  while(transmitting()){
 //   regRead(STATUS,&statb,1);if(stat!=statb){
 //     timeStat(&nrfTime3,&nrfTime0,&stat,"trans ");
 //   }
  }
  
//Serial.print("...transmitted...");
//delayMicroseconds(500);timeStat(&nrfTime0,&nrfTime3,&stat,"trans ");
  long readTo=0;
  stat=0;
  while(!(stat & (1 << RX_DR)) && readTo>=0 ){
    regRead(STATUS,&stat,1);
    readTo=10000-(micros() - time_beg);
  }
  
  if(readTo<0){cntko++;Serial.print(cntko);Serial.println(" Pas de pong");delay(1000);return;}

  dataRead(message);
  
  time_end=micros();
  
  Serial.print("received...");Serial.print((char*)message);Serial.print("...in:");
  Serial.print(time_end - time_beg); 
  Serial.println("us");

  delay(500);
#endif CLI_NRF
#ifdef SRV_NRF

  CE_HIGH
  stat=0;
  while((stat & (1 << RX_DR))==0 ){
    regRead(STATUS,&stat,1);
  }
  CE_LOW
  Serial.print("received ");
  dataRead(message);Serial.print((char*)message);
  delayMicroseconds(500);
  dataWrite(message);

  Serial.println(" transmit...");
  
  while(transmitting()){}

#endif SRV_NRF
} 



/* ********************************************************** */

void regRead(uint8_t reg,byte* data,uint8_t len)
{
    CSN_LOW
    SPI.transfer((R_REGISTER | (REGISTER_MASK & reg)));
    for(len;len>0;len--){data[len-1]=SPI.transfer(data[len-1]);}
    CSN_HIGH
}

void regWrite(uint8_t reg,byte* data,uint8_t len) 
{
    CSN_LOW
    SPI.transfer((W_REGISTER | (REGISTER_MASK & reg)));
    for(len;len>0;len--){SPI.transfer(data[len-1]);}
    CSN_HIGH
}

void pwrUpRx()
{
    CE_LOW
    if(txrxMode!=RX){
      conf=CONFWORD|(1<<PWR_UP)|(1<<PRIM_RX);       // powerUP, Rx
      txrxMode=RX;                                  // set RX mode
      regWrite(CONFIG,&conf,1);                        
    }              
}

void pwrUpTx()
{
    CE_LOW
    if(txrxMode!=TX){
      conf=CONFWORD|(1<<PWR_UP)|(0<<PRIM_RX);       // powerUP, Tx
      txrxMode=TX;                                  // set TX mode
      regWrite(CONFIG,&conf,1);
    }
}  

void dataRead(byte* data)
{
    CSN_LOW
    SPI.transfer(R_RX_PAYLOAD);
    SPI.transfer(data,MAX_PAYLOAD_LENGTH);
    CSN_HIGH
          
    stat=(1<<RX_DR);
    regWrite(STATUS,&stat,1);     // clear RX_DR bit
}

void dataWrite(byte* data)
{
    pwrUpTx();
    
    flushTx();
    
    stat=(1<<TX_DS) | (1<<MAX_RT);
    regWrite(STATUS,&stat,1);     // clear TX_DS & MAX_RT bits

    CSN_LOW
    SPI.transfer(W_TX_PAYLOAD);
    for(int i=0;i<MAX_PAYLOAD_LENGTH;i++){SPI.transfer(data[i]);}
    CSN_HIGH

    CE_HIGH                       // transmit

// transmitting() doit être testé pour attendre la fin de l'envoi du paquet
}

bool transmitting()               // if trans -> true else turn to rxMode -> false
{
    if(txrxMode==TX){
      regRead(STATUS,&stat,1);                                
      if((stat & ((1 << TX_DS)  | (1 << MAX_RT)))){
      
        stat|=(1<<TX_DS)|(1 << MAX_RT);     // clear TX_DS & MAX_RT bits
        regWrite(STATUS,&stat,1);
        pwrUpRx();
        return false; 
      }
      return true;
    }
    return false;
}

void flushTx()
{
    CSN_LOW
    SPI.transfer(FLUSH_TX);
    CSN_HIGH  
}

void flushRx()
{
    CSN_LOW
    SPI.transfer(FLUSH_RX);
    CSN_HIGH 
}

void hprint(byte* stat)
{
  if(((*stat)&0xf0)==0){Serial.print('0');}
  Serial.println(*stat,HEX);
}

void timeStat(unsigned long* timeS0,unsigned long* timeS,byte* stat,char* lib)
{
  timeS=micros();
  if(*timeS0==0){*timeS0=micros();}
  regRead(STATUS,stat,1);
  Serial.print(lib);Serial.print(" ");
  Serial.print(micros()-*timeS0);
  Serial.print(" stat=");hprint(stat);
}

