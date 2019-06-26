#include <SPI.h>
#include <nRF24L01.h>


#define CE_PIN  10
#define CSN_PIN 9

#define CE_HIGH   digitalWrite(CE_PIN,HIGH);
#define CE_LOW    digitalWrite(CE_PIN,LOW);
#define CSN_HIGH  digitalWrite(CSN_PIN,HIGH);
#define CSN_LOW   digitalWrite(CSN_PIN,LOW);

#define MAX_PAYLOAD_LENGTH 32
#define ADDR_LENGTH 5
#define CHANNEL 1
#define RE_ADDR "peri1"
#define TR_ADDR "serv1"

#define RX 1
#define TX 0

#define CONFWORD (1<<MASK_RX_DR) | (1<<MASK_TX_DS) | (1<<MASK_MAX_RT) | (1<<EN_CRC) | (0<<CRCO) | (0<<PWR_UP) | (1<<PRIM_RX)

uint8_t confValue=CONFWORD;
uint8_t txrxMode=RX;
uint8_t stat,conf;
uint8_t channel=CHANNEL;
uint8_t maxLength=MAX_PAYLOAD_LENGTH;

unsigned long cnt=0;
unsigned long cntko=0;
unsigned long time_message;
unsigned long time_beg;
unsigned long time_end;

byte message[MAX_PAYLOAD_LENGTH+1]={"ABCDEFGHIJKLMNOPQRSTUVWXYZ012345"};

/* prototypes */

void regRead(uint8_t reg,byte* data,uint8_t len);
void regWrite(uint8_t reg,byte* data,uint8_t len) ;
void pwrUpRx();
void pwrUpRx();
bool sending();
void flushRx();
void flushTx();
void dataRead(byte* data);
void dataWrite(byte* data);
 
void setup() {
  
  Serial.begin(115200);

  pinMode(CE_PIN,OUTPUT);
  CE_LOW
  pinMode(CSN_PIN,OUTPUT);
  CSN_HIGH

  SPI.beginTransaction(SPISettings(1000000,MSBFIRST,SPI_MODE0));
  SPI.begin();

/* inits nrf24L01+ */

  regWrite(RF_CH,&channel,1);
  regWrite(RX_PW_P0,&maxLength,1);
  pwrUpTx();
  flushTx();
  pwrUpRx();
  flushRx();
  regRead(STATUS,&stat,1);
  regWrite(RX_ADDR_P0,RE_ADDR,ADDR_LENGTH);
  regWrite(TX_ADDR,TR_ADDR,ADDR_LENGTH);
  
  Serial.println("Go !"); 
}
 
void loop() {
  
  while(sending()){}

  stat=0;
  while(!(stat & (1 << RX_DR)) ){
    regRead(STATUS,&stat,1);
  }

  dataRead(message);

Serial.println((char*)message);

  dataWrite(message);

  Serial.print("sending...");
  
} 



/* ********************************************************** */
/*
    Principe :
    Le nrf24L01 est mis en mode RX à l'initialisation
    Lorsque une transmission doit être effectuée, il passe en mode TX jusqu'à épuisement des paquets
    Dès la fin de celle-ci il revient en mode RX 
    (la gestion des paquets multiples est en attente, retour au mode RX dès la réception d'un paquet)
*/

void regRead(uint8_t reg,byte* data,uint8_t len)
{
    CSN_LOW
    SPI.transfer((R_REGISTER | (REGISTER_MASK & reg)));
    SPI.transfer(data,len);
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
    conf=confValue|(1<<PWR_UP)|(1<<PRIM_RX);      // powerUP, Rx
    txrxMode=RX;                                  // set RX mode
    regWrite(CONFIG,&conf,1);                        
    CE_HIGH                                         
}  

void pwrUpTx()
{
    CE_LOW
    conf=0x7A; //confValue|(1<<PWR_UP)|(0<<PRIM_RX);      // powerUP, Tx
    txrxMode=TX;                                    // set TX mode
    regWrite(CONFIG,&conf,1);                         
    CE_HIGH
}  

bool sending()                                             // if sending -> true else turn to rxMode -> false
{
  if(txrxMode==TX){
    regRead(STATUS,&stat,1);
    //Serial.print("status ");Serial.println(status,HEX);                                    
    if((stat & ((1 << TX_DS)  | (1 << MAX_RT)))){
      
      stat|=(1<<TX_DS);                                   // clear TX_DS bit
      regWrite(STATUS,&stat,1);
      pwrUpRx();
      return false; 
    }
    return true;
  }
  return false;
}

void dataRead(byte* data)
{
    CSN_LOW
    SPI.transfer(R_RX_PAYLOAD);
    SPI.transfer(data,MAX_PAYLOAD_LENGTH);
    CSN_HIGH

    regRead(STATUS,&stat,1);                                // clear RX_DR bit
    stat|=(1<<RX_DR);
    regWrite(STATUS,&stat,1);
}

void dataWrite(byte* data)
{
    pwrUpTx();
    flushTx();
    regRead(STATUS,&stat,1);
    CSN_LOW
    SPI.transfer(W_TX_PAYLOAD);
    SPI.transfer(data,MAX_PAYLOAD_LENGTH);
    CSN_HIGH
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


