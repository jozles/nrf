
#include <nRF24L01.h>
#include "nrf24l01p.h"

Nrfp nrfp;

#define CE_PIN  9
#define CSN_PIN 10

  /* périphérique */
  // pipe 1   4 octets d'adresse du concentrateur + 1 octet n° de périphérique
  // pipe 0   adresse de broadcast lorsque le périphérique est en PRX 
  //          adresse du concentrateur lorsque le périphérique est en PTX
  // la lib effectue les initialisations / commutations
  
  #define NRF_MODE 'P'     // mode périphérique pour configurer la lib
  #define BR_ADDR  "bcast"
  #define P1_ADDR  "C0011" // concentrateur 001 périph 1
  #define TX_ADDR  "C0011" // idem P1 pour messages

unsigned long cnt=0;
unsigned long cntko=0;
unsigned long time_beg;
unsigned long time_end;
uint8_t pipe;
uint8_t pldLength;

byte message[MAX_PAYLOAD_LENGTH+1]={"ABCDEFGHIJKLMNOPQRSTUVWXYZ012345"};

/* prototypes */

void timeStat(unsigned long* timeS0,unsigned long* timeS,byte* stat,char* lib);
void hprint(byte* stat);
 
void setup() {
  
  Serial.begin(115200);

  nrfp.ce_pin=9;
  nrfp.csn_pin=10;  
  nrfp.hardInit();
  
  nrfp.channel=1;

  nrfp.mode=NRF_MODE;
  nrfp.r1_addr=(byte*)P1_ADDR;  // pipe 1 pour réception commandes avec ACK
  nrfp.tr_addr=(byte*)TX_ADDR;  // adresse serveur pour ce périphérique
  nrfp.br_addr=(byte*)BR_ADDR;  // adresse commune de broadcast

  nrfp.config();

  Serial.println("start");

}
 
void loop() {

  while(!nrfp.available()){}
  
  Serial.print("received ");delay(1000);
  
  nrfp.dataRead(message,&pipe,&pldLength);
  Serial.print((char*)message);
  Serial.print("p/l:");
  Serial.print(pipe);
  Serial.print("/");
  Serial.print(pldLength);
  
  if(pipe!=0){                              // sinon message de balise
    nrfp.dataWrite(message,'A',pldLength);
    Serial.println(" transmit...");
  
    while(nrfp.transmitting()){}
  }
} 



/* ********************************************************** */

void hprint(byte* stat)
{
  if(((*stat)&0xf0)==0){Serial.print('0');}
  Serial.println(*stat,HEX);
}

void timeStat(unsigned long* timeS0,unsigned long* timeS,byte* stat,char* lib)
{
  timeS=micros();
  if(*timeS0==0){*timeS0=micros();}
  nrfp.regRead(STATUS,stat,1);
  Serial.print(lib);Serial.print(" ");
  Serial.print(micros()-*timeS0);
  Serial.print(" stat=");hprint(stat);
}

