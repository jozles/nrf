
#include <nRF24L01.h>
#include "nrf24l01p.h"

Nrfp nrfp;

#define CE_PIN  9
#define CSN_PIN 10

#define RE_ADDR "nrf02" // ^peri1"
#define TR_ADDR "nrf01" // "serv1"

unsigned long cnt=0;
unsigned long cntko=0;
unsigned long time_beg;
unsigned long time_end;

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
  nrfp.re_addr="clie1";
  nrfp.tr_addr="serv1";  
  nrfp.config();

  Serial.println("start");

}
 
void loop() {

  while(!nrfp.available()){}
  
  Serial.print("received ");
  
  nrfp.dataRead(message);Serial.print((char*)message);
  
  nrfp.dataWrite(message);

  Serial.println(" transmit...");
  
  while(nrfp.transmitting()){}

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

