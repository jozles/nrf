
#include <nRF24L01.h>
#include "nrf24l01p.h"

Nrfp nrfp;

#define CE_PIN  9
#define CSN_PIN 10

//#define RE_ADDR "nrf01" // "serv1"
//#define TR_ADDR "nrf02" // "peri1"

unsigned long cnt=0;
unsigned long cntko=0;
unsigned long time_beg;
unsigned long time_end;

long readTo=0;

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
  nrfp.re_addr=(byte*)"serv1";
  nrfp.tr_addr=(byte*)"clie1";  
  nrfp.config();

  Serial.println("start");

}

void loop() {
  
  cnt++;
  sprintf(message,"%05d",cnt);
  message[strlen(message)]='*';
  time_beg = micros();

  nrfp.dataWrite(message);

  Serial.print((char*)message);
  
  while(nrfp.transmitting()){}
  
  //Serial.print("...transmitted...");

  while(!nrfp.available()){   
    readTo=10000-micros()+time_beg;
    if(readTo<0){break;}
  }
  
  if(readTo<0){cntko++;Serial.print(cntko);Serial.println(" time out");}
  else{
  
    nrfp.dataRead(message);
  
    time_end=micros();
    Serial.print(" received ");  
    Serial.print((char*)message);Serial.print(" in:");
    Serial.print(time_end - time_beg); 
    Serial.println("us");
    
  }

  delay(2000);

} 


