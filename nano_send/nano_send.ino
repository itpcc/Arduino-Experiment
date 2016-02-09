#include <SPI.h>
#include <nRF24L01p.h>
#include <dht11.h>

nRF24L01p transmitter(7,8);//CSN,CE
dht11 dht;
#define DHT11PIN 5

void setup(){
  delay(150);
  Serial.begin(115200);
  Serial.println("SETTING UP");
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  transmitter.channel(80);
  transmitter.TXaddress("ALL");
  transmitter.init();
  Serial.println("Transmitter READY");
  
  Serial.println("Sensor READY");
}

String message;
unsigned long lastTemprature = 0;

void loop(){
  /*delay(1000);
  transmitter.txPL("MEAW -> "+String(millis(), DEC));
  transmitter.send(SLOW);
  Serial.println("MEAW -> "+String(millis(), DEC));*/
  /* Get a new sensor event */
  
  if(Serial.available()>0){
    char character=Serial.read();
    if(character=='\n'){
      Serial.println(message);
      do{
        if(message.length() > 26){
          transmitter.txPL((message.substring(0, 26))+"\a\a\a");
          message = message.substring(26);
        }else{
          transmitter.txPL(message);
          message = "";
        }
        transmitter.send(SLOW);        
      }while(message.length());      
      message="";
    }else{
      message+=character;
    }
  }
  if(millis() - lastTemprature >= 1000){
    Serial.print("Read DHT11: ");
    switch (dht.read(DHT11PIN)){
      case 0:    
        message = "***01|"+String((float)dht.humidity, 2)+"|"+String((float)dht.temperature, 2);
        Serial.println(message);
        transmitter.txPL(message);
        transmitter.send(SLOW);
        message = "";        
      break;
      case -1: Serial.println("Checksum error"); break;
      case -2: Serial.println("Time out error"); break;
      default: Serial.println("Unknown error"); break;
    }
    lastTemprature = millis();
  }
}
