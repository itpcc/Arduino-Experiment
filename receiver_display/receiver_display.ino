/*  OLED          arduino
     SCL----------10
     SDA-----------9
     RST----------13
     DC-----------11
     VCC----------5V
     GND----------GND*/
/*  RTC           arduino
 *   SCL----------21
 *   SDA----------20
 */
/* DHT11 Pin 22 */
#include <OLED.h>
#include <DS3232RTC.h>    //http://github.com/JChristensen/DS3232RTC
#include <Time.h>         //http://www.arduino.cc/playground/Code/Time  
#include <Wire.h>         //http://arduino.cc/en/Reference/Wire (included with Arduino IDE)
#include <SPI.h>
#include <nRF24L01p.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_HMC5883_U.h>
#include "number.h"
#include <ESP8266.h>

#define RESULT_LEN_SIZE 2048
#define SCROLL_DELAY 1000
#define MENU_BUTTON 48
#define MENU_THRESHOLD 150

struct reportData_T {
  float tempDHT;
  float humidDHT;
  float tempBMP;
  float pressureBMP;
  float altBMP; 
};

#define WIFI_SSID        "YOUR_AP_NAME"
#define WIFI_PASSWORD    "YOUR_AP_PASSWORD"
#define WIFI_CONNECT_LIMIT 1000
const String HOST_NAME = "YOUR_HOST_DOMAIN";
const String HOST_LOCATION = "/YOUR_SUBDIR";
const String HOST_USERNAME = "YOUR_USERNAME";
const String HOST_PASSWORD = "YOUR_PASSWORD";

OLED OLED;
nRF24L01p receiver(7,8);//CSN,CE
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
sensor_t sensor;
sensors_event_t event;
ESP8266 wifi(Serial1, 115200);

unsigned long lastShowTime = 0, lastDisplay = 0, lastClock = 0, lastGY651 = 0, lastShowMenu = 0, lastWiFISend = 0;
String msg, bufferReceiveText, currTime, displayText;
byte currReceiveLine = 0, currMenu = 0;
int clockNumber = 0;
bool isTextJoin = false;
short wifiState = 0;
int serverCMD = 0;

float headingDegree(float x, float y){
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(y, x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  return heading * 180/M_PI; 
}

String formatDigit(int digits){
  String str = ":";
  if(digits < 10)
    str += "0";
  str += digits;
  return str;
}

void printLargeNumber(unsigned char number, unsigned char x, unsigned char y){
  int i, j;
  unsigned char bufferChar[20];
  //Serial.print("==================================>"); Serial.println(number);
  for(i = 0; i < 5; ++i){
    for(j = 0; j < 20; ++j){
      bufferChar[j] = pgm_read_byte(&numberDisplay[(i*220)+((number*20)+j)]);
      //bufferChar[j] = 255;
    }
    OLED.LED_PrintBMP(x, i+y, x+20, i+y, bufferChar);
  }
  //Serial.println();
}

void fillScreenArea(unsigned char xFrom, unsigned char yFrom, unsigned char xTo, unsigned char yTo, unsigned char value){
  unsigned char x,y;
  for(y=yFrom;y<=yTo;y++){
    OLED.LED_Set_Pos(xFrom,y);        
    for(x=xFrom;x<xTo;x++){      
      OLED.LED_WrDat(value);       
    }
  }
}

void clearScreen(){
  fillScreenArea(0, 2, 128, 7, 0);
}

void printText(){
  //Characters per line : 21
  int bufferLen, newLinePos, i;
  bufferLen   = displayText.length();
  newLinePos  = displayText.indexOf('\n');
  if(currReceiveLine == 6){
    lastDisplay = millis(); //used for delay display message
    currReceiveLine = 0;
  }else if(bufferLen > 0){
    if(currReceiveLine == 0 && lastDisplay != 0)  clearScreen();
    
    if(bufferLen < 21){
      if(newLinePos != -1){
        displayText.replace("\n","");
        bufferLen = displayText.length();
      }
      OLED.LED_P6x8Str(0,2+currReceiveLine,(char*)displayText.c_str());
      for(i = bufferLen+1; i < 21; i++) OLED.LED_P6x8Char(6*i, 2+currReceiveLine,' ');
      displayText = "";
      currReceiveLine++;
    }else{
      String bufferText;
      if(newLinePos != -1 && newLinePos < 21){
        bufferText = displayText.substring(0, newLinePos-1);
        displayText = displayText.substring(newLinePos+1);
        bufferLen = bufferText.length();
        OLED.LED_P6x8Str(0,2+currReceiveLine,(char*)bufferText.c_str());
        for(i = bufferLen+1; i < 21; i++) OLED.LED_P6x8Char(6*i, 2+currReceiveLine,' ');
      }else{
        bufferText = displayText.substring(0, 21);
        displayText = displayText.substring(21);
        OLED.LED_P6x8Str(0,2+currReceiveLine,(char*)bufferText.c_str());
        //Serial.println("===========>"+bufferText);
      }
      currReceiveLine++;
    }
  }
}

void enqueueLine(String str){
  int displayTextLen = displayText.length();
  if(displayTextLen){
    if(currMenu != 0 && displayTextLen > 126) // 126 was from 21 chars * 6 lines
      displayText = "";
    else 
      displayText += "\n";
  }
  displayText += str;
}

void changeMenu(byte menuId){
  currMenu = menuId%3;
  clearScreen();
  switch(currMenu){
    case 0:
      OLED.LED_P8x16Str(0,4,(char*)String(F("     CONSOLE    ")).c_str());
      lastDisplay = millis(); //used for delay display message
      currReceiveLine = 0;
    break;
    case 1:
      OLED.LED_P8x16Str(0,4,(char*)String(F("      TIME      ")).c_str());
      lastClock = millis(); //used for delay display message
    break;
    case 2:
      OLED.LED_P8x16Str(0,4,(char*)String(F("     GY-651     ")).c_str());
      lastGY651 = millis(); //used for delay display message
    break;
  }
  lastShowMenu = millis();
}

void connectWIFI(){
  unsigned long startOperateTime = millis();
  if(!wifi.kick()) return;
  switch(wifiState){
    case 0:
      OLED.LED_P8x16Str(0,0,(char*)String(F("Set station...")).c_str());
      if(wifi.setOprToStation()){
        enqueueLine(String("+>Station SET"));
        wifiState = 1;
      }
    if(millis() - startOperateTime > WIFI_CONNECT_LIMIT) break;
    case 1:
      OLED.LED_P8x16Str(0,0,(char*)String(F("AP Joining.....")).c_str());
      if(wifi.joinAP(WIFI_SSID, WIFI_PASSWORD)){
        enqueueLine(String("+>AP "+String(WIFI_SSID)+" JOINED"));
        wifiState = 9;
      }
    break;
  }
}

void clearReportData(reportData_T *data){
  data->tempDHT = -999;
  data->humidDHT  = -999;
  data->tempBMP = -999;
  data->pressureBMP = -999;
  data->altBMP  = -999;
}

/* Converts an integer value to its hex character*/
char to_hex(char code) {
  static char hex[] = "0123456789ABCDEF";
  return hex[code & 15];
}

String url_encode(String str) {
  char buf;
  int len = str.length(), i;
  String result = "";
  for(i = 0; i < len; i++){
    buf = str[i];
    if(isAlphaNumeric(buf) || buf == '-' || buf == '_' || buf == '.' || buf == '~'){ 
      result += buf;
    }else if (buf == ' '){
      result += '+';
    }else{
      result += '%';
      result += to_hex(buf >> 4);
      result += to_hex(buf & 15);
    }
  }
  return result;
}

bool sendReport(reportData_T *data){
  String report = "";
  uint8_t resultBuffer[RESULT_LEN_SIZE] = {0};
  uint32_t resultLen, i;
  int pos;

  if(!wifi.kick() || wifiState != 9) return false;
  if(year() != 0){
    report += "datetime: \""+String(day())+"/"+String(month())+"/"+String(year())+" ";
    report += String(hour())+":"+String(minute())+":"+String(second())+"\",";
  }
  if(data->tempDHT != -999) report += "temp_dht: "+String(data->tempDHT)+",";
  if(data->humidDHT != -999)  report += "humid_dht: "+String(data->humidDHT)+",";
  if(data->tempBMP != -999) report += "temp_bmp: "+String(data->tempBMP)+",";
  if(data->pressureBMP != -999) report += "pressure_bmp: "+String(data->pressureBMP)+",";
  if(data->altBMP != -999)  report += "alt_bmp: "+String(data->altBMP)+",";

  report = "sensor="+url_encode("{"+report+"}")+"&username="+url_encode(HOST_USERNAME)+"&password="+url_encode(HOST_PASSWORD);
  OLED.LED_P8x16Str(0,0,(char*)String(F("Sending report..")).c_str());
  if(wifi.createTCP(HOST_NAME, 80)) {
    Serial.print("create tcp ok\r\n");
    report = "POST "+HOST_LOCATION+" HTTP/1.1\r\nHost: "+HOST_NAME+"\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: "+(report.length())+"\r\nConnection: close\r\n\r\n"+report+"\r\n";
    wifi.send((const uint8_t*)(report.c_str()), report.length());
    resultLen = wifi.recv(resultBuffer, sizeof(resultBuffer), 10000);
    if(resultLen > 0){
      report = "";
      for(i = 0; i < resultLen; i++){
        report += (char)resultBuffer[i];
      }

      report = report.substring(report.indexOf("\r\n\r\n")+4);
      Serial.println("SERVER->"+report+"("+resultLen+")");
      //enqueueLine("*>"+report+"("+resultLen+")\n");
      pos = report.indexOf("COMMAND:");
      if(pos != -1){
        serverCMD = (report.substring(pos+8)).toInt();
      }
      Serial.print("Server command ->");
      Serial.println(serverCMD, HEX);
      if(serverCMD!= 0)
        enqueueLine("*>CMD="+String(serverCMD, HEX));
      else
        enqueueLine("*>CMD=0");
      pos = report.indexOf("\"unix\":");
      if(pos != -1){
        enqueueLine("*>UNIX "+report.substring(pos+7, pos+17));
      }
      
    }
    if(wifi.releaseTCP()){
      Serial.print("release tcp ok\r\n");
      return true;
    }else{
        Serial.print("release tcp err\r\n");
        return false;
    }
  }else{
      Serial.print("create tcp err\r\n");
      return false;
  }
}

void setup(){
   OLED.LEDPIN_Init(10,9,13,11);
   OLED.LED_Init();
   Serial.begin(115200);
   setSyncProvider(RTC.get);   // the function to get the time from the RTC
    if(timeStatus() != timeSet) 
        Serial.println("Unable to sync with the RTC");
    else
        Serial.println("RTC has set the system time");
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    receiver.channel(80);  // ตั้งช่องความถี่ให้ตรงกัน
    receiver.RXaddress((char*)String("ALL").c_str());  // ตั้งชื่อตำแห่นงให้ตรงกัน ชื่อตั้งได้สูงสุด 5 ตัวอักษร
    receiver.init();
    
    Serial.println("Pressure Sensor Test"); Serial.println("");  
    /* Initialise the sensor */
    if(!bmp.begin()){
      /* There was a problem detecting the BMP085 ... check your connections */
      Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
    /* Initialise the sensor */
    if(!mag.begin()){
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      while(1);
    }
    
    pinMode(MENU_BUTTON, INPUT); 
    changeMenu(0);
}

void loop(){
    int i, j;
    reportData_T sensorData;
    clearReportData(&sensorData);

    if(millis() - lastShowTime >= 1000){
      currTime = (hour() > 10)?(String(hour(), DEC)):("0"+String(hour(), DEC));
      currTime += formatDigit(minute());
      currTime += formatDigit(second());
      currTime += "---";  currTime += (day() > 10)?(String(day(), DEC)):("0"+String(day(), DEC));
      currTime += "/";  currTime += (month() > 10)?(String(month(), DEC)):("0"+String(month(), DEC));
      currTime += "/";  currTime += year();
      OLED.LED_P6x8Str(0,0,(char*)currTime.c_str());
      Serial.println(currTime);
      lastShowTime = millis();
    }

    if(receiver.available()){
      msg = "";
      receiver.read(); // สั่งให้เริ่มอ่าน
      receiver.rxPL(msg); // สั่งใหอ่านเก็บไว้ที่ตัวแปร
      if(msg.length() > 0){
        msg.trim();
        Serial.println("MSG: =>"+msg+String(msg.endsWith("\a\a\a")));
        if(msg.indexOf("***") == 0){
          //Serial.print("----TYPE--:");
          msg = msg.substring(3);
          //Serial.println(msg.substring(3).toInt());
          switch(msg.toInt()){
            case 1: // Temperature
              i = msg.indexOf('|');
              if(i != -1){
                j = msg.indexOf('|', i+1);
                if(j != -1){
                  sensorData.tempDHT = msg.substring(i+1, j).toFloat();
                  sensorData.humidDHT = msg.substring(j+1).toFloat();
                  OLED.LED_P6x8Str(0, 1,(char*)(String("HUD:"+String(sensorData.tempDHT)+"%").c_str()));
                  OLED.LED_P6x8Str(60, 1,(char*)(String("TEMP:"+String(sensorData.humidDHT)+"C").c_str()));
                  Serial.print("Humidity (%): "+String(sensorData.tempDHT));        
                  Serial.println("\t\tTemperature (oC): "+String(sensorData.humidDHT));
                }
              }
            break;
          }
        }else if(msg.endsWith("\a\a\a")){
          if(!isTextJoin){
            bufferReceiveText = "";
            isTextJoin = true;
          }
          bufferReceiveText += msg;          
        }else{
          if(isTextJoin){
            msg = bufferReceiveText+msg;
            msg.replace("\a\a\a", "");
            isTextJoin = false;
          }            
          msg = "->"+msg;
          enqueueLine(msg);
        }
      }
    }
    
    if(Serial.available()) {
      msg = Serial.readString();
      msg.trim();  msg.replace("\n", "");
      if(msg == "CLEAR"){
        clearScreen();     
      }else if(msg == "CONSOLE"){
        changeMenu(0);
      }else if(msg == "TIME"){
        changeMenu(1);
      }else if(msg == "GY651"){
        changeMenu(2);
      }else if(msg == "DISPLAYTEXT"){
        Serial.println("------------> "+displayText);
      }else{
        msg = "=>"+msg;
        enqueueLine(msg);
        Serial.println("MSG Serial :"+msg);
      }
    }

    if(digitalRead(MENU_BUTTON) == HIGH){
      delay(MENU_THRESHOLD);
      if(digitalRead(MENU_BUTTON) == HIGH)
      changeMenu(currMenu+1);
    }

    switch(currMenu){
      case 0:
        if(millis() - lastDisplay >= SCROLL_DELAY){
          printText();
        }
      break;
      case 1:
        if(millis() - lastClock >= 1000){
          i = hour();
          printLargeNumber((int)(i/10)%10, 10, 3);
          printLargeNumber(i%10, 30, 3);
          if(second()%2 == 1){ fillScreenArea(50, 3, 70, 7, 0); }else{ printLargeNumber(10, 50, 3); }
          i = minute();
          printLargeNumber((int)(i/10)%10, 70, 3);
          printLargeNumber(i%10, 90, 3);
          lastClock = millis();
        }
      break;
      case 2:
        if(millis() - lastGY651 >= 500){
          bmp.getSensor(&sensor);
          OLED.LED_P8x16Str(0,2,(char*)sensor.name);
          bmp.getEvent(&event); 
          /* Display the results (barometric pressure is measure in hPa) */
          if (event.pressure){
            /* Display atmospheric pressue in hPa */
            Serial.print("Pressure:    ");
            Serial.print(event.pressure);
            Serial.println(" hPa");
            
            /* Calculating altitude with reasonable accuracy requires pressure    *
             * sea level pressure for your position at the moment the data is     *
             * converted, as well as the ambient temperature in degress           *
             * celcius.  If you don't have these values, a 'generic' value of     *
             * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
             * in sensors.h), but this isn't ideal and will give variable         *
             * results from one day to the next.                                  *
             *                                                                    *
             * You can usually find the current SLP value by looking at weather   *
             * websites or from environmental information centers near any major  *
             * airport.                                                           *
             *                                                                    *
             * For example, for Paris, France you can check the current mean      *
             * pressure and sea level at: http://bit.ly/16Au8ol                   */
             
            /* First we get the current temperature from the BMP085 */
            float temperature;
            bmp.getTemperature(&temperature);
            Serial.print("Temperature: ");
            Serial.print(temperature);
            Serial.println(" C");
        
            /* Then convert the atmospheric pressure, and SLP to altitude         */
            /* Update this next line with the current SLP for better results      */
            float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
            Serial.print("Altitude:    "); 
            Serial.print(bmp.pressureToAltitude(seaLevelPressure,
                                                event.pressure)); 
            Serial.println(" m");
            OLED.LED_P6x8Str(0, 4, (char*)String(String(event.pressure)+"hPa").c_str());
            OLED.LED_P6x8Str(0, 5, (char*)String(String(temperature)+" C").c_str());
            OLED.LED_P6x8Str(0, 6, (char*)String(String(bmp.pressureToAltitude(seaLevelPressure,event.pressure))+" m").c_str());

            sensorData.tempBMP = temperature;
            sensorData.pressureBMP = event.pressure;
            sensorData.altBMP = bmp.pressureToAltitude(seaLevelPressure, event.pressure);
          }else{
            OLED.LED_P8x16Str(0, 5, (char*)String(F(" ERROR ")).c_str());
          }

          mag.getSensor(&sensor);
          mag.getEvent(&event);
          OLED.LED_P8x16Str(65,2,(char*)sensor.name);
          fillScreenArea(65, 4, 66, 7, 0xFF);
          /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
          Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
          Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
          Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
          Serial.print("Heading (degrees): "); Serial.println(headingDegree(event.magnetic.x, event.magnetic.y));
          OLED.LED_P6x8Str(67, 4, (char*)String("X:"+String(event.magnetic.x)).c_str());
          OLED.LED_P6x8Str(67, 5, (char*)String("Y:"+String(event.magnetic.y)).c_str());
          OLED.LED_P6x8Str(67, 6, (char*)String("Z:"+String(event.magnetic.z)+"uT").c_str());
          OLED.LED_P6x8Str(67, 7, (char*)String("H:"+String(headingDegree(event.magnetic.x, event.magnetic.y))).c_str());

          
          lastGY651 = millis();
        }
      break;
    }

    if(lastShowMenu != 0 && currMenu != 1 && millis() - lastShowMenu > 1000){
      fillScreenArea(0, 4, 125, 5, 0);
      lastShowMenu = 0;
    }

    
    if(millis() - lastWiFISend > 5000){
      if(wifiState != 9)
        connectWIFI();
      else
        sendReport(&sensorData);
      lastWiFISend = millis();
    }
    

    //delay(300);
    //for(int i = 1; i < 8; i++) OLED.LED_P6x8Str(0, i, (char*)msg.c_str());
    
    //OLED.LED_P8x16Str(0,5,"u");
    //while(1);
}
