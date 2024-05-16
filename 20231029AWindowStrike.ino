/*Info about this code
PROGRAM NAME: Window Streak

PROGRAMMER: Simão Pedrosa

DATE: 2023

HARDWARE NEEDED:
- 1x ESP32
- 1x PCF8575
- 4x MAX31856
- 4x PT100
- Host for Home Assistant - PC or Raspery Pi

DESCRIPTION:

LABEL's:
//(HA)-Information from Home Assistant
//NTP - Network Time Protocol
//GMT - Greenwich Mean Time
//VExtUp - Vent in Exterior side, in the top of the window
//VExtDw - Vent in Exterior side, in the bottom of the window
//VIntUp - Vent in Interior side, in the top of the window
//VIntDw - Vent in Interior side, in the bottom of the window
//TExt - Temperature value from the exterior
//TInt - Temperature value from the interior of the space/room
//TWUp - Temperature value from the interior of the window in the top
//TWDw - Temperature value from the interior of the window in the bottom
//WSpcm - WindowShutter with PCM side for exterior
//WSClose - WindowShutter totally down
//SimMode - Simulated temperatures from HA
//ManMode - Manual Mode
//SimTxxx - Simulated temperature value
//M_ - Manual Mode variable
//A_ - Automatic Mode variable (not Manual Mode)
//B_ - Digital button related variable
//L_ - Led type variable


/*Libraries*/
#include <WiFi.h> //Wifi
#include <SPI.h> //SPI
#include <Adafruit_MAX31865.h> //MAX31856
#include <PubSubClient.h> //MQTT
#include <ArduinoJson.h> //JSON
#include <string.h> //String manipulation
#include <PCF8575.h> //PCF control
#include <time.h> //Date and Time from NTP Server (WiFi provided)
#include <FS.h>//Handle files
#include <SD.h>//SD Card Interface

/****************************************************************************Constants setup*/
/*Wifi constants*/
/*Casa*/
const char* ssid = "";//Wifi Name - Service Set IDentifier [Simon's Home]
const char* password = "";//Wifi Password [Simon's Home]
/*Hotspot/
const char* ssid = "SimaoPedro";
const char* password = "simaopedro";
/*Leiria/
const char* ssid = "";
const char* password = "";

/*MQTT constants*/
const char* MQTTBroker = "192.168.1.159";//MQQTBroker [Torre]
//const char* MQTTBroker = "192.168.1.169";//MQQTBroker [Portatil]
const char* MQTTPassword = "mqtt12345";//MQTTPassword
const char* MQTTUser = "mqttbroker";//MQTTUser
//#define MQTT_VERSION MQTT_VERSION_3_1_1

/*Date and Time constants*/
const char* NTPServer="pool.ntp.org"; //cluster of timeservers
const long GMTplus=0; //Time zone offset (GMT) in seconds. Portugal=0
const int DayLightOffset=3600; //Offset in seconds for daylight saving time. Generally 1h.

/*Manual constants*/
uint8_t ManMode=0;//Manual Mode (0/1)
uint8_t M_VExtUp;//Manual VExtUp (0/1)
uint8_t M_VExtDw;//Manual VExtDw (0/1)
uint8_t M_VIntUp;//Manual VIntUp (0/1)
uint8_t M_VIntDw;//Manual VIntDw (0/1)
uint8_t M_WSClose;//Manual WSClose (0/1)
uint8_t M_WSpcm;//Manual WSpcm (0/1)

/*HA Manual constants*/
String HA_ManMode;//Manual Mode (on/off) (HA)
String HA_VExtUp;//Manual VExtUp (on/off) (HA)
String HA_VExtDw;//Manual VExtDw (on/off) (HA)
String HA_VIntUp;//Manual VIntUp (on/off) (HA)
String HA_VIntDw;//Manual VIntDw (on/off) (HA)
String HA_WSClose;//Manual WSClose (on/off) (HA)
String HA_WSpcm;//Manual WSpcm (on/off) (HA)

/*Automatic constants*/
uint8_t A_VExtUp;//Automatic VExtUp (0/1)
uint8_t A_VExtDw;//Automatic VExtDw (0/1)
uint8_t A_VIntUp;//Automatic VIntUp (0/1)
uint8_t A_VIntDw;//Automatic VIntDw (0/1)
uint8_t A_WSClose;//Automatic WSClose (0/1)
uint8_t A_WSpcm;//Automatic WSpcm (0/1)

/*Button constants*/
uint8_t B_VExtUp;//Button VExtUp (0/1)
uint8_t B_VExtDw;//Button VExtDw (0/1)
uint8_t B_VIntUp;//Button VIntUp (0/1)
uint8_t B_VIntDw;//Button VIntDw (0/1)
uint8_t B_WSClose;//ButtonWSClose (0/1)
uint8_t B_WSpcm;//Button WSpcm (0/1)
uint8_t B_ManMode;//Button ManMode (0/1)

/*Status constants*/
uint8_t VExtUp;//Value (0/1)
uint8_t VExtDw;//Value (0/1)
uint8_t VIntUp;//Value (0/1)
uint8_t VIntDw;//Value (0/1)
uint8_t WSClose;//Value (0/1)
uint8_t WSpcm;//Value (0/1)

/*Sun constants*/
String SunInfo;//above_horizon or below_horizon
uint8_t Sun; //Value (0/1)

/*Occupation constants*/
String HA_Occupation;//Occupation of the room (0/1) (HA)
uint8_t Occupation=0;//Occupation of the room (0/1) ESP32

/*Temperature sensor initialize*/
float STInt_value;//Sensor info (float)
float STExt_value;//Sensor info (float)
float STWUp_value;//Sensor info (float)
float STWDw_value;//Sensor info (float)

/*Error MAX31856 initialize*/
uint8_t Fault_STInt;
uint8_t Fault_STExt;
uint8_t Fault_STWUp;
uint8_t Fault_STWDw;
float S_valueError=100;

/*Temperature constant*/
float TInt;//Value used (from sensor or simulation)
float TExt;//Value used (from sensor or simulation)
float TWUp;//Value used (from sensor or simulation)
float TWDw;//Value used (from sensor or simulation)
float TConf=20;//Value (Default or from HA)
float Tpcm=50;//Value (Default or from HA)

/*Simulation constants*/
String HA_SimMode;//Simulation Mode (0/1) (HA)
uint8_t SimMode=0;//Simulation Mode (0/1) ESP32
float SimTInt;//Simulated TInt (number) (HA)
float SimTExt;//Simulated TExt (number) (HA)
float SimTWDw;//Simulated TWDw (number) (HA)
float SimTWUp;//Simulated TWUp (number) (HA)

/*SPI Pinouts*/
#define MOSI_VSPI 23
#define MISO_VSPI 19
#define SCK_VSPI 18

#define MOSI_HSPI 27
#define MISO_HSPI 26
#define SCK_HSPI 25

//CS its the same that SS
#define STInt_Pin 14
#define STExt_Pin 4
#define STWUp_Pin 32
#define STWDw_Pin 33

//CS for SDCard
#define SDCard_Pin 5

//Configuration of SPI
/*MAX31865 Instances*/
Adafruit_MAX31865 STExt_Sensor=Adafruit_MAX31865(STExt_Pin, MOSI_HSPI, MISO_HSPI, SCK_HSPI); 
Adafruit_MAX31865 STInt_Sensor=Adafruit_MAX31865(STInt_Pin, MOSI_HSPI, MISO_HSPI, SCK_HSPI); 
///Adafruit_MAX31865 STWUp_Sensor=Adafruit_MAX31865(STWUp_Pin, MOSI_VSPI, MISO_VSPI, SCK_VSPI);
///Adafruit_MAX31865 STWDw_Sensor=Adafruit_MAX31865(STWDw_Pin, MOSI_VSPI, MISO_VSPI, SCK_VSPI);

/*SDCard*/
//SPIClass SDCard_SPI=SPIClass(HSPI);
SPIClass SDCard_SPI=SPIClass(VSPI); 

/*PCF8575 address*/
PCF8575 pcf(0x20);

/*SD Card variables*/
uint8_t cardType=SD.cardType();
const char* filename="/windowstreak.txt";

/*Resistors*/
#define Rref 430.0
//For PT100=430 and PT1000=4300 but always confirm in MAX31856!
#define Rnonimal 101.7
//For PT100=100 and PT1000=1000 but depends if it's 2,3 or 4 wire... And ajust for better values

/*IO's*/
uint8_t VExtUpPin=P0;
uint8_t VExtDwPin=P1;
uint8_t VIntUpPin=P2;
uint8_t VIntDwPin=P3;
uint8_t WSClosePin=P4;
uint8_t WSpcmPin=P5;
//uint8_t=P6;
uint8_t L_ManModePin=P7;
uint8_t L_ErrorPin=P8;
uint8_t B_ManModePin=P9;
uint8_t B_WSpcmPin=P10;
uint8_t B_WSClosePin=P11;
uint8_t B_VExtDwPin=P12;
uint8_t B_VIntDwPin=P13;
uint8_t B_VExtUpPin=P14;
uint8_t B_VIntUpPin=P15;






/*System String/
String with all info about:
- Date and Time
- Simulation Temperature Mode and Temperature Values
- Manual Mode and I&O's info*/
char aux_sys_date[50];//Date and Time char buff
String sys_info;//String with all info
String sys_date;//Date and Time string
String sys_temp;//Temperature string
String sys_ios;//I&O's string
String sys_occup;//Occupation string
String sys_error;//Error 0/1 string

/*Error Constants*/
uint8_t ERROR=0; //If Error=1 set the Error Led On
uint8_t ConnectionAttempts=2; //Number of attempts to reconnect/connect to wifi and MQTT Broker
uint8_t WifiConnectAttempts=30; //Number of attempts to connect to WiFi
uint8_t MAX31856Error=0; //If isn't 0: Error in MAX31856
uint8_t SDCardError=0; //If 1 Error in SDCard

/*Others*/
uint8_t Antiloop=0; //Run once
uint8_t delaybutton=200; //delay for button's work propelly
unsigned long lastMsg = 0; //Initilization for JSON Document
uint8_t Lifebit=0; //Lifebit toggle

#define Led_Esp32 2//ESP32 Built-in Led

WiFiClient esp32wificlient;//Create client

/**************************************************************************************************************Setup Wifi*/

void setupWifi(){

Serial.print("Wifi setup in progress...");
//Serial.print("2");

  //Initializes Wifi
  Serial.print("\nConnecting to wifi ");Serial.print(ssid);
  WiFi.begin(ssid, password);//Wifi begin

  //While waiting for connection
  while (WiFi.status()!=WL_CONNECTED and WifiConnectAttempts>0) {
    //While server is waiting for connection
    delay(1000);
    Serial.print(".");
    WifiConnectAttempts=WifiConnectAttempts-1;
  }

  //When is connected
  if(WiFi.status()==WL_CONNECTED){
  Serial.print("\nConnected to wifi ");Serial.println(ssid);
  }
}//void setupWifi()

/***************************************************************************************************************MQTTCallback*/

//Received messages by MQTT
void MQTTCallback(char* topic, byte* payload, unsigned int lenght){

//Serial.print("4");

//Conversion of Payload Byte to String
payload[lenght]=0;
String rcvpayload=String((char*)payload);
 
  //Save the Payload in right variable

  //home/ha/sw1/temp
  if(strcmp(topic,"home/ha/sw1/temp/simstatus")==0){//Comparation of the two string
    HA_SimMode=rcvpayload;//Write the payload in the variable
    if(HA_SimMode=="on"){SimMode=1;} else{SimMode=0;}
    Serial.print("SimMode: ");Serial.println(HA_SimMode);//Print the value
  }
  else if(strcmp(topic,"home/ha/sw1/temp/simtint")==0){
    SimTInt=rcvpayload.toFloat();
    Serial.print("SimTInt: ");Serial.println(SimTInt);
  }
  else if(strcmp(topic,"home/ha/sw1/temp/simtext")==0){
    SimTExt=rcvpayload.toFloat();
    Serial.print("SimTExt: ");Serial.println(SimTExt);
  }
  else  if(strcmp(topic,"home/ha/sw1/temp/simtwup")==0){
    SimTWUp=rcvpayload.toFloat();
    Serial.print("SimTWUp: ");Serial.println(SimTWUp);
  }
  else if(strcmp(topic,"home/ha/sw1/temp/simtwdw")==0){
    SimTWDw=rcvpayload.toFloat();
    Serial.print("SimTWDw: ");Serial.println(SimTWDw);
  }

  //home/ha/sw1/vent
  else if(strcmp(topic,"home/ha/sw1/vent/manual")==0){
    HA_ManMode=rcvpayload;
    Serial.print("HA_ManMode: ");Serial.println(HA_ManMode);
  }
  else if(strcmp(topic,"home/ha/sw1/vent/vextup")==0){
    HA_VExtUp=rcvpayload;
    Serial.print("HA_VExtUp: ");Serial.println(HA_VExtUp);
  }
  else if(strcmp(topic,"home/ha/sw1/vent/vextdw")==0){
    HA_VExtDw=rcvpayload;
    Serial.print("HA_VExtDw: ");Serial.println(HA_VExtDw);  
  }
  else if(strcmp(topic,"home/ha/sw1/vent/vintup")==0){
    HA_VIntUp=rcvpayload;
    Serial.print("HA_IntUp: ");Serial.println(HA_VIntUp);  
  }
  else if(strcmp(topic,"home/ha/sw1/vent/vintdw")==0){
    HA_VIntDw=rcvpayload;
    Serial.print("HA_IntDw: ");Serial.println(HA_VIntDw);  
  }

  //home/ha/sw1/wshutter
  else if(strcmp(topic,"home/ha/sw1/wshutter/wsclose")==0){
    HA_WSClose=rcvpayload;
    Serial.print("HA_WSClose: ");Serial.println(HA_WSClose);
  }
  else if(strcmp(topic,"home/ha/sw1/wshutter/wspcm")==0){
    HA_WSpcm=rcvpayload;
    Serial.print("HA_WSpcm: ");Serial.println(HA_WSpcm);
  }

  //home/ha/sw1/occupation
  else if(strcmp(topic,"home/ha/sw1/occupation")==0){
    HA_Occupation=rcvpayload;
    if(HA_Occupation=="on"){Occupation=1;} else{Occupation=0;}
    Serial.print("Occupation: ");Serial.println(HA_Occupation);
  } 

  //home/ha/settings
  else if(strcmp(topic,"home/ha/settings/temp/tconf")==0){
    TConf=rcvpayload.toFloat();
    Serial.print("TConf: ");Serial.println(TConf);
  }
  else if(strcmp(topic,"home/ha/settings/temp/tpcm")==0){
    Tpcm=rcvpayload.toFloat();
    Serial.print("TPCM: ");Serial.println(Tpcm);
  }

  //home/ha/sun
  else if(strcmp(topic,"home/ha/sun")==0){
    SunInfo=rcvpayload;
    Serial.println(SunInfo);
    if(SunInfo="above_horizon"){Sun=1;}
    else if(SunInfo="below_horizon"){Sun=0;}
    //Serial.println(Sun);
  }
}//void MQTTCallback

PubSubClient mqttclient(MQTTBroker,1833,MQTTCallback,esp32wificlient);//Partially initialised client instant

/*********************************************************************************Setup MQTT*/

//Connect and reconnect MQTT
void setupMQTT(){

Serial.println("MQTT Setup in progress...");
Serial.print("C");

  while(!mqttclient.connected()){
    Serial.print("Connecting to MQQT Broker ");
    Serial.println(MQTTBroker);

    //If connected
    if(mqttclient.connect("mqttbroker",MQTTUser,MQTTPassword)){
      Serial.print("Connected to MQQT Broker ");Serial.println(MQTTBroker);

      /*Listen Topics*/

      //Wildcards (Makes ESP32 too slow)
      //client.subscribe("/home/ha/sw1/#");
      //client.subscribe("/home/ha/settings/temp/#");
      
      //home/ha/sw1/temp
      mqttclient.subscribe("home/ha/sw1/temp/simtint");
      mqttclient.subscribe("home/ha/sw1/temp/simtext");
      mqttclient.subscribe("home/ha/sw1/temp/simtwup");
      mqttclient.subscribe("home/ha/sw1/temp/simtwdw");
      mqttclient.subscribe("home/ha/sw1/temp/simstatus");

      //home/ha/sw1/vent
      mqttclient.subscribe("home/ha/sw1/vent/manual");
      mqttclient.subscribe("home/ha/sw1/vent/vextup");
      mqttclient.subscribe("home/ha/sw1/vent/vextdw");
      mqttclient.subscribe("home/ha/sw1/vent/vintup");
      mqttclient.subscribe("home/ha/sw1/vent/vintdw");

      //home/ha/sw1/wshutter
      mqttclient.subscribe("home/ha/sw1/wshutter/wsclose");
      mqttclient.subscribe("home/ha/sw1/wshutter/wspcm");

      //home/ha/sw/occupation
      mqttclient.subscribe("home/ha/sw1/occupation");

      //home/ha/settings
      mqttclient.subscribe("home/ha/settings/temp/tconf");
      mqttclient.subscribe("home/ha/settings/temp/tpcm"); 

      //home/ha/sun
      mqttclient.subscribe("home/ha/sun");
      
      Serial.println("MQTT Setup completed!");
    }

    //If not connected
    else if(ConnectionAttempts>0){
      ConnectionAttempts=ConnectionAttempts-1; //Counting

      /*While waiting for connection*/
      Serial.print("Trying connecting again. Attempts left:");
      Serial.println(ConnectionAttempts);
      //break;//Exit cycle and try again
    }

    //If connection fails (while and void exit)
    else{Serial.println("MQTT Connection not possible!");break;}

}//while
}//setupMQTT()

/*************************************************************************************SDCard Write*/

void SDWrite(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n",path);
  File file=fs.open(path,FILE_WRITE);
  if(!file){Serial.println("Open file for writing failed!"); return;}

  if(file.print(message)){Serial.println("File written.");}
  else{Serial.println("Write failed!");}

  file.close();
  Serial.println("SD Card setup completed!");
}//SDWrite

/*************************************************************************************SDCard AddInfo*/

void SDAddInfo(fs::FS &fs, const char * path, const char * message){
  File file=fs.open(path,FILE_APPEND);
  if(!file){Serial.println("Open file for append failed!");SDCardError=1;}//return;}
  if(!file.print(message)){Serial.println("Append info in SD file failed!");SDCardError=1;}
}

/******************************************************************************************SD Setup*/
//Setup process for SDCard
void setupSDCard(){

//Serial.print("7");

  if(!SD.begin()){
    //If SD Card not initalize
    Serial.println("SD Card not initilize!");
    SDCardError=1;
    
  }
  else{Serial.println("SD Card setup in progress...");}
  
  //Serial.print("SD Card Type: ");
  switch(cardType){
    case 'CARD_NONE': Serial.println("SD Card not attached!"); return;
    case 'CARD_MMC': Serial.println("MMC");
    case 'CARD_SD': Serial.println("SD");
    case 'CARD_SDHC': Serial.println("SDHC");
    //default: Serial.print("Unkown"); //Serial.println(cardType);
  }//switch(cardType)

  if(SD.begin()){
  File file=SD.open(filename);
  if(!file){
    Serial.print(filename);Serial.println(" don't exist. Creating file!");
    SDWrite(SD,filename, "Date;Time;SimMode;TExt;TInt;TWUp;TWDw;Tconf;Tpcm;ManMode;VExtUp;VExtDw;VIntUp;VIntDw;WSClose;WSpcm;Occupation;Error \r\n");
    file.close();}

  }//if(SD.begin())

}//void setupSDCard()

/**************************************************************************************ReadSTExt*/

//Temperatura measurement cycle
void ReadSTExt(){
digitalWrite(STExt_Pin,LOW);
digitalWrite(STInt_Pin,HIGH);
digitalWrite(STWUp_Pin,HIGH);
digitalWrite(STWDw_Pin,HIGH);
STExt_Sensor.begin(MAX31865_2WIRE);
//other options: 3WIRE and 4WIRE. The MAX31856 solded config change to! Depend of PT100/PT1000

//Read
STExt_value=STExt_Sensor.temperature(Rnonimal,Rref);
//Serial.print("STExt=");Serial.println(STExt_value);

  /*Errors/
  Fault_STInt=STInt_Sensor.readFault();
  if(Fault_STInt or STInt_value>S_valueError){Serial.println("Error in STInt Sensor");MAX31856Error=1;}
  //digitalWrite(STInt_Pin,HIGH);
  /**/
digitalWrite(STExt_Pin,HIGH);

}//void ReadSTExt()

/**************************************************************************************ReadSTInt*/

//Temperatura measurement cycle
void ReadSTInt(){
digitalWrite(STInt_Pin,LOW);
digitalWrite(STExt_Pin,HIGH);
digitalWrite(STWUp_Pin,HIGH);
digitalWrite(STWDw_Pin,HIGH);
STInt_Sensor.begin(MAX31865_2WIRE);
//other options: 3WIRE and 4WIRE. The MAX31856 solded config change to! Depend of PT100/PT1000

//Read
STInt_value=STInt_Sensor.temperature(Rnonimal,Rref);
//Serial.print("STInt=");Serial.println(STInt_value);

  /*Errors/
  Fault_STInt=STInt_Sensor.readFault();
  if(Fault_STInt or STInt_value>S_valueError){Serial.println("Error in STInt Sensor");MAX31856Error=1;}
  //digitalWrite(STInt_Pin,HIGH);
  /**/
digitalWrite(STInt_Pin,HIGH);

}//void ReadSTInt()

/**************************************************************************************Setup*/

//Setup up of ESP32 and all components
void setup(){

delay(10000);//Time to Serial monitor starts

Serial.begin(115200);//Data rate in bits

//Serial.println("ESP32 setup in progress...");
//Serial.print("1");

  /*Wifi*/
  setupWifi();

//If Wi-Fi is connected
if(WiFi.status()==WL_CONNECTED){

  /*Configuration of Date and Time*/
  configTime(GMTplus,DayLightOffset,NTPServer);
  //Serial.print("3");

  /*Broker*/
  mqttclient.setServer(MQTTBroker,1883);
  //Serial.print("4");
  mqttclient.setCallback(MQTTCallback);
  //Serial.print("5");

}//wifi connected

  /*ESP32 Pins*/
  pinMode(Led_Esp32, OUTPUT);

  /*SPI SD Card Start*/
  //digitalWrite(SDCard_Pin,LOW);
  //SDCard_SPI.begin(SCK_HSPI,MISO_HSPI,MOSI_HSPI,SDCard_Pin);
  SDCard_SPI.begin(SCK_VSPI,MISO_VSPI,MOSI_VSPI,SDCard_Pin);
  //Serial.print("6");
  setupSDCard();
  //digitalWrite(SDCard_Pin,HIGH);

  /*PCF8575 setup*/
  pcf.pinMode(VExtUpPin,OUTPUT);
  pcf.pinMode(VExtDwPin,OUTPUT);
  pcf.pinMode(VIntUpPin,OUTPUT);
  pcf.pinMode(VIntDwPin,OUTPUT);
  pcf.pinMode(WSClosePin,OUTPUT);
  pcf.pinMode(WSpcmPin,OUTPUT);
  //pcf.PinMode(P6,);
  pcf.pinMode(L_ManModePin,OUTPUT);
  pcf.pinMode(L_ErrorPin,OUTPUT);
  pcf.pinMode(B_ManModePin,INPUT);
  pcf.pinMode(B_VExtUpPin,INPUT);
  pcf.pinMode(B_VExtDwPin,INPUT);
  pcf.pinMode(B_VIntUpPin,INPUT);
  pcf.pinMode(B_VIntDwPin,INPUT);
  pcf.pinMode(B_WSClosePin,INPUT);
  pcf.pinMode(B_WSpcmPin,INPUT);
  //Serial.print("8");

  /*PCF Begin*/
  pcf.begin();
  //Serial.println("9");

  /*Led's Start Off*/
  pcf.digitalWrite(VExtUpPin, HIGH);
  pcf.digitalWrite(VExtDwPin, HIGH);
  pcf.digitalWrite(VIntUpPin, HIGH);
  pcf.digitalWrite(VIntDwPin, HIGH);
  pcf.digitalWrite(WSClosePin, HIGH);
  pcf.digitalWrite(WSpcmPin, HIGH);
  pcf.digitalWrite(L_ManModePin, HIGH);
  pcf.digitalWrite(L_ErrorPin, HIGH);

Serial.println("ESP32 setup completed!");

/*I2C Pins/
Serial.print("SDA: ");Serial.println(SDA,DEC);
Serial.print("SCL: ");Serial.println(SCL,DEC);*/

/*SPI - VSPI/
It's the way to find the MOSI, MISO, SCK and SS
Serial.print("MOSI: ");Serial.println(MOSI,DEC);//in this case 23
Serial.print("MISO: ");Serial.println(MISO,DEC);//in this case 19
Serial.print("SCK: ");Serial.println(SCK,DEC);//in this case 18
Serial.print("SS: ");Serial.println(SS,DEC);// first one 5*/

}//void setup()

/***************************************************************************************Loop*/

//ESP32 normal loop
void loop() {

//Serial.println("ESP32 in loop");
unsigned long tic=millis();

//Serial.print("A");

/**********************************************************************Wifi - If Disconnected*/
if(WiFi.status()==WL_DISCONNECTED){
  Serial.println("Wifi connection lost");
  WifiConnectAttempts=WifiConnectAttempts-1;
  setupWifi();
}

/***********************************************************************MQTT Connection Loop*/

/*MQTT - Connection and Reconnection and possible error's*/
while(ConnectionAttempts>0){

//Life bit
  if(Lifebit==0){mqttclient.publish("home/esp32/sw1/lifebit","0");Lifebit=1;}
  else if(Lifebit==1){mqttclient.publish("home/esp32/sw1/lifebit","1");Lifebit=0;}

//Serial.print("B");
  //Serial.println(client.state());

  /*Possible errors*/
  switch(mqttclient.state()){
    case -1: Serial.println("MQTT Client - Not connected");break;
    case 1: Serial.println("MQTT Server - Bad Protocol");break;
    case 2: Serial.println("MQTT Server - Bad Client ID");break;
    case 3: Serial.println("MQTT Server - Unavailable");break;
    case 4: Serial.println("MQTT Server - Bad Credential");break;
    case 5: Serial.println("MQTT Server - Client unauthorized");break;
    case -2: Serial.println("MQTT Network - Connection failed");break;
    case -3: Serial.println("MQTT Network - Connection lost");break;
    case -4: Serial.println("MQTT Server - Timeout");break;
  }
  /**/

  //If is not connected, try connect  
  if(!mqttclient.connected()){
    setupMQTT();//Serial.print("C");
  }

else{break;}//Exit while
}//while

//Request info to HomeAssistant when ESP32 starts
if(mqttclient.connected()==true and Antiloop==0){
mqttclient.publish("home/esp32/sw1/request/info","1");
Serial.println("Info requested to Home Assistant");
Antiloop=1;}
/*Info request:
    - Simulation Mode and Temperatures      
    - Sun
    - Occupation
*/

/********************************************************************************Temperature*/

//ReadSTInt();//Serial.print("D");
ReadSTExt();
ReadSTInt();

/*Temperature analises*/
//Simulation Mode On: Temperature give by Home Assistant
if(SimMode==1){
  TInt=SimTInt;
  TExt=SimTExt;
  TWUp=SimTWUp;
  TWDw=SimTWDw;
}
//Simulation Mode Off: Temperature give by Max31685
else if (SimMode==0){
  TInt=STInt_value;
  TExt=STExt_value;
  TWUp=21;//STWUp_value;
  TWDw=22;//STWDw_value;
}

/**********************************************************************MQTT Temperature JSON*/

//Serial.print("E");

/*Print temperature information in json*/
StaticJsonDocument<200> tempdoc;
char TempDoc[200];
long now = millis(); //Delay between sending message
if (now-lastMsg>300){

  //Add variables to JSON Document
  tempdoc["tint"]=TInt;
  tempdoc["text"]=TExt;
  tempdoc["twup"]=TWUp;
  tempdoc["twdw"]=TWDw;
  tempdoc["tconf"]=TConf;
  tempdoc["tpcm"]=Tpcm;

  //Serialise JSON and send Document
  serializeJson(tempdoc,TempDoc);
  mqttclient.publish("home/esp32/sw1/temp",TempDoc);
}//if

/***********************************************************************************Button's*/

//Serial.print("F");

/*Digital Button's*/
  //Read and Save
    if(pcf.digitalRead(B_VExtUpPin)==HIGH){B_VExtUp=1;delay(delaybutton);}
    if(pcf.digitalRead(B_VExtDwPin)==HIGH){B_VExtDw=1;delay(delaybutton);}
    if(pcf.digitalRead(B_VIntUpPin)==HIGH){B_VIntUp=1;delay(delaybutton);}
    if(pcf.digitalRead(B_VIntDwPin)==HIGH){B_VIntDw=1;delay(delaybutton);}
    if(pcf.digitalRead(B_WSClosePin)==HIGH){B_WSClose=1;delay(delaybutton);}
    if(pcf.digitalRead(B_WSpcmPin)==HIGH){B_WSpcm=1;delay(delaybutton);}
    if(pcf.digitalRead(B_ManModePin)==HIGH){B_ManMode=1;delay(delaybutton);}

/*Digital Button's and HA Buttons*/
//ManMode
if(HA_ManMode=="request" or B_ManMode==1){
  //Reset
  HA_ManMode="off";B_ManMode=0;
  //Toggle
  ManMode=!ManMode;
}
//M_VExtUp
if(HA_VExtUp=="request" or B_VExtUp==1){
  HA_VExtUp="off";B_VExtUp=0;
  M_VExtUp=!M_VExtUp;
}
//M_VExtDw
if(HA_VExtDw=="request" or B_VExtDw==1){
  HA_VExtDw="off";B_VExtDw=0;
  M_VExtDw=!M_VExtDw;
}
//M_VIntUp
if(HA_VIntUp=="request" or B_VIntUp==1){
  HA_VIntUp="off";B_VIntUp=0;
  M_VIntUp=!M_VIntUp;
}
//M_VIntDw
if(HA_VIntDw=="request" or B_VIntDw==1){
  HA_VIntDw="off";B_VIntDw=0;
  M_VIntDw=!M_VIntDw;
}
//M_WSClose
if(HA_WSClose=="request" or B_WSClose==1){
  HA_WSClose="off";B_WSClose=0;
  M_WSClose=!M_WSClose;
}
//M_WSpcm
if(HA_WSpcm=="request" or B_WSpcm==1){
  HA_WSpcm="off";B_WSpcm=0;
  M_WSpcm=!M_WSpcm;
}

/**********************************************************************************Auto Mode*/

//Serial.print("G");

if(ManMode==0){
  /*Temperature logic*/
    //Aquecimento pela janela
      if(TInt-TConf<0 and TExt-TInt<=0 and TWDw-TInt>0){
      A_VExtUp=0;A_VExtDw=0;A_VIntUp=1;A_VIntDw=1;
      //Serial.println("Aquec. janela");
      }
    //Aquecimento pelo exterior
      else if(TInt-TConf<0 and TExt-TInt>0){
      A_VExtUp=1;A_VExtDw=0;A_VIntUp=0;A_VIntDw=1;
      //Serial.println("Aquec. exterior");
      }
    //Arrefecimento pelo exterior
      else if(TInt-TConf>0 and TExt-TInt<0){
      A_VExtUp=0;A_VExtDw=1;A_VIntUp=1;A_VIntDw=0;
      //Serial.println("Arref. exterior");
      }
    //Arrefecimento pela janela
      else if(TInt-TConf>0 and TExt-TInt>=0 and TWDw-TInt<0){
      A_VExtUp=0;A_VExtDw=0;A_VIntUp=1;A_VIntDw=1;
      //Serial.println("Arref. janela");
      }
    //Recirculação
      else if(abs(TInt-TConf)<1 and abs(TExt-TInt)<1){
      A_VExtUp=1;A_VExtDw=1;A_VIntUp=1;A_VIntDw=1;
      //Serial.println("Recirculação");
      }
    //Fechado
      else{A_VExtUp=0;A_VExtDw=0;A_VIntUp=0;A_VIntDw=0;}
  
  /*Window Shutter logic*/
    //Open/Close
      if(Sun==0){A_WSClose=1;}
      else if(Sun==1 and Occupation==1){A_WSClose=0;}
      else if(Sun==1 and Occupation==0){A_WSClose=1;}
    //Side
      if(TWDw>Tpcm){A_WSpcm=0;}
      else {A_WSpcm=1;}

}//if ManMode==off

/***********************************************************************************Unit Map*/

//Serial.print("H");

  //VExtUp
    if(ManMode==1 and M_VExtUp==1){VExtUp=1;}
    else if(ManMode==0 and A_VExtUp==1){VExtUp=1;}
    else {VExtUp=0;}  
  //VExtDw
    if(ManMode==1 and M_VExtDw==1){VExtDw=1;}
    else if(ManMode==0 and A_VExtDw==1){VExtDw=1;}
    else{VExtDw=0;}
  //VintUp
    if(ManMode==1 and M_VIntUp==1){VIntUp=1;}
    else if(ManMode==0 and A_VIntUp==1){VIntUp=1;}
    else{VIntUp=0;}
  //VintDw
    if(ManMode==1 and M_VIntDw==1){VIntDw=1;}
    else if(ManMode==0 and A_VIntDw==1){VIntDw=1;}
    else{VIntDw=0;}
  //WSClose
    if(ManMode==1 and M_WSClose==1){WSClose=1;}
    else if(ManMode==0 and A_WSClose==1){WSClose=1;}
    else{WSClose=0;}
  //WSpcm
    if(ManMode==1 and M_WSpcm==1){WSpcm=1;}
    else if(ManMode==0 and A_WSpcm==1){WSpcm=1;}
    else{WSpcm=0;}

/****************************************************************************IO's Activation*/
  //VExtUp
    if(VExtUp==1){pcf.digitalWrite(VExtUpPin, LOW);}
    if(VExtUp==0){pcf.digitalWrite(VExtUpPin, HIGH);}
  //VExtDw
    if(VExtDw==1){pcf.digitalWrite(VExtDwPin, LOW);}
    if(VExtDw==0){pcf.digitalWrite(VExtDwPin, HIGH);}
  //VintUp
    if(VIntUp==1){pcf.digitalWrite(VIntUpPin, LOW);}
    if(VIntUp==0){pcf.digitalWrite(VIntUpPin, HIGH);}
  //VintDw
    if(VIntDw==1){pcf.digitalWrite(VIntDwPin, LOW);}
    if(VIntDw==0){pcf.digitalWrite(VIntDwPin, HIGH);}
  //WSClose
    if(WSClose==1){pcf.digitalWrite(WSClosePin, LOW);}
    if(WSClose==0){pcf.digitalWrite(WSClosePin, HIGH);}
  //WSpcm
    if(WSpcm==1){pcf.digitalWrite(WSpcmPin, LOW);}
    if(WSpcm==0){pcf.digitalWrite(WSpcmPin, HIGH);}
  //Led Error
    if(ERROR==1){pcf.digitalWrite(L_ErrorPin, LOW);}
  //ManModeLed
    if(ManMode==1){pcf.digitalWrite(L_ManModePin,LOW);}
    if(ManMode==0){pcf.digitalWrite(L_ManModePin,HIGH);}


/*****************************************************************************MQTT IO's JSON*/

/*Print status information in json*/
StaticJsonDocument<200> statusdoc;
char StatusDoc[200];
now = millis(); //Delay between sending message
if(now-lastMsg>300){

  //Add variables to JSON Document
  if(VExtUp==1){statusdoc["vextup"]="on";}
  else{statusdoc["vextup"]="off";}
  if(VExtDw==1){statusdoc["vextdw"]="on";}
  else{statusdoc["vextdw"]="off";}
  if(VIntUp==1){statusdoc["vintup"]="on";}
  else{statusdoc["vintup"]="off";}
  if(VIntDw==1){statusdoc["vintdw"]="on";}
  else{statusdoc["vintdw"]="off";}
  if(WSClose==1){statusdoc["wsclose"]="on";}
  else{statusdoc["wsclose"]="off";}
  if(WSpcm==1){statusdoc["wspcm"]="on";}
  else{statusdoc["wspcm"]="off";}
  if(ManMode==1){statusdoc["manmode"]="on";}
  else{statusdoc["manmode"]="off";}
  if(SimMode==1){statusdoc["simmode"]="on";}
  else{statusdoc["simmode"]="off";}

  //Serialise JSON and send Document
  serializeJson(statusdoc,StatusDoc);
  mqttclient.publish("home/esp32/sw1/status",StatusDoc);
}



/*************************************************************************************Errors*/
/*MQTT Error's*/
if(ConnectionAttempts==0){Serial.println("ERROR: MQTT Connection failed!");ERROR=1;}
//For MQTT Error's check MQTT Connection Loop

/*Wifi Error's*/
if(WifiConnectAttempts==0){Serial.println("ERROR: Wifi Connection failed!");ERROR=1;}
//Info print in Wifi Setup


mqttclient.loop();//MQTT Client Loop - Keep Connection


Time();//Time void call


/************************************************************************************Code time*/
//Time between the start and end of the void (loop)
unsigned long toc=millis();
unsigned long ping=toc-tic;
//Serial.print("Ping: ");Serial.print(ping);Serial.println("ms");
if(ping>1000){Serial.println("Taking to long...");digitalWrite(Led_Esp32,HIGH);}
else{digitalWrite(Led_Esp32,LOW);}
/**/

}//void loop()

/*****************************************************************************Time and Save String in SDCard*/
void Time(){
  //Save info in the struct
  struct tm datetime;
  //If fail obtain time info
  if(!getLocalTime(&datetime)){Serial.println("Time and date obtaining fail!");return;}

  //Date and Time - Struct to String
  strftime(aux_sys_date,sizeof(aux_sys_date),"%F;%T;",&datetime);//Format time as char

  //sys_xxxx
  sys_date=String(aux_sys_date);//Save in string
  sys_temp=String(SimMode,DEC)+";"+String(TExt,3)+";"+String(TInt,3)+";"+String(TWUp,3)+";"+String(TWDw,3)+";"+String(Tpcm,3)+";"+String(TConf,3)+";";
  sys_ios=String(ManMode,DEC)+";"+String(VExtUp,DEC)+";"+String(VIntUp,DEC)+";"+String(VIntDw,DEC)+";"+String(WSClose,DEC)+";"+String(WSpcm,DEC)+";";
  sys_occup=String(Occupation,DEC)+";";
  sys_error=String(ERROR,DEC);

  //sys_info
  sys_info=sys_date+sys_temp+sys_ios+sys_occup+sys_error+"\r\n";
  
  //Serial.println(sys_info);delay(2000);
  if(SDCardError==0){SDAddInfo(SD,filename,sys_info.c_str());}

}//void Time()
