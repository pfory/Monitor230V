//ESP8266-01
//monitor sitoveho napeti 230V
//kompilovat jako Generic ESP8266 Module

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <WiFiManager.h> 
#include "Sender.h"

#define verbose
#ifdef verbose
  #define DEBUG_PRINT(x)                     Serial.print (x)
  #define DEBUG_PRINTDEC(x)                  Serial.print (x, DEC)
  #define DEBUG_PRINTLN(x)                   Serial.println (x)
  #define DEBUG_PRINTF(x, y)                 Serial.printf (x, y)
  #define PORTSPEED                          115200             
  #define SERIAL_BEGIN                       Serial.begin(PORTSPEED)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTDEC(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, y)
#endif 


#define SEND_DELAY                           30000  //prodleva mezi poslanim dat v ms
#define SENDSTAT_DELAY                       60000  //poslani statistiky kazdou minutu
#define TESTPIN                                 10  //test pinu


#define STATUSLED     2 //GPI02
#define PINMONITOR    3 //Rx

#define ota
#ifdef ota
#define HOSTNAMEOTA   "Monitor230V"
#include <ArduinoOTA.h>
#endif

#define AUTOCONNECTNAME   HOSTNAMEOTA
#define AUTOCONNECTPWD    "password"

//for LED status
#include <Ticker.h>
Ticker ticker;

#include <timer.h>
auto timer = timer_create_default(); // create a timer with default settings
Timer<> default_timer; // save as above


void tick()
{
  //toggle state
  int state = digitalRead(STATUSLED);  // get the current state of GPIO1 pin
  digitalWrite(STATUSLED, !state);     // set pin to the opposite state
}

WiFiClient client;
WiFiManager wifiManager;

uint32_t heartBeat                    = 0;
byte stavSite                         = LOW;

char      mqtt_server[40]             = "192.168.1.56";
uint16_t  mqtt_port                   = 1883;
char      mqtt_username[40]           = "datel";
char      mqtt_key[20]                = "hanka12";
char      mqtt_base[60]               = "/home/Monitor230V";
char      static_ip[16]               = "192.168.1.115";
char      static_gw[16]               = "192.168.1.1";
char      static_sn[16]               = "255.255.255.0";


//gets called when WiFiManager enters configuration mode
void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
}

ADC_MODE(ADC_VCC);
#define MILIVOLT_TO_VOLT 1000.0

  //SW name & version
#define     VERSION                          "0.31"
#define     SW_NAME                          "Monitor 230V"

void setup(void) {
  SERIAL_BEGIN;
  DEBUG_PRINT(F(SW_NAME));
  DEBUG_PRINT(F(" "));
  DEBUG_PRINTLN(F(VERSION));
  //set led pin as output
  pinMode(STATUSLED, OUTPUT);

  ticker.attach(1, tick);
  
  rst_info *_reset_info = ESP.getResetInfoPtr();
  uint8_t _reset_reason = _reset_info->reason;
  DEBUG_PRINT("Boot-Mode: ");
  DEBUG_PRINTLN(_reset_reason);
  heartBeat = _reset_reason;

  WiFiManager wifiManager;
  //reset settings - for testing
  //wifiManager.resetSettings();
  
  IPAddress _ip,_gw,_sn;
  _ip.fromString(static_ip);
  _gw.fromString(static_gw);
  _sn.fromString(static_sn);

  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  
  DEBUG_PRINTLN(_ip);
  DEBUG_PRINTLN(_gw);
  DEBUG_PRINTLN(_sn);

  //wifiManager.setConfigPortalTimeout(60); 
  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  if (!wifiManager.autoConnect(AUTOCONNECTNAME, AUTOCONNECTPWD)) { 
    DEBUG_PRINTLN("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  } 
  
  #ifdef ota
  //OTA
  // Port defaults to 8266
  // ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAMEOTA);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    // String type;
    // if (ArduinoOTA.getCommand() == U_FLASH)
      // type = "sketch";
    // else // U_SPIFFS
      // type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    //DEBUG_PRINTLN("Start updating " + type);
    DEBUG_PRINTLN("Start updating ");
  });
  ArduinoOTA.onEnd([]() {
   DEBUG_PRINTLN("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_PRINTF("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_PRINTF("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) DEBUG_PRINTLN("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) DEBUG_PRINTLN("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) DEBUG_PRINTLN("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) DEBUG_PRINTLN("Receive Failed");
    else if (error == OTA_END_ERROR) DEBUG_PRINTLN("End Failed");
  });
  ArduinoOTA.begin();
#endif

  void * a;
  timer.every(SEND_DELAY, sendDataHA);
  timer.every(SENDSTAT_DELAY, sendStatisticHA);
  timer.every(TESTPIN, testPin);
  sendStatisticHA(a);
  ticker.detach();
}

void loop(void) {
  timer.tick(); // tick the timer
#ifdef ota
  ArduinoOTA.handle();
#endif
}


bool testPin(void *) {
  //monitor
  pinMode(PINMONITOR, FUNCTION_3); 

  if (stavSite==0) { //aby se uchoval stav pokud sit vypadne jen na kratky okamzik
    stavSite = digitalRead(PINMONITOR); //0 - 230V, 1 - 0V
  }

  //********** CHANGE PIN FUNCTION  TO TX/RX **********
  pinMode(PINMONITOR, FUNCTION_0); 
  digitalWrite(STATUSLED, stavSite);  //0 - sviti, 1 - nesviti
  return true;
}

bool sendStatisticHA(void *) {
  DEBUG_PRINTLN(F(" - I am sending statistic to HA"));

  SenderClass sender;
  sender.add("VersionSW", VERSION);
  sender.add("HeartBeat", heartBeat++);
  sender.add("RSSI", WiFi.RSSI());
  sender.add("Napeti",  ESP.getVcc());
  
  DEBUG_PRINTLN(F("Calling MQTT"));
  
  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);
  return true;
}

bool sendDataHA(void *) {
  SenderClass sender;

  digitalWrite(STATUSLED, !stavSite);

  sender.add("status",   stavSite);
  sender.sendMQTT(mqtt_server, mqtt_port, mqtt_username, mqtt_key, mqtt_base);

  digitalWrite(STATUSLED, stavSite);
  stavSite = 1;
  return true;
}