//ESP8266-01
//monitor sitoveho napeti 230V
//kompilovat jako Generic ESP8266 Module

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include <WiFiManager.h> 

#define AIO_SERVER      "192.168.1.56"
#define AIO_SERVERPORT  1883
#define AIO_USERNAME    "datel"
#define AIO_KEY         "hanka12"

#define verbose
#ifdef verbose
 #define DEBUG_PRINT(x)         Serial.print (x)
 #define DEBUG_PRINTDEC(x)      Serial.print (x, DEC)
 #define DEBUG_PRINTLN(x)       Serial.println (x)
 #define DEBUG_PRINTF(x, y)     Serial.printf (x, y)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTDEC(x)
 #define DEBUG_PRINTLN(x)
 #define DEBUG_PRINTF(x, y)
#endif 

#define PORTSPEED     115200
#define SENDINTERVAL  5000  //60sec
#define MEASINTERVAL   10  //10ms

#define STATUSLED     2 //GPI02
#define PINMONITOR    3 //Rx

//for LED status
#include <Ticker.h>
Ticker ticker;

void tick()
{
  //toggle state
  int state = digitalRead(STATUSLED);  // get the current state of GPIO1 pin
  digitalWrite(STATUSLED, !state);     // set pin to the opposite state
}

WiFiClient client;
WiFiManager wifiManager;

uint32_t heartBeat                    = 0;
unsigned long milisLastSend           = 0;
unsigned long milisLastMeas           = 0;
byte stavSite                         = LOW;


IPAddress _ip           = IPAddress(192, 168, 1, 115);
IPAddress _gw           = IPAddress(192, 168, 1, 1);
IPAddress _sn           = IPAddress(255, 255, 255, 0);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

unsigned int const sendTimeDelay  = 10000;
signed long lastSendTime          = sendTimeDelay * -1;

/****************************** Feeds ***************************************/
#define MQTTBASE "/home/Monitor230V/"

Adafruit_MQTT_Publish version             = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "VersionSW");
Adafruit_MQTT_Publish hb                  = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "HeartBeat");
Adafruit_MQTT_Publish status              = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "status");
Adafruit_MQTT_Publish voltage            = Adafruit_MQTT_Publish(&mqtt, MQTTBASE "Voltage");

void MQTT_connect(void);

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

float versionSW                   = 0.3;
String versionSWString            = "Monitor 230V v";


void setup(void) {
  Serial.begin(PORTSPEED);
  DEBUG_PRINT(versionSWString);
  DEBUG_PRINT(versionSW);
  //set led pin as output
  pinMode(STATUSLED, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);
  
  DEBUG_PRINTLN(ESP.getResetReason());
  if (ESP.getResetReason()=="Software/System restart") {
    heartBeat=1;
  } else if (ESP.getResetReason()=="Power on") {
    heartBeat=2;
  } else if (ESP.getResetReason()=="External System") {
    heartBeat=3;
  } else if (ESP.getResetReason()=="Hardware Watchdog") {
    heartBeat=4;
  } else if (ESP.getResetReason()=="Exception") {
    heartBeat=5;
  } else if (ESP.getResetReason()=="Software Watchdog") {
    heartBeat=6;
  } else if (ESP.getResetReason()=="Deep-Sleep Wake") {
    heartBeat=7;
  }

  wifiManager.setConnectTimeout(10);

  //set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);
  
  //WiFi.config(ip); 
  wifiManager.setSTAStaticIPConfig(_ip, _gw, _sn);
  if (!wifiManager.autoConnect("Monitor 230V", "password")) {
    DEBUG_PRINTLN("failed to connect, we should reset as see if it connects");
    delay(3000);
    ESP.reset();
    delay(5000);
  }
  ticker.detach();
}

void loop(void) {

  //test site 230V na pinu
  if (millis() - milisLastMeas > MEASINTERVAL) {
    milisLastMeas = millis();
    //monitor
    pinMode(PINMONITOR, FUNCTION_3); 

    if (stavSite==0) { //aby se uchoval stav pokud sit vypadne jen na kratky okamzik
      stavSite = digitalRead(PINMONITOR); //0 - 230V, 1 - 0V
    }
    
    //********** CHANGE PIN FUNCTION  TO TX/RX **********
    pinMode(PINMONITOR, FUNCTION_0); 
    digitalWrite(STATUSLED, stavSite);  //0 - sviti, 1 - nesviti
  }
  
  if (millis() - milisLastSend > SENDINTERVAL) {
    milisLastSend = millis();
    digitalWrite(STATUSLED, !stavSite);
    delay(100);
    MQTT_connect();
    if (! version.publish(versionSW)) {
      DEBUG_PRINTLN("version SW send failed");
    } else {
      DEBUG_PRINTLN("version SW send OK!");
    }
    if (! hb.publish(heartBeat)) {
      DEBUG_PRINTLN("HB send failed");
    } else {
      DEBUG_PRINTLN("HB send OK!");
    }
    heartBeat++;
    
    if (! status.publish(!stavSite)) {
      DEBUG_PRINTLN("status send failed");
    } else {
      DEBUG_PRINTLN("status send OK!");
    }

    if (! voltage.publish((float)ESP.getVcc()/MILIVOLT_TO_VOLT)) {
      DEBUG_PRINTLN("Voltage failed");
    } else {
      DEBUG_PRINTLN("Voltage OK!");
    }  
    
    digitalWrite(STATUSLED, stavSite);
    stavSite = 1;
  }
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }

  DEBUG_PRINT("Connecting to MQTT... ");

  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       DEBUG_PRINTLN(mqtt.connectErrorString(ret));
       DEBUG_PRINTLN("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  DEBUG_PRINTLN("MQTT Connected!");
}
