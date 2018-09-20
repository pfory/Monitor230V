//ESP8266-01
//monitor sitoveho napeti
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
#define SENDINTERVAL  60000  //60sec

#define STATUSLED     3
#define PINMONITOR    1

//for LED status
#include <Ticker.h>
Ticker ticker;

void tick()
{
  //toggle state
  int state = digitalRead(BUILTIN_LED);  // get the current state of GPIO1 pin
  digitalWrite(BUILTIN_LED, !state);     // set pin to the opposite state
}

WiFiClient client;
WiFiManager wifiManager;

uint32_t heartBeat                    = 0;
unsigned long milisLastRunMinOld      = 0;
byte stavSite                         = LOW;


IPAddress _ip           = IPAddress(192, 168, 1, 115);
IPAddress _gw           = IPAddress(192, 168, 1, 1);
IPAddress _sn           = IPAddress(255, 255, 255, 0);

Adafruit_MQTT_Client mqtt(&client, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

unsigned int const sendTimeDelay  = 10000;
signed long lastSendTime          = sendTimeDelay * -1;

/****************************** Feeds ***************************************/
Adafruit_MQTT_Publish version             = Adafruit_MQTT_Publish(&mqtt, "/home/Monitor230V/VersionSW");
Adafruit_MQTT_Publish hb                  = Adafruit_MQTT_Publish(&mqtt, "/home/Monitor230V/HeartBeat");
Adafruit_MQTT_Publish status              = Adafruit_MQTT_Publish(&mqtt, "/home/Monitor230V/status");

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

float versionSW                   = 0.1;
String versionSWString            = "Monitor 230V v";


void setup(void) {
  Serial.begin(PORTSPEED);
  DEBUG_PRINT(versionSWString);
  DEBUG_PRINT(versionSW);
  //set led pin as output
  pinMode(BUILTIN_LED, OUTPUT);
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

  wifiManager.setConnectTimeout(600); //5min

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
  //keep LED on
  digitalWrite(BUILTIN_LED, HIGH);
  
}

void loop(void) {
  //monitor
  //********** CHANGE PIN FUNCTION  TO GPIO **********
  //GPIO 1 (TX) swap the pin to a GPIO.
  pinMode(PINMONITOR, FUNCTION_3); 
  //GPIO 3 (RX) swap the pin to a GPIO.
  pinMode(STATUSLED, FUNCTION_3); 
  //**************************************************

  stavSite = digitalRead(PINMONITOR);
  digitalWrite(STATUSLED,stavSite);

  //********** CHANGE PIN FUNCTION  TO TX/RX **********
  //GPIO 1 (TX) swap the pin to a TX.
  pinMode(PINMONITOR, FUNCTION_0); 
  //GPIO 3 (RX) swap the pin to a RX.
  pinMode(STATUSLED, FUNCTION_0); 
  //***************************************************
  
  
  if (millis() - milisLastRunMinOld > SENDINTERVAL) {
    digitalWrite(BUILTIN_LED,LOW);
    MQTT_connect();
    milisLastRunMinOld = millis();
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

    if (! status.publish(stavSite)) {
      DEBUG_PRINTLN("status send failed");
    } else {
      DEBUG_PRINTLN("status send OK!");
    }
    digitalWrite(BUILTIN_LED,HIGH);
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
