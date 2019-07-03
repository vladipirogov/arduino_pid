#include <DallasTemperature.h>
#include <pid.h>
#include <TimerOne.h>
#include <ELClient.h>
#include <ELClientCmd.h>
#include <ELClientMqtt.h>
#include <EEPROM.h>
#include "hashmap.h"   

// Arduino pin number with sensor connected
#define PIN_DS18B20 8
//pin 7 connection to web encoder
#define pin_DT  2 
//pin 8 connection to CLK encoder
#define pin_CLK 3
#define pin_OUT 5

// Initialize a connection to esp-link using the normal hardware serial port both for
// SLIP and for debug messages.
ELClient esp(&Serial, &Serial);

// Initialize CMD client (for GetTime)
ELClientCmd cmd(&esp);

// Initialize the MQTT client
ELClientMqtt mqtt(&esp);

// Creating a OneWire object
OneWire oneWire(PIN_DS18B20);

// Create a DallasTemperature object to work with sensors, passing it a link to an object to work with 1-Wire.
DallasTemperature dallasSensors(&oneWire);

//PIR regulator
Pid pid;

int pos_encoder = 0; // The initial position of the encoder is 0
int last;            // previous encoder position value
int DT;
boolean left;
float error = 0;
float realTemp = 0;
float out = 0;

//Connection status
bool connected;

/**
 * Auxiliary function of printing the temperature value for the device
 */
float printTemperature() {
  float tempC = dallasSensors.getTempCByIndex(0);
  return tempC;
}

/**
 * 
 */
void print_values() {
  Serial.print("Temperature: ");     
  Serial.println(realTemp);
  Serial.print ("  ");

  Serial.print ("ERROR: ");
  Serial.print (error);
  Serial.print ("  ");

  Serial.print ("OUT: ");
  Serial.print (out);
  esp.Process();
   String result = "{real_temperature: " + String(realTemp) + 
                    ", reference_value:" + String(pos_encoder) + 
                    ", error: " + String(error) + 
                    ", out: " + String(out) + "}";
   char buf[100];
   result.toCharArray(buf, 100);
    mqtt.publish("pid/value", buf);
}

void do_encoder() {
  DT = digitalRead(pin_DT);
   if (DT != last) { // if they are not equal then the encoder has changed the position
     if (digitalRead(pin_CLK) != DT) { 
       pos_encoder++;
       left = false;
     } else { 
       left = true;
       pos_encoder--;
     }
     Serial.print ("DIRECTION: ");
     if (!left){  
       Serial.println("clockwise ->");
     }else{
       Serial.println("counterclockwise  <-");
     }
     Serial.print("POSITION: ");     
     Serial.println(pos_encoder);
   } 
   last = DT;
}



// Callback made from esp-link to notify of wifi status changes
// Here we just print something out for grins
void wifiCb(void* response) {
  ELClientResponse *res = (ELClientResponse*)response;
  if (res->argc() == 1) {
    uint8_t status;
    res->popArg(&status, 1);

    if(status == STATION_GOT_IP) {
      Serial.println("WIFI CONNECTED");
    } else {
      Serial.print("WIFI NOT READY: ");
      Serial.println(status);
    }
}
}



// Callback when MQTT is connected
void mqttConnected(void* response) {
  Serial.println("MQTT connected!");
  mqtt.subscribe("home/commandtopic");
  connected = true;
}

// Callback when MQTT is disconnected
void mqttDisconnected(void* response) {
  //Serial.println("MQTT disconnected");
  connected = false;
}

void mqttData(void* response) {
  ELClientResponse *res = (ELClientResponse *)response;
  String topic = res->popString();

  String serial_str = res->popString();

  Serial.println(serial_str);
}

void mqttPublished(void* response) {
  Serial.println("MQTT published");
}

/**
 * 
 */
void setup() {
  Serial.begin(9600);
  pinMode (pin_DT,INPUT);
  pinMode (pin_CLK,INPUT);
  last = digitalRead(pin_DT);

  Timer1.initialize(2000000);
  Timer1.attachInterrupt( print_values );
   attachInterrupt(digitalPinToInterrupt(pin_DT), do_encoder, CHANGE);

   Serial.println("EL-Client starting!");

 esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)
  bool ok;
  do {
    ok = esp.Sync();      // sync up with esp-link, blocks for up to 2 seconds
    if (!ok) Serial.println("EL-Client sync failed!");
    if(Serial.available() > 0 ) {
      String str = Serial.readString();
      if(str == "break") {
        Serial.println("Breack synced!");
        break;
      }
    }
  } while(!ok);
  Serial.println("EL-Client synced!");

  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();
    Serial.println("EL-MQTT ready");
}

/**
 * 
 */
void loop(void){
  
  
  dallasSensors.requestTemperatures(); 

  realTemp = printTemperature();

  error = pos_encoder - realTemp;
  

  out = pid.eval(error);
  
  analogWrite(pin_OUT, out);
}