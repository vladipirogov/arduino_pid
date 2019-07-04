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
  esp.Process();
   String result = "{real_temperature: " + String(realTemp) + 
                    ", reference_value:" + String(pos_encoder) + 
                    ", error: " + String(error) + 
                    ", out: " + String(out) + "}";
   Serial.println(result);
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
  mqtt.subscribe("pid/reference");
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

  if(serial_str == "getkoef") {
    String result = "{kp:" + String(pid.kp.get()) + 
                    ", ki:" + String(pid.ki.get()) + 
                    ", kd:" + String(pid.kd.get()) + "}";
    char buf1[50];
    result.toCharArray(buf1, result.length() + 1);
    mqtt.publish("home/feedbacktopic", buf1);
  }

  char buf[serial_str.length() + 1]; 
  serial_str.toCharArray(buf, serial_str.length() + 1);
  char *array[serial_str.length() + 2];
  int i = 0;
  array[i] = strtok(buf,",");
    while(array[i]!=NULL) {
      array[++i] = strtok(NULL,",");
    }
  int reference = atoi(array[0]);
  int kp = atoi(array[1]);
  int ki = atoi(array[2]);
  int kd = atoi(array[3]);
  if(pos_encoder != reference) pos_encoder = reference;
  if(pid.kp.get() != kp) pid.kp.set(kp);
  if(pid.ki.get() != ki) pid.ki.set(ki);
  if(pid.kd.get() != kp) pid.kd.set(kd);

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