#include <DallasTemperature.h>
#include <pid.h>
#include <TimerOne.h>

// Arduino pin number with sensor connected
#define PIN_DS18B20 8
//pin 7 connection to web encoder
#define pin_DT  2 
//pin 8 connection to CLK encoder
#define pin_CLK 3
#define pin_OUT 5

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