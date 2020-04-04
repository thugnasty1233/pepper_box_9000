/***************************************************************************
  This is a library for the BME680 gas, humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BME680 Breakout
  ----> http://www.adafruit.com/products/3660

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

//begRelay

#define relay1 8//light
#define relay2 7 // heat
#define relay3 4 //airpump
#define relay4 12 // humidifier
#define fan1PWM 11
#define fan2PWM 10
//endRelay

//Temp in C, 60% max humidity to prevent condensation.
#define onTemp 24
#define offTemp 26
#define onHumid 50
#define offHumid 52


const unsigned long second = 1000;
const unsigned long hour = 3600*second;
const unsigned long day = hour*24;

//begEdit
//Normal cycle
//const unsigned long eventTime_daytimestart = 1*second;
//const unsigned long eventTime_daytimeend = 10*hour;
//const unsigned long eventTime_nighttimestart = 10*hour + 1*second;
//const unsigned long eventTime_nighttimeend = 24*hour;

//Test cycle
const unsigned long eventTime_daytimestart = 10*second;
const unsigned long eventTime_daytimeend = 20*second;
//const unsigned long eventTime_nighttimestart = 30*second;
//const unsigned long eventTime_nighttimeend =40*second;

unsigned long previousTime = 0;
//unsigned long previousTime_1 = 0;
//unsigned long previousTime_2 = 0;
//unsigned long previousTime_3 = 0;
//unsigned long previousTime_4 = 0;
//unsigned long scratch = 0;
//endEdit


//determines if you want to start with daytime/nighttime
bool dayTime = true;

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"

//begSERVOedit
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position
//endSERVOedit


//begDHTedit
#include <DHT.h>
#include <DHT_U.h>

#define DHTPIN 2
// Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

// Uncomment the type of sensor in use:
#define DHTTYPE    DHT11     // DHT 11
//#define DHTTYPE    DHT22     // DHT 22 (AM2302)
//#define DHTTYPE    DHT21     // DHT 21 (AM2301)

// See guide for details on sensor wiring and usage:
//   https://learn.adafruit.com/dht/overview

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

//endDHTedit


#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME680 bme; // I2C
//Adafruit_BME680 bme(BME_CS); // hardware SPI
//Adafruit_BME680 bme(BME_CS, BME_MOSI, BME_MISO,  BME_SCK);



void setup() {

    Serial.begin(9600);
  while (!Serial);
  Serial.println(F("BME680 Ready:"));

  //begSERVOedit
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  //endSERVOedit

  //begDHTedit
// Initialize device.
  dht.begin();
  Serial.println(F("DHTxx Unified Sensor Example"));
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
  //endDHTedit

  if (!bme.begin()) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms  

  //begRelay
 if(dayTime == true) {
  pinMode(relay1, OUTPUT);
  digitalWrite(relay1, LOW);

  pinMode(relay2, OUTPUT);
  digitalWrite(relay2, HIGH);

  pinMode(relay3, OUTPUT);
  digitalWrite(relay3, LOW);

  pinMode(relay4, OUTPUT);
  digitalWrite(relay4, HIGH);

  pinMode(fan1PWM, OUTPUT);
  analogWrite(fan1PWM, 255);

    pinMode(fan2PWM, OUTPUT);
  analogWrite(fan2PWM, 255);
 }
 
  else{
  pinMode(relay1, OUTPUT);
  digitalWrite(relay1, HIGH);

  pinMode(relay2, OUTPUT);
  digitalWrite(relay2, HIGH);

  pinMode(relay3, OUTPUT);
  digitalWrite(relay3, LOW);

  pinMode(relay4, OUTPUT);
  digitalWrite(relay4, HIGH);

  pinMode(fan1PWM, OUTPUT);
  analogWrite(fan1PWM, 255);

    pinMode(fan2PWM, OUTPUT);
  analogWrite(fan2PWM, 255);
  }

  //endRelay

//PLX-DAQbeg
 Serial.println("LABEL,Computer Time (?),Temperature (C.),Barometric Pressure (hPa.),Humidity (%),VO2 (KOhms.),Approximate Altitude (m.),Heating Element,Humidifier,O2 Intake,Fan1,Fan2,Overhead Light");  
//PLX-DAQend
}

void loop() {

  unsigned long currentTime = millis();
//  if (previousTime > currentTime){
//    previousTime_1 = currentTime;
//    previousTime_2 = currentTime;
//    previousTime_3 = currentTime;
//    previousTime_4 = currentTime;
//    previousTime = 0;
//    }
//  previousTime = currentTime;
  if( currentTime - previousTime > 12 * hour){ // day
    if(!dayTime){
      digitalWrite(relay1, LOW);
      //digitalWrite(relay3, LOW);
      analogWrite(fan1PWM, 255);
      analogWrite(fan2PWM, 255);
      Serial.println("Goodmorning");
      dayTime = true;
      previousTime = currentTime;
    }
    else{ //night
    digitalWrite(relay1, HIGH);
    //digitalWrite(relay3, HIGH);
    analogWrite(fan1PWM, 0);
    analogWrite(fan2PWM, 0);
   
    Serial.println("Goodnight");
    dayTime = false;
    previousTime = currentTime;
    } 
  }
 
 

// if( currentTime - previousTime_3 >= eventTime_nighttimestart ){
//    digitalWrite(relay1, LOW);
//    digitalWrite(relay3, HIGH);
//    analogWrite(fan1PWM, 255);
//    analogWrite(fan2PWM, 255);
//
//    Serial.println("Night");
//
//    previousTime_3 = currentTime;
//
//  }
//
//   if( currentTime - previousTime_4 >= eventTime_nighttimeend ){
//    digitalWrite(relay1, HIGH);
//    digitalWrite(relay3, HIGH);
//    analogWrite(fan1PWM, 0);
//    analogWrite(fan2PWM, 0);
//
//    Serial.println("Good Morning");
//   
//    previousTime_4 = currentTime;
//
//  }  



//Sensor array to do:
//PID controller

if (! bme.performReading()) {
  Serial.println("Failed to perform reading :(");
  return;
}
Serial.print("Temperature = ");
Serial.print(bme.temperature);
Serial.println(" *C");
if (bme.temperature < onTemp && digitalRead(relay2)){
    digitalWrite(relay2, LOW);
    analogWrite(fan1PWM, 255);
    analogWrite(fan2PWM, 255);
}
if (bme.temperature > offTemp || !digitalRead(relay2)){
  digitalWrite(relay2, HIGH);
  if(!digitalRead(relay2)){
    //analogWrite(fan1PWM, 0);
    //analogWrite(fan2PWM, 0);
  }
}

Serial.print("Pressure = ");
Serial.print(bme.pressure / 100.0);
Serial.println(" hPa");

Serial.print("Humidity = ");
Serial.print(bme.humidity);
Serial.println(" %");

if (bme.humidity < onHumid){
  digitalWrite(relay4, LOW);
  analogWrite(fan1PWM, 255);
  analogWrite(fan2PWM, 255);
}
if (bme.humidity > offHumid){
  digitalWrite(relay4, HIGH);
  analogWrite(fan1PWM, 0);
  analogWrite(fan2PWM, 0);
  }

Serial.print("Gas = ");
Serial.print(bme.gas_resistance / 1000.0);
Serial.println(" KOhms");

Serial.print("Approx. Altitude = ");
Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
Serial.println(" m");

 Serial.print("Light ");
 Serial.println(digitalRead(relay1), DEC);
 Serial.print("Heat ");
 Serial.println(digitalRead(relay2), DEC);
 Serial.print("Pump ");
 Serial.println(digitalRead(relay3), DEC);
 Serial.print("Humidifier ");
 Serial.println(digitalRead(relay4), DEC);

//servo to do!
//if statement with incremental opening.

//INSTALL PHOTOREISTOR

//begSERVOedit
if(bme.humidity >= 58 || bme.temperature >= 26.667){
  for (pos = 55; pos <= 160; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(50);                       // waits __ms for the servo to reach the position
  }
  for (pos = 160; pos >= 55; pos -= 1) { // goes from 180 degrees to 0 degrees
   myservo.write(pos);              // tell servo to go to position in variable 'pos'
   delay(50);                       // waits __ms for the servo to reach the position
  }
}
//endSERVOedit

//begDHTedit
  // Delay between measurements.
  delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
//endDHTedit

//PLX-DAQbeg
  Serial.print("DATA,TIME,");
  //BME680
  Serial.print(bme.temperature, DEC);
  Serial.print(",");  
  Serial.print(bme.pressure / 100.0, DEC);
  Serial.print(","); 
  Serial.print(bme.humidity, DEC);
  Serial.print(",");
  Serial.print(bme.gas_resistance / 1000.0, DEC);
  Serial.print(",");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA), DEC);
  Serial.print(",");
  Serial.print(!digitalRead(relay2), DEC);
  Serial.print(",");
  Serial.print(!digitalRead(relay4), DEC);
  Serial.print(",");
  Serial.print(!digitalRead(relay3), DEC);
  Serial.print(",");
  Serial.print(((float)analogRead(fan1PWM))/1024 * 100, DEC);
  Serial.print(",");  
  Serial.print(((float)analogRead(fan2PWM))/1024 * 100, DEC);
  Serial.print(",");
  Serial.print(!digitalRead(relay1), DEC);
  Serial.print(",");  
  //PLX-DAQend

Serial.println();
delay(2000);

delay(10*second);


}
