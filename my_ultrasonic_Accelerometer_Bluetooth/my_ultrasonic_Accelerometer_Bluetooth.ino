// Definitions for Accelerometer

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <EEPROM.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 accelerometer;

int16_t accel_x, accel_y, accel_z;

// Definitions for Bluetooth

//#include <SoftwareSerial.h>

const int rxPin = 10; // Connected to TXD on the Bluetooth module
const int txPin = 11; // Connected to RXD on the Bluetooth module

//SoftwareSerial myserial(rxPin, txPin);

// Definitions for Ultrasonic sensor

const int trigPin = 2;
const int echoPin = 3;

const int PROGRAM_MODE_PIN = 7;
// Each integer is 2 bytes long, therefore a sample is 6 bytes.
int SAMPLE_SIZE_BYTES = 6;
int MAX_EEPROM_ADDR = EEPROM.length() - SAMPLE_SIZE_BYTES;

float duration, cm;

void setup() {

  // Set up accelerometer
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  
  accelerometer.initialize();

  // Set up Bluetooth
  
  Serial.begin(9600);
//  Serial.println("Hello, Serial ready");

//  myserial.begin(9600);
//  myserial.println("Hello, SoftwareSerial (Bluetooth) ready");

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // Set up Ultrasonic sensor

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  pinMode(PROGRAM_MODE_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);


  if (digitalRead(PROGRAM_MODE_PIN) == HIGH) {
    // dump the contents of the arduino EEPROM to serial
    digitalWrite(LED_BUILTIN, HIGH);
    
    int eeAddr = 0;
    
    while(eeAddr <= MAX_EEPROM_ADDR) {
      int k;
      EEPROM.get(eeAddr, k);
      Serial.print(k);
      Serial.print(",");
      eeAddr += sizeof(int);
      
      EEPROM.get(eeAddr, k);
      Serial.print( ((float)k)/10 );
      Serial.print(",");
      eeAddr += sizeof(int);

      EEPROM.get(eeAddr, k);
      Serial.println(k);
      eeAddr += sizeof(int);
    }
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    int eeAddr = 0;
    digitalWrite(LED_BUILTIN, HIGH);

    while (eeAddr <= MAX_EEPROM_ADDR) {
      // Obtain distance from Ultrasonic sensor

      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
    
      duration = pulseIn(echoPin, HIGH);
      cm = (duration/2) * 0.0343;
    
      delay(1);
    
      // Obtain accelerometer data
    
      accelerometer.getAcceleration(&accel_x, &accel_y, &accel_z);
    
      // Print to Bluetooth serial monitor


      // OUTPUT FORMAT OF CODE PROVIDED:
//      myserial.print(millis()); // Print time in milliseconds
//      myserial.print(",");
//      myserial.print(cm); // Print distance in centimeters
//      myserial.print(",");
//      myserial.print(accel_x); // Print accelerometer x output
//      myserial.print(",");
//      myserial.print(accel_y); // Print accelerometer y output
//      myserial.print(",");
//      myserial.println(accel_z); // Print accelerometer z output
       int currTime_INT = millis();
       int currDistMM_INT = cm * 10.0;
       // change acceleration if axis is different:
       int accel_INT = accel_x;  

       EEPROM.put(eeAddr, currTime_INT);
       eeAddr += sizeof(int);
       EEPROM.put(eeAddr, currDistMM_INT);
       eeAddr += sizeof(int);
       EEPROM.put(eeAddr, accel_INT);
       eeAddr += sizeof(int);

      
    
      // Delay between samples (20 milliseconds)
      // with a delay of roughly 20 milliseconds, the program runs for about 7.2 seconds. This implies the rest of the loop takes an average of 23 ms to complete (7.2 seconds means 42 m/s per loop since there are 1000/6 loops)
      delay(20);
    }
    digitalWrite(LED_BUILTIN, LOW);
    
  }
}

void loop() {

  
}
