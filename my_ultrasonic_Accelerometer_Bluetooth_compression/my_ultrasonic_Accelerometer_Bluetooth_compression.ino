// Definitions for Accelerometer

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <EEPROM.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 accelerometer;

int ACCEL_SCALE_FACTOR = 50;

int LOOP_DURATION_MS = 50;

int16_t accel_x, accel_y, accel_z;

// Definitions for Ultrasonic sensor

const int trigPin = 2;
const int echoPin = 3;

const int PROGRAM_MODE_PIN = 7;

// END_PADDING represents how many bytes of padding we want to leave at the end of the EEPROM
// we leave some for safety so that we don't write to invalid locations
int END_PADDING = 6;
int MAX_EEPROM_ADDR = (EEPROM.length()/3) - END_PADDING;

float duration, cm;

void setup() {

  // Set up accelerometer
  
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif
  
  accelerometer.initialize();


  // Set up Ultrasonic sensor

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  pinMode(PROGRAM_MODE_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);


  if (digitalRead(PROGRAM_MODE_PIN) == HIGH) {
    // dump the contents of the arduino EEPROM to serial
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.begin(9600);
    
    int eeAddr = 0;

    int currDistMM;
    int currAccelScaled;

    bool has_overflowed = false;

    EEPROM.get(eeAddr, has_overflowed);
    eeAddr += sizeof(bool);
    EEPROM.get(eeAddr, currDistMM);
    eeAddr += sizeof(int);
    EEPROM.get(eeAddr, currAccelScaled);
    eeAddr += sizeof(int);

    if (has_overflowed) {
      Serial.println("The following data contains an overflow and is thus unreliable:");
    }

    long loop_counter = 0;
    
    while(eeAddr <= MAX_EEPROM_ADDR) {
      loop_counter ++;
      int8_t k;
      
      EEPROM.get(eeAddr, k);
      currDistMM += k;
      eeAddr += sizeof(int8_t);
      
      EEPROM.get(eeAddr, k);
      currAccelScaled += k;
      eeAddr += sizeof(int8_t);

      long currTime = loop_counter * LOOP_DURATION_MS;
      long currAccel = currAccelScaled * ACCEL_SCALE_FACTOR;

      Serial.print(currTime);
      Serial.print(",");
      Serial.print(currDistMM);
      Serial.print(",");
      Serial.println(currAccel);
      
    }
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    int eeAddr = 0;
    digitalWrite(LED_BUILTIN, HIGH);

    

    bool has_overflowed = false;

    // Obtain data
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    accelerometer.getAcceleration(&accel_x, &accel_y, &accel_z);
    int prevDistMM = (duration/2) * 0.343;
    int prevAccelScaled = accel_x * ACCEL_SCALE_FACTOR;

    EEPROM.put(eeAddr, false);
    eeAddr += sizeof(bool);
    EEPROM.put(eeAddr, prevDistMM);
    eeAddr += sizeof(int);
    EEPROM.put(eeAddr, prevAccelScaled);
    eeAddr += sizeof(int);

    long INITIAL_TIME = millis();
    long loop_counter = 0;

    while (eeAddr <= MAX_EEPROM_ADDR) {
      loop_counter ++;
      while (millis() < loop_counter * ((long) LOOP_DURATION_MS)) {
        // do nothing (wait for the correct time to begin collecting data)
      }


      
      // Obtain distance from Ultrasonic sensor

      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      accelerometer.getAcceleration(&accel_x, &accel_y, &accel_z);
      int currDistMM = (duration/2) * 0.343;
      int currAccelScaled = accel_x * ACCEL_SCALE_FACTOR;
      

      
      int8_t DistMM_DIFF = currDistMM - prevDistMM;
      int8_t AccelScaled_DIFF = currAccelScaled - prevAccelScaled;

      if (currDistMM - prevDistMM > 125 || currDistMM - prevDistMM < -125) {
        has_overflowed = true;
      }
      if (currAccelScaled - prevAccelScaled > 125  || currAccelScaled - prevAccelScaled < -125) {
        has_overflowed = true;
      }

      prevDistMM = currDistMM;
      prevAccelScaled = currAccelScaled;
    

       

       EEPROM.put(eeAddr, DistMM_DIFF);
       eeAddr += sizeof(int8_t);
       EEPROM.put(eeAddr, AccelScaled_DIFF);
       eeAddr += sizeof(int8_t);    
    }

    if (has_overflowed) {
      EEPROM.put(0, true);
      while (true) {
        digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
        delay(1000);                       // wait for a second
        digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
        delay(1000);
      }
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }

    
    
    
  }
}

void loop() {

  
}
