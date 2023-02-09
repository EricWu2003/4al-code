// Definitions for Accelerometer

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 accelerometer;

int16_t accel_x, accel_y, accel_z;

// Definitions for Bluetooth

#include <SoftwareSerial.h>

const int rxPin = 10; // Connected to TXD on the Bluetooth module
const int txPin = 11; // Connected to RXD on the Bluetooth module

SoftwareSerial myserial(rxPin, txPin);

// Definitions for Ultrasonic sensor

const int trigPin = 2;
const int echoPin = 3;

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
  Serial.println("Hello, Serial ready");

  myserial.begin(9600);
  myserial.println("Hello, SoftwareSerial (Bluetooth) ready");

  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);

  // Set up Ultrasonic sensor

  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
}

void loop() {

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

  myserial.print(millis()); // Print time in milliseconds
  myserial.print(",");
  myserial.print(cm); // Print distance in centimeters
  myserial.print(",");
  myserial.print(accel_x); // Print accelerometer x output
  myserial.print(",");
  myserial.print(accel_y); // Print accelerometer y output
  myserial.print(",");
  myserial.println(accel_z); // Print accelerometer z output

  // Delay between samples (2 milliseconds)

  delay(2);
}
