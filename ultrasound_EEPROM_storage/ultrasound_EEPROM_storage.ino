#include <EEPROM.h>


const int TRIG_PIN = 2;
const int ECHO_PIN = 3;


// pin 7 will be used to set the mode of the program at runtime.
// if pin 7 is connected to 5V at the beginning of the program, then we will write data to serial
// otherwise, we will collect data, overwriting the contents of the EEPROM
const int PROGRAM_MODE_PIN = 7;


// For each sample, this program will write two integers, 
// representing the current time in milliseconds, and the distance detected in mm
//
// Each integer is 2 bytes long, therefore a sample is 4 bytes.
int SAMPLE_SIZE_BYTES = 4;


int MAX_EEPROM_ADDR = EEPROM.length() - SAMPLE_SIZE_BYTES;


float duration;
int distance;
int currentTime;


void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(PROGRAM_MODE_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);
  
  Serial.begin(9600);



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
      Serial.println(k);
      eeAddr += sizeof(int);
    }
    digitalWrite(LED_BUILTIN, LOW);
    
  }
  else {
    //collect data for about 16 seconds, filling the EEPROM and overwriting anything that was previously stored.
    
    int eeAddr = 0;
    digitalWrite(LED_BUILTIN, HIGH);
    
    while(eeAddr <= MAX_EEPROM_ADDR) {
      // main loop here
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH); //Send pulse
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
      duration = pulseIn(ECHO_PIN, HIGH); // Recieve pulse
      distance = (duration/2)*0.343;
  
    
      currentTime = millis();
  

      // write a single sample to EEPROM  
      EEPROM.put(eeAddr, currentTime);
      eeAddr += sizeof(int);
      EEPROM.put(eeAddr, distance);
      eeAddr += sizeof(int);

      // Delay in between samples
      delay(50); 
    }

    digitalWrite(LED_BUILTIN, LOW);
    
  }



}

void loop() {

}
