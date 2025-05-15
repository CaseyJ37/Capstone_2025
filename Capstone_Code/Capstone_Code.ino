//Right now, serial command is written for both the USB and the RX and TX pins

#define _PWM_LOGLEVEL_       4

// Select false to use PWM
#define USING_TIMER       false   //true

#include "nRF52_PWM.h"

#include <Adafruit_TinyUSB.h>
#include <ArduinoJson.h>
#include<SPI.h>
#include<SD.h>

#define pinToUse       A0
//creates pwm instance
nRF52_PWM* PWM_Instance;

float frequency;
float dutyCycle;

float nonZeroReadingsVoltage[100] = {0};
int VoltageCount;
int nonZeroIndexVoltage = 0;
float nonZeroReadingsADC[100] = {0}; 
int ADCCount;
int nonZeroIndexADC = 0;
float nonZeroReadingsConductivity[100] = {0}; 
int ConductivityCount;
int nonZeroIndexConductivity = 0;
int sample_number = 1;
File MyFile;

void setup() {
  // put your setup code here, to run once:
  //Starts a serial connection with the RX and TX pins, and the USB port
  Serial1.begin(9600);
  Serial.begin(9600);

  frequency = 1000000.0f;
  dutyCycle = 53.5f; //Should be 50, in practice this is the number that gets us a 50% duty cycle
  PWM_Instance = new nRF52_PWM(pinToUse, frequency, dutyCycle);

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  if(!SD.begin(5))
    Serial.println("SD failed to initialize");
}

void loop() {
  // put your main code here, to run repeatedly:

  delay(50);
  // Serial.print("Sample number: ");
  // Serial.println(sample_number);

  //Reads the voltage from the Conductivity sensor, and sends that value to the serial port.
  int adc_value = analogRead(A1);
  float voltage = adc_value * (3.3/1023.0);
  float conductivity = (adc_value - 50.0) * 2.5;
  // Serial1.print("ADC value: ");
  // Serial.print("ADC value: ");
  // Serial1.println(adc_value);
  // Serial.println(adc_value);
  // Serial1.print("Voltage: ");
  // Serial.print("Voltage: ");
  // Serial1.println(voltage);
  // Serial.println(voltage);
  adc_value = processADC(adc_value);
  voltage = processVoltage(voltage);
  conductivity = processConductivity(conductivity);

  MyFile = SD.open("Data.txt", FILE_WRITE);
  if(MyFile)
  {
    MyFile.print("Sample number: ");
    MyFile.print(sample_number);
    sample_number++;
    MyFile.print(" ADC value: ");
    MyFile.print(adc_value);
    MyFile.print(" Voltage: ");
    MyFile.println(voltage);
    MyFile.print(" Conductivity: ");
    MyFile.println(conductivity);
    MyFile.close();
  }

  String receivedMessage;

  //Reads data from the serial port (RX and TX pins) if it's available, and shows the recieved message
  while (Serial1.available() > 0) {
    char receivedChar = Serial1.read();
    if (receivedChar == '\n') {
      Serial1.print("Received message: ");
      Serial1.println(receivedMessage);  // Print the received message
      Serial.print("Received message: ");
      Serial.println(receivedMessage);  // Print the received message
      receivedMessage = "";  // Reset the received message
    } 
    else {
      receivedMessage += receivedChar;  // Append characters to the received message
    }
  }

    //Reads data from the serial port (USB) if it's available, and shows the recieved message
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      Serial.print("Received message: ");
      Serial.println(receivedMessage);  // Print the received message
      receivedMessage = "";  // Reset the received message
    } 
    else {
      receivedMessage += receivedChar;  // Append characters to the received message
    }
  }

}

float processVoltage(float value) { 
  VoltageCount++;
  if (value == 0.0f) {
    return 0; // Ignore zero values
  }

  float average = 0;
  nonZeroReadingsVoltage[nonZeroIndexVoltage++] = value;

  if (VoltageCount >= 100) { 
    float sum = 0.0f; 
    for (int i = 0; i < nonZeroIndexVoltage; i++) { 
      sum += nonZeroReadingsVoltage[i]; 
      } 
      
      average = sum/100; 
      // Serial.print("Recent Average Voltage: "); 
      // Serial.println(average);

    nonZeroIndexVoltage = 0; // Reset counters
    VoltageCount = 0;

  }
  return average; 
}

float processADC(float value) { 
  ADCCount++;
  if (value == 0.0f) {
    return 0; // Ignore zero values
  }

  float average = 0;
  nonZeroReadingsADC[nonZeroIndexADC++] = value;

  if (ADCCount >= 100) { 
    float sum = 0.0f; 
    for (int i = 0; i < nonZeroIndexADC; i++) { 
      sum += nonZeroReadingsADC[i]; 
      } 
      
      average = sum/100; 
      // Serial.print("Recent Average ADC Value: "); 
      // Serial.println(average);

    nonZeroIndexADC = 0; // Reset counters
    ADCCount = 0;

  } 
  return average;
}

float processConductivity(float value) { 
  ConductivityCount++;
  // if (value <= 0.0f) {
  //   return 0; // Ignore zero values
  // }

  float average = 0;
  nonZeroReadingsConductivity[nonZeroIndexConductivity++] = value;

  if (ConductivityCount >= 100) { 
    float sum = 0.0f; 
    for (int i = 0; i < nonZeroIndexConductivity; i++) { 
      sum += nonZeroReadingsConductivity[i]; 
      } 
      
      average = sum/100; 
      Serial.print("Recent Average Conductivity Value: "); 
      Serial.println(average);

    nonZeroIndexConductivity = 0; // Reset counters
    ConductivityCount = 0;

  } 
  return average;
}
