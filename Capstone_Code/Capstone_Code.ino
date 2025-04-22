//Right now, serial command is written for both the USB and the RX and TX pins

#define _PWM_LOGLEVEL_       4

// Select false to use PWM
#define USING_TIMER       false   //true

#include "nRF52_PWM.h"

#include <Adafruit_TinyUSB.h>

#define pinToUse       A0
//creates pwm instance
nRF52_PWM* PWM_Instance;

float frequency;

float dutyCycle;

void setup() {
  // put your setup code here, to run once:
  //Starts a serial connection with the RX and TX pins, and the USB port
  Serial1.begin(9600);
  Serial.begin(9600);

  frequency = 500000.0f;

  dutyCycle = 53.5f; //Should be 50, in practice this is the number that gets us a 50% duty cycle

  PWM_Instance = new nRF52_PWM(pinToUse, frequency, dutyCycle);

  // Serial.print(F("Change PWM Freq to "));
  // Serial.println(frequency);
  //PWM_Instance->setPWM(pinToUse, frequency, dutyCycle);

}

void loop() {
  // put your main code here, to run repeatedly:


  delay(50);

  //Reads the voltage from the Conductivity sensor, and sends that value to the serial port.
  int adc_value = analogRead(A1);
  //float voltage = adc_value * (3.3/1023.0);
  Serial1.print("ADC value: ");
  Serial.print("ADC value: ");
  Serial1.println(adc_value);
  Serial.println(adc_value);


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
