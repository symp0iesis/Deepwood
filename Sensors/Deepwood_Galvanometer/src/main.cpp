//DEEPWOOD SENSING STACK
//V1 - biodata galvanometer (Sam Cusumanu) implementation with custom pcb

#include <Arduino.h>
#include "helper.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// I/O Pin declarations
// int buttonPin = 13;
// int potPin = A2;
const byte interruptPin = 12;         //galvanometer input

//OLED
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

void setup() {
  Serial.begin(115200);
  // while (!Serial) { } // Wait for the Serial to be ready

  //Pins
  pinMode(interruptPin, INPUT_PULLUP); //pulse input
  attachInterrupt(interruptPin, sample, CHANGE);  //begin sampling from interrupt OPTION: CHANGE

  ///OLED
  display.begin(0x3C, true); // Address 0x3C default
  Serial.println("OLED begun");
  // Clear the buffer.
  display.clearDisplay();
  display.setRotation(3);
  display.setTextSize(2);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0,0);
  display.println("Deepwood");
  display.println("Galvanometer");
  display.display();
  
  // Button initialization
  //pinMode(buttonA, INPUT_PULLUP);
  //pinMode(buttonB, INPUT_PULLUP);
  //pinMode(buttonC, INPUT_PULLUP);

  Serial.print("Setup OK");
  display.println("Setup OK");
  display.display();
  delay(5000);
}

void loop() {;
  //Serial.println("loop:");
  if( sampleIndex >= samplesize )  { analyzeSample(); }

  // Serial Print data

  Serial.print("GalvAvera,");
  Serial.print(rawAverage);
  Serial.print(",GalvStdevi,");
  Serial.print(rawStdevi);
  Serial.print(",GalvDelta,");
  Serial.println(rawDelta);

  // Display data on OLED 
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Galvanometer:");
  display.println("");
  display.print("Average: ");
  display.println(rawAverage);
  display.print("Stdevi: ");
  display.println(rawStdevi);
  //display.print("ThreshDelta: ");
  //display.println(rawThreshDelta);
  display.print("Delta: ");
  display.println(rawDelta);
  display.display();

  delay(100);
}