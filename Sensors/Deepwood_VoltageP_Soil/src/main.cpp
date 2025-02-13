#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_seesaw.h>

// Inits
#define VoltagePin A0
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // Use dedicated hardware SPI pins for display
Adafruit_seesaw ss;
float moisture = 0;
float temperature = 0;
float voltage = 0;

void setup() {
  Serial.begin(115200);
  //while (!Serial) delay(10);   // wait until serial port is opened
  Wire.begin();
  
    // ## Display
  // turn on backlite
  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  // turn on the TFT / I2C power supply
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(10);

  // initialize TFT
  tft.init(135, 240); // Init ST7789 240x135
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(0, 0); // x = 10, y = 20
  tft.println("Deepwood");
  tft.println("Voltage");
  tft.println("+ Soil");
  tft.println("");

  // # Soil  
  while (!ss.begin(0x36)) { // Retry initialization until successful
  Serial.println(F("seesaw not found! Retrying..."));
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.setCursor(10, 80);
  tft.println("Retrying Soil Sensor...");
  delay(1000); // Wait 1 second before retrying
  }

  tft.setTextSize(1);
  Serial.println(F("Soil Sensor Found!"));
  tft.println("Soil Sensor Found!");

  // Voltage Pin
  pinMode(VoltagePin, INPUT);
  
  Serial.println("Setup done");
  tft.println("Setup Done");

  delay(5000);
}

void loop() {

  // Soil
  moisture = ss.touchRead(0);  // Read capacitive moisture
  temperature = ss.getTemp(); // Read temperature

  // Voltage
  voltage = analogRead(VoltagePin);

  //Display
  
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(2);
  tft.setCursor(0, 0); // x = 10, y = 20
  tft.println("Voltage Potential:");
  tft.println(voltage);
  tft.println();
  tft.println("Soil Moisture:");
  tft.println(moisture);
  tft.println();
  tft.println("Soil Temperature:");
  tft.println(temperature);

  // Serial
  Serial.print("SoilMoist,");
  Serial.print(moisture);
  Serial.print(",SoilTemp,");
  Serial.print(temperature);
  Serial.print(",VoltagePotential,");
  Serial.println(voltage);


  delay(100); // 100ms sampling
}