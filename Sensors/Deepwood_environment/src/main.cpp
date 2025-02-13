#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX1704X.h>
#include <Adafruit_Sensor.h>
#include <SensirionI2CScd4x.h>
#include <Adafruit_BME680.h>

// Auxiliary Variables
unsigned long lastCO2ReadingTime = 0; // Tracks the last CO2 sensor reading time
unsigned long lastAmbSampleTime = 0; // Tracks the last microphone sampling time

const unsigned long co2Interval = 5000; // Interval for CO2 readings (5 seconds)
const unsigned long AmbInterval = 100;  // Interval for microphone and Atmosphere readings (10 ms for ~100 Hz sampling)

float micLevel = 0; // To store the microphone reading
float temperature, humidity, pressure, gas; // for Atmosphere readings
float temperature_co2, humidity_co2; // variables for CO2 sensor readings (less accurate than BME)
uint16_t co2 = 0;
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
Adafruit_BME680 bme; // I2C


// Initializations
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); // Use dedicated hardware SPI pins for display
Adafruit_MAX17048 lipo;                           // Battery
int MicPin = A0;   // Anolog Mic input


// CO2
SensirionI2CScd4x scd4x;

void printUint16Hex(uint16_t value) {
    Serial.print(value < 4096 ? "0" : "");
    Serial.print(value < 256 ? "0" : "");
    Serial.print(value < 16 ? "0" : "");
    Serial.print(value, HEX);
}

void printSerialNumber(uint16_t serial0, uint16_t serial1, uint16_t serial2) {
    Serial.print("Serial: 0x");
    printUint16Hex(serial0);
    printUint16Hex(serial1);
    printUint16Hex(serial2);
    Serial.println();
}

void setup(void) {
  Serial.begin(115200);
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
  tft.setCursor(10, 20); // x = 10, y = 20
  tft.println("Deepwood");
  tft.println("Environment");

  /*
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLDOWN);
  pinMode(2, INPUT_PULLDOWN);
  */

  
  // ## Atmosphere sensor
  if (!bme.begin(0x77)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    tft.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }
  bme.setTemperatureOversampling(BME680_OS_8X);   // Set up oversampling and filter initialization
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
  

  // ##CO2 sensor

  uint16_t error;
  char errorMessage[256];
  // initialize sensor
  scd4x.begin(Wire);

  // stop potentially previously started measurement
  error = scd4x.stopPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute stopPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 0); // x = 10, y = 20
    tft.println("Error:");
    tft.println(errorMessage);
  }

  uint16_t serial0;
  uint16_t serial1;
  uint16_t serial2;
  error = scd4x.getSerialNumber(serial0, serial1, serial2);
  if (error) {
    Serial.print("Error trying to execute getSerialNumber(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 0); // x = 10, y = 20
    tft.println("Error:");
    tft.println(errorMessage);
  } else {
    printSerialNumber(serial0, serial1, serial2);
  }

  // Start Measurement
  error = scd4x.startPeriodicMeasurement();
  if (error) {
    Serial.print("Error trying to execute startPeriodicMeasurement(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(0, 0); // x = 10, y = 20
    tft.println("Error:");
    tft.println(errorMessage);
  }

  Serial.println("Waiting for first measurement... (5 sec)");

  // Microphone Module

  pinMode(MicPin, INPUT); 

  Serial.println("Setup done");
  tft.println("Setup Done");
  delay(5000);
}


void loop() {
  unsigned long currentTime = millis();

  // Handle microphone and Atmosphere readings (every 10 ms)
  if (currentTime - lastAmbSampleTime >= AmbInterval) {
    lastAmbSampleTime = currentTime;

    // ## Mic Reading
    micLevel = analogRead(MicPin);
    // Optional: Process micLevel here if needed (e.g., store in a buffer for FFT)

    // ## Atmosphere readings
    bme.performReading();
    temperature = bme.temperature;
    humidity = bme.humidity;
    pressure = bme.pressure/100.0;
    gas = bme.gas_resistance;

    // Print to Serial
    Serial.print("Mic,");
    Serial.print(micLevel);
    Serial.print(",CO2,");
    Serial.print(co2);
    Serial.print(",Temp,");
    Serial.print(temperature);
    Serial.print(",Humi,");
    Serial.print(humidity);
    Serial.print(",Pres,");
    Serial.print(pressure);
    Serial.print(",Gas,");
    Serial.println(gas);

    // Display TFT
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0); // x = 10, y = 20
    tft.print("Mic:");
    tft.println(micLevel);
    tft.print("Co2: ");
    tft.println(co2);
    tft.print("Temp: ");
    tft.print(temperature);
    tft.println(" C");
    tft.print("Humi: ");
    tft.print(humidity);
    tft.println(" %");
    tft.print("Press: ");
    tft.print(pressure);
    tft.println(" hPa");
    tft.print("Gas: ");
    tft.print(gas);
    tft.println(" Ohms");
  }

  // Handle CO2 sensor readings (every 5 seconds)
  if (currentTime - lastCO2ReadingTime >= co2Interval) {
    lastCO2ReadingTime = currentTime;

    uint16_t error;
    char errorMessage[256];

    // Read CO2 sensor data
    error = scd4x.readMeasurement(co2, temperature_co2, humidity_co2);
    if (error) {
      Serial.print("Error reading CO2 sensor: ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
    } else if (co2 == 0) {
      Serial.println("Invalid CO2 sample detected, skipping.");
    } else {
      /*
      Serial.print("CO2,");
      Serial.println(co2);
      Serial.print(" ppm, Temp: ");
      Serial.print(temperature);
      Serial.print(" °C, Humidity: ");
      Serial.print(humidity);
      Serial.println(" %");
      */
    }
    /*
    // Display TFT
    tft.fillScreen(ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setCursor(0, 0); // x = 10, y = 20
    tft.print("Mic:");
    tft.println(micLevel);
    tft.print("Co2: ");
    tft.println(co2);
    tft.print("Temp: ");
    tft.print(temperature);
    tft.println(" C");
    tft.print("Humi: ");
    tft.print(humidity);
    tft.println(" %");
    tft.print("Press: ");
    tft.print(pressure);
    tft.println(" hPa");
    tft.print("Gas: ");
    tft.print(gas);
    tft.println(" Ohms");
    */
  }
}