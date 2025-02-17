////////// M5 Atom //////////
#include "M5Atom.h"

////////// Temperature Sensors //////////
#include <HDC2080.h>
#define ADDR 0x40
#define ADDR2 0x41
HDC2080 sensor(ADDR);
HDC2080 sensor2(ADDR2);
#define HEAT_PIN_SWITCH 22

/* GLOBAL VARIABLES */

////////// FreeRTOS tasks //////////
#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

TaskHandle_t xTaskLoopHandle = NULL;
void TaskHeatPulse(void *pvParameters);

unsigned long tempSensorTimer = 0;

//Using millis instead of delay
unsigned long currentMillis;
unsigned long previousMillis = 0;

//Counter for millis since heat pulse was fired
unsigned long millisSinceHeatPulse = 0;

//Internal counter for starting/stopping heat pulse
unsigned long millisStartHeatPulse = 0;

//Millis value for last time heater was turned on
unsigned long previousHeaterOnTime = 0;

//Counter for millis since we started tracking reference temperatures
unsigned long millisStartReferenceTemp = 0;
unsigned long millisSinceReferenceTemp = 0;

float temp1;
float temp2;

struct HeatRatioDataPoint
{
  float temp1;
  float temp2;
  unsigned long millisSinceHeatPulse;
  uint32_t rtcUnixTimestamp;
  unsigned long millisSinceReferenceTemp;
};

HeatRatioDataPoint datapoints[100];
int datapointIndex = 0;

float meanHeatRatio = -1;

////////// Heat Pulse //////////
unsigned long heatPulseDuration = 10000;
unsigned long timeBetweenHeatPulses = 900; /* 1800 seconds = 30 minutes */

/////////////// Serial /////////////////

#define rxPin 26
#define txPin 32

void setup() {
  M5.begin(false, false, true);
  
  // Set heat pin as an output
  pinMode(HEAT_PIN_SWITCH, OUTPUT);

  // Setup Serial
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  Serial.begin(115200);
  //Serial2.begin(115200, SERIAL_8N1, rxPin, txPin);
  //while (!Serial2)
  //  ;
  //delay(5000);

  // Initialize I2C communication
  sensor.begin(25, 21);
  sensor2.begin(25, 21);

  // Begin with a device reset
  sensor.reset();
  sensor2.reset();

  // Configure Measurements
  sensor.setMeasurementMode(TEMP_AND_HUMID); // Set measurements to temperature and humidity
  sensor2.setMeasurementMode(TEMP_AND_HUMID);
  sensor.setRate(ONE_HZ); // Set measurement frequency to 1 Hz
  sensor2.setRate(ONE_HZ);
  sensor.setTempRes(FOURTEEN_BIT);
  sensor2.setTempRes(FOURTEEN_BIT);
  sensor.setHumidRes(FOURTEEN_BIT);
  sensor2.setHumidRes(FOURTEEN_BIT);

  //begin measuring
  sensor.triggerMeasurement();
  sensor2.triggerMeasurement();

  xTaskCreatePinnedToCore(
      TaskHeatPulse, "TaskHeatPulse" // A name just for humans
      ,
      4096 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL, ARDUINO_RUNNING_CORE);

  xTaskCreatePinnedToCore(
      TaskWriteDataToSerial, "TaskWriteDataToSerial" // A name just for humans
      ,
      4096 // This stack size can be checked & adjusted by reading the Stack Highwater
      ,
      NULL, 2 // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,
      NULL, ARDUINO_RUNNING_CORE);
}

void loop() {
  // Empty, since FreeRTOS takes care of task management
}

void recordDatapoint(
    float temp1,
    float temp2,
    unsigned long millisSinceHeatPulse,
    unsigned long millisSinceReferenceTemp)
{
  // write to RAM
  datapoints[datapointIndex] = {
      temp1,
      temp2,
      millisSinceHeatPulse,
      millisSinceReferenceTemp};

  //Serial.print(F("Recorded datapoint at datapointIndex: "));
  //Serial.println(datapointIndex);
  //Serial.print(F("millisSinceReferenceTemp at current index: "));
  //Serial.println(datapoints[datapointIndex].millisSinceReferenceTemp);

  datapointIndex++;
}

float calculateMeanHeatRatio(
    unsigned long heatRatioWindowStart,
    unsigned long heatRatioWindowEnd)
{
  // Mean = cumulative sum / number of datapoints
  float cumulativeSumReferenceTemp1 = 0;
  int cumulativeSumReferenceTemp1DataPointsUsed = 0;

  float cumulativeSumReferenceTemp2 = 0;
  int cumulativeSumReferenceTemp2DataPointsUsed = 0;

  float heatRatioCumulativeSum = 0;
  int heatRatioDataPointPairsUsed = 0;

  // Mean values
  float referenceTemp1;
  float referenceTemp2;
  float meanHeatRatio = -1;

  for (const HeatRatioDataPoint &datapoint : datapoints)
  {
    if (
        datapoint.temp1 > 0 && datapoint.temp1 < 70 && datapoint.temp2 > 0 && datapoint.temp2 < 70)
    {
      if (datapoint.millisSinceReferenceTemp < 10000)
      {
        cumulativeSumReferenceTemp1 += datapoint.temp1;
        cumulativeSumReferenceTemp1DataPointsUsed++;
        cumulativeSumReferenceTemp2 += datapoint.temp2;
        cumulativeSumReferenceTemp2DataPointsUsed++;
      }
    }
  }

  // guard against division by 0
  if (
      !cumulativeSumReferenceTemp1DataPointsUsed || !cumulativeSumReferenceTemp2DataPointsUsed)
  {
    return -1;
  }

  referenceTemp1 = cumulativeSumReferenceTemp1 / cumulativeSumReferenceTemp1DataPointsUsed;
  referenceTemp2 = cumulativeSumReferenceTemp2 / cumulativeSumReferenceTemp2DataPointsUsed;

  for (const HeatRatioDataPoint &datapoint : datapoints)
  {
    if (
        datapoint.temp1 > 0 && datapoint.temp1 < 40 && datapoint.temp2 > 0 && datapoint.temp2 < 40)
    {
      if (
          datapoint.millisSinceHeatPulse > heatRatioWindowStart && datapoint.millisSinceHeatPulse < heatRatioWindowEnd)
      {

        heatRatioCumulativeSum += (datapoint.temp2 - referenceTemp2) / (datapoint.temp1 - referenceTemp1);
        heatRatioDataPointPairsUsed++;
      }
    }
  }

  // guard against division by 0
  if (!heatRatioDataPointPairsUsed)
  {
    return -1;
  }

  meanHeatRatio = heatRatioCumulativeSum / heatRatioDataPointPairsUsed;
  return meanHeatRatio;
}

void TaskHeatPulse(void *pvParameters) // This is a task.
{
  (void)pvParameters;

  unsigned long millisSinceHeatPulse;
  unsigned long millisSinceReferenceTemp;

  unsigned long millisStartReferenceTemp;
  unsigned long millisStartHeatPulse;
  uint32_t heatPulseId;

  char databuf[512];
  int numDatapointsRecorded;

  for (;;)
  {
    numDatapointsRecorded = 0;
    // Record reference temperatures
    //    Serial.println(F("Recording baseline reference temperatures..."));
    millisStartReferenceTemp = millis();
    heatPulseId = esp_random();

    for (int i = 0; i < 10; i++)
    {
      millisSinceHeatPulse = 0;
      millisSinceReferenceTemp = millis() - millisStartReferenceTemp;
      //Serial.print(F("millisSinceReferenceTemp: "));
      //Serial.println(millisSinceReferenceTemp);
      
      temp1 = sensor.readTemp();
      //Serial.print(F("Sensor 1 Temperature (C): "));
      //Serial.print(temp1, 10);
      temp2 = sensor2.readTemp();
      //Serial.print(F(" Sensor 2 Temperature (C): "));
      //Serial.print(temp2, 10);
      //Serial.print(F("millisSinceHeatPulse: "));
      //Serial.println(millisSinceHeatPulse);
      recordDatapoint(temp1, temp2, millisSinceHeatPulse, millisSinceReferenceTemp);
      numDatapointsRecorded++;

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Fire heat pulse and record temperatures
    //Serial.print(F("Firing heat pulse for: "));
    //Serial.print(heatPulseDuration / 1000);
    //Serial.println(F(" seconds"));
    
    digitalWrite(HEAT_PIN_SWITCH, HIGH);
    millisStartHeatPulse = millis();

    for (int i = 0; i < _min((heatPulseDuration / 1000), 30); i++)
    {
      millisSinceHeatPulse = millis() - millisStartHeatPulse;
      millisSinceReferenceTemp = millis() - millisStartReferenceTemp;
      //Serial.print(F("millisSinceReferenceTemp: "));
      //Serial.println(millisSinceReferenceTemp);

      temp1 = sensor.readTemp();
      //Serial.print(F("Sensor 1 Temperature (C): "));
      //Serial.print(temp1, 10);
      temp2 = sensor2.readTemp();
      //Serial.print(F(" Sensor 2 Temperature (C): "));
      //Serial.print(temp2, 10);
      //Serial.print(F("millisSinceHeatPulse: "));
      //Serial.println(millisSinceHeatPulse);
      recordDatapoint(temp1, temp2, millisSinceHeatPulse, millisSinceReferenceTemp);
      numDatapointsRecorded++;

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    //Serial.println(F("Turning off heat pulse"));
    digitalWrite(HEAT_PIN_SWITCH, LOW);

    while (numDatapointsRecorded < 100)
    {
      millisSinceHeatPulse = millis() - millisStartHeatPulse;
      millisSinceReferenceTemp = millis() - millisStartReferenceTemp;
      //Serial.print(F("millisSinceReferenceTemp: "));
      //Serial.println(millisSinceReferenceTemp);

      temp1 = sensor.readTemp();
      //Serial.print(F("Sensor 1 Temperature (C): "));
      //Serial.print(temp1, 10);
      temp2 = sensor2.readTemp();
      //Serial.print(F(" Sensor 2 Temperature (C): "));
      //Serial.print(temp2, 10);
      //Serial.print(F("millisSinceHeatPulse: "));
      //Serial.println(millisSinceHeatPulse);
      recordDatapoint(temp1, temp2, millisSinceHeatPulse, millisSinceReferenceTemp);
      numDatapointsRecorded++;

      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    //    Serial.println("Deleting heat pulse task.");
    //    vTaskDelete(NULL);

    meanHeatRatio = calculateMeanHeatRatio(55000, 75000);
    //Serial.print(F("Waiting. Next heat pulse in: "));
    //Serial.println(String(timeBetweenHeatPulses) + " seconds");
    vTaskDelay(timeBetweenHeatPulses * 1000 / portTICK_PERIOD_MS);
    ESP.restart();
  }
}

void TaskWriteDataToSerial(void *pvParameters) // This is a task.
{
  (void)pvParameters;
  for(;;){ // infinite loop 
    // Print to Serial
    temp1 = sensor.readTemp();
    Serial.print("TreeTemp1,");
    Serial.print(temp1);
    temp2 = sensor2.readTemp();
    Serial.print(",TreeTemp2,");
    Serial.print(temp2);
    Serial.print(",SapFlow,");
    Serial.println(meanHeatRatio);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
