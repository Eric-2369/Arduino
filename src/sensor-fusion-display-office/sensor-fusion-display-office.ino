#include "arduino_secrets.h"
#include "thingProperties.h"

#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2cSht3x.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2CSgp40.h>
#include <Adafruit_BMP3XX.h>
#include <Arduino_LED_Matrix.h>
#include "animation.h"
#include <VOCGasIndexAlgorithm.h>

SensirionI2cSht3x sht30;
SensirionI2CScd4x scd40;
SensirionI2CSgp40 sgp40;
Adafruit_BMP3XX bmp390;
ArduinoLEDMatrix matrix;
VOCGasIndexAlgorithm vocAlgorithm;

float sht30Temperature = NAN;
float sht30Humidity = NAN;
float scd40Temperature = NAN;
float scd40Humidity = NAN;
uint16_t scd40CO2Concentration = 0;
uint16_t sgp40VOCRaw = 0;
int32_t sgp40VOCIndex = 0;
float bmp390Temperature = NAN;
float bmp390Pressure = NAN;

unsigned long lastSHT30ReadTime = 0;
unsigned long lastSCD40ReadTime = 0;
unsigned long lastSGP40ReadTime = 0;
unsigned long lastBMP390ReadTime = 0;

const unsigned long readInterval = 1000;

void initializeSHT30() {
  sht30.begin(Wire, SHT30_I2C_ADDR_44);
  sht30.stopMeasurement();
  delay(1);
  sht30.softReset();
  delay(100);
  sht30.startPeriodicMeasurement(REPEATABILITY_HIGH, MPS_ONE_PER_SECOND);
}

void initializeSCD40() {
  scd40.begin(Wire);
  scd40.stopPeriodicMeasurement();
  scd40.startPeriodicMeasurement();
}

void initializeSGP40() {
  sgp40.begin(Wire);
  uint16_t testResult;
  uint16_t error = sgp40.executeSelfTest(testResult);
  if (error || testResult != 0xD400) {
    Serial.println("SGP40 initialization failed.");
  }
}

void initializeBMP390() {
  if (bmp390.begin_I2C()) {
    bmp390.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp390.setOutputDataRate(BMP3_ODR_25_HZ);
  } else {
    Serial.println("BMP390 initialization failed.");
  }
}

void initializeLEDMatrix() {
  matrix.loadSequence(frames);
  matrix.begin();
  matrix.play(true);
}

void readSHT30Data() {
  float tempTemperature = 0.0;
  float tempHumidity = 0.0;
  int16_t sht30Error = sht30.blockingReadMeasurement(tempTemperature, tempHumidity);
  if (sht30Error == NO_ERROR) {
    sht30Temperature = tempTemperature;
    sht30Humidity = tempHumidity;
  } else {
    Serial.println("SHT30 measurement error.");
  }
}

void readSCD40Data() {
  float tempSCD40Temperature = 0.0f;
  float tempSCD40Humidity = 0.0f;
  uint16_t tempCO2Concentration = 0;
  bool isDataReady = false;
  scd40.getDataReadyFlag(isDataReady);
  if (isDataReady) {
    uint16_t scd40Error = scd40.readMeasurement(tempCO2Concentration, tempSCD40Temperature, tempSCD40Humidity);
    if (scd40Error == 0 && tempCO2Concentration != 0) {
      scd40Temperature = tempSCD40Temperature;
      scd40Humidity = tempSCD40Humidity;
      scd40CO2Concentration = tempCO2Concentration;
    } else {
      Serial.println("SCD40 measurement error.");
    }
  }
}

void readSGP40Data() {
  uint16_t sgp40Error;
  uint16_t defaultRh = 0x8000;
  uint16_t defaultT = 0x6666;
  uint16_t compensationRh = 0;
  uint16_t compensationT = 0;

  if (!isnan(sht30Temperature) && !isnan(sht30Humidity)) {
    compensationT = static_cast<uint16_t>((sht30Temperature + 45) * 65535 / 175);
    compensationRh = static_cast<uint16_t>(sht30Humidity * 65535 / 100);
  } else {
    compensationRh = defaultRh;
    compensationT = defaultT;
  }

  sgp40Error = sgp40.measureRawSignal(compensationRh, compensationT, sgp40VOCRaw);

  if (sgp40Error == 0) {
    sgp40VOCIndex = vocAlgorithm.process(sgp40VOCRaw);
  } else {
    Serial.println("SGP40 measurement error.");
  }
}

void readBMP390Data() {
  if (bmp390.performReading()) {
    bmp390Temperature = bmp390.temperature;
    bmp390Pressure = bmp390.pressure / 1000.0;
  } else {
    Serial.println("BMP390 measurement error.");
  }
}

void printDataToSerial() {
  Serial.print("SHT30 Temperature: ");
  Serial.print(isnan(sht30Temperature) ? "N/A " : String(sht30Temperature) + "C ");
  Serial.print("Humidity: ");
  Serial.print(isnan(sht30Humidity) ? "N/A | " : String(sht30Humidity) + "% | ");


  Serial.print("SCD40 Temperature: ");
  Serial.print(isnan(scd40Temperature) ? "N/A " : String(scd40Temperature) + "C ");
  Serial.print("Humidity: ");
  Serial.print(isnan(scd40Humidity) ? "N/A " : String(scd40Humidity) + "% ");
  Serial.print("CO2 Concentration: ");
  Serial.print(scd40CO2Concentration != 0 ? String(scd40CO2Concentration) + "ppm | " : "N/A | ");

  Serial.print("SGP40 VOC Index: ");
  Serial.print(sgp40VOCIndex != 0 ? String(sgp40VOCIndex) + " | " : "N/A | ");

  Serial.print("BMP390 Temperature: ");
  Serial.print(isnan(bmp390Temperature) ? "N/A " : String(bmp390Temperature) + "C ");
  Serial.print("Pressure: ");
  Serial.print(isnan(bmp390Pressure) ? "N/A | " : String(bmp390Pressure, 3) + "kPa");

  Serial.println();
}

void updateCloudVariables() {
  cloud_sht30Temperature = sht30Temperature;
  cloud_sht30Humidity = sht30Humidity;
  cloud_scd40Temperature = scd40Temperature;
  cloud_scd40Humidity = scd40Humidity;
  cloud_scd40CO2Concentration = scd40CO2Concentration;
  cloud_sgp40VOCRaw = sgp40VOCRaw;
  cloud_sgp40VOCIndex = sgp40VOCIndex;
  cloud_bmp390Temperature = bmp390Temperature;
  cloud_bmp390Pressure = bmp390Pressure;
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }
  delay(1500);

  Wire.begin();

  initializeSHT30();
  initializeSCD40();
  initializeSGP40();
  initializeBMP390();
  initializeLEDMatrix();

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  Serial.println("All devices have been initialized.");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastSHT30ReadTime >= readInterval) {
    lastSHT30ReadTime = currentMillis;
    readSHT30Data();
  }

  if (currentMillis - lastSCD40ReadTime >= readInterval) {
    lastSCD40ReadTime = currentMillis;
    readSCD40Data();
  }

  if (currentMillis - lastSGP40ReadTime >= readInterval) {
    lastSGP40ReadTime = currentMillis;
    readSGP40Data();
  }

  if (currentMillis - lastBMP390ReadTime >= readInterval) {
    lastBMP390ReadTime = currentMillis;
    readBMP390Data();
  }

  printDataToSerial();

  updateCloudVariables();
  ArduinoCloud.update();
}
