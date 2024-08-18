#include "arduino_secrets.h"
#include "thingProperties.h"

#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2cSht4x.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2CSgp40.h>
#include <BMP180I2C.h>
#include <U8g2lib.h>
#include <Arduino_LED_Matrix.h>
#include "animation.h"
#include <VOCGasIndexAlgorithm.h>

SensirionI2cSht4x sht45;
SensirionI2CScd4x scd41;
SensirionI2CSgp40 sgp40;
BMP180I2C bmp180(0x77);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
ArduinoLEDMatrix matrix;
VOCGasIndexAlgorithm vocAlgorithm;

float sht45Temperature = NAN;
float sht45Humidity = NAN;
float scd41Temperature = NAN;
float scd41Humidity = NAN;
uint16_t scd41CO2Concentration = 0;
uint16_t sgp40VocRaw = 0;
int32_t sgp40VocIndex = 0;
float bmp180Temperature = NAN;
float bmp180Pressure = NAN;

unsigned long lastSHT45ReadTime = 0;
unsigned long lastSCD41ReadTime = 0;
unsigned long lastSGP40ReadTime = 0;
unsigned long lastBMP180ReadTime = 0;

const unsigned long readInterval = 1000;

void initializeSHT45() {
  sht45.begin(Wire, SHT40_I2C_ADDR_44);
  sht45.softReset();
}

void initializeSCD41() {
  scd41.begin(Wire);
  scd41.stopPeriodicMeasurement();
  scd41.startPeriodicMeasurement();
}

void initializeSGP40() {
  sgp40.begin(Wire);
  uint16_t testResult;
  uint16_t error = sgp40.executeSelfTest(testResult);
  if (error || testResult != 0xD400) {
    Serial.println("SGP40 self-test failed.");
  }
}

void initializeBMP180() {
  if (bmp180.begin()) {
    bmp180.resetToDefaults();
    bmp180.setSamplingMode(BMP180MI::MODE_UHR);
  } else {
    Serial.println("BMP180 initialization failed.");
  }
}

void initializeOLED() {
  u8g2.setI2CAddress(0x3C * 2);
  u8g2.begin();
  u8g2.enableUTF8Print();
}

void initializeLEDMatrix() {
  matrix.loadSequence(frames);
  matrix.begin();
  matrix.play(true);
}

void readSHT45Data() {
  float tempTemperature = 0.0;
  float tempHumidity = 0.0;
  int16_t sht45Error = sht45.measureHighPrecision(tempTemperature, tempHumidity);
  if (sht45Error == 0) {
    sht45Temperature = tempTemperature;
    sht45Humidity = tempHumidity;
  } else {
    Serial.println("SHT45 measurement error.");
  }
}

void readSCD41Data() {
  uint16_t tempCO2Concentration = 0;
  float tempScd41Temperature = 0.0f;
  float tempScd41Humidity = 0.0f;
  bool isDataReady = false;
  scd41.getDataReadyFlag(isDataReady);
  if (isDataReady) {
    uint16_t scd41Error = scd41.readMeasurement(tempCO2Concentration, tempScd41Temperature, tempScd41Humidity);
    if (scd41Error == 0 && tempCO2Concentration != 0) {
      scd41CO2Concentration = tempCO2Concentration;
      scd41Temperature = tempScd41Temperature;
      scd41Humidity = tempScd41Humidity;
    } else {
      Serial.println("SCD41 measurement error or invalid sample.");
    }
  }
}

void readSGP40Data() {
  uint16_t sgp40Error;
  uint16_t defaultRh = 0x8000;
  uint16_t defaultT = 0x6666;
  uint16_t compensationRh = 0;
  uint16_t compensationT = 0;

  if (!isnan(sht45Temperature) && !isnan(sht45Humidity)) {
    compensationT = static_cast<uint16_t>((sht45Temperature + 45) * 65535 / 175);
    compensationRh = static_cast<uint16_t>(sht45Humidity * 65535 / 100);
  } else {
    compensationRh = defaultRh;
    compensationT = defaultT;
  }

  uint16_t tempVocRaw = 0;
  sgp40Error = sgp40.measureRawSignal(compensationRh, compensationT, tempVocRaw);
  if (sgp40Error == 0) {
    sgp40VocRaw = tempVocRaw;
    sgp40VocIndex = vocAlgorithm.process(sgp40VocRaw);
  } else {
    Serial.println("SGP40 measurement error.");
  }
}

void readBMP180Data() {
  if (bmp180.measureTemperature()) {
    while (!bmp180.hasValue()) {
      delay(100);
    }
    bmp180Temperature = bmp180.getTemperature();
  } else {
    Serial.println("BMP180 temperature measurement error.");
  }

  if (bmp180.measurePressure()) {
    while (!bmp180.hasValue()) {
      delay(100);
    }
    bmp180Pressure = bmp180.getPressure();
  } else {
    Serial.println("BMP180 pressure measurement error.");
  }
}

void displayData() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont12_mf);
    u8g2.setFontPosTop();

    u8g2.setCursor(0, 0);
    u8g2.print("SHT45 ");
    u8g2.print(isnan(sht45Temperature) ? "N/A " : String(sht45Temperature) + "C ");
    u8g2.print(isnan(sht45Humidity) ? "N/A" : String(sht45Humidity) + "%");

    u8g2.setCursor(0, 10);
    u8g2.print("SCD41 ");
    u8g2.print(isnan(scd41Temperature) ? "N/A " : String(scd41Temperature) + "C ");
    u8g2.print(isnan(scd41Humidity) ? "N/A" : String(scd41Humidity) + "%");

    u8g2.setCursor(0, 20);
    u8g2.print("SCD41 CO2 ");
    u8g2.print(scd41CO2Concentration != 0 ? String(scd41CO2Concentration) + "ppm" : "N/A");

    u8g2.setCursor(0, 30);
    u8g2.print("SGP40 VOC ");
    u8g2.print(sgp40VocIndex != 0 ? String(sgp40VocIndex) : "N/A");

    u8g2.setCursor(0, 40);
    u8g2.print("BMP180 ");
    u8g2.print(isnan(bmp180Temperature) ? "N/A " : String(bmp180Temperature, 1) + "C ");
    u8g2.print(isnan(bmp180Pressure) ? "N/A" : String(bmp180Pressure / 1000.0, 3) + "kPa");
  } while (u8g2.nextPage());
}

void printDataToSerial() {
  Serial.print("SHT45 Temperature: ");
  Serial.print(isnan(sht45Temperature) ? "N/A " : String(sht45Temperature) + "C ");
  Serial.print("Humidity: ");
  Serial.print(isnan(sht45Humidity) ? "N/A | " : String(sht45Humidity) + "% | ");

  Serial.print("SCD41 Temperature: ");
  Serial.print(isnan(scd41Temperature) ? "N/A " : String(scd41Temperature) + "C ");
  Serial.print("Humidity: ");
  Serial.print(isnan(scd41Humidity) ? "N/A " : String(scd41Humidity) + "% ");
  Serial.print("CO2: ");
  Serial.print(scd41CO2Concentration != 0 ? String(scd41CO2Concentration) + "ppm | " : "N/A | ");

  Serial.print("SGP40 VOC: ");
  Serial.print(sgp40VocIndex != 0 ? String(sgp40VocIndex) + " | " : "N/A | ");

  Serial.print("BMP180 Temperature: ");
  Serial.print(isnan(bmp180Temperature) ? "N/A " : String(bmp180Temperature, 1) + "C ");
  Serial.print("Pressure: ");
  Serial.print(isnan(bmp180Pressure) ? "N/A" : String(bmp180Pressure / 1000.0, 3) + "kPa");

  Serial.println();
}

void updateCloudVariables() {
  cloud_sht45Temperature = sht45Temperature;
  cloud_sht45Humidity = sht45Humidity;
  cloud_scd41Temperature = scd41Temperature;
  cloud_scd41Humidity = scd41Humidity;
  cloud_scd41CO2Concentration = scd41CO2Concentration;
  cloud_sgp40VocRaw = sgp40VocRaw;
  cloud_sgp40VocIndex = sgp40VocIndex;
  cloud_bmp180Temperature = bmp180Temperature;
  cloud_bmp180Pressure = bmp180Pressure;
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }
  delay(1500);

  Wire.begin();

  initializeSHT45();
  initializeSCD41();
  initializeSGP40();
  initializeBMP180();
  initializeOLED();
  initializeLEDMatrix();

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  Serial.println("All devices have been initialized.");
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastSHT45ReadTime >= readInterval) {
    lastSHT45ReadTime = currentMillis;
    readSHT45Data();
  }

  if (currentMillis - lastSCD41ReadTime >= readInterval) {
    lastSCD41ReadTime = currentMillis;
    readSCD41Data();
  }

  if (currentMillis - lastSGP40ReadTime >= readInterval) {
    lastSGP40ReadTime = currentMillis;
    readSGP40Data();
  }

  if (currentMillis - lastBMP180ReadTime >= readInterval) {
    lastBMP180ReadTime = currentMillis;
    readBMP180Data();
  }

  displayData();
  printDataToSerial();

  updateCloudVariables();
  ArduinoCloud.update();
}
