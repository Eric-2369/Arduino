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
uint16_t scd41CO2 = 0;
uint16_t sgp40RawVoc = 0;
int32_t sgp40VocIndex = 0;
float bmp180Temperature = NAN;
float bmp180Pressure = NAN;

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
  } else {
    Serial.println("SGP40 self-test passed.");
  }
}

void initializeBMP180() {
  if (!bmp180.begin()) {
    Serial.println("BMP180 initialization failed.");
    while (1)
      ;
  }
  bmp180.resetToDefaults();
  bmp180.setSamplingMode(BMP180MI::MODE_UHR);
}

void initializeOLED() {
  u8g2.setI2CAddress(0x3C * 2);
  u8g2.begin();
  u8g2.enableUTF8Print();
}

void initializeLEDMatrix() {
  matrix.loadSequence(frames);
  matrix.begin();
  matrix.play(false);
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
  uint16_t tempCO2 = 0;
  float tempScd41Temperature = 0.0f;
  float tempScd41Humidity = 0.0f;
  bool isDataReady = false;
  scd41.getDataReadyFlag(isDataReady);
  if (isDataReady) {
    uint16_t scd41Error = scd41.readMeasurement(tempCO2, tempScd41Temperature, tempScd41Humidity);
    if (scd41Error == 0 && tempCO2 != 0) {
      scd41CO2 = tempCO2;
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

  uint16_t tempSrawVoc = 0;
  sgp40Error = sgp40.measureRawSignal(compensationRh, compensationT, tempSrawVoc);
  if (sgp40Error == 0) {
    sgp40RawVoc = tempSrawVoc;
    sgp40VocIndex = vocAlgorithm.process(sgp40RawVoc);
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
    u8g2.print(scd41CO2 != 0 ? String(scd41CO2) + "ppm" : "N/A");

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
  Serial.print(scd41CO2 != 0 ? String(scd41CO2) + "ppm | " : "N/A | ");

  Serial.print("SGP40 VOC: ");
  Serial.print(sgp40VocIndex != 0 ? String(sgp40VocIndex) + " | " : "N/A | ");

  Serial.print("BMP180 Temperature: ");
  Serial.print(isnan(bmp180Temperature) ? "N/A " : String(bmp180Temperature, 1) + "C ");
  Serial.print("Pressure: ");
  Serial.print(isnan(bmp180Pressure) ? "N/A" : String(bmp180Pressure / 1000.0, 3) + "kPa");

  Serial.println();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  Wire.begin();

  initializeSHT45();
  initializeSCD41();
  initializeSGP40();
  initializeBMP180();
  initializeOLED();
  initializeLEDMatrix();

  Serial.println("All devices have been initialized.");
}

void loop() {
  readSHT45Data();
  readSCD41Data();
  readSGP40Data();
  readBMP180Data();

  displayData();
  printDataToSerial();

  if (matrix.sequenceDone()) {
    matrix.play(false);
  }

  delay(1000);
}