#ifndef DEVICE_HANDLERS_H
#define DEVICE_HANDLERS_H

#include <SensirionI2cSht4x.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2CSgp41.h>
#include <SensirionI2CSfa3x.h>
#include <VOCGasIndexAlgorithm.h>
#include <NOxGasIndexAlgorithm.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_TSL2561_U.h>
#include <LiquidCrystal_PCF8574.h>
#include <U8g2lib.h>

void initializeSHT4x(SensirionI2cSht4x& sht4x);
void initializeSCD4x(SensirionI2CScd4x& scd4x);
void initializeSGP40(SensirionI2CSgp40& sgp40);
void initializeSGP41(SensirionI2CSgp41& sgp41);
void initializeSFA3x(SensirionI2CSfa3x& sfa3x);
void initializeBMP3xx(Adafruit_BMP3XX& bmp3xx);
void initializeTSL2561(Adafruit_TSL2561_Unified& tsl2561);
void initializeLCD(LiquidCrystal_PCF8574& lcd);
void initializeOLED(U8G2_SSD1306_128X64_NONAME_1_HW_I2C& u8g2);

void readSHT4xData(SensirionI2cSht4x& sht4x, float& temperature, float& humidity);
void readSCD4xData(SensirionI2CScd4x& scd4x, float& temperature, float& humidity, uint16_t& co2Concentration);
void readSGP40Data(SensirionI2CSgp40& sgp40, VOCGasIndexAlgorithm& vocAlgorithm, float temperature, float humidity, uint16_t& vocRaw, int32_t& vocIndex);
void readSGP41Data(SensirionI2CSgp41& sgp41, VOCGasIndexAlgorithm& vocAlgorithm, NOxGasIndexAlgorithm& noxAlgorithm, float temperature, float humidity, uint16_t& vocRaw, uint16_t& noxRaw, int32_t& vocIndex, int32_t& noxIndex, uint16_t& conditioningTime);
void readSFA3xData(SensirionI2CSfa3x& sfa3x, float& temperature, float& humidity, float& ch2oConcentration);
void readBMP3xxData(Adafruit_BMP3XX& bmp3xx, float& temperature, float& pressure);
void readTSL2561Data(Adafruit_TSL2561_Unified& tsl2561, float& illuminance);

#endif  // DEVICE_HANDLERS_H
