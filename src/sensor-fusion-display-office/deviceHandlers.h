#ifndef DEVICE_HANDLERS_H
#define DEVICE_HANDLERS_H

#include <SensirionI2cSht4x.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2CSgp40.h>
#include <VOCGasIndexAlgorithm.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_TSL2561_U.h>
#include <LiquidCrystal_PCF8574.h>

void initializeSHT4x(SensirionI2cSht4x& sht4x);
void initializeSCD4x(SensirionI2CScd4x& scd4x);
void initializeSGP40(SensirionI2CSgp40& sgp40);
void initializeBMP3xx(Adafruit_BMP3XX& bmp3xx);
void initializeTSL2561(Adafruit_TSL2561_Unified& tsl2561);
void initializeLCD(LiquidCrystal_PCF8574& lcd);

void readSHT4xData(SensirionI2cSht4x& sht4x, float& temperature, float& humidity);
void readSCD4xData(SensirionI2CScd4x& scd4x, float& temperature, float& humidity, uint16_t& co2Concentration);
void readSGP40Data(SensirionI2CSgp40& sgp40, VOCGasIndexAlgorithm& vocAlgorithm, float temperature, float humidity, uint16_t& vocRaw, int32_t& vocIndex);
void readBMP3xxData(Adafruit_BMP3XX& bmp3xx, float& temperature, float& pressure);
void readTSL2561Data(Adafruit_TSL2561_Unified& tsl2561, float& illuminance);

#endif  // DEVICE_HANDLERS_H
