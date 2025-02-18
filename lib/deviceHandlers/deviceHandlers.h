#ifndef DEVICE_HANDLERS_H
#define DEVICE_HANDLERS_H

#include <SensirionI2cSht4x.h>
#include <SensirionI2cScd4x.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2CSgp41.h>
#include <SensirionI2CSfa3x.h>
#include <WZ.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_TSL2591.h>

#include <VOCGasIndexAlgorithm.h>
#include <NOxGasIndexAlgorithm.h>

uint8_t clearI2C(uint8_t sdaPin, uint8_t sclPin);

uint8_t initializeSHT4x(SensirionI2cSht4x& sht4x, TwoWire& wire);
uint8_t initializeSCD4x(SensirionI2cScd4x& scd4x, TwoWire& wire);
uint8_t initializeSGP40(SensirionI2CSgp40& sgp40, TwoWire& wire);
uint8_t initializeSGP41(SensirionI2CSgp41& sgp41, TwoWire& wire);
uint8_t initializeSFA3x(SensirionI2CSfa3x& sfa3x, TwoWire& wire);
uint8_t initializeWZ(WZ& wz);
uint8_t initializeBMP3xx(Adafruit_BMP3XX& bmp3xx, TwoWire& wire);
uint8_t initializeTSL2561(Adafruit_TSL2561_Unified& tsl2561, TwoWire& wire);
uint8_t initializeTSL2591(Adafruit_TSL2591& tsl2591, TwoWire& wire);

uint8_t readSHT4xData(SensirionI2cSht4x& sht4x, float& temperature, float& humidity);
uint8_t readSCD4xData(SensirionI2cScd4x& scd4x, float& temperature, float& humidity, float& co2Concentration);
uint8_t readSGP40Data(SensirionI2CSgp40& sgp40, VOCGasIndexAlgorithm& vocAlgorithm, float temperature, float humidity, float& vocRaw, float& vocIndex);
uint8_t readSGP41Data(SensirionI2CSgp41& sgp41, VOCGasIndexAlgorithm& vocAlgorithm, NOxGasIndexAlgorithm& noxAlgorithm, float temperature, float humidity, float& vocRaw, float& noxRaw, float& vocIndex, float& noxIndex, uint16_t& conditioningTime);
uint8_t readSFA3xData(SensirionI2CSfa3x& sfa3x, float& temperature, float& humidity, float& ch2oConcentration);
uint8_t readWZData(WZ& wz, float& ch2oConcentration);
uint8_t readBMP3xxData(Adafruit_BMP3XX& bmp3xx, float& temperature, float& pressure);
uint8_t readTSL2561Data(Adafruit_TSL2561_Unified& tsl2561, float& illuminance);
uint8_t readTSL2591Data(Adafruit_TSL2591& tsl2591, float& illuminance);

#endif  // DEVICE_HANDLERS_H
