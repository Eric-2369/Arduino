#include "deviceHandlers.h"

void initializeSHT4x(SensirionI2cSht4x& sht4x) {
  sht4x.begin(Wire, SHT40_I2C_ADDR_44);
  int16_t error = sht4x.softReset();
  if (error) {
    Serial.println(F("SHT4x initialization failed."));
  }
}

void initializeSCD4x(SensirionI2CScd4x& scd4x) {
  scd4x.begin(Wire);
  uint16_t stopError = scd4x.stopPeriodicMeasurement();
  uint16_t startError = scd4x.startPeriodicMeasurement();
  if (stopError || startError) {
    Serial.println(F("SCD4x initialization failed."));
  }
}

void initializeSGP40(SensirionI2CSgp40& sgp40) {
  sgp40.begin(Wire);
  uint16_t testResult;
  uint16_t error = sgp40.executeSelfTest(testResult);
  if (error || testResult != 0xD400) {
    Serial.println(F("SGP40 initialization failed."));
  }
}

void initializeBMP3xx(Adafruit_BMP3XX& bmp3xx) {
  if (bmp3xx.begin_I2C()) {
    bmp3xx.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp3xx.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp3xx.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp3xx.setOutputDataRate(BMP3_ODR_25_HZ);
  } else {
    Serial.println(F("BMP3xx initialization failed."));
  }
}

void initializeTSL2561(Adafruit_TSL2561_Unified& tsl2561) {
  if (tsl2561.begin()) {
    tsl2561.enableAutoRange(false);
    tsl2561.setGain(TSL2561_GAIN_1X);
    tsl2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
  } else {
    Serial.println(F("TSL2561 initialization failed."));
  }
}

void initializeLCD(LiquidCrystal_PCF8574& lcd) {
  lcd.begin(16, 2);
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
}

void readSHT4xData(SensirionI2cSht4x& sht4x, float& temperature, float& humidity) {
  float tempTemperature = 0.0;
  float tempHumidity = 0.0;
  int16_t error = sht4x.measureHighPrecision(tempTemperature, tempHumidity);
  if (error == 0) {
    temperature = tempTemperature;
    humidity = tempHumidity;
  } else {
    Serial.println(F("SHT4x measurement error."));
  }
}

void readSCD4xData(SensirionI2CScd4x& scd4x, float& temperature, float& humidity, uint16_t& co2Concentration) {
  float tempTemperature = 0.0f;
  float tempHumidity = 0.0f;
  uint16_t tempCO2Concentration = 0;
  bool isDataReady = false;
  scd4x.getDataReadyFlag(isDataReady);
  if (isDataReady) {
    uint16_t error = scd4x.readMeasurement(tempCO2Concentration, tempTemperature, tempHumidity);
    if (error == 0) {
      temperature = tempTemperature;
      humidity = tempHumidity;
      co2Concentration = tempCO2Concentration;
    } else {
      Serial.println(F("SCD4x measurement error."));
    }
  }
}

void readSGP40Data(SensirionI2CSgp40& sgp40, VOCGasIndexAlgorithm& vocAlgorithm, float temperature, float humidity, uint16_t& vocRaw, int32_t& vocIndex) {
  uint16_t error;
  uint16_t defaultTemperature = 0x6666;
  uint16_t defaultHumidity = 0x8000;
  uint16_t compensationTemperature = 0;
  uint16_t compensationHumidity = 0;

  if (!isnan(temperature) && !isnan(humidity)) {
    compensationTemperature = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
    compensationHumidity = static_cast<uint16_t>(humidity * 65535 / 100);
  } else {
    compensationTemperature = defaultTemperature;
    compensationHumidity = defaultHumidity;
  }

  uint16_t tempVOCRaw = 0;
  error = sgp40.measureRawSignal(compensationHumidity, compensationTemperature, tempVOCRaw);
  if (error == 0) {
    int32_t tempVOCIndex = vocAlgorithm.process(tempVOCRaw);
    vocRaw = tempVOCRaw;
    vocIndex = tempVOCIndex;
  } else {
    Serial.println(F("SGP40 measurement error."));
  }
}

void readBMP3xxData(Adafruit_BMP3XX& bmp3xx, float& temperature, float& pressure) {
  if (bmp3xx.performReading()) {
    temperature = bmp3xx.temperature;
    pressure = bmp3xx.pressure;
  } else {
    Serial.println(F("BMP3xx measurement error."));
  }
}

void readTSL2561Data(Adafruit_TSL2561_Unified& tsl2561, float& illuminance) {
  sensors_event_t event;
  tsl2561.getEvent(&event);
  if (event.light) {
    illuminance = event.light;
  } else {
    Serial.println(F("TSL2561 measurement error."));
  }
}
