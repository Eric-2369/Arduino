#include "deviceHandlers.h"

uint8_t clearI2C(uint8_t sdaPin, uint8_t sclPin) {
  pinMode(sdaPin, INPUT_PULLUP);
  pinMode(sclPin, INPUT_PULLUP);

  delay(2000);

  boolean SCL_LOW = (digitalRead(sclPin) == LOW);
  boolean SDA_LOW = (digitalRead(sdaPin) == LOW);

  if (SCL_LOW) {
    Serial.println(F("I2C bus error. Could not clear"));
    Serial.println(F("SCL clock line held low"));
    return 1;
  } else if (SDA_LOW) {
    int clockCount = 20;

    while (SDA_LOW && (clockCount > 0)) {
      clockCount--;
      pinMode(sclPin, INPUT);
      pinMode(sclPin, OUTPUT);
      delayMicroseconds(10);
      pinMode(sclPin, INPUT);
      pinMode(sclPin, INPUT_PULLUP);
      delayMicroseconds(10);
      SCL_LOW = (digitalRead(sclPin) == LOW);
      int counter = 20;
      while (SCL_LOW && (counter > 0)) {
        counter--;
        delay(100);
        SCL_LOW = (digitalRead(sclPin) == LOW);
      }
      if (SCL_LOW) {
        Serial.println(F("I2C bus error. Could not clear"));
        Serial.println(F("SCL clock line held low by slave clock stretch"));
        return 2;
      }
      SDA_LOW = (digitalRead(sdaPin) == LOW);
    }
    if (SDA_LOW) {
      Serial.println(F("I2C bus error. Could not clear"));
      Serial.println(F("SDA data line held low"));
      return 3;
    } else {
      pinMode(sdaPin, INPUT);
      pinMode(sdaPin, OUTPUT);
      delayMicroseconds(10);
      pinMode(sdaPin, INPUT);
      pinMode(sdaPin, INPUT_PULLUP);
      delayMicroseconds(10);
      pinMode(sdaPin, INPUT);
      pinMode(sclPin, INPUT);

      Serial.println(F("I2C bus cleared successfully"));
      return 0;
    }
  } else {
    Serial.println(F("I2C bus was already free"));
    return 0;
  }
}

uint8_t initializeSHT4x(SensirionI2cSht4x& sht4x, TwoWire& wire) {
  sht4x.begin(wire, SHT40_I2C_ADDR_44);
  int16_t error = sht4x.softReset();
  if (error) {
    Serial.println(F("SHT4x initialization failed."));
    return 1;
  } else {
    return 0;
  }
}

uint8_t initializeSCD4x(SensirionI2cScd4x& scd4x, TwoWire& wire) {
  scd4x.begin(wire, SCD40_I2C_ADDR_62);
  uint16_t stopError = scd4x.stopPeriodicMeasurement();
  uint16_t startError = scd4x.startPeriodicMeasurement();
  if (stopError || startError) {
    Serial.println(F("SCD4x initialization failed."));
    return 1;
  } else {
    return 0;
  }
}

uint8_t initializeSGP40(SensirionI2CSgp40& sgp40, TwoWire& wire) {
  sgp40.begin(wire);
  uint16_t testResult;
  uint16_t error = sgp40.executeSelfTest(testResult);
  if (error || testResult != 0xD400) {
    Serial.println(F("SGP40 initialization failed."));
    return 1;
  } else {
    return 0;
  }
}

uint8_t initializeSGP41(SensirionI2CSgp41& sgp41, TwoWire& wire) {
  sgp41.begin(wire);
  uint16_t testResult;
  uint16_t error = sgp41.executeSelfTest(testResult);
  if (error || testResult != 0xD400) {
    Serial.println(F("SGP41 initialization failed."));
    return 1;
  } else {
    return 0;
  }
}

uint8_t initializeSFA3x(SensirionI2cSfa3x& sfa3x, TwoWire& wire) {
  sfa3x.begin(wire, SFA3X_I2C_ADDR_5D);
  uint16_t resetError = sfa3x.deviceReset();
  uint16_t startError = sfa3x.startContinuousMeasurement();
  if (resetError || startError) {
    Serial.println(F("SFA3x initialization failed."));
    return 1;
  } else {
    return 0;
  }
}

uint8_t initializeWZ(WZ& wz) {
  wz.passiveMode();
  return 0;
}

uint8_t initializeBMP3xx(Adafruit_BMP3XX& bmp3xx, TwoWire& wire) {
  if (bmp3xx.begin_I2C(BMP3XX_DEFAULT_ADDRESS, &wire)) {
    bmp3xx.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp3xx.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp3xx.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp3xx.setOutputDataRate(BMP3_ODR_25_HZ);
    return 0;
  } else {
    Serial.println(F("BMP3xx initialization failed."));
    return 1;
  }
}

uint8_t initializeTSL2561(Adafruit_TSL2561_Unified& tsl2561, TwoWire& wire) {
  if (tsl2561.begin(&wire)) {
    tsl2561.enableAutoRange(false);
    tsl2561.setGain(TSL2561_GAIN_1X);
    tsl2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
    return 0;
  } else {
    Serial.println(F("TSL2561 initialization failed."));
    return 1;
  }
}

uint8_t initializeTSL2591(Adafruit_TSL2591& tsl2591, TwoWire& wire) {
  if (tsl2591.begin(&wire, TSL2591_ADDR)) {
    tsl2591.setGain(TSL2591_GAIN_MED);
    tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS);
    return 0;
  } else {
    Serial.println(F("TSL2591 initialization failed."));
    return 1;
  }
}

uint8_t initializeAS7265X(AS7265X& as7265x, TwoWire& wire) {
  if (as7265x.begin(wire)) {
    as7265x.setGain(AS7265X_GAIN_1X);
    as7265x.setIntegrationCycles(255);
    as7265x.setMeasurementMode(AS7265X_MEASUREMENT_MODE_6CHAN_CONTINUOUS);
    as7265x.setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS7265x_LED_WHITE);
    as7265x.setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS7265x_LED_IR);
    as7265x.setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS7265x_LED_UV);
    as7265x.setIndicatorCurrent(AS7265X_INDICATOR_CURRENT_LIMIT_1MA);
    as7265x.disableBulb(AS7265x_LED_WHITE);
    as7265x.disableBulb(AS7265x_LED_IR);
    as7265x.disableBulb(AS7265x_LED_UV);
    as7265x.disableIndicator();
    as7265x.disableInterrupt();
    return 0;
  } else {
    Serial.println(F("AS7265X initialization failed."));
    return 1;
  }
}

uint8_t readSHT4xData(SensirionI2cSht4x& sht4x, float& temperature, float& humidity) {
  float tempTemperature = 0.0;
  float tempHumidity = 0.0;
  int16_t error = sht4x.measureHighPrecision(tempTemperature, tempHumidity);
  if (error == 0) {
    temperature = tempTemperature;
    humidity = tempHumidity;
    return 0;
  } else {
    Serial.println(F("SHT4x measurement error."));
    return 1;
  }
}

uint8_t readSCD4xData(SensirionI2cScd4x& scd4x, float& temperature, float& humidity, float& co2Concentration) {
  float tempTemperature = 0.0f;
  float tempHumidity = 0.0f;
  uint16_t tempCO2Concentration = 0;
  bool isDataReady = false;
  scd4x.getDataReadyStatus(isDataReady);
  if (isDataReady) {
    uint16_t error = scd4x.readMeasurement(tempCO2Concentration, tempTemperature, tempHumidity);
    if (error == 0) {
      temperature = tempTemperature;
      humidity = tempHumidity;
      co2Concentration = tempCO2Concentration;
      return 0;
    } else {
      Serial.println(F("SCD4x measurement error."));
      return 1;
    }
  } else {
    return 0;
  }
}

uint8_t readSGP40Data(SensirionI2CSgp40& sgp40, VOCGasIndexAlgorithm& vocAlgorithm, float temperature, float humidity, float& vocRaw, float& vocIndex) {
  uint16_t defaultTemperature = 0x6666;
  uint16_t defaultHumidity = 0x8000;
  uint16_t compensationTemperature = 0;
  uint16_t compensationHumidity = 0;
  uint16_t tempVOCRaw = 0;
  int32_t tempVOCIndex = 0;

  if (!isnan(temperature) && !isnan(humidity)) {
    compensationTemperature = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
    compensationHumidity = static_cast<uint16_t>(humidity * 65535 / 100);
  } else {
    compensationTemperature = defaultTemperature;
    compensationHumidity = defaultHumidity;
  }

  uint16_t error = sgp40.measureRawSignal(compensationHumidity, compensationTemperature, tempVOCRaw);
  if (error == 0) {
    tempVOCIndex = vocAlgorithm.process(tempVOCRaw);

    vocRaw = tempVOCRaw;
    vocIndex = tempVOCIndex;
    return 0;
  } else {
    Serial.println(F("SGP40 measurement error."));
    return 1;
  }
}

uint8_t readSGP41Data(SensirionI2CSgp41& sgp41, VOCGasIndexAlgorithm& vocAlgorithm, NOxGasIndexAlgorithm& noxAlgorithm, float temperature, float humidity, float& vocRaw, float& noxRaw, float& vocIndex, float& noxIndex, uint16_t& conditioningTime) {
  uint16_t error;
  uint16_t defaultTemperature = 0x6666;
  uint16_t defaultHumidity = 0x8000;
  uint16_t compensationTemperature = 0;
  uint16_t compensationHumidity = 0;
  uint16_t tempVOCRaw = 0;
  uint16_t tempNOXRaw = 0;
  int32_t tempVOCIndex = 0;
  int32_t tempNOXIndex = 0;

  if (!isnan(temperature) && !isnan(humidity)) {
    compensationTemperature = static_cast<uint16_t>((temperature + 45) * 65535 / 175);
    compensationHumidity = static_cast<uint16_t>(humidity * 65535 / 100);
  } else {
    compensationTemperature = defaultTemperature;
    compensationHumidity = defaultHumidity;
  }

  if (conditioningTime > 0) {
    error = sgp41.executeConditioning(compensationHumidity, compensationTemperature, tempVOCRaw);
    conditioningTime--;
  } else {
    error = sgp41.measureRawSignals(compensationHumidity, compensationTemperature, tempVOCRaw, tempNOXRaw);
  }

  if (error == 0) {
    tempVOCIndex = vocAlgorithm.process(tempVOCRaw);
    tempNOXIndex = noxAlgorithm.process(tempNOXRaw);

    vocRaw = tempVOCRaw;
    noxRaw = tempNOXRaw;
    vocIndex = tempVOCIndex;
    noxIndex = tempNOXIndex;
    return 0;
  } else {
    Serial.println(F("SGP41 measurement error."));
    return 1;
  }
}

uint8_t readSFA3xData(SensirionI2cSfa3x& sfa3x, float& temperature, float& humidity, float& ch2oConcentration) {
  float tempTemperature = 0.0f;
  float tempHumidity = 0.0f;
  float tempCH2OConcentration = 0.0f;
  int16_t error = sfa3x.readMeasuredValues(tempCH2OConcentration, tempHumidity, tempTemperature);
  if (error == 0) {
    temperature = tempTemperature;
    humidity = tempHumidity;
    ch2oConcentration = tempCH2OConcentration;
    return 0;
  } else {
    Serial.println(F("SFA3x measurement error."));
    return 1;
  }
}

uint8_t readWZData(WZ& wz, float& ch2oConcentration) {
  WZ::DATA ch2oData;
  wz.requestRead();
  if (wz.readUntil(ch2oData)) {
    ch2oConcentration = ch2oData.HCHO_PPB;
    return 0;
  } else {
    Serial.println(F("WZ measurement error."));
    return 1;
  }
}

uint8_t readBMP3xxData(Adafruit_BMP3XX& bmp3xx, float& temperature, float& pressure) {
  if (bmp3xx.performReading()) {
    temperature = bmp3xx.temperature;
    pressure = bmp3xx.pressure;
    return 0;
  } else {
    Serial.println(F("BMP3xx measurement error."));
    return 1;
  }
}

uint8_t readTSL2561Data(Adafruit_TSL2561_Unified& tsl2561, float& illuminance) {
  sensors_event_t event;
  if (tsl2561.getEvent(&event)) {
    illuminance = event.light;
    return 0;
  } else {
    Serial.println(F("TSL2561 measurement error."));
    return 1;
  }
}

uint8_t readTSL2591Data(Adafruit_TSL2591& tsl2591, float& illuminance) {
  sensors_event_t event;
  if (tsl2591.getEvent(&event)) {
    illuminance = event.light;
    return 0;
  } else {
    Serial.println(F("TSL2591 measurement error."));
    return 1;
  }
}

uint8_t readAS7265XData(AS7265X& as7265x, float& as72651Temperature, float& as72652Temperature, float& as72653Temperature, float& as72653Spectrum410Irradiance, float& as72653Spectrum435Irradiance, float& as72653Spectrum460Irradiance, float& as72653Spectrum485Irradiance, float& as72653Spectrum510Irradiance, float& as72653Spectrum535Irradiance, float& as72652Spectrum560Irradiance, float& as72652Spectrum585Irradiance, float& as72651Spectrum610Irradiance, float& as72652Spectrum645Irradiance, float& as72651Spectrum680Irradiance, float& as72652Spectrum705Irradiance, float& as72651Spectrum730Irradiance, float& as72651Spectrum760Irradiance, float& as72651Spectrum810Irradiance, float& as72651Spectrum860Irradiance, float& as72652Spectrum900Irradiance, float& as72652Spectrum940Irradiance) {
  as72651Temperature = as7265x.getTemperature(AS72651_NIR);
  as72652Temperature = as7265x.getTemperature(AS72652_VISIBLE);
  as72653Temperature = as7265x.getTemperature(AS72653_UV);
  as72653Spectrum410Irradiance = as7265x.getCalibratedA();
  as72653Spectrum435Irradiance = as7265x.getCalibratedB();
  as72653Spectrum460Irradiance = as7265x.getCalibratedC();
  as72653Spectrum485Irradiance = as7265x.getCalibratedD();
  as72653Spectrum510Irradiance = as7265x.getCalibratedE();
  as72653Spectrum535Irradiance = as7265x.getCalibratedF();
  as72652Spectrum560Irradiance = as7265x.getCalibratedG();
  as72652Spectrum585Irradiance = as7265x.getCalibratedH();
  as72651Spectrum610Irradiance = as7265x.getCalibratedR();
  as72652Spectrum645Irradiance = as7265x.getCalibratedI();
  as72651Spectrum680Irradiance = as7265x.getCalibratedS();
  as72652Spectrum705Irradiance = as7265x.getCalibratedJ();
  as72651Spectrum730Irradiance = as7265x.getCalibratedT();
  as72651Spectrum760Irradiance = as7265x.getCalibratedU();
  as72651Spectrum810Irradiance = as7265x.getCalibratedV();
  as72651Spectrum860Irradiance = as7265x.getCalibratedW();
  as72652Spectrum900Irradiance = as7265x.getCalibratedK();
  as72652Spectrum940Irradiance = as7265x.getCalibratedL();
  return 0;
}
