#include "deviceHandlers.h"

int clearI2C() {
  pinMode(SDA, INPUT_PULLUP);
  pinMode(SCL, INPUT_PULLUP);

  delay(2000);

  boolean SCL_LOW = (digitalRead(SCL) == LOW);
  if (SCL_LOW) {
    Serial.println(F("I2C bus error. Could not clear"));
    Serial.println(F("SCL clock line held low"));
    return 1;
  }

  boolean SDA_LOW = (digitalRead(SDA) == LOW);
  int clockCount = 20;

  while (SDA_LOW && (clockCount > 0)) {
    clockCount--;
    pinMode(SCL, INPUT);
    pinMode(SCL, OUTPUT);
    delayMicroseconds(10);
    pinMode(SCL, INPUT);
    pinMode(SCL, INPUT_PULLUP);
    delayMicroseconds(10);
    SCL_LOW = (digitalRead(SCL) == LOW);
    int counter = 20;
    while (SCL_LOW && (counter > 0)) {
      counter--;
      delay(100);
      SCL_LOW = (digitalRead(SCL) == LOW);
    }
    if (SCL_LOW) {
      Serial.println(F("I2C bus error. Could not clear"));
      Serial.println(F("SCL clock line held low by slave clock stretch"));
      return 2;
    }
    SDA_LOW = (digitalRead(SDA) == LOW);
  }
  if (SDA_LOW) {
    Serial.println(F("I2C bus error. Could not clear"));
    Serial.println(F("SDA data line held low"));
    return 3;
  }

  pinMode(SDA, INPUT);
  pinMode(SDA, OUTPUT);
  delayMicroseconds(10);
  pinMode(SDA, INPUT);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(10);
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);

  Serial.println(F("I2C bus cleared successfully"));
  return 0;
}

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

void initializeSGP41(SensirionI2CSgp41& sgp41) {
  sgp41.begin(Wire);
  uint16_t testResult;
  uint16_t error = sgp41.executeSelfTest(testResult);
  if (error || testResult != 0xD400) {
    Serial.println(F("SGP41 initialization failed."));
  }
}

void initializeSFA3x(SensirionI2CSfa3x& sfa3x) {
  sfa3x.begin(Wire);
  uint16_t stopError = sfa3x.stopMeasurement();
  uint16_t startError = sfa3x.startContinuousMeasurement();
  if (stopError || startError) {
    Serial.println(F("SFA3x initialization failed."));
  }
}

void initializeWZ(WZ& wz) {
  wz.passiveMode();
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

void initializeTSL2591(Adafruit_TSL2591& tsl2591) {
  if (tsl2591.begin()) {
    tsl2591.setGain(TSL2591_GAIN_MED);
    tsl2591.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  } else {
    Serial.println(F("TSL2591 initialization failed."));
  }
}

void initializeLCD(LiquidCrystal_PCF8574& lcd) {
  lcd.begin(16, 2);
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
}

void initializeOLED(U8G2_SSD1306_128X64_NONAME_1_HW_I2C& oled) {
  oled.setI2CAddress(0x3C * 2);
  if (oled.begin()) {
    oled.enableUTF8Print();
  } else {
    Serial.println(F("OLED initialization failed."));
  }
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

void readSCD4xData(SensirionI2CScd4x& scd4x, float& temperature, float& humidity, float& co2Concentration) {
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

void readSGP40Data(SensirionI2CSgp40& sgp40, VOCGasIndexAlgorithm& vocAlgorithm, float temperature, float humidity, float& vocRaw, float& vocIndex) {
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
  } else {
    Serial.println(F("SGP40 measurement error."));
  }
}

void readSGP41Data(SensirionI2CSgp41& sgp41, VOCGasIndexAlgorithm& vocAlgorithm, NOxGasIndexAlgorithm& noxAlgorithm, float temperature, float humidity, float& vocRaw, float& noxRaw, float& vocIndex, float& noxIndex, uint16_t& conditioningTime) {
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
  } else {
    Serial.println(F("SGP41 measurement error."));
  }
}

void readSFA3xData(SensirionI2CSfa3x& sfa3x, float& temperature, float& humidity, float& ch2oConcentration) {
  int16_t tempTemperature = 0.0f;
  int16_t tempHumidity = 0.0f;
  int16_t tempCH2OConcentration = 0;
  uint16_t error = sfa3x.readMeasuredValues(tempCH2OConcentration, tempHumidity, tempTemperature);
  if (error == 0) {
    temperature = tempTemperature / 200.0;
    humidity = tempHumidity / 100.0;
    ch2oConcentration = tempCH2OConcentration / 5.0;
  } else {
    Serial.println(F("SFA3x measurement error."));
  }
}

void readWZData(WZ& wz, float& ch2oConcentration) {
  WZ::DATA ch2oData;
  wz.requestRead();
  if (wz.readUntil(ch2oData)) {
    ch2oConcentration = ch2oData.HCHO_PPB;
  } else {
    Serial.println(F("WZ measurement error."));
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
  if (tsl2561.getEvent(&event)) {
    illuminance = event.light;
  } else {
    Serial.println(F("TSL2561 measurement error."));
  }
}

void readTSL2591Data(Adafruit_TSL2591& tsl2591, float& illuminance) {
  sensors_event_t event;
  if (tsl2591.getEvent(&event)) {
    illuminance = event.light;
  } else {
    Serial.println(F("TSL2591 measurement error."));
  }
}
