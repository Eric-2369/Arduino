#include "arduino_secrets.h"
#include "thingProperties.h"

#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2cSht4x.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2CSgp41.h>
#include <SensirionI2CSfa3x.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_TSL2561_U.h>
#include <U8g2lib.h>
#include <Arduino_LED_Matrix.h>
#include "animation.h"
#include <VOCGasIndexAlgorithm.h>
#include <NOxGasIndexAlgorithm.h>

SensirionI2cSht4x sht45;
SensirionI2CScd4x scd41;
SensirionI2CSgp41 sgp41;
SensirionI2CSfa3x sfa30;
Adafruit_BMP3XX bmp390;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
ArduinoLEDMatrix matrix;
VOCGasIndexAlgorithm vocAlgorithm;
NOxGasIndexAlgorithm noxAlgorithm;

float sht45Temperature = NAN;
float sht45Humidity = NAN;
float scd41Temperature = NAN;
float scd41Humidity = NAN;
uint16_t scd41CO2Concentration = 0;
uint16_t sgp41VOCRaw = 0;
int32_t sgp41VOCIndex = 0;
uint16_t sgp41NOXRaw = 0;
int32_t sgp41NOXIndex = 0;
float sfa30Temperature = NAN;
float sfa30Humidity = NAN;
float sfa30CH2OConcentration = NAN;
float bmp390Temperature = NAN;
float bmp390Pressure = NAN;
float tsl2561Illuminance = NAN;

uint16_t sgp41ConditioningTime = 5;

unsigned long lastSHT45ReadTime = 0;
unsigned long lastSCD41ReadTime = 0;
unsigned long lastSGP41ReadTime = 0;
unsigned long lastSFA30ReadTime = 0;
unsigned long lastBMP390ReadTime = 0;
unsigned long lastTSL2561ReadTime = 0;

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

void initializeSGP41() {
  sgp41.begin(Wire);
  uint16_t testResult;
  uint16_t error = sgp41.executeSelfTest(testResult);
  if (error || testResult != 0xD400) {
    Serial.println(F("SGP41 initialization failed."));
  }
}

void initializeSFA30() {
  sfa30.begin(Wire);
  uint16_t error = sfa30.startContinuousMeasurement();
  if (error) {
    Serial.println(F("SFA30 initialization failed."));
  }
}

void initializeBMP390() {
  if (bmp390.begin_I2C()) {
    bmp390.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp390.setTemperatureOversampling(BMP3_OVERSAMPLING_2X);
    bmp390.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp390.setOutputDataRate(BMP3_ODR_25_HZ);
  } else {
    Serial.println(F("BMP390 initialization failed."));
  }
}

void initializeTSL2561() {
  if (tsl2561.begin()) {
    tsl2561.enableAutoRange(false);
    tsl2561.setGain(TSL2561_GAIN_1X);
    tsl2561.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);
  } else {
    Serial.println(F("TSL2561 initialization failed."));
  }
}

void initializeOLED() {
  u8g2.setI2CAddress(0x3C * 2);
  u8g2.begin();
  u8g2.enableUTF8Print();
}

void initializeLEDMatrix() {
  matrix.begin();
  matrix.loadSequence(frames);
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
    Serial.println(F("SHT45 measurement error."));
  }
}

void readSCD41Data() {
  float tempSCD41Temperature = 0.0f;
  float tempSCD41Humidity = 0.0f;
  uint16_t tempCO2Concentration = 0;
  bool isDataReady = false;
  scd41.getDataReadyFlag(isDataReady);
  if (isDataReady) {
    uint16_t scd41Error = scd41.readMeasurement(tempCO2Concentration, tempSCD41Temperature, tempSCD41Humidity);
    if (scd41Error == 0 && tempCO2Concentration != 0) {
      scd41Temperature = tempSCD41Temperature;
      scd41Humidity = tempSCD41Humidity;
      scd41CO2Concentration = tempCO2Concentration;
    } else {
      Serial.println(F("SCD41 measurement error."));
    }
  }
}

void readSGP41Data() {
  uint16_t sgp41Error;
  uint16_t defaultT = 0x6666;
  uint16_t defaultRh = 0x8000;
  uint16_t compensationT = 0;
  uint16_t compensationRh = 0;

  if (!isnan(sht45Temperature) && !isnan(sht45Humidity)) {
    compensationT = static_cast<uint16_t>((sht45Temperature + 45) * 65535 / 175);
    compensationRh = static_cast<uint16_t>(sht45Humidity * 65535 / 100);
  } else {
    compensationT = defaultT;
    compensationRh = defaultRh;
  }

  if (sgp41ConditioningTime > 0) {
    sgp41Error = sgp41.executeConditioning(compensationRh, compensationT, sgp41VOCRaw);
    sgp41ConditioningTime--;
  } else {
    sgp41Error = sgp41.measureRawSignals(compensationRh, compensationT, sgp41VOCRaw, sgp41NOXRaw);
  }

  if (sgp41Error == 0) {
    sgp41VOCIndex = vocAlgorithm.process(sgp41VOCRaw);
    sgp41NOXIndex = noxAlgorithm.process(sgp41NOXRaw);
  } else {
    Serial.println(F("SGP41 measurement error."));
  }
}

void readSFA30Data() {
  uint16_t error;
  int16_t temperature;
  int16_t humidity;
  int16_t concentration;
  error = sfa30.readMeasuredValues(concentration, humidity, temperature);
  if (error == 0) {
    sfa30Temperature = temperature / 200.0;
    sfa30Humidity = humidity / 100.0;
    sfa30CH2OConcentration = concentration / 5.0;
  } else {
    Serial.println(F("SFA30 measurement error."));
  }
}

void readBMP390Data() {
  if (bmp390.performReading()) {
    bmp390Temperature = bmp390.temperature;
    bmp390Pressure = bmp390.pressure / 1000.0;
  } else {
    Serial.println(F("BMP390 measurement error."));
  }
}

void readTSL2561Data() {
  sensors_event_t event;
  tsl2561.getEvent(&event);
  if (event.light) {
    tsl2561Illuminance = event.light;
  } else {
    Serial.println(F("TSL2561 measurement error."));
  }
}

void displayDataOnScreen() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_profont11_mf);
    u8g2.setFontPosTop();

    u8g2.setCursor(0, 0);
    u8g2.print("SHT45 ");
    u8g2.print(isnan(sht45Temperature) ? "N/A " : String(sht45Temperature) + "C ");
    u8g2.print(isnan(sht45Humidity) ? "N/A" : String(sht45Humidity) + "%");

    u8g2.setCursor(0, 9);
    u8g2.print("SCD41 ");
    u8g2.print(isnan(scd41Temperature) ? "N/A " : String(scd41Temperature) + "C ");
    u8g2.print(isnan(scd41Humidity) ? "N/A" : String(scd41Humidity) + "%");

    u8g2.setCursor(0, 18);
    u8g2.print("SCD41 CO2 ");
    u8g2.print(scd41CO2Concentration == 0 ? "N/A" : String(scd41CO2Concentration) + "PPM");

    u8g2.setCursor(0, 27);
    u8g2.print("SGP41 VOC ");
    u8g2.print(sgp41VOCIndex == 0 ? "N/A " : String(sgp41VOCIndex) + " ");
    u8g2.print("NOX ");
    u8g2.print(sgp41NOXIndex == 0 ? "N/A" : String(sgp41NOXIndex));

    u8g2.setCursor(0, 36);
    u8g2.print("SFA30 ");
    u8g2.print(isnan(sfa30Temperature) ? "N/A " : String(sfa30Temperature) + "C ");
    u8g2.print(isnan(sfa30Humidity) ? "N/A" : String(sfa30Humidity) + "%");

    u8g2.setCursor(0, 45);
    u8g2.print("SFA30 CH2O ");
    u8g2.print(isnan(sfa30CH2OConcentration) ? "N/A" : String(sfa30CH2OConcentration) + "PPB");

    u8g2.setCursor(0, 54);
    u8g2.print("BMP390 ");
    u8g2.print(isnan(bmp390Temperature) ? "N/A " : String(bmp390Temperature) + "C ");
    u8g2.print(isnan(bmp390Pressure) ? "N/A" : String(bmp390Pressure, 2) + "KPA");

    u8g2.setCursor(0, 63);
    u8g2.print("TSL2561 ");
    u8g2.print(isnan(tsl2561Illuminance) ? "N/A" : String(tsl2561Illuminance) + "LX");

  } while (u8g2.nextPage());
}

void displayDataOnSerial() {
  Serial.print("SHT45 Temperature: ");
  Serial.print(isnan(sht45Temperature) ? "N/A " : String(sht45Temperature) + "C ");
  Serial.print("Humidity: ");
  Serial.print(isnan(sht45Humidity) ? "N/A | " : String(sht45Humidity) + "% | ");

  Serial.print("SCD41 Temperature: ");
  Serial.print(isnan(scd41Temperature) ? "N/A " : String(scd41Temperature) + "C ");
  Serial.print("Humidity: ");
  Serial.print(isnan(scd41Humidity) ? "N/A " : String(scd41Humidity) + "% ");
  Serial.print("CO2 Concentration: ");
  Serial.print(scd41CO2Concentration == 0 ? "N/A | " : String(scd41CO2Concentration) + "ppm | ");

  Serial.print("SGP41 VOC Index: ");
  Serial.print(sgp41VOCIndex == 0 ? "N/A " : String(sgp41VOCIndex) + " ");
  Serial.print("SGP41 NOX Index: ");
  Serial.print(sgp41NOXIndex == 0 ? "N/A | " : String(sgp41NOXIndex) + " | ");

  Serial.print("SFA30 Temperature: ");
  Serial.print(isnan(sfa30Temperature) ? "N/A " : String(sfa30Temperature) + "C ");
  Serial.print("Humidity: ");
  Serial.print(isnan(sfa30Humidity) ? "N/A " : String(sfa30Humidity) + "% ");
  Serial.print("CH2O Concentration: ");
  Serial.println(isnan(sfa30CH2OConcentration) ? "N/A | " : String(sfa30CH2OConcentration) + "ppb | ");

  Serial.print("BMP390 Temperature: ");
  Serial.print(isnan(bmp390Temperature) ? "N/A " : String(bmp390Temperature) + "C ");
  Serial.print("Pressure: ");
  Serial.print(isnan(bmp390Pressure) ? "N/A | " : String(bmp390Pressure, 3) + "kPa | ");

  Serial.print("TSL2561 Illuminance: ");
  Serial.print(isnan(tsl2561Illuminance) ? "N/A" : String(tsl2561Illuminance) + "lx");

  Serial.println();
}

void initializeCloudVariables() {
  cloud_displayControl = true;
  cloud_sht45Temperature = 0.0;
  cloud_sht45Humidity = 0.0;
  cloud_scd41Temperature = 0.0;
  cloud_scd41Humidity = 0.0;
  cloud_scd41CO2Concentration = 0;
  cloud_sgp41VOCRaw = 0;
  cloud_sgp41VOCIndex = 0;
  cloud_sgp41NOXRaw = 0;
  cloud_sgp41NOXIndex = 0;
  cloud_sfa30Temperature = 0.0;
  cloud_sfa30Humidity = 0.0;
  cloud_sfa30CH2OConcentration = 0.0;
  cloud_bmp390Temperature = 0.0;
  cloud_bmp390Pressure = 0.0;
  cloud_tsl2561Illuminance = 0.0;
}

void updateCloudVariables() {
  cloud_sht45Temperature = sht45Temperature;
  cloud_sht45Humidity = sht45Humidity;
  cloud_scd41Temperature = scd41Temperature;
  cloud_scd41Humidity = scd41Humidity;
  cloud_scd41CO2Concentration = scd41CO2Concentration;
  cloud_sgp41VOCRaw = sgp41VOCRaw;
  cloud_sgp41VOCIndex = sgp41VOCIndex;
  cloud_sgp41NOXRaw = sgp41NOXRaw;
  cloud_sgp41NOXIndex = sgp41NOXIndex;
  cloud_sfa30Temperature = sfa30Temperature;
  cloud_sfa30Humidity = sfa30Humidity;
  cloud_sfa30CH2OConcentration = sfa30CH2OConcentration;
  cloud_bmp390Temperature = bmp390Temperature;
  cloud_bmp390Pressure = bmp390Pressure;
  cloud_tsl2561Illuminance = tsl2561Illuminance;
}

void onCloudDisplayControlChange() {
  if (cloud_displayControl) {
    u8g2.setPowerSave(0);
    matrix.loadSequence(frames);
    matrix.play(true);
  } else {
    u8g2.setPowerSave(1);
    matrix.clear();
  }
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
  initializeSGP41();
  initializeSFA30();
  initializeBMP390();
  initializeTSL2561();
  initializeOLED();
  initializeLEDMatrix();

  initProperties();
  initializeCloudVariables();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.printDebugInfo();

  Serial.println(F("All devices have been initialized."));
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

  if (currentMillis - lastSGP41ReadTime >= readInterval) {
    lastSGP41ReadTime = currentMillis;
    readSGP41Data();
  }

  if (currentMillis - lastSFA30ReadTime >= readInterval) {
    lastSFA30ReadTime = currentMillis;
    readSFA30Data();
  }

  if (currentMillis - lastBMP390ReadTime >= readInterval) {
    lastBMP390ReadTime = currentMillis;
    readBMP390Data();
  }

  if (currentMillis - lastTSL2561ReadTime >= readInterval) {
    lastTSL2561ReadTime = currentMillis;
    readTSL2561Data();
  }

  if (cloud_displayControl) {
    displayDataOnScreen();
    displayDataOnSerial();
  }

  updateCloudVariables();
  ArduinoCloud.update();
}
