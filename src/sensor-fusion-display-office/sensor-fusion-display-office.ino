#include "arduino_secrets.h"
#include "thingProperties.h"

#include <Arduino.h>
#include <Wire.h>
#include <SensirionI2cSht4x.h>
#include <SensirionI2CScd4x.h>
#include <SensirionI2CSgp40.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_TSL2561_U.h>
#include <LiquidCrystal_PCF8574.h>
#include <Arduino_LED_Matrix.h>
#include "animation.h"
#include <VOCGasIndexAlgorithm.h>

SensirionI2cSht4x sht40;
SensirionI2CScd4x scd40;
SensirionI2CSgp40 sgp40;
Adafruit_BMP3XX bmp390;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
LiquidCrystal_PCF8574 lcd(0x27);
ArduinoLEDMatrix matrix;
VOCGasIndexAlgorithm vocAlgorithm;

float sht40Temperature = NAN;
float sht40Humidity = NAN;
float scd40Temperature = NAN;
float scd40Humidity = NAN;
uint16_t scd40CO2Concentration = 0;
uint16_t sgp40VOCRaw = 0;
int32_t sgp40VOCIndex = 0;
float bmp390Temperature = NAN;
float bmp390Pressure = NAN;
float tsl2561Illuminance = NAN;

unsigned long lastSHT40ReadTime = 0;
unsigned long lastSCD40ReadTime = 0;
unsigned long lastSGP40ReadTime = 0;
unsigned long lastBMP390ReadTime = 0;
unsigned long lastTSL2561ReadTime = 0;

const unsigned long readInterval = 1000;

void initializeSHT40() {
  sht40.begin(Wire, SHT40_I2C_ADDR_44);
  int16_t error = sht40.softReset();
  if (error) {
    Serial.println(F("SHT40 initialization failed."));
  }
}

void initializeSCD40() {
  scd40.begin(Wire);
  uint16_t stopError = scd40.stopPeriodicMeasurement();
  uint16_t startError = scd40.startPeriodicMeasurement();
  if (stopError || startError) {
    Serial.println(F("SCD40 initialization failed."));
  }
}

void initializeSGP40() {
  sgp40.begin(Wire);
  uint16_t testResult;
  uint16_t error = sgp40.executeSelfTest(testResult);
  if (error || testResult != 0xD400) {
    Serial.println(F("SGP40 initialization failed."));
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

void initializeLCD() {
  lcd.begin(16, 2);
  lcd.setBacklight(255);
  lcd.home();
  lcd.clear();
}

void initializeLEDMatrix() {
  if (matrix.begin()) {
    matrix.loadSequence(frames);
    matrix.play(true);
  } else {
    Serial.println(F("LED Matrix initialization failed."));
  }
}

void readSHT40Data() {
  float tempTemperature = 0.0;
  float tempHumidity = 0.0;
  int16_t sht40Error = sht40.measureHighPrecision(tempTemperature, tempHumidity);
  if (sht40Error == 0) {
    sht40Temperature = tempTemperature;
    sht40Humidity = tempHumidity;
  } else {
    Serial.println(F("SHT40 measurement error."));
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
      Serial.println(F("SCD40 measurement error."));
    }
  }
}

void readSGP40Data() {
  uint16_t sgp40Error;
  uint16_t defaultT = 0x6666;
  uint16_t defaultRh = 0x8000;
  uint16_t compensationT = 0;
  uint16_t compensationRh = 0;

  if (!isnan(sht40Temperature) && !isnan(sht40Humidity)) {
    compensationT = static_cast<uint16_t>((sht40Temperature + 45) * 65535 / 175);
    compensationRh = static_cast<uint16_t>(sht40Humidity * 65535 / 100);
  } else {
    compensationT = defaultT;
    compensationRh = defaultRh;
  }

  sgp40Error = sgp40.measureRawSignal(compensationRh, compensationT, sgp40VOCRaw);

  if (sgp40Error == 0) {
    sgp40VOCIndex = vocAlgorithm.process(sgp40VOCRaw);
  } else {
    Serial.println(F("SGP40 measurement error."));
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
  lcd.setCursor(0, 0);
  lcd.print(isnan(sht40Temperature) ? "N/A " : String(sht40Temperature) + " ");
  lcd.print(isnan(sht40Humidity) ? "N/A " : String(sht40Humidity) + " ");
  lcd.print(scd40CO2Concentration == 0 ? "N/A" : String(scd40CO2Concentration));
  lcd.print("                ");

  lcd.setCursor(0, 1);
  lcd.print(sgp40VOCIndex == 0 ? "N/A " : String(sgp40VOCIndex) + " ");
  lcd.print(isnan(bmp390Pressure) ? "N/A " : String(bmp390Pressure, 3) + " ");
  lcd.print(isnan(tsl2561Illuminance) ? "N/A" : String(tsl2561Illuminance, 0));
  lcd.print("                ");
}

void displayDataOnSerial() {
  Serial.print("SHT40 Temperature: ");
  Serial.print(isnan(sht40Temperature) ? "N/A " : String(sht40Temperature) + "C ");
  Serial.print("Humidity: ");
  Serial.print(isnan(sht40Humidity) ? "N/A | " : String(sht40Humidity) + "% | ");

  Serial.print("SCD40 Temperature: ");
  Serial.print(isnan(scd40Temperature) ? "N/A " : String(scd40Temperature) + "C ");
  Serial.print("Humidity: ");
  Serial.print(isnan(scd40Humidity) ? "N/A " : String(scd40Humidity) + "% ");
  Serial.print("CO2 Concentration: ");
  Serial.print(scd40CO2Concentration == 0 ? "N/A | " : String(scd40CO2Concentration) + "ppm | ");

  Serial.print("SGP40 VOC Index: ");
  Serial.print(sgp40VOCIndex == 0 ? "N/A | " : String(sgp40VOCIndex) + " | ");

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
  cloud_sht40Temperature = 0.0;
  cloud_sht40Humidity = 0.0;
  cloud_scd40Temperature = 0.0;
  cloud_scd40Humidity = 0.0;
  cloud_scd40CO2Concentration = 0;
  cloud_sgp40VOCRaw = 0;
  cloud_sgp40VOCIndex = 0;
  cloud_bmp390Temperature = 0.0;
  cloud_bmp390Pressure = 0.0;
  cloud_tsl2561Illuminance = 0.0;
}

void updateCloudVariables() {
  cloud_sht40Temperature = sht40Temperature;
  cloud_sht40Humidity = sht40Humidity;
  cloud_scd40Temperature = scd40Temperature;
  cloud_scd40Humidity = scd40Humidity;
  cloud_scd40CO2Concentration = scd40CO2Concentration;
  cloud_sgp40VOCRaw = sgp40VOCRaw;
  cloud_sgp40VOCIndex = sgp40VOCIndex;
  cloud_bmp390Temperature = bmp390Temperature;
  cloud_bmp390Pressure = bmp390Pressure;
  cloud_tsl2561Illuminance = tsl2561Illuminance;
}

void onCloudDisplayControlChange() {
  if (cloud_displayControl) {
    lcd.setBacklight(255);
    lcd.display();
    matrix.loadSequence(frames);
    matrix.play(true);
  } else {
    lcd.setBacklight(0);
    lcd.noDisplay();
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

  initializeSHT40();
  initializeSCD40();
  initializeSGP40();
  initializeBMP390();
  initializeTSL2561();
  initializeLCD();
  initializeLEDMatrix();

  initProperties();
  initializeCloudVariables();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.printDebugInfo();

  Serial.println(F("All devices have been initialized."));
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastSHT40ReadTime >= readInterval) {
    lastSHT40ReadTime = currentMillis;
    readSHT40Data();
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
