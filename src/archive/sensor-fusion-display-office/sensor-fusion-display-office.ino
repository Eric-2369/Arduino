#include "arduino_secrets.h"
#include "thingProperties.h"
#include "deviceHandlers.h"
#include <Arduino_LED_Matrix.h>
#include "animation.h"

SensirionI2cSht4x sht40;
SensirionI2CScd4x scd40;
SensirionI2CSgp40 sgp40;
VOCGasIndexAlgorithm vocAlgorithm;
Adafruit_BMP3XX bmp390;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
LiquidCrystal_PCF8574 lcd(0x27);
ArduinoLEDMatrix matrix;

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

void initializeLEDMatrix() {
  if (matrix.begin()) {
    matrix.loadSequence(frames);
    matrix.play(true);
  } else {
    Serial.println(F("LED Matrix initialization failed."));
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
  lcd.print(isnan(bmp390Pressure) ? "N/A " : String(bmp390Pressure / 1000.0, 3) + " ");
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
  Serial.print(isnan(bmp390Pressure) ? "N/A | " : String(bmp390Pressure) + "Pa | ");

  Serial.print("TSL2561 Illuminance: ");
  Serial.print(isnan(tsl2561Illuminance) ? "N/A" : String(tsl2561Illuminance) + "lx");

  Serial.println();
}

void initializeCloudVariables() {
  cloud_systemReset = false;
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

void onCloudSystemResetChange() {
  if (cloud_systemReset) {
    NVIC_SystemReset();
  }
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

void checkCloudConnection() {
  static unsigned long lastCloudConnectedTime = 0;

  if (ArduinoCloud.connected()) {
    lastCloudConnectedTime = millis();
  } else {
    if (millis() - lastCloudConnectedTime >= 120 * 1000) {
      Serial.println(F("Cloud connection lost for 120 seconds. System will reset."));
      NVIC_SystemReset();
    }
  }
}

void setup() {
  Serial.begin(9600);
  delay(1000);
  if (clearI2C() == 0) {
    Wire.begin();
  }

  initializeSHT4x(sht40);
  initializeSCD4x(scd40);
  initializeSGP40(sgp40);
  initializeBMP3xx(bmp390);
  initializeTSL2561(tsl2561);
  initializeLCD(lcd);
  initializeLEDMatrix();

  initProperties();
  initializeCloudVariables();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection, false);
  ArduinoCloud.printDebugInfo();

  Serial.println(F("All devices have been initialized."));
}

void loop() {
  static unsigned long lastSHT40ReadTime = 0;
  static unsigned long lastSCD40ReadTime = 0;
  static unsigned long lastSGP40ReadTime = 0;
  static unsigned long lastBMP390ReadTime = 0;
  static unsigned long lastTSL2561ReadTime = 0;
  const unsigned long readInterval = 1000;
  unsigned long currentMillis = millis();

  if (currentMillis - lastSHT40ReadTime >= readInterval) {
    lastSHT40ReadTime = currentMillis;
    readSHT4xData(sht40, sht40Temperature, sht40Humidity);
  }

  if (currentMillis - lastSCD40ReadTime >= readInterval) {
    lastSCD40ReadTime = currentMillis;
    readSCD4xData(scd40, scd40Temperature, scd40Humidity, scd40CO2Concentration);
  }

  if (currentMillis - lastSGP40ReadTime >= readInterval) {
    lastSGP40ReadTime = currentMillis;
    readSGP40Data(sgp40, vocAlgorithm, sht40Temperature, sht40Humidity, sgp40VOCRaw, sgp40VOCIndex);
  }

  if (currentMillis - lastBMP390ReadTime >= readInterval) {
    lastBMP390ReadTime = currentMillis;
    readBMP3xxData(bmp390, bmp390Temperature, bmp390Pressure);
  }

  if (currentMillis - lastTSL2561ReadTime >= readInterval) {
    lastTSL2561ReadTime = currentMillis;
    readTSL2561Data(tsl2561, tsl2561Illuminance);
  }

  if (cloud_displayControl) {
    displayDataOnScreen();
    displayDataOnSerial();
  }

  checkCloudConnection();
  updateCloudVariables();
  ArduinoCloud.update();
}
