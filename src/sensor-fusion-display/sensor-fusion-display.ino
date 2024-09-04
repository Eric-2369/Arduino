#include "arduino_secrets.h"
#include "thingProperties.h"
#include "deviceHandlers.h"
#include <Arduino_LED_Matrix.h>
#include "animation.h"

SensirionI2cSht4x sht45;
SensirionI2CScd4x scd41;
SensirionI2CSgp41 sgp41;
VOCGasIndexAlgorithm vocAlgorithm;
NOxGasIndexAlgorithm noxAlgorithm;
SensirionI2CSfa3x sfa30;
Adafruit_BMP3XX bmp390;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
ArduinoLEDMatrix matrix;

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

void initializeLEDMatrix() {
  if (matrix.begin()) {
    matrix.loadSequence(frames);
    matrix.play(true);
  } else {
    Serial.println(F("LED Matrix initialization failed."));
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
    u8g2.print(isnan(bmp390Pressure) ? "N/A" : String(bmp390Pressure / 1000.0, 2) + "KPA");

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
  Serial.print(isnan(bmp390Pressure) ? "N/A | " : String(bmp390Pressure) + "Pa | ");

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

  initializeSHT4x(sht45);
  initializeSCD4x(scd41);
  initializeSGP41(sgp41);
  initializeSFA3x(sfa30);
  initializeBMP3xx(bmp390);
  initializeTSL2561(tsl2561);
  initializeOLED(u8g2);
  initializeLEDMatrix();

  initProperties();
  initializeCloudVariables();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.printDebugInfo();

  Serial.println(F("All devices have been initialized."));
}

void loop() {
  static uint16_t sgp41ConditioningTime = 5;
  static unsigned long lastSHT45ReadTime = 0;
  static unsigned long lastSCD41ReadTime = 0;
  static unsigned long lastSGP41ReadTime = 0;
  static unsigned long lastSFA30ReadTime = 0;
  static unsigned long lastBMP390ReadTime = 0;
  static unsigned long lastTSL2561ReadTime = 0;
  const unsigned long readInterval = 1000;
  unsigned long currentMillis = millis();

  if (currentMillis - lastSHT45ReadTime >= readInterval) {
    lastSHT45ReadTime = currentMillis;
    readSHT4xData(sht45, sht45Temperature, sht45Humidity);
  }

  if (currentMillis - lastSCD41ReadTime >= readInterval) {
    lastSCD41ReadTime = currentMillis;
    readSCD4xData(scd41, scd41Temperature, scd41Humidity, scd41CO2Concentration);
  }

  if (currentMillis - lastSGP41ReadTime >= readInterval) {
    lastSGP41ReadTime = currentMillis;
    readSGP41Data(sgp41, vocAlgorithm, noxAlgorithm, sht45Temperature, sht45Humidity, sgp41VOCRaw, sgp41NOXRaw, sgp41VOCIndex, sgp41NOXIndex, sgp41ConditioningTime);
  }

  if (currentMillis - lastSFA30ReadTime >= readInterval) {
    lastSFA30ReadTime = currentMillis;
    readSFA3xData(sfa30, sfa30Temperature, sfa30Humidity, sfa30CH2OConcentration);
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

  updateCloudVariables();
  ArduinoCloud.update();
}
