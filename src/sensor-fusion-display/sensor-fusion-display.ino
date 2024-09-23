#include "arduino_secrets.h"
#include "thingProperties.h"
#include "deviceHandlers.h"

#include <U8g2lib.h>
#include <Arduino_LED_Matrix.h>
#include "animation.h"

SensirionI2cSht4x sht45;
SensirionI2CScd4x scd41;
SensirionI2CSgp41 sgp41;
WZ wz(Serial1);
Adafruit_BMP3XX bmp390;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 2561);

VOCGasIndexAlgorithm vocAlgorithm;
NOxGasIndexAlgorithm noxAlgorithm;

U8G2_SH1106_128X64_NONAME_2_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
ArduinoLEDMatrix matrix;

const uint8_t RED_LED_PIN = 2;
const uint8_t YELLOW_LED_PIN = 3;
const uint8_t GREEN_LED_PIN = 4;
const uint8_t BLUE_LED_PIN = 5;

bool i2cInitialized = false;

float sht45Temperature = NAN;
float sht45Humidity = NAN;
float scd41Temperature = NAN;
float scd41Humidity = NAN;
float scd41CO2Concentration = NAN;
float sgp41VOCRaw = NAN;
float sgp41NOXRaw = NAN;
float sgp41VOCIndex = NAN;
float sgp41NOXIndex = NAN;
float wzCH2OConcentration = NAN;
float bmp390Temperature = NAN;
float bmp390Pressure = NAN;
float tsl2561Illuminance = NAN;

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
  cloud_wzCH2OConcentration = wzCH2OConcentration;
  cloud_bmp390Temperature = bmp390Temperature;
  cloud_bmp390Pressure = bmp390Pressure;
  cloud_tsl2561Illuminance = tsl2561Illuminance;
}

void initializeLED() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(YELLOW_LED_PIN, LOW);
  digitalWrite(GREEN_LED_PIN, LOW);
  digitalWrite(BLUE_LED_PIN, LOW);
}

void checkCloudConnection() {
  static uint32_t lastCloudConnectedTime = 0;

  if (ArduinoCloud.connected()) {
    lastCloudConnectedTime = millis();
    digitalWrite(YELLOW_LED_PIN, LOW);
    digitalWrite(GREEN_LED_PIN, HIGH);
  } else {
    digitalWrite(YELLOW_LED_PIN, HIGH);
    digitalWrite(GREEN_LED_PIN, LOW);
    if (millis() - lastCloudConnectedTime >= 120 * 1000) {
      Serial.println(F("Cloud connection lost for 120 seconds. System will reset."));
      digitalWrite(RED_LED_PIN, HIGH);
      delay(10 * 1000);
      digitalWrite(RED_LED_PIN, LOW);
      NVIC_SystemReset();
    }
  }
}

void onCloudSystemResetChange() {
  if (cloud_systemReset) {
    digitalWrite(RED_LED_PIN, HIGH);
    delay(10 * 1000);
    digitalWrite(RED_LED_PIN, LOW);
    NVIC_SystemReset();
  }
}

void initializeLEDMatrix() {
  if (matrix.begin()) {
    matrix.loadSequence(frames);
    matrix.play(true);
  } else {
    Serial.println(F("LED Matrix initialization failed."));
  }
}

void initializeOLED(U8G2_SH1106_128X64_NONAME_2_HW_I2C& oled) {
  oled.setI2CAddress(0x3C * 2);
  if (oled.begin()) {
    oled.enableUTF8Print();
  } else {
    Serial.println(F("OLED initialization failed."));
  }
}

void displayDataOnScreen() {
  oled.firstPage();
  do {
    oled.setFont(u8g2_font_profont11_mf);
    oled.setFontPosTop();

    oled.setCursor(0, 0);
    oled.print("SHT45 ");
    oled.print(String(sht45Temperature, 2) + "C ");
    oled.print(String(sht45Humidity, 2) + "%");

    oled.setCursor(0, 9);
    oled.print("SCD41 ");
    oled.print(String(scd41Temperature, 2) + "C ");
    oled.print(String(scd41Humidity, 2) + "%");

    oled.setCursor(0, 18);
    oled.print("SCD41 CO2 ");
    oled.print(String(scd41CO2Concentration, 0) + "PPM");

    oled.setCursor(0, 27);
    oled.print("SGP41 VOC ");
    oled.print(String(sgp41VOCIndex, 0) + " ");
    oled.print("NOX ");
    oled.print(String(sgp41NOXIndex, 0));

    oled.setCursor(0, 36);
    oled.print("WZ CH2O ");
    oled.print(String(wzCH2OConcentration, 0) + "PPB");

    oled.setCursor(0, 45);
    oled.print("BMP390 ");
    oled.print(String(bmp390Temperature, 2) + "C ");
    oled.print(String(bmp390Pressure, 0) + "PA");

    oled.setCursor(0, 54);
    oled.print("TSL2561 ");
    oled.print(String(tsl2561Illuminance, 0) + "LX");

  } while (oled.nextPage());
}

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);

  initializeLED();
  initializeLEDMatrix();

  if (clearI2C() == 0) {
    Wire.begin();
    initializeSHT4x(sht45);
    initializeSCD4x(scd41);
    initializeSGP41(sgp41);
    initializeWZ(wz);
    initializeBMP3xx(bmp390);
    initializeTSL2561(tsl2561);
    initializeOLED(oled);
    i2cInitialized = true;
  }

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection, false);
  ArduinoCloud.printDebugInfo();

  Serial.println(F("All devices have been initialized."));
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delayMicroseconds(10000);
  digitalWrite(LED_BUILTIN, LOW);
  delayMicroseconds(10000);

  static uint16_t sgp41ConditioningTime = 5;
  static uint32_t lastReadTime = 0;

  if (i2cInitialized && (millis() - lastReadTime >= 1000)) {
    lastReadTime = millis();
    digitalWrite(BLUE_LED_PIN, HIGH);
    readSHT4xData(sht45, sht45Temperature, sht45Humidity);
    readSCD4xData(scd41, scd41Temperature, scd41Humidity, scd41CO2Concentration);
    readSGP41Data(sgp41, vocAlgorithm, noxAlgorithm, sht45Temperature, sht45Humidity, sgp41VOCRaw, sgp41NOXRaw, sgp41VOCIndex, sgp41NOXIndex, sgp41ConditioningTime);
    readWZData(wz, wzCH2OConcentration);
    readBMP3xxData(bmp390, bmp390Temperature, bmp390Pressure);
    readTSL2561Data(tsl2561, tsl2561Illuminance);
    updateCloudVariables();
    displayDataOnScreen();
    digitalWrite(BLUE_LED_PIN, LOW);
  }

  checkCloudConnection();
  ArduinoCloud.update();
}
