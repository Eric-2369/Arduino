#include "arduino_secrets.h"
#include "thingProperties.h"
#include "deviceHandlers.h"

SensirionI2cSht4x sht45;
SensirionI2CScd4x scd41;
SensirionI2CSgp41 sgp41;
SensirionI2CSfa3x sfa30;
Adafruit_BMP3XX bmp390;
Adafruit_TSL2591 tsl2591 = Adafruit_TSL2591(2591);

VOCGasIndexAlgorithm vocAlgorithm;
NOxGasIndexAlgorithm noxAlgorithm;

const uint8_t RED_LED_PIN = 2;
const uint8_t YELLOW_LED_PIN = 3;
const uint8_t GREEN_LED_PIN = 4;
const uint8_t BLUE_LED_PIN = 5;

bool i2cInitialized = false;

float sht45Temperature = 0.0;
float sht45Humidity = 0.0;
float scd41Temperature = 0.0;
float scd41Humidity = 0.0;
float scd41CO2Concentration = 0.0;
float sgp41VOCRaw = 0.0;
float sgp41NOXRaw = 0.0;
float sgp41VOCIndex = 0.0;
float sgp41NOXIndex = 0.0;
float sfa30Temperature = 0.0;
float sfa30Humidity = 0.0;
float sfa30CH2OConcentration = 0.0;
float bmp390Temperature = 0.0;
float bmp390Pressure = 0.0;
float tsl2591Illuminance = 0.0;

void updateCloudVariables() {
  cloud_sht45Temperature = sht45Temperature;
  cloud_sht45Humidity = sht45Humidity;
  cloud_scd41Temperature = scd41Temperature;
  cloud_scd41Humidity = scd41Humidity;
  cloud_scd41CO2Concentration = scd41CO2Concentration;
  cloud_sgp41VOCRaw = sgp41VOCRaw;
  cloud_sgp41NOXRaw = sgp41NOXRaw;
  cloud_sgp41VOCIndex = sgp41VOCIndex;
  cloud_sgp41NOXIndex = sgp41NOXIndex;
  cloud_sfa30Temperature = sfa30Temperature;
  cloud_sfa30Humidity = sfa30Humidity;
  cloud_sfa30CH2OConcentration = sfa30CH2OConcentration;
  cloud_bmp390Temperature = bmp390Temperature;
  cloud_bmp390Pressure = bmp390Pressure;
  cloud_tsl2591Illuminance = tsl2591Illuminance;
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

void setup() {
  Serial.begin(9600);

  initializeLED();

  if (clearI2C() == 0) {
    Wire.begin();
    initializeSHT4x(sht45);
    initializeSCD4x(scd41);
    initializeSGP41(sgp41);
    initializeSFA3x(sfa30);
    initializeBMP3xx(bmp390);
    initializeTSL2591(tsl2591);
    i2cInitialized = true;
  }

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection, false);
  ArduinoCloud.printDebugInfo();

  Serial.println(F("All devices have been initialized."));
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(10);
  digitalWrite(LED_BUILTIN, LOW);

  static uint16_t sgp41ConditioningTime = 5;
  static uint32_t lastReadTime = 0;

  if (i2cInitialized && (millis() - lastReadTime >= 1000)) {
    lastReadTime = millis();
    digitalWrite(BLUE_LED_PIN, HIGH);
    readSHT4xData(sht45, sht45Temperature, sht45Humidity);
    readSCD4xData(scd41, scd41Temperature, scd41Humidity, scd41CO2Concentration);
    readSGP41Data(sgp41, vocAlgorithm, noxAlgorithm, sht45Temperature, sht45Humidity, sgp41VOCRaw, sgp41NOXRaw, sgp41VOCIndex, sgp41NOXIndex, sgp41ConditioningTime);
    readSFA3xData(sfa30, sfa30Temperature, sfa30Humidity, sfa30CH2OConcentration);
    readBMP3xxData(bmp390, bmp390Temperature, bmp390Pressure);
    readTSL2591Data(tsl2591, tsl2591Illuminance);
    digitalWrite(BLUE_LED_PIN, LOW);
    updateCloudVariables();
  }

  checkCloudConnection();
  ArduinoCloud.update();
}
