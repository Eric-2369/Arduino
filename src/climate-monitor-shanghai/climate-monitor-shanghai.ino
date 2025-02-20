#include "arduino_secrets.h"
#include "thingProperties.h"
#include "deviceHandlers.h"

SensirionI2cSht4x sht45;
SensirionI2cScd4x scd41;
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

float sht45Temperature = NAN;
float sht45Humidity = NAN;
float scd41Temperature = NAN;
float scd41Humidity = NAN;
float scd41CO2Concentration = NAN;
float sgp41VOCRaw = NAN;
float sgp41NOXRaw = NAN;
float sgp41VOCIndex = NAN;
float sgp41NOXIndex = NAN;
float sfa30Temperature = NAN;
float sfa30Humidity = NAN;
float sfa30CH2OConcentration = NAN;
float bmp390Temperature = NAN;
float bmp390Pressure = NAN;
float tsl2591Illuminance = NAN;

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
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
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

  if (clearI2C(WIRE_SDA_PIN, WIRE_SCL_PIN) == 0) {
    Wire.begin();
    initializeSHT4x(sht45, Wire);
    initializeSCD4x(scd41, Wire);
    initializeSGP41(sgp41, Wire);
    initializeSFA3x(sfa30, Wire);
    initializeBMP3xx(bmp390, Wire);
    initializeTSL2591(tsl2591, Wire);
    i2cInitialized = true;
  }

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection, false);
  ArduinoCloud.printDebugInfo();

  Serial.println(F("All devices have been initialized."));
}

void loop() {
  digitalWrite(BLUE_LED_PIN, HIGH);
  delayMicroseconds(10000);
  digitalWrite(BLUE_LED_PIN, LOW);
  delayMicroseconds(10000);

  static uint16_t sgp41ConditioningTime = 5;
  static uint32_t lastReadTime = 0;

  if (i2cInitialized && (millis() - lastReadTime >= 1000)) {
    lastReadTime = millis();
    readSHT4xData(sht45, sht45Temperature, sht45Humidity);
    readSCD4xData(scd41, scd41Temperature, scd41Humidity, scd41CO2Concentration);
    readSGP41Data(sgp41, vocAlgorithm, noxAlgorithm, sht45Temperature, sht45Humidity, sgp41VOCRaw, sgp41NOXRaw, sgp41VOCIndex, sgp41NOXIndex, sgp41ConditioningTime);
    readSFA3xData(sfa30, sfa30Temperature, sfa30Humidity, sfa30CH2OConcentration);
    readBMP3xxData(bmp390, bmp390Temperature, bmp390Pressure);
    readTSL2591Data(tsl2591, tsl2591Illuminance);
    updateCloudVariables();
  }

  checkCloudConnection();
  ArduinoCloud.update();
}
