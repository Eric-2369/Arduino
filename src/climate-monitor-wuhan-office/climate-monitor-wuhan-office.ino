#include "arduino_secrets.h"
#include "thingProperties.h"
#include "deviceHandlers.h"

SensirionI2cSht4x sht40;
SensirionI2CScd4x scd40;
SensirionI2CSgp40 sgp40;
Adafruit_BMP3XX bmp390;
Adafruit_TSL2561_Unified tsl2561 = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

VOCGasIndexAlgorithm vocAlgorithm;

LiquidCrystal_PCF8574 lcd(0x27);

const uint8_t RED_LED_PIN = 2;
const uint8_t YELLOW_LED_PIN = 3;
const uint8_t GREEN_LED_PIN = 4;
const uint8_t BLUE_LED_PIN = 5;

bool i2cInitialized = false;

float sht40Temperature = 0.0;
float sht40Humidity = 0.0;
float scd40Temperature = 0.0;
float scd40Humidity = 0.0;
float scd40CO2Concentration = 0.0;
float sgp40VOCRaw = 0.0;
float sgp40VOCIndex = 0.0;
float bmp390Temperature = 0.0;
float bmp390Pressure = 0.0;
float tsl2561Illuminance = 0.0;

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

void displayDataOnScreen() {
  lcd.setCursor(0, 0);
  lcd.print(String(sht40Temperature, 2) + " ");
  lcd.print(String(sht40Humidity, 2) + " ");
  lcd.print(String(scd40CO2Concentration, 0));
  lcd.print("                ");

  lcd.setCursor(0, 1);
  lcd.print(String(sgp40VOCIndex, 0) + " ");
  lcd.print(String(bmp390Pressure, 0) + " ");
  lcd.print(String(tsl2561Illuminance, 0));
  lcd.print("                ");
}

void setup() {
  Serial.begin(9600);

  initializeLED();

  if (clearI2C() == 0) {
    Wire.begin();
    initializeSHT4x(sht40);
    initializeSCD4x(scd40);
    initializeSGP40(sgp40);
    initializeBMP3xx(bmp390);
    initializeTSL2561(tsl2561);
    initializeLCD(lcd);
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

  static uint32_t lastReadTime = 0;

  if (i2cInitialized && (millis() - lastReadTime >= 1000)) {
    lastReadTime = millis();
    digitalWrite(BLUE_LED_PIN, HIGH);
    readSHT4xData(sht40, sht40Temperature, sht40Humidity);
    readSCD4xData(scd40, scd40Temperature, scd40Humidity, scd40CO2Concentration);
    readSGP40Data(sgp40, vocAlgorithm, sht40Temperature, sht40Humidity, sgp40VOCRaw, sgp40VOCIndex);
    readBMP3xxData(bmp390, bmp390Temperature, bmp390Pressure);
    readTSL2561Data(tsl2561, tsl2561Illuminance);
    digitalWrite(BLUE_LED_PIN, LOW);
    displayDataOnScreen();
    updateCloudVariables();
  }

  checkCloudConnection();
  ArduinoCloud.update();
}
