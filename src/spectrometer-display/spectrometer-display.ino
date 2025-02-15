#include "arduino_secrets.h"
#include "thingProperties.h"
#include "deviceHandlers.h"

#include <SparkFun_AS7265X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

AS7265X as7265x;
Adafruit_ST7735 lcd = Adafruit_ST7735(/* CS  */ 10, /* DC  */ 3, /* RST */ 2);

const int wavelengths[18] = {
  410, 435, 460, 485, 510, 535,
  560, 585, 610, 645, 680, 705,
  730, 760, 810, 860, 900, 940
};

const uint16_t spectrumColors[18] = {
  0x001F, 0x041F, 0x07FF, 0x07DF, 0x07E0, 0x5FE0,
  0xFFE0, 0xFBE0, 0xF800, 0xB800, 0x7800, 0x7000,
  0x6000, 0x4000, 0x2000, 0x1000, 0x0800, 0x0400
};

bool i2cInitialized = false;

float as72651Temperature = NAN;
float as7265xTemperature = NAN;
float as72653Spectrum410Irradiance = NAN;
float as72653Spectrum435Irradiance = NAN;
float as72653Spectrum460Irradiance = NAN;
float as72653Spectrum485Irradiance = NAN;
float as72653Spectrum510Irradiance = NAN;
float as72653Spectrum535Irradiance = NAN;
float as72652Spectrum560Irradiance = NAN;
float as72652Spectrum585Irradiance = NAN;
float as72651Spectrum610Irradiance = NAN;
float as72652Spectrum645Irradiance = NAN;
float as72651Spectrum680Irradiance = NAN;
float as72652Spectrum705Irradiance = NAN;
float as72651Spectrum730Irradiance = NAN;
float as72651Spectrum760Irradiance = NAN;
float as72651Spectrum810Irradiance = NAN;
float as72651Spectrum860Irradiance = NAN;
float as72652Spectrum900Irradiance = NAN;
float as72652Spectrum940Irradiance = NAN;

void updateCloudVariables() {
  cloud_as72651Temperature = as72651Temperature;
  cloud_as7265xTemperature = as7265xTemperature;
  cloud_as72653Spectrum410Irradiance = as72653Spectrum410Irradiance;
  cloud_as72653Spectrum435Irradiance = as72653Spectrum435Irradiance;
  cloud_as72653Spectrum460Irradiance = as72653Spectrum460Irradiance;
  cloud_as72653Spectrum485Irradiance = as72653Spectrum485Irradiance;
  cloud_as72653Spectrum510Irradiance = as72653Spectrum510Irradiance;
  cloud_as72653Spectrum535Irradiance = as72653Spectrum535Irradiance;
  cloud_as72652Spectrum560Irradiance = as72652Spectrum560Irradiance;
  cloud_as72652Spectrum585Irradiance = as72652Spectrum585Irradiance;
  cloud_as72651Spectrum610Irradiance = as72651Spectrum610Irradiance;
  cloud_as72652Spectrum645Irradiance = as72652Spectrum645Irradiance;
  cloud_as72651Spectrum680Irradiance = as72651Spectrum680Irradiance;
  cloud_as72652Spectrum705Irradiance = as72652Spectrum705Irradiance;
  cloud_as72651Spectrum730Irradiance = as72651Spectrum730Irradiance;
  cloud_as72651Spectrum760Irradiance = as72651Spectrum760Irradiance;
  cloud_as72651Spectrum810Irradiance = as72651Spectrum810Irradiance;
  cloud_as72651Spectrum860Irradiance = as72651Spectrum860Irradiance;
  cloud_as72652Spectrum900Irradiance = as72652Spectrum900Irradiance;
  cloud_as72652Spectrum940Irradiance = as72652Spectrum940Irradiance;
}

void checkCloudConnection() {
  static uint32_t lastCloudConnectedTime = 0;

  if (ArduinoCloud.connected()) {
    lastCloudConnectedTime = millis();
  } else {
    if (millis() - lastCloudConnectedTime >= 120 * 1000) {
      Serial.println(F("Cloud connection lost for 120 seconds. System will reset."));
      delay(10 * 1000);
      NVIC_SystemReset();
    }
  }
}

void onCloudSystemResetChange() {
  if (cloud_systemReset) {
    delay(10 * 1000);
    NVIC_SystemReset();
  }
}

void drawSpectrumHistogram() {
  lcd.fillScreen(ST77XX_BLACK);  // 清屏

  int barWidth = 6;        // 每个柱状条的宽度
  int maxBarHeight = 100;  // 最大柱状条高度
  int xOffset = 22;        // X 轴起始偏移量
  int yOffset = 8;         // Y 轴起始偏移量

  // 绘制 Y 轴（波长标注）
  lcd.setTextSize(1);
  lcd.setTextColor(ST77XX_WHITE);
  for (int i = 0; i < 18; i++) {
    lcd.setCursor(2, yOffset + i * (barWidth + 2));
    lcd.print(wavelengths[i]);
  }

  // 绘制柱状图
  float spectrumData[18] = {
    as72653Spectrum410Irradiance,
    as72653Spectrum435Irradiance,
    as72653Spectrum460Irradiance,
    as72653Spectrum485Irradiance,
    as72653Spectrum510Irradiance,
    as72653Spectrum535Irradiance,
    as72652Spectrum560Irradiance,
    as72652Spectrum585Irradiance,
    as72651Spectrum610Irradiance,
    as72652Spectrum645Irradiance,
    as72651Spectrum680Irradiance,
    as72652Spectrum705Irradiance,
    as72651Spectrum730Irradiance,
    as72651Spectrum760Irradiance,
    as72651Spectrum810Irradiance,
    as72651Spectrum860Irradiance,
    as72652Spectrum900Irradiance,
    as72652Spectrum940Irradiance
  };

  for (int i = 0; i < 18; i++) {
    int barHeight = spectrumData[i] * maxBarHeight;          // 计算柱状条高度
    if (barHeight > maxBarHeight) barHeight = maxBarHeight;  // 限制最大高度

    int x = xOffset;                       // 柱状条 X 坐标
    int y = yOffset + i * (barWidth + 2);  // 柱状条 Y 坐标

    // 绘制柱状条
    lcd.fillRect(x, y, barHeight, barWidth, spectrumColors[i]);
  }

  // 绘制 X 轴
  lcd.drawLine(xOffset, yOffset, xOffset, yOffset + 18 * (barWidth + 2), ST77XX_WHITE);
}

void setup() {
  Serial.begin(9600);

  if (clearI2C() == 0) {
    Wire.begin();
    Wire1.begin();
    if (!as7265x.begin(Wire1)) {
      Serial.println("传感器未连接，请检查接线。");
      while (1);
    }

    as7265x.setGain(AS7265X_GAIN_1X);
    as7265x.setMeasurementMode(AS7265X_MEASUREMENT_MODE_6CHAN_CONTINUOUS);
    as7265x.setIntegrationCycles(255);
    as7265x.setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS7265x_LED_WHITE);
    as7265x.setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS7265x_LED_UV);
    as7265x.setBulbCurrent(AS7265X_LED_CURRENT_LIMIT_12_5MA, AS7265x_LED_IR);
    as7265x.disableIndicator();
    as7265x.setIndicatorCurrent(AS7265X_INDICATOR_CURRENT_LIMIT_8MA);
    as7265x.disableInterrupt();

    lcd.initR(INITR_BLACKTAB);
    lcd.fillScreen(ST77XX_BLACK);
    lcd.setRotation(0);

    i2cInitialized = true;
  }

  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection, false);
  ArduinoCloud.printDebugInfo();

  Serial.println(F("All devices have been initialized."));
}

void loop() {
  static uint32_t lastReadTime = 0;

  if (i2cInitialized && (millis() - lastReadTime >= 1000)) {
    lastReadTime = millis();
    as7265x.takeMeasurements();
    as72653Spectrum410Irradiance = as7265x.getCalibratedA() / 1000.0;
    as72653Spectrum435Irradiance = as7265x.getCalibratedB() / 1000.0;
    as72653Spectrum460Irradiance = as7265x.getCalibratedC() / 1000.0;
    as72653Spectrum485Irradiance = as7265x.getCalibratedD() / 1000.0;
    as72653Spectrum510Irradiance = as7265x.getCalibratedE() / 1000.0;
    as72653Spectrum535Irradiance = as7265x.getCalibratedF() / 1000.0;
    as72652Spectrum560Irradiance = as7265x.getCalibratedG() / 1000.0;
    as72652Spectrum585Irradiance = as7265x.getCalibratedH() / 1000.0;
    as72651Spectrum610Irradiance = as7265x.getCalibratedR() / 1000.0;
    as72652Spectrum645Irradiance = as7265x.getCalibratedI() / 1000.0;
    as72651Spectrum680Irradiance = as7265x.getCalibratedS() / 1000.0;
    as72652Spectrum705Irradiance = as7265x.getCalibratedJ() / 1000.0;
    as72651Spectrum730Irradiance = as7265x.getCalibratedT() / 1000.0;
    as72651Spectrum760Irradiance = as7265x.getCalibratedU() / 1000.0;
    as72651Spectrum810Irradiance = as7265x.getCalibratedV() / 1000.0;
    as72651Spectrum860Irradiance = as7265x.getCalibratedW() / 1000.0;
    as72652Spectrum900Irradiance = as7265x.getCalibratedK() / 1000.0;
    as72652Spectrum940Irradiance = as7265x.getCalibratedL() / 1000.0;
    as72651Temperature = as7265x.getTemperature();
    as7265xTemperature = as7265x.getTemperatureAverage();

    updateCloudVariables();
    drawSpectrumHistogram();
  }

  checkCloudConnection();
  ArduinoCloud.update();
}
