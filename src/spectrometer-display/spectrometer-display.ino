#include "arduino_secrets.h"
#include "thingProperties.h"
#include "deviceHandlers.h"

#include <SparkFun_AS7265X.h>
#include <Adafruit_ST7735.h>

AS7265X as7265x;
Adafruit_ST7735 lcd = Adafruit_ST7735(/* CS  */ 10, /* DC  */ 3, /* RST */ 2);

const int wavelengths[18] = {
  410, 435, 460, 485, 510, 535,
  560, 585, 610, 645, 680, 705,
  730, 760, 810, 860, 900, 940
};

const uint16_t spectrumColors[18] = {
  0x781B, 0x201F, 0x03DF, 0x075F, 0x07E0, 0x77E0,
  0xC7E0, 0xFF60, 0xFCC0, 0xF800, 0xF800, 0xF000,
  0xC800, 0x8800, 0x6000, 0x6000, 0x6000, 0x6000
};

bool i2cInitialized = false;

float as72651Temperature = NAN;
float as72652Temperature = NAN;
float as72653Temperature = NAN;
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
  cloud_as72652Temperature = as72652Temperature;
  cloud_as72653Temperature = as72653Temperature;
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

void initializeLCD(Adafruit_ST7735& lcd) {
  lcd.initR(INITR_BLACKTAB);
  lcd.setRotation(0);
  lcd.fillScreen(ST77XX_BLACK);
}

void displayDataOnScreen() {
  lcd.fillScreen(ST77XX_BLACK);  // 清屏

  int barWidth = 6;        // 每个柱状条的宽度
  int maxBarHeight = 100;  // 最大柱状条高度
  int xOffset = 22;        // X 轴起始偏移量
  int yOffset = 8;         // Y 轴起始偏移量

  // 获取光谱数据
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

  // 1. 找到数据中的最大值
  float maxValue = 0;
  for (int i = 0; i < 18; i++) {
    if (spectrumData[i] > maxValue) {
      maxValue = spectrumData[i];
    }
  }

  // 2. 设置最小缩放值，防止 maxValue 过小
  float minScale = 20.0;  // 设定最小比例尺
  if (maxValue < minScale) {
    maxValue = minScale;
  }

  // 3. 绘制 Y 轴（波长标注）
  lcd.setTextSize(1);
  lcd.setTextColor(ST77XX_WHITE);
  for (int i = 0; i < 18; i++) {
    lcd.setCursor(2, yOffset + i * (barWidth + 2));
    lcd.print(wavelengths[i]);
  }

  // 4. 绘制柱状图
  for (int i = 0; i < 18; i++) {
    int barHeight = (spectrumData[i] / maxValue) * maxBarHeight;  // 归一化到 maxBarHeight
    if (barHeight > maxBarHeight) barHeight = maxBarHeight;       // 限制最大高度

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

  if (clearI2C(WIRE1_SDA_PIN, WIRE1_SCL_PIN) == 0) {
    Wire1.begin();
    initializeAS7265X(as7265x, Wire1);
    initializeLCD(lcd);
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
    readAS7265XData(as7265x, as72651Temperature, as72652Temperature, as72653Temperature, as72653Spectrum410Irradiance, as72653Spectrum435Irradiance, as72653Spectrum460Irradiance, as72653Spectrum485Irradiance, as72653Spectrum510Irradiance, as72653Spectrum535Irradiance, as72652Spectrum560Irradiance, as72652Spectrum585Irradiance, as72651Spectrum610Irradiance, as72652Spectrum645Irradiance, as72651Spectrum680Irradiance, as72652Spectrum705Irradiance, as72651Spectrum730Irradiance, as72651Spectrum760Irradiance, as72651Spectrum810Irradiance, as72651Spectrum860Irradiance, as72652Spectrum900Irradiance, as72652Spectrum940Irradiance);
    updateCloudVariables();
    displayDataOnScreen();
  }

  checkCloudConnection();
  ArduinoCloud.update();
}
