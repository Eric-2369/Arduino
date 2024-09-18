#include "arduino_secrets.h"
#include "thingProperties.h"
#include "deviceHandlers.h"
#include <ArduinoGraphics.h>
#include <Arduino_LED_Matrix.h>

WZ wz(Serial1);
ArduinoLEDMatrix matrix;

float wzCH2OConcentration = NAN;

void displayDataOnMatrix() {
  matrix.beginDraw();
  matrix.beginText(0, 1, 0xFFFFFF);

  matrix.textFont(Font_5x7);
  String displayText = isnan(wzCH2OConcentration) ? "NA    " : String(static_cast<int>(wzCH2OConcentration)) + "    ";
  matrix.println(displayText);

  matrix.endText();
  matrix.endDraw();
}

void displayDataOnSerial() {
  Serial.print("WZ CH2O Concentration: ");
  Serial.print(isnan(wzCH2OConcentration) ? "N/A" : String(wzCH2OConcentration, 0) + "ppb");
  Serial.println();
}

void initializeCloudVariables() {
  cloud_wzCH2OConcentration = 0.0;
}

void updateCloudVariables() {
  cloud_wzCH2OConcentration = wzCH2OConcentration;
}

void onCloudSystemResetChange() {
  if (cloud_systemReset) {
    NVIC_SystemReset();
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
  Serial1.begin(9600);
  delay(1000);

  initializeWZ(wz);
  matrix.begin();

  initProperties();
  initializeCloudVariables();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection, false);
  ArduinoCloud.printDebugInfo();

  Serial.println(F("All devices have been initialized."));
}

void loop() {
  static unsigned long lastWZReadTime = 0;
  const unsigned long readInterval = 1000;
  unsigned long currentMillis = millis();

  if (currentMillis - lastWZReadTime >= readInterval) {
    lastWZReadTime = currentMillis;
    readWZData(wz, wzCH2OConcentration);
    displayDataOnMatrix();
    displayDataOnSerial();
  }

  checkCloudConnection();
  updateCloudVariables();
  ArduinoCloud.update();
}
