#include "arduino_secrets.h"
#include "thingProperties.h"

#include <Arduino.h>
#include <WZ.h>
#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"

WZ wz(Serial1);
ArduinoLEDMatrix matrix;

float wzCH2OConcentration = NAN;

unsigned long lastWZReadTime = 0;

const unsigned long readInterval = 1000;

void initializeWZ() {
  Serial1.begin(9600);
  wz.passiveMode();
}

void readWZData() {
  WZ::DATA ch2oData;
  wz.requestRead();
  if (wz.readUntil(ch2oData)) {
    wzCH2OConcentration = ch2oData.HCHO_PPB;
  } else {
    Serial.println(F("WZ measurement error."));
  }
}

void displayDataOnSerial() {
  Serial.print("WZ CH2O Concentration: ");
  Serial.print(isnan(wzCH2OConcentration) ? "N/A" : String(wzCH2OConcentration, 0) + "ppb");
  Serial.println();
}

void displayDataOnMatrix() {
  matrix.beginDraw();
  matrix.beginText(0, 1, 0xFFFFFF);

  matrix.textFont(Font_5x7);
  String displayText = isnan(wzCH2OConcentration) ? "NA    " : String(static_cast<int>(wzCH2OConcentration)) + "    ";
  matrix.println(displayText);

  matrix.endText();
  matrix.endDraw();
}

void initializeCloudVariables() {
  cloud_wzCH2OConcentration = 0.0;
}

void updateCloudVariables() {
  cloud_wzCH2OConcentration = wzCH2OConcentration;
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    delay(100);
  }
  delay(1500);

  initializeWZ();
  matrix.begin();

  initProperties();
  initializeCloudVariables();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  ArduinoCloud.printDebugInfo();
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastWZReadTime >= readInterval) {
    lastWZReadTime = currentMillis;
    readWZData();
    displayDataOnMatrix();
    displayDataOnSerial();
  }

  updateCloudVariables();
  ArduinoCloud.update();
}
