// Code generated by Arduino IoT Cloud, DO NOT EDIT.

#include <ArduinoIoTCloud.h>
#include <Arduino_ConnectionHandler.h>

const char SSID[] = SECRET_SSID;           // Network SSID (name)
const char PASS[] = SECRET_OPTIONAL_PASS;  // Network password (use for WPA, or use as key for WEP)

void onCloudSystemResetChange();

CloudSwitch cloud_systemReset;
CloudCounter cloud_wzCH2OConcentration;

void initProperties() {
  ArduinoCloud.addProperty(cloud_systemReset, READWRITE, 1 * SECONDS, onCloudSystemResetChange);
  ArduinoCloud.addProperty(cloud_wzCH2OConcentration, READ, 1 * SECONDS, NULL);
}

WiFiConnectionHandler ArduinoIoTPreferredConnection(SSID, PASS);