#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7);
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

void configureSensor() {
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true); /* Auto-gain ... switches automatically between 1x and 16x */

  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);   /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS); /* 16-bit data but slowest conversions */
}

void setup() {
  Serial.begin(9600);
  Serial.println("TSL2561 Test");
  lcd.begin(16, 2);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home();
  lcd.print("TSL2561 Test");
  configureSensor();
}

void loop() {
  sensors_event_t event;
  tsl.getEvent(&event);
  if (event.light) {
    Serial.print(event.light);
    Serial.println(" lux");
    lcd.setCursor(0, 1);
    lcd.print(event.light);
    lcd.print(" lux     ");
  } else {
    Serial.println("Sensor overload");
    lcd.setCursor(0, 1);
    lcd.print("Sensor overload");
  }
  delay(500);
}
