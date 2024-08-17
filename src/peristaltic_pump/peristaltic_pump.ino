#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7);

const int Pause = 2;
const int StepPulse = 4;
const int StepDirection = 5;
const int StepEnable = 6;
const int ButtonLeft = 8;
const int ButtonRight = 9;
const int ButtonClick = 7;
int Direction = 0;
int Flow = 10;  //ml/h 12800 step/ml
long StartTime = 0;

void LcdInitialize() {
  lcd.begin(16, 2);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  delay(1000);
  lcd.home();
  lcd.print("Peristaltic Pump");
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);
}

void LcdPrint() {
  lcd.home();
  lcd.print("Dir:");
  if (Direction == 0) {
    lcd.print("Forward      ");
    digitalWrite(StepDirection, HIGH);
  };
  if (Direction == 1) {
    lcd.print("Reverse      ");
    digitalWrite(StepDirection, LOW);
  };
  lcd.setCursor(0, 1);
  lcd.print("Flow:");
  lcd.print(Flow);
  lcd.print("mL/h      ");
}

void SetState() {
Direction:
  while (1) {
    if (digitalRead(ButtonLeft) == LOW) {
      Direction = 0;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(ButtonRight) == LOW) {
      Direction = 1;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(ButtonClick) == LOW) {
      LcdPrint();
      delay(1000);
      goto Flow;
    }
  }
Flow:
  while (1) {
    if (digitalRead(ButtonLeft) == LOW) {
      Flow = Flow - 1;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(ButtonRight) == LOW) {
      Flow = Flow + 1;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(ButtonClick) == LOW) {
      LcdPrint();
      delay(1000);
      //goto Direction;
      break;
    }
  }
}

void PuaseFun() {
  if (digitalRead(Pause) == LOW) {
    while (1) {
      if (digitalRead(Pause) == HIGH) {
        break;
      }
    }
  }
}

void setup() {
  pinMode(Pause, INPUT_PULLUP);
  pinMode(StepEnable, OUTPUT);
  pinMode(StepDirection, OUTPUT);
  pinMode(StepPulse, OUTPUT);
  pinMode(ButtonLeft, INPUT_PULLUP);
  pinMode(ButtonRight, INPUT_PULLUP);
  pinMode(ButtonClick, INPUT_PULLUP);
  digitalWrite(StepEnable, LOW);
  digitalWrite(StepDirection, HIGH);
  attachInterrupt(digitalPinToInterrupt(2), PuaseFun, FALLING);
  LcdInitialize();
  LcdPrint();
  SetState();
  StartTime = millis();
  double FlowNow = 128 / 36 * Flow;
  tone(StepPulse, int(FlowNow));
  while (1) {
    lcd.setCursor(0, 1);
    lcd.print("Flow:");
    lcd.print(Flow);
    lcd.print("mL/h ");
    lcd.print(int((millis() - StartTime) / 60000));
    lcd.print("mins ");
  }
}

void loop() {
}
