#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7);

const int Pause = 2;
const int StepPulse = 4;
const int StepDirection = 5;
const int StepEnable = 6;
const int Hold = 10;
const int LED = 13;
double StartProportion = 0.1;
double EndProportion = 0.9;
double TotalTime = 10;
double Flow = 1;  //ml/s 0.25
double FlowNow = 0;
int StartTime = 0;

void LcdInitialize() {
  lcd.begin(16, 2);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  delay(1000);
  lcd.home();
  lcd.print("Gradient Pump");
  delay(1000);
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(5000);
}

void LcdPrint() {
  lcd.home();
  lcd.print("From ");
  lcd.print(int(StartProportion * 100));
  lcd.print("%");
  lcd.print(" to ");
  lcd.print(int(EndProportion * 100));
  lcd.print("% ");
  lcd.setCursor(0, 1);
  lcd.print(Flow);
  lcd.print("mL/s");
  lcd.print(" ");
  lcd.print(int(TotalTime));
  lcd.print("mins ");
}

void SetState() {
Start:
  while (1) {
    if (digitalRead(7) == LOW) {
      StartProportion = StartProportion - 0.01;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(8) == LOW) {
      StartProportion = StartProportion + 0.01;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(9) == LOW) {
      LcdPrint();
      delay(1000);
      goto End;
    }
  }
End:
  while (1) {
    if (digitalRead(7) == LOW) {
      EndProportion = EndProportion - 0.01;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(8) == LOW) {
      EndProportion = EndProportion + 0.01;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(9) == LOW) {
      LcdPrint();
      delay(1000);
      goto Flow;
    }
  }
Flow:
  while (1) {
    if (digitalRead(7) == LOW) {
      Flow = Flow - 0.01;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(8) == LOW) {
      Flow = Flow + 0.01;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(9) == LOW) {
      LcdPrint();
      delay(1000);
      goto Time;
    }
  }
Time:
  while (1) {
    if (digitalRead(7) == LOW) {
      TotalTime = TotalTime - 1;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(8) == LOW) {
      TotalTime = TotalTime + 1;
      LcdPrint();
      delay(10);
    }
    if (digitalRead(9) == LOW) {
      LcdPrint();
      delay(1000);
      //goto Start;
      break;
    }
  }
}

void PuaseFun() {
  PauseCheck();
}

void PauseCheck() {
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
  pinMode(Hold, INPUT_PULLUP);
  digitalWrite(StepEnable, LOW);
  digitalWrite(StepDirection, LOW);
  digitalWrite(Hold, HIGH);
  pinMode(11, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), PuaseFun, FALLING);

  LcdInitialize();
  LcdPrint();
  SetState();
  if (digitalRead(11) == LOW) {
    StartTime = millis();
    digitalWrite(Hold, LOW);
    long TotalMilliSecond = TotalTime * 60 * 1000;
    double ProportionRatio = (EndProportion - StartProportion) / TotalMilliSecond;
    while (millis() < TotalMilliSecond) {
      int FlowNow = 12800 * Flow * (StartProportion + ProportionRatio * millis());
      tone(StepPulse, FlowNow, 1000);
      delay(1000);
      noTone(StepPulse);
      lcd.home();
      lcd.print("Now ");
      lcd.print(100 * (StartProportion + ProportionRatio * millis()));
      lcd.print("%      ");
      lcd.setCursor(0, 1);
      lcd.print(Flow);
      lcd.print("mL/s");
      lcd.print(" ");
      lcd.print(int(TotalTime - (millis() - StartTime) / 60000));
      lcd.print("mins ");
      //HoldCheck();
    }
    digitalWrite(Hold, HIGH);
    lcd.clear();
    lcd.home();
    lcd.print("Done!");
  }
}

void loop() {
}
