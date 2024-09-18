const int RX = 0;
const int TX = 1;
const int StepPulseX = 2;
const int StepPulseY = 3;
const int StepPulseZ = 4;
const int DirectionX = 5;
const int DirectionY = 6;
const int DirectionZ = 7;
const int StepperEnable = 8;
const int LimitX = 9;
const int LimitY = 10;
const int LimitZ = 11;
const int SpindleEnable = 12;
const int SpindleDirection = 13;

const int Abort = A0;
const int Hold = A1;
const int Resume = A2;
const int CoolantEnable = A3;
const int NC0 = A4;
const int NC1 = A5;

const double StepsPerCentimetre = 800;
const double LengthX = 7.1;   //13
const double LengthY = 16.9;  //18
const double SpacingX = 1.85;
const double SpacingY = 1.85;
const double DelayTime = 5000;
//const double DelayDropTime = 100;
//const double DelayPumpTime = 80000;

void HoldCheck() {
  if (digitalRead(Hold) == LOW || digitalRead(SpindleDirection) == LOW) {
    while (1) {
      digitalWrite(SpindleEnable, LOW);
      if (digitalRead(Hold) == HIGH && digitalRead(SpindleDirection) == HIGH) {
        break;
      }
    }
  }
}

void PumpDrop() {
  delay(DelayTime);
  digitalWrite(SpindleEnable, LOW);

  delay(DelayTime);
}

void AutoReset() {
  digitalWrite(DirectionY, HIGH);
  digitalWrite(DirectionX, HIGH);
  while (digitalRead(LimitY) == HIGH) {
    digitalWrite(StepPulseY, HIGH);
    delayMicroseconds(200);
    digitalWrite(StepPulseY, LOW);
    delayMicroseconds(200);
    HoldCheck();
  }
  while (digitalRead(LimitX) == HIGH) {
    digitalWrite(StepPulseX, HIGH);
    delayMicroseconds(200);
    digitalWrite(StepPulseX, LOW);
    delayMicroseconds(200);
    HoldCheck();
  }
  digitalWrite(DirectionY, LOW);
  digitalWrite(DirectionX, LOW);
}

void setup() {
  pinMode(RX, INPUT);
  pinMode(TX, OUTPUT);
  pinMode(StepPulseX, OUTPUT);
  pinMode(StepPulseY, OUTPUT);
  pinMode(StepPulseZ, OUTPUT);
  pinMode(DirectionX, OUTPUT);
  pinMode(DirectionY, OUTPUT);
  pinMode(DirectionZ, OUTPUT);
  pinMode(StepperEnable, OUTPUT);
  pinMode(LimitX, INPUT_PULLUP);
  pinMode(LimitY, INPUT_PULLUP);
  pinMode(LimitZ, INPUT_PULLUP);
  pinMode(SpindleEnable, OUTPUT);
  pinMode(SpindleDirection, INPUT_PULLUP);

  pinMode(Abort, INPUT_PULLUP);
  pinMode(Hold, INPUT_PULLUP);
  pinMode(Resume, INPUT_PULLUP);
  pinMode(CoolantEnable, INPUT_PULLUP);
  pinMode(NC0, INPUT_PULLUP);
  pinMode(NC1, INPUT_PULLUP);

  digitalWrite(StepperEnable, LOW);
  digitalWrite(DirectionX, LOW);
  digitalWrite(DirectionY, LOW);
  digitalWrite(DirectionZ, LOW);
  digitalWrite(SpindleEnable, HIGH);
}

void loop() {
  AutoReset();
  if (digitalRead(Abort) == LOW) {

    for (int X = 0; X < int(LengthX / SpacingX); X++) {
      PumpDrop();
      for (int Y = 0; Y < int(LengthY / SpacingY); Y++) {
        for (double S = 0; S < SpacingY * StepsPerCentimetre; S++) {
          digitalWrite(StepPulseY, HIGH);
          delayMicroseconds(150);
          digitalWrite(StepPulseY, LOW);
          delayMicroseconds(150);
          HoldCheck();
        }
        PumpDrop();
      }
      digitalWrite(DirectionY, HIGH);
      for (int Y = 0; Y < int(LengthY / SpacingY); Y++) {
        for (double S = 0; S < SpacingY * StepsPerCentimetre; S++) {
          digitalWrite(StepPulseY, HIGH);
          delayMicroseconds(100);
          digitalWrite(StepPulseY, LOW);
          delayMicroseconds(100);
          HoldCheck();
        }
      }
      digitalWrite(DirectionY, LOW);
      for (double S = 0; S < SpacingX * StepsPerCentimetre; S++) {
        digitalWrite(StepPulseX, HIGH);
        delayMicroseconds(150);
        digitalWrite(StepPulseX, LOW);
        delayMicroseconds(150);
        HoldCheck();
      }
      delay(DelayTime);
    }
    PumpDrop();
    for (int Y = 0; Y < int(LengthY / SpacingY); Y++) {
      for (double S = 0; S < SpacingY * StepsPerCentimetre; S++) {
        digitalWrite(StepPulseY, HIGH);
        delayMicroseconds(150);
        digitalWrite(StepPulseY, LOW);
        delayMicroseconds(150);
        HoldCheck();
      }
      PumpDrop();
    }
  }
}
