// ==== [ Motor Pins ] ========================================================
#define motorA1                 6   // Motor A1 LF
#define motorA2                 9   // Motor A2 LB
#define motorB1                 10  // Motor B1 RF
#define motorB2                 11  // Motor B2 RB

// ==== [ Line Sensor Pins ] ===================================================
const int linePins[] = {A0, A1, A2, A3, A4, A5, A6, A7}; // 8 Line sensors

// ==== [ Gripper Pin ] ========================================================
#define gripperPin              5 // Gripper servo

// ==== [ Line Sensor Threshold ] ==============================================
#define lineThreshold           800 // 0-1023
#define heavyTurnAdjustment     255
#define hardTurnAdjustment      175
#define strongTurnAdjustment    125
#define veerAdjustment          25

// ==== [ Motor Speeds ] =======================================================
#define baseSpeed               255 // The base speed of the motors
#define startSpeed              210 // The speed of the motors when starting

// ==== [ Setup ] ==============================================================
void setup() {
  setupLineSensors();
  setupMotorPins();
  gripperOpen();
  initializeMotors();
}

void setupLineSensors() {
  for (int i = 0; i < 8; i++) {
    pinMode(linePins[i], INPUT);
  }
}

void setupMotorPins() {
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(gripperPin, OUTPUT);
}

void gripperOpen() {
  moveGripper(1700);
}

void gripperClose() {
  moveGripper(1050);
}

void moveGripper(int pulseDuration) {
  for (int i = 0; i < 10; i++) {
    delay(20);
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(pulseDuration);
    digitalWrite(gripperPin, LOW);
  }
}

void initializeMotors() {
  analogWrite(motorA1, startSpeed);
  analogWrite(motorB1, startSpeed);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);
  delay(1750);
  analogWrite(motorA1, 0);
  analogWrite(motorB1, 0);
  gripperClose();
  delay(500);
  analogWrite(motorA2, startSpeed);
  analogWrite(motorB1, startSpeed);
  delay(500);
  analogWrite(motorA2, 0);
  analogWrite(motorB2, 0);
  delay(1000);
}

// ==== [ Loop ] ===============================================================
void loop() {
  int sensorValues[8];
  readLineSensors(sensorValues);
  determineLineFollowing(sensorValues);
}

void readLineSensors(int sensorValues[]) {
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(linePins[i]);
  }
}

void determineLineFollowing(int sensorValues[]) {
  bool extremeLeft = sensorValues[0] > lineThreshold;
  bool farLeftOnLine = sensorValues[1] > lineThreshold;
  bool lessFarLeftOnLine = sensorValues[2] > lineThreshold;
  bool evenlessFarLeftOnLine = sensorValues[3] > lineThreshold;
  bool extremeRight = sensorValues[7] > lineThreshold;
  bool farRightOnLine = sensorValues[6] > lineThreshold;
  bool lessFarRightOnLine = sensorValues[5] > lineThreshold;
  bool evenlessFarRightOnLine = sensorValues[4] > lineThreshold;
  bool sensors3And4Active = sensorValues[3] > lineThreshold && sensorValues[4] > lineThreshold;
  bool allCenterSensorsActive = lessFarLeftOnLine && evenlessFarLeftOnLine && evenlessFarRightOnLine && lessFarRightOnLine;

  if (allCenterSensorsActive && !sensors3And4Active) {
    goStraight();
    delay(500);
    if (allCenterSensorsActive && !sensors3And4Active) {
      stopMotors();
      dropGripper();
      delay(500);
      moveBackward();
    }
  } else if (farLeftOnLine) {
    hardRightTurn();
  } else if (lessFarLeftOnLine) {
    strongRightTurn();
  } else if (lessFarRightOnLine) {
    strongLeftTurn();
  } else if (farRightOnLine) {
    hardLeftTurn();
  }

  if (extremeLeft && !sensors3And4Active) {
    rotateHeavyLeft();
  } else if (extremeRight && !sensors3And4Active) {
    rotateHeavyRight();
  } else if (allCenterSensorsActive) {
    moveForward();
  }
}

void goStraight() {
  analogWrite(motorA1, baseSpeed);
  analogWrite(motorB1, baseSpeed);
}

void hardRightTurn() {
  analogWrite(motorA1, baseSpeed - hardTurnAdjustment);
  analogWrite(motorB1, baseSpeed);
}

void strongRightTurn() {
  analogWrite(motorA1, baseSpeed - strongTurnAdjustment);
  analogWrite(motorB1, baseSpeed);
}

void strongLeftTurn() {
  analogWrite(motorA1, baseSpeed);
  analogWrite(motorB1, baseSpeed - strongTurnAdjustment * 2);
}

void hardLeftTurn() {
  analogWrite(motorA1, baseSpeed);
  analogWrite(motorB1, baseSpeed - hardTurnAdjustment * 2);
}

void rotateHeavyLeft() {
  analogWrite(motorA1, baseSpeed - heavyTurnAdjustment);
  analogWrite(motorB1, baseSpeed);
}

void rotateHeavyRight() {
  analogWrite(motorA1, baseSpeed);
  analogWrite(motorB1, baseSpeed - heavyTurnAdjustment);
}

void stopMotors() {
  analogWrite(motorA1, 0);
  analogWrite(motorB1, 0);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);
}

void dropGripper() {
  gripperOpen();
}

void moveBackward() {
  digitalWrite(motorA1, LOW);
  digitalWrite(motorB1, LOW);
  digitalWrite(motorA2, baseSpeed);
  digitalWrite(motorB2, baseSpeed);
  delay(2000);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);
}

void moveForward() {
  digitalWrite(motorA2, baseSpeed);
  digitalWrite(motorB2, baseSpeed);
  delay(1000);
  if (digitalRead(motorA2) == HIGH && digitalRead(motorB2) == HIGH) {
    analogWrite(motorA1, 0);
    analogWrite(motorB1, 0);
  }
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);
}