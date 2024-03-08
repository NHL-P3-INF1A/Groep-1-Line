#include <Arduino.h>

/*///////////////// define variables /////////////////*/

const int motorA1 = 6;
const int motorA2 = 9;
const int motorB1 = 10;
const int motorB2 = 11;

const int linePins[] = {A0, A1, A2, A3, A4, A5, A6, A7};

const int lineThreshold = 700;

const int gripperPin = 5;

int heavyTurnAdjustment = 255;
int hardTurnAdjustment = 200;

int strongTurnAdjustment = 150;
int veerAdjustment = 25;

// Base speed of the robot
const int baseSpeedLeft = 240;
const int baseSpeedRight = 255;
const int startSpeed = 200;

int delayTime = 1000;

bool endOfLine = false;

const int SENSOR_COUNT = 8;
int maxValues[SENSOR_COUNT] = {0};
int minValues[SENSOR_COUNT] = {1023};
int calibrationThreshold[SENSOR_COUNT];

bool endOfBlackSquare = analogRead(A4) > lineThreshold;

/*///////////////// Function ///////////////*/

void calibrateSensors()
{
  analogWrite(motorA1, startSpeed);
  analogWrite(motorB1, startSpeed);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);
  delay(1500);

  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    int sensorValue = analogRead(linePins[i]);
    maxValues[i] = max(maxValues[i], sensorValue);
    minValues[i] = min(minValues[i], sensorValue);

    calibrationThreshold[i] = (maxValues[i] + minValues[i]) / 2 + 100;
  }

  if (endOfBlackSquare)
  {
    analogWrite(motorA1, 0);
    analogWrite(motorB1, 0);
    gripperClose();
    delay(500);
    analogWrite(motorA2, 250);
    analogWrite(motorB1, 250);
    delay(500);
    analogWrite(motorA2, 0);
    analogWrite(motorB2, 0);
  }
}

void gripperOpen()
{
  moveGripper(1700);
}

void gripperClose()
{
  moveGripper(1050);
}

void moveGripper(int pulseDuration)
{
  for (int i = 0; i < 10; i++)
  {
    delay(20);
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(pulseDuration);
    digitalWrite(gripperPin, LOW);
  }
}

/*///////////////// setup /////////////////*/

void setup()
{
  gripperOpen();
  // Initialize motor control pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Initialize line sensor pins as inputs
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  calibrateSensors();
}

/*///////////////// follow line loop /////////////////*/

void loop()
{
  bool extremeLeft = analogRead(A0) > lineThreshold;
  bool farLeftOnLine = analogRead(A1) > lineThreshold;
  bool lessFarLeftOnLine = analogRead(A2) > lineThreshold;
  bool evenlessFarLeftOnLine = analogRead(A3) > lineThreshold;
  bool extremeRight = analogRead(A7) > lineThreshold;
  bool farRightOnLine = analogRead(A6) > lineThreshold;
  bool lessFarRightOnLine = analogRead(A5) > lineThreshold;
  bool evenlessFarRightOnLine = analogRead(A4) > lineThreshold;

  if (farLeftOnLine)
  {
    // Hard right turn
    analogWrite(motorA1, baseSpeedLeft - hardTurnAdjustment); // Even slower left motor
    analogWrite(motorB1, baseSpeedRight - 50);
  }
  else if (lessFarLeftOnLine)
  {
    // Strong right turn
    analogWrite(motorA1, baseSpeedLeft - strongTurnAdjustment);
    analogWrite(motorB1, baseSpeedRight - 35);
  }
  else if (evenlessFarLeftOnLine)
  {
    // Veer left
    analogWrite(motorA1, baseSpeedLeft);
    analogWrite(motorB1, baseSpeedRight - veerAdjustment);
  }
  else if (evenlessFarRightOnLine)
  {
    // Veer right
    analogWrite(motorA1, baseSpeedLeft - veerAdjustment);
    analogWrite(motorB1, baseSpeedRight);
  }
  else if (lessFarRightOnLine)
  {
    // Strong left turn
    analogWrite(motorA1, baseSpeedLeft - 35);
    analogWrite(motorB1, baseSpeedRight);
  }
  else if (farRightOnLine)
  {
    // Hard left turn
    analogWrite(motorA1, baseSpeedLeft - 50);
    analogWrite(motorB1, baseSpeedRight);
  }
  else if (extremeLeft)
  {
    // Hard right turn
    analogWrite(motorA1, 150);
    analogWrite(motorB1, baseSpeedRight - 50);
  }
  else if (extremeRight)
  {
    // Hard left turn
    analogWrite(motorA1, baseSpeedLeft - 50);
    analogWrite(motorB1, 150);
  }
  else
  {
    // Move forward
    analogWrite(motorA1, baseSpeedLeft);
    analogWrite(motorB1, baseSpeedRight);
  }
}