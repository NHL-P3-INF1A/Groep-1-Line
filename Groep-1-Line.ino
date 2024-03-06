#include <Arduino.h>

/*///////////////// define variables /////////////////*/

const int motorA1 = 6;
const int motorA2 = 9;
const int motorB1 = 10;
const int motorB2 = 11;

const int linePins[] = {A0, A1, A2, A3, A4, A5, A6, A7};

const int lineThreshold = 700;

int heavyTurnAdjustment = 255;
int hardTurnAdjustment = 200;

int strongTurnAdjustment = 150;
int veerAdjustment = 25;

// Base speed of the robot
const int baseSpeedLeft = 255;
const int baseSpeedRight = 255;

int delayTime = 1000;

bool endOfLine = false;

/*///////////////// setup /////////////////*/

void setup()
{
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
}
/*///////////////// Function ///////////////*/

void blacksquare()
{
  if ((evenlessFarLeftOnLine + evenlessFarRightOnLine + lessFarLeftOnLine + lessFarRightOnLine + farLeftOnLine + farRightOnLine + extremeLeft + extremeRight) >= 3)
  {
    analogWrite(motorA1, baseSpeedLeft);
    analogWrite(motorB1, baseSpeedRight);
    delay(delayTime);
    if ((evenlessFarLeftOnLine + evenlessFarRightOnLine + lessFarLeftOnLine + lessFarRightOnLine + farLeftOnLine + farRightOnLine + extremeLeft + extremeRight) >= 3)
    {
      endOfLine = true;
    }
  }
}

/*///////////////// follow line loop /////////////////*/

void loop()

// Read sensor values
int sensorValues[8];
for (int i = 0; i < 8; i++)
{
  sensorValues[i] = analogRead(linePins[i]);
}

// Determine if sensors are on the line
  bool extremeLeft = sensorValues[0] > lineThreshold;
  bool farLeftOnLine = sensorValues[1] > lineThreshold;
  bool lessFarLeftOnLine = sensorValues[2] > lineThreshold;
  bool evenlessFarLeftOnLine = sensorValues[3] > lineThreshold;
  bool extremeRight = sensorValues[7] > lineThreshold;
  bool farRightOnLine = sensorValues[6] > lineThreshold;
  bool lessFarRightOnLine = sensorValues[5] > lineThreshold;
  bool evenlessFarRightOnLine = sensorValues[4] > lineThreshold;
{

  blacksquare();
  if (endOfLine)
  {
    analogWrite(motorA1, 0);
    analogWrite(motorB1, 0);
  }
  

  // Determine motor speeds
  if (farLeftOnLine)
  {
    // Hard right turn
    analogWrite(motorA1, baseSpeedLeft - hardTurnAdjustment);
    analogWrite(motorB1, baseSpeedRight);
  }
  else if (lessFarLeftOnLine)
  {
    // Strong right turn
    analogWrite(motorA1, baseSpeedLeft - strongTurnAdjustment);
    analogWrite(motorB1, baseSpeedRight);
  }
  else if (evenlessFarLeftOnLine)
  {
    // Veer left
    analogWrite(motorA1, baseSpeedLeft - veerAdjustment);
    analogWrite(motorB1, baseSpeedRight);
  }
  else if (evenlessFarRightOnLine)
  {
    // Veer right
    analogWrite(motorA1, baseSpeedLeft);
    analogWrite(motorB1, baseSpeedRight - veerAdjustment);
  }
  else if (lessFarRightOnLine)
  {
    // Strong left turn
    analogWrite(motorA1, baseSpeedLeft);
    analogWrite(motorB1, baseSpeedRight - strongTurnAdjustment);
  }
  else if (farRightOnLine)
  {
    // Hard left turn
    analogWrite(motorA1, baseSpeedLeft);
    analogWrite(motorB1, baseSpeedRight - hardTurnAdjustment);
  }
  else
  {
    // Drive straight
    analogWrite(motorA1, baseSpeedLeft);
    analogWrite(motorB1, baseSpeedRight);
  }

  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);
}