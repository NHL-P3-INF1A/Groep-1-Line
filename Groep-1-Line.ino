#include <Arduino.h>

/*///////////////// define variables /////////////////*/

const int motorA1 = 6;
const int motorA2 = 9;
const int motorB1 = 10;
const int motorB2 = 11;

const int linePins[] = {A0, A1, A2, A3, A4, A5, A6, A7};

const int lineThreshold = 700;

int heavyTurnAdjustment = 255;
int hardTurnAdjustment = 175;
int strongTurnAdjustment = 125;
int veerAdjustment = 25;

// Base speed of the robot
const int baseSpeedLeft = 255;
const int baseSpeedRight = 255;
const int startSpeed = 200; 

/*///////////////// functions /////////////////*/

void checkIfOnBlackSquare() {
  // Define the missing variables
  bool evenlessFarLeftOnLine;
  bool evenlessFarRightOnLine;
  bool lessFarLeftOnLine;
  bool lessFarRightOnLine;
  bool farLeftOnLine;
  bool farRightOnLine;
  bool extremeLeft;
  bool extremeRight;
  int baseSpeed = 255;
  int delayTime = 1000;

  if ((evenlessFarLeftOnLine + evenlessFarRightOnLine + lessFarLeftOnLine + lessFarRightOnLine + farLeftOnLine + farRightOnLine + extremeLeft + extremeRight) >= 3)
  {
    analogWrite(motorA1, baseSpeed);
    analogWrite(motorB1, baseSpeed);
    delay(delayTime); // Add missing parentheses and semicolon
    if ((evenlessFarLeftOnLine + evenlessFarRightOnLine + lessFarLeftOnLine + lessFarRightOnLine + farLeftOnLine + farRightOnLine + extremeLeft + extremeRight) >= 3)
    {
      analogWrite(motorA1, 0);
      analogWrite(motorB1, 0);
    }
  } 
}

/*///////////////// setup /////////////////*/

void setup() {
  // Initialize motor control pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Initialize line sensor pins as inputs
  pinMode (A0, INPUT);
  pinMode (A1, INPUT);
  pinMode (A2, INPUT);
  pinMode (A3, INPUT);
  pinMode (A4, INPUT);
  pinMode (A5, INPUT);
  pinMode (A6, INPUT);
  pinMode (A7, INPUT);
}


/*///////////////// follow line loop /////////////////*/

void loop() {
  // Read sensor values
  int sensorValues[8];
  for (int i = 0; i < 8; i++) {
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

  // Check if sensors 3 and 4 are active
  bool sensors3And4Active = sensorValues[3] > lineThreshold && sensorValues[4] > lineThreshold;

  // NEW SECTION: Check for cross-section
  bool allCenterSensorsActive = lessFarLeftOnLine && evenlessFarLeftOnLine && evenlessFarRightOnLine && lessFarRightOnLine;

  if (allCenterSensorsActive && !sensors3And4Active) {
    // Cross-section detected - go straight
    analogWrite(motorA1, baseSpeedLeft);
    analogWrite(motorB1, baseSpeedRight);
  } else if (farLeftOnLine) {
    // Hard right turn
    analogWrite(motorA1, baseSpeedLeft - hardTurnAdjustment); // Even slower left motor
    analogWrite(motorB1, baseSpeedRight - 50);
  } else if (lessFarLeftOnLine) {
    // Strong right turn
    analogWrite(motorA1, baseSpeedLeft - strongTurnAdjustment);
    analogWrite(motorB1, baseSpeedRight - 35);
  } else if (evenlessFarLeftOnLine) {
    // Veer left
    analogWrite(motorA1, baseSpeedLeft);
    analogWrite(motorB1, baseSpeedRight - veerAdjustment);
  } else if (evenlessFarRightOnLine) {
    // Veer right
    analogWrite(motorA1, baseSpeedLeft - veerAdjustment);
    analogWrite(motorB1, baseSpeedRight);
  } else if (lessFarRightOnLine) {
    // Strong left turn
    analogWrite(motorA1, baseSpeedLeft - 35);
    analogWrite(motorB1, baseSpeedRight - strongTurnAdjustment * 2);
  } else if (farRightOnLine) {
    // Hard left turn
    analogWrite(motorA1, baseSpeedLeft - 50);
    analogWrite(motorB1, baseSpeedRight - hardTurnAdjustment * 2);
  }

  if (extremeLeft && !sensors3And4Active) {
    // Lost the line to the extreme left - rotate heavy
    analogWrite(motorA1, baseSpeedLeft - heavyTurnAdjustment);
    analogWrite(motorB1, baseSpeedRight);
  } else if (extremeRight && !sensors3And4Active) {
    // Lost the line to the extreme right - rotate heavy
    analogWrite(motorA1, baseSpeedLeft);
    analogWrite(motorB1, baseSpeedRight - heavyTurnAdjustment);
  }

  checkIfOnBlackSquare();

  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);
}
