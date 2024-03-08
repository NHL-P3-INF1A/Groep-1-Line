#include <Arduino.h>

/*///////////////// define variables /////////////////*/

const int motorA1 = 6;
const int motorA2 = 9;
const int motorB1 = 10;
const int motorB2 = 11;

const int linePins[] = {A0, A1, A2, A3, A4, A5, A6, A7};
const int SENSOR_COUNT = 8;

const int lineThreshold = 700;
const int gripperPin = 5;

const int MAX_TURN_ADJUSTMENT = 255;
const int HARD_TURN_ADJUSTMENT = 200;
const int STRONG_TURN_ADJUSTMENT = 150;
const int VEER_ADJUSTMENT = 25;
const int baseSpeedLeft = 240;
const int baseSpeedRight = 255;
const int startSpeed = 200;

int maxValues[SENSOR_COUNT] = {0};
int minValues[SENSOR_COUNT] = {1023};
int calibrationThreshold[SENSOR_COUNT];

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

void startSetup() {

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

void hardRightTurn() {
  analogWrite(motorA1, baseSpeedLeft - MAX_TURN_ADJUSTMENT); 
  analogWrite(motorB1, baseSpeedRight); 
}

void strongRightTurn() {
  analogWrite(motorA1, baseSpeedLeft - STRONG_TURN_ADJUSTMENT);
  analogWrite(motorB1, baseSpeedRight - STRONG_TURN_ADJUSTMENT / 2); 
}

void veerRight() {
  analogWrite(motorA1, baseSpeedLeft - VEER_ADJUSTMENT);
  analogWrite(motorB1, baseSpeedRight);
}

void hardLeftTurn() {
  analogWrite(motorA1, baseSpeedLeft); 
  analogWrite(motorB1, baseSpeedRight - MAX_TURN_ADJUSTMENT); 
}

void strongLeftTurn() {
  analogWrite(motorA1, baseSpeedLeft - STRONG_TURN_ADJUSTMENT);
  analogWrite(motorB1, baseSpeedRight - STRONG_TURN_ADJUSTMENT / 2); 
}

void veerLeft() {
  analogWrite(motorA1, baseSpeedLeft);
  analogWrite(motorB1, baseSpeedRight - VEER_ADJUSTMENT);
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
  for (int i = 0; i < SENSOR_COUNT; i++) {
    pinMode(linePins[i], INPUT);
  }

  calibrateSensors();
}

/*///////////////// follow line loop /////////////////*/

void loop()
{
  int sensorReadings[SENSOR_COUNT]; 

  // Read sensor values into array
  for (int i = 0; i < SENSOR_COUNT; i++) {
    sensorReadings[i] = analogRead(linePins[i]);
  }

  // Line Following Logic
  if (sensorReadings[0] > lineThreshold) {
    hardRightTurn();  // Turn hard right
  } else if (sensorReadings[7] > lineThreshold) {
    hardLeftTurn();
  } else if (sensorReadings[1] > lineThreshold) { 
    hardRightTurn();
  } else if (sensorReadings[2] > lineThreshold) {
    strongRightTurn();
  } else if (sensorReadings[3] > lineThreshold) { 
    veerRight(); 
  } else if (sensorReadings[4] > lineThreshold) {  
    veerLeft(); 
  } else if (sensorReadings[5] > lineThreshold) { 
    strongLeftTurn(); 
  } else if (sensorReadings[6] > lineThreshold) { 
    hardLeftTurn(); 
  } else {
    analogWrite(motorA1, baseSpeedLeft); 
    analogWrite(motorB1, baseSpeedRight); 
  }
}