// Define motor control pins
const int motorA1 = 6;
const int motorA2 = 9;
const int motorB1 = 10;
const int motorB2 = 11;

// Define analog input pins for line detection
const int linePins[] = {A0, A1, A2, A3, A4, A5};

void setup() {
  // Initialize motor control pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  // Initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Read sensor values for line detection
  int sensorValues[6];
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(linePins[i]);
  }

  // Calculate the difference between left and right sensors
  int leftSensorValue = sensorValues[0] + sensorValues[1];
  int rightSensorValue = sensorValues[4] + sensorValues[5];

  // Read sensor values for center sensors (A2 and A3)
  int sensorValue1 = analogRead(linePins[2]); // Assuming A2 is the first center sensor
  int sensorValue2 = analogRead(linePins[3]); // Assuming A3 is the second center sensor

  // Calculate the center value
  int centerSensorValue = (sensorValue1 + sensorValue2) / 2;

  // Adjust motor speeds based on sensor readings
  int motorSpeedA = 200;
  int motorSpeedB = 200;

  // Calculate the deviation from the center sensors
  int deviation = centerSensorValue - 200; // Assuming centerSensorValue ~ 512 when on the line

  // Adjust motor speeds based on deviation
  motorSpeedA += deviation / 2; // Reduce the effect of deviation on one motor
  motorSpeedB -= deviation / 2; // Increase the effect of deviation on the other motor

  // Ensure motor speeds are within the valid range (0-255)
  motorSpeedA = constrain(motorSpeedA, 0, 255);
  motorSpeedB = constrain(motorSpeedB, 0, 255);

  // Move the robot forward or backward based on motor speeds
  if (motorSpeedA >= 0) {
    analogWrite(motorA1, motorSpeedA);
    digitalWrite(motorA2, LOW);
  } else {
    analogWrite(motorA1, LOW);
    digitalWrite(motorA2, -motorSpeedA);
  }

  if (motorSpeedB >= 0) {
    analogWrite(motorB1, motorSpeedB);
    digitalWrite(motorB2, LOW);
  } else {
    analogWrite(motorB1, LOW);
    digitalWrite(motorB2, -motorSpeedB);
  }

  // Add a short delay to avoid reading the sensors too frequently
  delay(25);
}
