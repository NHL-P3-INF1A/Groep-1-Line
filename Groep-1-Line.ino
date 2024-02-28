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
const int baseSpeed = 255;
const int startSpeed = 200; 

/*///////////////// setup /////////////////*/

void setup() {
  // Initialize motor control pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  
  //Auto rijdt door voor 0.7 seconden en daarna vuurt die weer de normale code uit.
  analogWrite(motorA1, startSpeed);
  analogWrite(motorB1, startSpeed);
  
  delay(700);
  analogWrite(motorA1, LOW);
  analogWrite(motorB1, LOW);
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
    analogWrite(motorA1, baseSpeed);
    analogWrite(motorB1, baseSpeed);
  } else if (farLeftOnLine) {
    // Hard right turn
    analogWrite(motorA1, baseSpeed - hardTurnAdjustment); // Even slower left motor
    analogWrite(motorB1, baseSpeed - 50);
  } else if (lessFarLeftOnLine) {
    // Strong right turn
    analogWrite(motorA1, baseSpeed - strongTurnAdjustment);
    analogWrite(motorB1, baseSpeed - 35);
  } else if (evenlessFarLeftOnLine) {
    // Veer left
    analogWrite(motorA1, baseSpeed);
    analogWrite(motorB1, baseSpeed - veerAdjustment);
  } else if (evenlessFarRightOnLine) {
    // Veer right
    analogWrite(motorA1, baseSpeed - veerAdjustment);
    analogWrite(motorB1, baseSpeed);
  } else if (lessFarRightOnLine) {
    // Strong left turn
    analogWrite(motorA1, baseSpeed - 35);
    analogWrite(motorB1, baseSpeed - strongTurnAdjustment * 2);
  } else if (farRightOnLine) {
    // Hard left turn
    analogWrite(motorA1, baseSpeed - 50);
    analogWrite(motorB1, baseSpeed - hardTurnAdjustment * 2);
  }

  if (extremeLeft && !sensors3And4Active) {
    // Lost the line to the extreme left - rotate heavy
    analogWrite(motorA1, baseSpeed - heavyTurnAdjustment);
    analogWrite(motorB1, baseSpeed);
  } else if (extremeRight && !sensors3And4Active) {
    // Lost the line to the extreme right - rotate heavy
    analogWrite(motorA1, baseSpeed);
    analogWrite(motorB1, baseSpeed - heavyTurnAdjustment);
  }

  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);
}
