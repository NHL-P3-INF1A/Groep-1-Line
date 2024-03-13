// ==== [ Motor Pins ] ========================================================
const int motorA1 = 6; // Motor A1 LF
const int motorA2 = 9; // Motor A2 LB
const int motorB1 = 10; // Motor B1 RF
const int motorB2 = 11; // Motor B2 RB

// ==== [ Line Sensor Pins ] ===================================================
const int linePins[] = {A0, A1, A2, A3, A4, A5, A6, A7}; // 8 Line sensors

// ==== [ Gripper Pin ] ========================================================
const int gripperPin = 5; // Gripper servo

// ==== [ Line Sensor Threshold ] ==============================================
const int lineThreshold = 700; // 0-1023
int heavyTurnAdjustment = 255;
int hardTurnAdjustment = 175;
int strongTurnAdjustment = 125;
int veerAdjustment = 25; //

// ==== [ Motor Speeds ] =======================================================
const int baseSpeed = 255;
const int startSpeed = 210; 

/*///////////////// setup /////////////////*/

void setup() {
  pinMode(linePins[0], INPUT);
  pinMode(linePins[1], INPUT);
  pinMode(linePins[2], INPUT);
  pinMode(linePins[3], INPUT);
  pinMode(linePins[4], INPUT);
  pinMode(linePins[5], INPUT);
  pinMode(linePins[6], INPUT);
  pinMode(linePins[7], INPUT);

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);

  pinMode(gripperPin, OUTPUT);

  // Initialize motor control pins as outputs
  gripperOpen();
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

void gripperOpen()
{moveGripper(1700);}

void gripperClose()
{moveGripper(1050);}

void moveGripper(int pulseDuration)
{
  for(int i = 0; i < 10; i++)
  {
    delay(20);
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(pulseDuration);
    digitalWrite(gripperPin, LOW);
  }
}
/*///////////////// follow line loop /////////////////*/
void loop() {
  //Read sensor values
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
  else if(allCenterSensorsActive){
    digitalWrite(motorA2, baseSpeed);
    digitalWrite(motorB2, baseSpeed);
    delay(200);
    if(allCenterSensorsActive){
    analogWrite(motorA1, 0);
    analogWrite(motorB1, 0);}
  }
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);

}
