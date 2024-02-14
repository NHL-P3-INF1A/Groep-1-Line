// Define motor control pins
const int motorA1 = 6; 
const int motorA2 = 9; 
const int motorB1 = 10;
const int motorB2 = 11;

// Define analog input pins for line detection
const int linePins[] = {A0, A1, A2, A3, A4, A5}; 

// Threshold to determine if a sensor is on or off the line
const int lineThreshold = 700; // ADJUST this based on your actual sensor readings

// Base speed of the robot
const int baseSpeed = 240;  // ADJUST for your robot

void setup() {
  // Initialize motor control pins as outputs
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
}

void loop() {
  // Read sensor values
  int sensorValues[6];
  for (int i = 0; i < 6; i++) {
    sensorValues[i] = analogRead(linePins[i]); 
  }

  // Determine if sensors are on the line
  bool farLeftOnLine = sensorValues[0] > lineThreshold; 
  bool lessFarLeftOnLine = sensorValues[1] > lineThreshold;
  bool evenlessFarLeftOnLine = sensorValues[2] > lineThreshold;
  bool farRightOnLine = sensorValues[5] > lineThreshold;
  bool lessFarRightOnLine = sensorValues[4] > lineThreshold;
  bool evenlessFarRightOnLine = sensorValues[3] > lineThreshold;

  // Graduated turning logic 
  if (farLeftOnLine) {
    // Hard right turn   
    analogWrite(motorA1, baseSpeed - 200);
    analogWrite(motorB1, baseSpeed - 100); 
  } 
   else if (farRightOnLine) {
    // Hard left turn
    analogWrite(motorB1, baseSpeed - 200);  
    analogWrite(motorA1, baseSpeed - 100); 
  } 
  else if (lessFarLeftOnLine) {
    // Strong right turn
    analogWrite(motorA1, baseSpeed - 125); 
    analogWrite(motorB1, baseSpeed - 50);
  } 
  else if (lessFarRightOnLine) {
    // Strong left turn    
    analogWrite(motorB1, baseSpeed - 125);
    analogWrite(motorA1, baseSpeed - 50); 
  }
  else if (evenlessFarLeftOnLine) {
    // Veer right 
    analogWrite(motorA1, baseSpeed - 25); 
    analogWrite(motorB1, baseSpeed); 
  } 
  else if (evenlessFarRightOnLine) { 
    // Veer left
    analogWrite(motorB1, baseSpeed - 25);  
    analogWrite(motorA1, baseSpeed);
  } 
  
  else {
    // Lost the line - try rotating a bit
    analogWrite(motorA1, 150);  // Stop left
    analogWrite(motorB1, 01);  // Turn slowly right
  }

  // Move the robot forward (or adjust for turning state above)
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW); 
}

