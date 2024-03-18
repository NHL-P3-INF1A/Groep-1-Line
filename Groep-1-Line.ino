#include <NewPing.h>
#include <Adafruit_NeoPixel.h>

// ==== [ Motor Pins ] ========================================================
#define motorA1                 6   // Motor A1 LF
#define motorA2                 9   // Motor A2 LB
#define motorB1                 10  // Motor B1 RF
#define motorB2                 11  // Motor B2 RB

// ==== [ Lights ] =============================================================
const int RED[]                 = {255, 0, 0};
const int GREEN[]               = {0, 255, 0};
const int BLUE[]                = {0, 0, 255};
const int ORANGE[]              = {255, 80, 0};
const int WHITE[]               = {255, 255, 255};

// ==== [ Line Sensor Pins ] ===================================================
const int linePins[] = {A0, A1, A2, A3, A4, A5, A6, A7}; // 8 Line sensors

// ==== [ Gripper Pin ] ========================================================
#define gripperPin              5 // Gripper servo

// ==== [ Line Sensor Threshold ] ==============================================
#define lineThreshold           700 // 0-1023
#define heavyTurnAdjustment     255
#define hardTurnAdjustment      175
#define strongTurnAdjustment    125
#define veerAdjustment          25

// ==== [ Motor Speeds ] =======================================================
#define baseSpeed               255 // The base speed of the motors
#define startSpeed              210 // The speed of the motors when starting

// ==== [ Echo Sensor Pins ] ===================================================
#define TRIG_PIN 8
#define ECHO_PIN 3

#define MAX_DISTANCE 40 // Maximum distance in cm we want to check for the flag
#define TRIGGER_DISTANCE 10 // Distance in cm to trigger obstacle avoidance

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 

// ==== [ LED Pins ] ===========================================================
#define   LED_PIN                 13   // Pin that the neopixels are connected to
#define   LED_COUNT               4   // Amount of LEDs
#define   BRIGHTNESS 50  // NeoPixel brightness (0-255)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

#define   LED_LEFT_BACK           0   // Left Back
#define   LED_RIGHT_BACK          1   // Right Back
#define   LED_RIGHT_FRONT         2   // Right Front
#define   LED_LEFT_FRONT          3   // Left Front

// ==== [ Setup ] ==============================================================
void setup() {
  Serial.begin(115200);
  setupLineSensors();
  setupMotorPins();
  setupLights();
}

void setupLights(){
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();
  turnOnLED(2, BLUE[0], BLUE[1], BLUE[2]);
  turnOnLED(3, BLUE[0], BLUE[1], BLUE[2]);
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
  Serial.println("Opening gripper.");
}

void gripperClose() {
  moveGripper(1050);
  Serial.println("Closing gripper.");
}

void moveGripper(int pulseDuration) {
  for (int i = 0; i < 10; i++) {
    delay(20);
    digitalWrite(gripperPin, HIGH);
    delayMicroseconds(pulseDuration);
    digitalWrite(gripperPin, LOW);
  }
}

void turnOnLED(int led, uint8_t red, uint8_t green, uint8_t blue) {
  // Set the color of the specified LED
  strip.setPixelColor(led, red, green, blue);
  strip.show();
}

void initializeMotors() {
  analogWrite(motorA1, startSpeed);
  analogWrite(motorB1, startSpeed);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);
  delay(1650);
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
  Serial.println("Motors initialized.");
}

// ==== [ Loop ] ===============================================================
void loop() {
  static bool flagDetected = false; // Initialize flag detection variable
  
  if (!flagDetected) {
    int distance = sonar.ping_cm();
    Serial.print("Distance: ");
    Serial.println(distance);
    if (distance > TRIGGER_DISTANCE) {
      Serial.println("Flag detected. Starting robot logic.");
      gripperOpen();
      initializeMotors();
      flagDetected = true; // Set flag to true to indicate detection
    } else {
      Serial.println("Waiting before re-checking distance.");
      delay(150); 
    }
  } else {
    int sensorValues[8]; 
    readLineSensors(sensorValues); 
    determineLineFollowing(sensorValues);
    if (isObstacleDetected()) {
      performObstacleAvoidance();
    }
  }
}


void readLineSensors(int sensorValues[]) {
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(linePins[i]);
  }
}

void determineLineFollowing(int sensorValues[]) {
  bool extremeLeft = sensorValues[0] > lineThreshold;
  Serial.println("Extreme left:");
  Serial.println(sensorValues[0]);
  bool farLeftOnLine = sensorValues[1] > lineThreshold;
  Serial.println("Far left:");
  Serial.println(sensorValues[1]);
  bool lessFarLeftOnLine = sensorValues[2] > lineThreshold;
  Serial.println("Less far left:");
  Serial.println(sensorValues[2]);
  bool evenlessFarLeftOnLine = sensorValues[3] > lineThreshold;
  Serial.println("Even less far left:");
  Serial.println(sensorValues[3]);
  bool extremeRight = sensorValues[7] > lineThreshold;
  Serial.println("Extreme right:");
  Serial.println(sensorValues[7]);
  bool farRightOnLine = sensorValues[6] > lineThreshold;
  Serial.println("Far right:");
  Serial.println(sensorValues[6]);
  bool lessFarRightOnLine = sensorValues[5] > lineThreshold;
  Serial.println("Less far right:");
  Serial.println(sensorValues[5]);  
  bool evenlessFarRightOnLine = sensorValues[4] > lineThreshold;
  Serial.println("Even less far right:");
  Serial.println(sensorValues[4]);
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
    lightsLeft();
  } else if (extremeRight && !sensors3And4Active) {
    rotateHeavyRight();
    lightRight();
  } else if (allCenterSensorsActive) {
    moveForward();
  }
}

void goStraight() {
  analogWrite(motorA1, baseSpeed);
  analogWrite(motorB1, baseSpeed);
  Serial.println("Going straight.");
}

void hardRightTurn() {
  analogWrite(motorA1, baseSpeed - hardTurnAdjustment);
  analogWrite(motorB1, baseSpeed);
  lightRight();
  Serial.println("Performing hard right turn.");
}

void strongRightTurn() {
  analogWrite(motorA1, baseSpeed - strongTurnAdjustment);
  analogWrite(motorB1, baseSpeed);
  lightRight();
  Serial.println("Performing strong right turn.");
}

void strongLeftTurn() {
  analogWrite(motorA1, baseSpeed);
  analogWrite(motorB1, baseSpeed - strongTurnAdjustment);
  lightsLeft();
  Serial.println("Performing strong left turn.");
}

void hardLeftTurn() {
  analogWrite(motorA1, baseSpeed);
  analogWrite(motorB1, baseSpeed - hardTurnAdjustment);
  lightsLeft();
  Serial.println("Performing hard left turn.");
}

void rotateHeavyLeft() {
  analogWrite(motorA1, baseSpeed - heavyTurnAdjustment);
  analogWrite(motorB1, baseSpeed);
  lightsLeft();
  Serial.println("Performing heavy left turn.");
}

void rotateHeavyRight() {
  analogWrite(motorA1, baseSpeed);
  analogWrite(motorB1, baseSpeed - heavyTurnAdjustment);
  lightRight();
  Serial.println("Performing heavy right turn.");
}

void stopMotors() {
  analogWrite(motorA1, 0);
  analogWrite(motorB1, 0);
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);
  Serial.println("Stopping motors.");
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

bool isObstacleDetected() {
  int distance = sonar.ping_cm();
  if (distance != 0 && distance < 20) {
    return true;
  } else {
    return false;
  }
}

void performObstacleAvoidance() {
  // Simple example - turn right for a bit, then resume
  strongRightTurn();
  Serial.println("Performing obstacle avoidance: turning right.");
  delay(2500); // Adjust the time as needed
  goStraight();
  Serial.println("Resuming straight movement after obstacle avoidance.");
  delay(500); 
}

void lightsLeft() {
  turnOnLED(0, ORANGE[0], ORANGE[1], ORANGE[2]);
  turnOnLED(1, BLUE[0], BLUE[1], BLUE[2]);
  turnOnLED(2, BLUE[0], BLUE[1], BLUE[2]);
  turnOnLED(3, ORANGE[0], ORANGE[1], ORANGE[2]);
}

void lightRight() {
  turnOnLED(0, BLUE[0], BLUE[1], BLUE[2]);
  turnOnLED(1, ORANGE[0], ORANGE[1], ORANGE[2]);
  turnOnLED(2, ORANGE[0], ORANGE[1], ORANGE[2]);
  turnOnLED(3, BLUE[0], BLUE[1], BLUE[2]);
}