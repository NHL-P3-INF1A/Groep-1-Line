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

// ==== [ LED Pins ] ===========================================================
#include <Adafruit_NeoPixel.h>
#define   LED_PIN                 13   // Pin that the neopixels are connected to
#define   LED_COUNT               4   // Amount of LEDs
#define   BRIGHTNESS 50  // NeoPixel brightness (0-255)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

#define   LED_LEFT_BACK           0   // Left Back
#define   LED_RIGHT_BACK          1   // Right Back
#define   LED_RIGHT_FRONT         2   // Right Front
#define   LED_LEFT_FRONT          3   // Left Front

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

void setupLights(){
  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();
  turnOnLED(2, BLUE[0], BLUE[1], BLUE[2]);
  turnOnLED(3, BLUE[0], BLUE[1], BLUE[2]);
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

void turnOnLED(int led, uint8_t red, uint8_t green, uint8_t blue) {
  // Set the color of the specified LED
  strip.setPixelColor(led, red, green, blue);
  strip.show();
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
    lightRight();
  } else if (lessFarLeftOnLine) {
    strongRightTurn();
    lightRight(); 
  } else if (lessFarRightOnLine) {
    strongLeftTurn();
    lightsLeft();
  } else if (farRightOnLine) {
    hardLeftTurn();
    lightsLeft();
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