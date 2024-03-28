#include <NewPing.h>
#include <Adafruit_NeoPixel.h>

// ==== [ Motor Pins ] ========================================================
#define LEFTFORWARD             6   // Motor A1 LF
#define LEFTBACK                9   // Motor A2 LB
#define RIGHTFORWARD            10  // Motor B1 RF
#define RIGHTBACK               11  // Motor B2 RB

// ==== [ Lights ] =============================================================
const int RED[]                 = {255, 0, 0};
const int GREEN[]               = {0, 255, 0};
const int BLUE[]                = {0, 0, 255};
const int ORANGE[]              = {255, 80, 0};
const int WHITE[]               = {255, 255, 255};

// ==== [ Line Sensor Pins ] ===================================================
const int linePins[]            = {A0, A1, A2, A3, A4, A5, A6, A7}; // 8 Line sensors

// ==== [ Gripper Pin ] ========================================================
#define GRIPPERPIN              5 // Gripper servo

// ==== [ Line Sensor Threshold ] ==============================================

#define LINETHRESHOLD           700 // 0-1023
#define HEAVYTURNADJUSTMENT     200
#define HARDTURNADJUSTMENT      150
#define STRONGTURNADJUSTMENT    75
#define VEERADJUSTMENT          25

// ==== [ Motor Speeds ] =======================================================

#define BASESPEED               255 // The base speed of the motors
#define STARTSPEED              210 // The speed of the motors when starting

// ==== [ Echo Sensor Pins ] ===================================================

#define TRIG_PIN                8 // 
#define ECHO_PIN                3 // 

#define MAX_DISTANCE            40 // Maximum distance in cm we want to check for the flag
#define TRIGGER_DISTANCE        10 // Distance in cm to trigger obstacle avoidance

NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE); 

// ==== [ LED Pins ] ===========================================================

#define   LED_PIN                13   // Pin that the neopixels are connected to
#define   LED_COUNT              4   // Amount of LEDs
#define   BRIGHTNESS             50  // NeoPixel brightness (0-255)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

#define   LED_LEFT_BACK          0   // Left Back
#define   LED_RIGHT_BACK         1   // Right Back
#define   LED_RIGHT_FRONT        2   // Right Front
#define   LED_LEFT_FRONT         3   // Left Front

// ==== [ Setup ] ==============================================================

void setup() {
  Serial.begin(115200);
  setupLineSensors();
  setupMotorPins();
  setupLights();
}

// ==== [ Loop ] ===============================================================

void loop() {
  static bool flagDetected = false;
  static bool obstacleAvoided = false;
  
  if (!flagDetected) {
    int distance = sonar.ping_cm();
    Serial.print("Distance: ");
    Serial.println(distance);
    if (distance > TRIGGER_DISTANCE) {
      Serial.println("Flag detected. Starting robot logic.");
      gripperOpen();
      initializeMotors();
      flagDetected = true;
    } else {
      Serial.println("Waiting before re-checking distance.");
      delay(150); 
    }
  } else { 
    determineLineFollowing();
    if (isObstacleDetected() && obstacleAvoided == false) {
      performObstacleAvoidance();
      obstacleAvoided = true;
    }
  }
}

// ==== [ Setup Functions ] ====================================================

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
  pinMode(LEFTFORWARD, OUTPUT);
  pinMode(LEFTBACK, OUTPUT);
  pinMode(RIGHTFORWARD, OUTPUT);
  pinMode(RIGHTBACK, OUTPUT);
  pinMode(GRIPPERPIN, OUTPUT);
}

void turnOnLED(int led, uint8_t red, uint8_t green, uint8_t blue) {
  // Set the color of the specified LED
  strip.setPixelColor(led, red, green, blue);
  strip.show();
}

void initializeMotors() {
  analogWrite(LEFTFORWARD, STARTSPEED);
  analogWrite(RIGHTFORWARD, STARTSPEED);
  digitalWrite(LEFTBACK, LOW);
  digitalWrite(RIGHTBACK, LOW);
  delay(1650);
  analogWrite(LEFTFORWARD, 0);
  analogWrite(RIGHTFORWARD, 0);
  gripperClose();
  delay(500);
  analogWrite(LEFTBACK, STARTSPEED);
  analogWrite(RIGHTFORWARD, STARTSPEED);
  delay(500);
  analogWrite(LEFTBACK, 0);
  analogWrite(RIGHTBACK, 0);
  delay(1000);
  Serial.println("Motors initialized.");
}

// ==== [ Line Following functions ] =====================================================

void readLineSensors(int sensorValues[]) {
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = analogRead(linePins[i]);
  }
}

void determineLineFollowing() {
  int sensorValues[8]; 
  readLineSensors(sensorValues);

  bool extremeLeft = sensorValues[0] > LINETHRESHOLD;
  bool farLeftOnLine = sensorValues[1] > LINETHRESHOLD;
  bool lessFarLeftOnLine = sensorValues[2] > LINETHRESHOLD;
  bool evenlessFarLeftOnLine = sensorValues[3] > LINETHRESHOLD;
  bool extremeRight = sensorValues[7] > LINETHRESHOLD;
  bool farRightOnLine = sensorValues[6] > LINETHRESHOLD;
  bool lessFarRightOnLine = sensorValues[5] > LINETHRESHOLD;
  bool evenlessFarRightOnLine = sensorValues[4] > LINETHRESHOLD;
  bool sensors3And4Active = sensorValues[3] > LINETHRESHOLD && sensorValues[4] > LINETHRESHOLD;
  bool allCenterSensorsActive = lessFarLeftOnLine && evenlessFarLeftOnLine && evenlessFarRightOnLine && lessFarRightOnLine;

if (blackCheck()) {
  goStraight(10);
  delay(300);
  if (blackCheck()) {
    stopMotors();
    goBack(200);
    gripperOpen();
    goBack(1000);
    bool i = true;
    while (i == true) {
      stopMotors();
    }
  } 
}
  else if (farLeftOnLine) {
    goRight(HARDTURNADJUSTMENT);
  } else if (lessFarLeftOnLine) {
    goRight(STRONGTURNADJUSTMENT);
  } else if (lessFarRightOnLine) {
    goLeft(STRONGTURNADJUSTMENT);
  } else if (farRightOnLine) {
    goLeft(HARDTURNADJUSTMENT);
  }

  if (extremeLeft && !sensors3And4Active) {
    goRight(HEAVYTURNADJUSTMENT);
  } else if (extremeRight && !sensors3And4Active) {
    goLeft(HEAVYTURNADJUSTMENT);
  }
}

bool blackCheck() {
  Serial.println("Checking if all sensors are black."); 
  int sensorValues[8]; 
  readLineSensors(sensorValues);
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] < LINETHRESHOLD) {
      return false;
    }
  }
  return true;
}

// ==== [ Move Functions ] ====================================================

void goStraight() {
  analogWrite(LEFTFORWARD, BASESPEED);
  analogWrite(RIGHTFORWARD, BASESPEED);
  digitalWrite(LEFTBACK, 0);
  digitalWrite(RIGHTBACK, 0);
  Serial.println("Going straight.");
}

void goBack() {
  analogWrite(LEFTFORWARD, 0);
  analogWrite(RIGHTFORWARD, 0);
  digitalWrite(LEFTBACK, BASESPEED);
  digitalWrite(RIGHTBACK, BASESPEED);
  Serial.println("Going back.");
}

void goLeft(int speedAdjustment) {
  analogWrite(LEFTFORWARD, BASESPEED);
  analogWrite(RIGHTFORWARD, BASESPEED - speedAdjustment);
  lightsLeft();
  Serial.println("Going left.");
}

void goRight(int speedAdjustment) {
  analogWrite(LEFTFORWARD, BASESPEED - speedAdjustment);
  analogWrite(RIGHTFORWARD, BASESPEED);
  Serial.println("Going right.");
}

void stopMotors() {
  analogWrite(LEFTFORWARD, 0);
  analogWrite(RIGHTFORWARD, 0);
  analogWrite(LEFTBACK, 0);
  analogWrite(RIGHTBACK, 0);
  Serial.println("Stopping motors.");
}

void moveForward() {
  digitalWrite(LEFTBACK, BASESPEED);
  digitalWrite(RIGHTBACK, BASESPEED);
}

// ==== [ Obstacle Avoidance ] =================================================

bool isObstacleDetected() {
  int distance = sonar.ping_cm();
  if (distance != 0 && distance < 40) {
    return true;
  } else {
    return false;
  }
}

void performObstacleAvoidance() {
  stopMotors();
  unsigned long startTime = millis();
  goBack();
  while (millis() - startTime < 250) {
  }

  startTime = millis();
  while (millis() - startTime < 1000) {
    analogWrite(motorA2, 160);
    analogWrite(motorB1, 255); 
    analogWrite(motorB2, 0);
    analogWrite(motorA1, 0);
  }

  bool blackDetected = false;

  while (!blackDetected) {
    startTime = millis();
    while (millis() - startTime < 500){
      readSensors();
      if (anyBlack()) {
        blackDetected = true;
        break; 
      }
      analogWrite(motorA2, 255); 
      analogWrite(motorB1, 150); 
      analogWrite(motorB2, 0);
      analogWrite(motorA1, 0);
    }

    if (!blackDetected) {
      while (!anyBlack()) {
        analogWrite(motorA2, 255); 
        analogWrite(motorB1, 120); 
        analogWrite(motorB2, 0);
        analogWrite(motorA1, 0);
        readSensors();
      }
    }
  }

  stopDriving();
  lastSensor = 1;
}

bool anyBlack() {
  for (int i = 0; i < 8; i++) {
    if (lineSensorValue[i] >= colorBlack) {
      return true;
    }
  }
  return false;
}

// ==== [ LED Functions ] ======================================================

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

// ==== [ Gripper Functions ] =================================================

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
    digitalWrite(GRIPPERPIN, HIGH);
    delayMicroseconds(pulseDuration);
    digitalWrite(GRIPPERPIN, LOW);
  }
}

void goAroundObject() 
{
}