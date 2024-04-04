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

int LINETHRESHOLD               = 700; // 0-1023
#define HEAVYTURNADJUSTMENT     255
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
#define   BRIGHTNESS             255  // NeoPixel brightness (0-255)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

#define   LED_LEFT_BACK          0   // Left Back
#define   LED_RIGHT_BACK         1   // Right Back
#define   LED_RIGHT_FRONT        2   // Right Front
#define   LED_LEFT_FRONT         3   // Left Front

// ==== [ Setup ] ==============================================================

void setup() {
  setupLineSensors();
  setupMotorPins();
  setupGripper();
  setupLights();
  stopMotors();
}

// ==== [ Loop ] ===============================================================

void loop() {
  static bool flagDetected = false;
  
  if (!flagDetected) {
    int distance = sonar.ping_cm();
    if (distance > TRIGGER_DISTANCE) {
      gripperOpen();
      LightsStart();
      startProcedure();
      flagDetected = true;
    } else {
      stopMotors();
    }
  } else { 
    determineLineFollowing();
    if (isObstacleDetected()) {
      performObstacleAvoidance();
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
}

void setupGripper() {
  pinMode(GRIPPERPIN, OUTPUT);
}

void turnOnLED(int led, uint8_t red, uint8_t green, uint8_t blue) {
  // Set the color of the specified LED
  strip.setPixelColor(led, red, green, blue);
  strip.show();
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

  if (allBlackCheck()) {
    goStraight();
    delay(300);
    if (allBlackCheck()) {
      LightsEnd();
      stopMotors();
      goBack(300);
      gripperOpen();
      goBack(1000);
      bool i = true;
      while (i == true) {
        stopMotors();
      }
    } 
  }
  else if (farLeftOnLine) {
    goLeft(HARDTURNADJUSTMENT);
  } else if (lessFarLeftOnLine) {
    goLeft(STRONGTURNADJUSTMENT);
  } else if (lessFarRightOnLine) {
    goRight(STRONGTURNADJUSTMENT);
  } else if (farRightOnLine) {
    goRight(HARDTURNADJUSTMENT);
  }

  if (extremeLeft && !sensors3And4Active) {
    goLeft(HEAVYTURNADJUSTMENT);
  } else if (extremeRight && !sensors3And4Active) {
    goRight(HEAVYTURNADJUSTMENT);
  }
}

// ==== [ Obstacle Avoidance ] =================================================

bool isObstacleDetected() {
  static int count = 0;
  static long downDuration;

  if(millis() > downDuration) {
    downDuration = millis() + 2;
    int distance = sonar.ping_cm();
    if (distance > 0 && distance < 10) {
      count++;
    }

    if(count >= 10) {
      count = 0;
      return true;

      }
    else { 
      return false;
      }
  }
}

void performObstacleAvoidance() {
  stopMotors();
  unsigned long startTime = millis();
  while (millis() - startTime < 250) {
    goBack(0);
  }

  startTime = millis();
  while (millis() - startTime < 1000) {
    goLeft(160);
  }

  bool blackDetected = false;

  while (!blackDetected) {
    startTime = millis();
    while (millis() - startTime < 2000){
      if (anyBlackCheck()) {
        blackDetected = true;
        break; 
      }
      goRight(150);
    }

    if (!blackDetected) {
      while (!anyBlackCheck()) {
        goRight(120);
      }
    }
  }
}

// ==== [ Move Functions ] ====================================================

void goStraight() {
  analogWrite(LEFTFORWARD, 240);
  analogWrite(RIGHTFORWARD, BASESPEED);
  digitalWrite(LEFTBACK, 0);
  digitalWrite(RIGHTBACK, 0);
}

void goBack() {
  analogWrite(LEFTFORWARD, 0);
  analogWrite(RIGHTFORWARD, 0);
  digitalWrite(LEFTBACK, BASESPEED);
  digitalWrite(RIGHTBACK, BASESPEED);
}

void goRight(int speedAdjustment) {
  analogWrite(LEFTFORWARD, BASESPEED);
  analogWrite(RIGHTFORWARD, BASESPEED - speedAdjustment);
  analogWrite(RIGHTBACK, 0);
  lightsRight();
}

void goLeft(int speedAdjustment) {
  analogWrite(LEFTFORWARD, BASESPEED - speedAdjustment);
  analogWrite(RIGHTFORWARD, BASESPEED);
  lightsLeft();
}

void stopMotors() {
  analogWrite(LEFTFORWARD, 0);
  analogWrite(RIGHTFORWARD, 0);
  analogWrite(LEFTBACK, 0);
  analogWrite(RIGHTBACK, 0);
}

void moveForward() {
  digitalWrite(LEFTBACK, BASESPEED);
  digitalWrite(RIGHTBACK, BASESPEED);
}


// ==== [ LED Functions ] ======================================================

void lightsLeft() {
  turnOnLED(0, ORANGE[0], ORANGE[1], ORANGE[2]);
  turnOnLED(1, BLUE[0], BLUE[1], BLUE[2]);
  turnOnLED(2, BLUE[0], BLUE[1], BLUE[2]);
  turnOnLED(3, ORANGE[0], ORANGE[1], ORANGE[2]);
}

void lightsRight() {
  turnOnLED(0, BLUE[0], BLUE[1], BLUE[2]);
  turnOnLED(1, ORANGE[0], ORANGE[1], ORANGE[2]);
  turnOnLED(2, ORANGE[0], ORANGE[1], ORANGE[2]);
  turnOnLED(3, BLUE[0], BLUE[1], BLUE[2]);
}

void LightsStart() {
  turnOnLED(0, GREEN[0], GREEN[1], GREEN[2]);
  turnOnLED(1, GREEN[0], GREEN[1], GREEN[2]);
  turnOnLED(2, GREEN[0], GREEN[1], GREEN[2]);
  turnOnLED(3, GREEN[0], GREEN[1], GREEN[2]);
}

void LightsEnd() {
  turnOnLED(0, RED[0], RED[1], RED[2]);
  turnOnLED(1, RED[0], RED[1], RED[2]);
  turnOnLED(2, RED[0], RED[1], RED[2]);
  turnOnLED(3, RED[0], RED[1], RED[2]);
}

// ==== [ Gripper Functions ] =================================================

void gripperOpen() {
  moveGripper(1700);
}

void gripperClose() {
  moveGripper(1050);
}

void moveGripper(int pulseDuration) {
  for (int i = 0; i < 10; i++) {
    delay(20);
    digitalWrite(GRIPPERPIN, HIGH);
    delayMicroseconds(10);
    delayMicroseconds(pulseDuration);
    delayMicroseconds(2);
    digitalWrite(GRIPPERPIN, LOW);
  }
}

// ==== [ Check Functions ] ===================================================

bool allBlackCheck() {
  int sensorValues[8]; 
  readLineSensors(sensorValues);
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] < LINETHRESHOLD) {
      return false;
    }
  }
  return true;
}

bool anyBlackCheck() {
  int sensorValues[8]; 
  readLineSensors(sensorValues);
  for (int i = 0; i < 8; i++) {
    if (sensorValues[i] > LINETHRESHOLD) {
      return true;
    }
  }
  return false;
}

// ==== [ Start Procedure Function ] =================================================

void startProcedure() {
  int sensorValues[8]; 
  readLineSensors(sensorValues);
  
  int blackLineSum = 0;
  int blackLineCount = 0;
  setupLights();
  goStraight();
  
  while(blackLineCount < 4) {
    while (true) {
      if (analogRead(sensorValues[3]) > LINETHRESHOLD) {
        break;
      }
    }
    while (true) {
      
      if (analogRead(sensorValues[3]) < LINETHRESHOLD) {
        break;
      }
    }
      blackLineSum += getAverageLightValue();    
      blackLineCount++; 
  }
    stopMotors();

  LINETHRESHOLD = blackLineSum / blackLineCount;

  delay(10);
  gripperClose();
  goRight(180);
  analogWrite(LEFTBACK, 180);
  analogWrite(RIGHTBACK, 0);
  delay(500);
  while(true) {
    if(analogRead(sensorValues[4]) > LINETHRESHOLD) {
      break;
    }
  }
  stopMotors();
}

int getAverageLightValue() {
  int sensorValues[8]; 
  readLineSensors(sensorValues);

  int sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(sensorValues[i]);
  }
  return sum / 8;
}