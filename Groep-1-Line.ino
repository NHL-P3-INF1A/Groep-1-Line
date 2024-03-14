/*///////////////// define variables /////////////////*/
const int motorA1 = 6;
const int motorA2 = 9;
const int motorB1 = 10;
const int motorB2 = 11;
const int linePins[] = {A0, A1, A2, A3, A4, A5, A6, A7};

#include <Adafruit_NeoPixel.h>
#define   LED_PIN                 13   // Pin that the neopixels are connected to
#define   LED_COUNT               4   // Amount of LEDs
#define   BRIGHTNESS 50  // NeoPixel brightness (0-255)
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

#define   LED_LEFT_BACK           0   // Left Back
#define   LED_RIGHT_BACK          1   // Right Back
#define   LED_RIGHT_FRONT         2   // Right Front
#define   LED_LEFT_FRONT          3   // Left Front

const int gripperPin = 5;

const int lineThreshold = 700;
int heavyTurnAdjustment = 255;
int hardTurnAdjustment = 175;
int strongTurnAdjustment = 125;
int veerAdjustment = 25;

const int RED[]                 = {255, 0, 0};
const int GREEN[]               = {0, 255, 0};
const int BLUE[]                = {0, 0, 255};
const int ORANGE[]              = {255, 80, 0};
const int WHITE[]               = {255, 255, 255};

// Base speed of the robot
const int baseSpeed = 255;
const int startSpeed = 210; 

/*///////////////// setup /////////////////*/

void setup() {
  // Initialize motor control pins as outputs
  gripperOpen();
  turnOnLED(2, BLUE[0], BLUE[1], BLUE[2]);
  turnOnLED(3, BLUE[0], BLUE[1], BLUE[2]);
  
  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(gripperPin, OUTPUT);

  strip.begin();
  strip.setBrightness(BRIGHTNESS);
  strip.show();

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
    turnOnLED(0, 255, 0, 0); 
  } else if (farLeftOnLine) {
    // Hard left turn
    analogWrite(motorA1, baseSpeed - hardTurnAdjustment); 
    analogWrite(motorB1, baseSpeed - 50);
    turnOnLED(0, ORANGE[0], ORANGE[1], ORANGE[2]);
    turnOnLED(1, BLUE[0], BLUE[1], BLUE[2]);
    turnOnLED(2, BLUE[0], BLUE[1], BLUE[2]);
    turnOnLED(3, ORANGE[0], ORANGE[1], ORANGE[2]);
  } else if (lessFarLeftOnLine) {
    // Strong left turn
    analogWrite(motorA1, baseSpeed - strongTurnAdjustment);
    analogWrite(motorB1, baseSpeed - 35);
    turnOnLED(0, ORANGE[0], ORANGE[1], ORANGE[2]);
    turnOnLED(1, BLUE[0], BLUE[1], BLUE[2]);
    turnOnLED(2, BLUE[0], BLUE[1], BLUE[2]);
    turnOnLED(3, ORANGE[0], ORANGE[1], ORANGE[2]);
  } else if (evenlessFarLeftOnLine) {
    // Veer left
    analogWrite(motorA1, baseSpeed);
    analogWrite(motorB1, baseSpeed - veerAdjustment);
  } else if (evenlessFarRightOnLine) {
    // Veer right
    analogWrite(motorA1, baseSpeed - veerAdjustment);
    analogWrite(motorB1, baseSpeed);
  } else if (lessFarRightOnLine) {
    // Strong right turn
    analogWrite(motorA1, baseSpeed - 35);
    analogWrite(motorB1, baseSpeed - strongTurnAdjustment * 2);
    turnOnLED(0, BLUE[0], BLUE[1], BLUE[2]);
    turnOnLED(1, ORANGE[0], ORANGE[1], ORANGE[2]);
    turnOnLED(2, ORANGE[0], ORANGE[1], ORANGE[2]);
    turnOnLED(3, BLUE[0], BLUE[1], BLUE[2]);
  } else if (farRightOnLine) {
    // Hard right turn
    analogWrite(motorA1, baseSpeed - 50);
    analogWrite(motorB1, baseSpeed - hardTurnAdjustment * 2);
    turnOnLED(0, BLUE[0], BLUE[1], BLUE[2]);
    turnOnLED(1, ORANGE[0], ORANGE[1], ORANGE[2]);
    turnOnLED(2, ORANGE[0], ORANGE[1], ORANGE[2]);
    turnOnLED(3, BLUE[0], BLUE[1], BLUE[2]);
  }
  else if (extremeLeft && !sensors3And4Active) {
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
    delay(300);
    if(allCenterSensorsActive){
    analogWrite(motorA1, 0);
    analogWrite(motorB1, 0);
    }
  }
  digitalWrite(motorA2, LOW);
  digitalWrite(motorB2, LOW);

}
