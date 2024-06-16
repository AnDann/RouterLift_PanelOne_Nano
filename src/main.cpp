#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <LiquidCrystalFast.h>
#include <Axis.h>
#include <Bounce2.h>

// Pins used
// Encoder
#define LE_ENCA  2 // D2 encoder pins
#define LE_ENCB  3 // D3
#define BUTTON_PIN  A0 // A2 Encoder click pin

// Stepper driver
#define STEP_PIN 12
#define DIR_PIN 11
#define ENABLE_PIN 10

// Endstops and Probe
#define ENDSTOP_MIN_PIN A2
#define ENDSTOP_MAX_PIN A3
#define PROBE_PIN A4

// LCD Pins
#define LCD_RS  4
#define LCD_EN  5
#define LCD_D4  6
#define LCD_D5  7
#define LCD_D6  8
#define LCD_D7  9

// Encoder steps per click
#define ENC_STEPS 4
#define DISPLAY_REFRESH_INTERVAL_MS 200
#define ENCODER_READ_INTERVAL_MS 50

// ***************************************************************************************************************
//                  Program start
// ***************************************************************************************************************

LiquidCrystalFast lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
Encoder encoder(LE_ENCA, LE_ENCB);
Bounce buttonOk = Bounce();
Axis lift(STEP_PIN, DIR_PIN, ENABLE_PIN, 200, 8, 8, 0.0, 119.0, ENDSTOP_MIN_PIN, ENDSTOP_MAX_PIN, PROBE_PIN);

// Global Variables
bool buttonPressed = false;
long _lastEncoderPosition = 0;
unsigned long _lastEncoderRead = 0, _lastDisplayUpdate = 0;

// LCD Texts
const char axisStateText[][14] PROGMEM = {"None", "Go to Target", "Go to Home", "Go to Probe", "In Position", "Max!", "Min!"};
const char homingStateText[][10] PROGMEM = {"None", "Backoff 1", "Move Fast", "Backoff 2", "Move slow", "Homed", "Error"};
const char probingStateText[][10] PROGMEM = {"None", "Backoff 1", "Move Fast", "Backoff 2", "Move slow", "Probed", "Error"};
const char menuOptions[][13] PROGMEM = {"Probing", "Homing", "Main Screen"};

enum State {
  MAIN_SCREEN,
  MENU_SCREEN,
  PROBING,
  HOMING
};

State currentState = MAIN_SCREEN;
int currentMenuIndex = 0;

// Function prototypes
int readEncoder(bool accelerated);
void displayMenu();

void setup(void)
{
  buttonOk.attach(BUTTON_PIN, INPUT_PULLUP);
  buttonOk.interval(5); // interval in ms

  lcd.begin(20, 4);
  lcd.setCursor(0, 0);
  lcd.print("RouterLift V0.01");
  lcd.setCursor(0, 1);
  lcd.print("Starting up...");
  lcd.setCursor(0, 2);

  Serial.begin(115200);
  _lastEncoderPosition = encoder.read() / ENC_STEPS;
  _lastEncoderRead = millis();
  _lastDisplayUpdate = millis();
}

void loop(void)
{
  lift.handle();
  buttonOk.update();

  switch (currentState) {
    case MAIN_SCREEN:
      if ((lift.inPosition() || lift.isError()) && (millis() - _lastDisplayUpdate > DISPLAY_REFRESH_INTERVAL_MS)) {
        lcd.setCursor(0, 0);
        lcd.print(F("Status:             "));
        lcd.setCursor(7, 0);
        lcd.print(axisStateText[lift.getState()]);

        lcd.setCursor(0, 1);
        lcd.print("         :          ");
        lcd.setCursor(0, 1);
        lcd.print(homingStateText[lift.getHomingState()]);
        lcd.setCursor(10, 1);
        lcd.print(probingStateText[lift.getProbingState()]);

        lcd.setCursor(0, 2);
        lcd.print(F("Soll:               "));
        lcd.setCursor(5, 2);
        lcd.print(lift.getTargetPosition());
        lcd.print(F("mm"));

        lcd.setCursor(0, 3);
        lcd.print(F("Ist:                "));
        lcd.setCursor(5, 3);
        lcd.print(lift.getCurrentPosition());
        lcd.print(F("mm "));
        lcd.print(lift.getWorkoffset());
        lcd.print(F("mm"));
        _lastDisplayUpdate = millis();
      }

      if (lift.isHomed()) {
        lift.setTargetPosition(lift.getTargetPosition() + (readEncoder(true) * 0.01));
      }

      if (buttonOk.rose() && buttonOk.previousDuration() < 1000) {
        lift.moveToTarget();
      } else if (buttonOk.read() == LOW && buttonOk.currentDuration() > 1000) {
        currentState = MENU_SCREEN;
        currentMenuIndex = 0;
        displayMenu();
      }
      break;

    case MENU_SCREEN:
    {
      int encoderMove = readEncoder(false);
      if (encoderMove != 0) {
        currentMenuIndex = (currentMenuIndex + encoderMove) % 3;
        if (currentMenuIndex < 0) currentMenuIndex += 3;
        displayMenu();
      }

      if (buttonOk.rose()) {
        if (currentMenuIndex == 0) {
          currentState = PROBING;
        } else if (currentMenuIndex == 1) {
          currentState = HOMING;
        } else {
          currentState = MAIN_SCREEN;
        }
      }
      break;
    }

    case PROBING:
      lift.probing();
      currentState = MAIN_SCREEN;
      break;

    case HOMING:
      lift.homing();
      currentState = MAIN_SCREEN;
      break;
    default:
      break;
  }
}

void displayMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Menu:"));
  for (int i = 0; i < 3; i++) {
    lcd.setCursor(0, i + 1);
    if (i == currentMenuIndex) {
      lcd.print(F("> "));
    } else {
      lcd.print(F("  "));
    }
    lcd.print(menuOptions[i]);
  }
}

int readEncoder(bool accelerated)
{
  if (millis() - _lastEncoderRead > ENCODER_READ_INTERVAL_MS)
  {
    int16_t currentEncoderPosition = encoder.read() / ENC_STEPS;
    if (currentEncoderPosition != _lastEncoderPosition)
    {
      int encoderMove = currentEncoderPosition - _lastEncoderPosition;
      _lastEncoderPosition = currentEncoderPosition;

      int encoderAccelerated = 0;
      if (abs(encoderMove) >= 6)
      {
        encoderAccelerated = encoderMove * 500;
      }
      else if (abs(encoderMove) >= 4)
      {
        encoderAccelerated = encoderMove * 50;
      }
      else if (abs(encoderMove) >= 2)
      {
        encoderAccelerated = encoderMove * 10;
      }
      else
      {
        encoderAccelerated = encoderMove;
      }
      _lastEncoderRead = millis();
      return accelerated ? encoderAccelerated : encoderMove;
    }
    _lastEncoderRead = millis();
  }
  return 0;
}
