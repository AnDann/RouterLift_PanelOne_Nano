#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <LiquidCrystalFast.h>
#include <Axis.h>
#include <Bounce2.h>
//#include <Streaming.h>

// Pins used
#define LE_ENCA  2 // D2 encoder pins
#define LE_ENCB  3 // D3
#define BUTTON_PIN  A0 // A2 Encoder click pin

// Encoder steps per click
#define ENC_STEPS 4
/*
PanelOne (5V) LCD pin to Nano
--------------------
	AUX2 1 -> +5V
	AUX2 2 -> GND

	AUX2  3 ENCB  -> D3
	AUX2  4 ENCA  -> D2
	AUX2  5 DB7   -> D9
	AUX2  6 RS    -> D4
	AUX2  7 DB6   -> D8
	AUX2  8 EN    -> D5
	AUX2  9 DB5   -> D7
	AUX2 10 DB4   -> D6

	AUX3  7 ENCSW -> A2


PanelOne (5V) SD pin to Smoothie
--------------------

	AUX3 3  CS -> Unused GPIO : 1.30
	AUX3 4 CLK -> SCK  : 0.15
	AUX3 5  DO -> MISO : 0.17
	AUX3 6  DI -> MOSI : 0.18

	AUX3 2 -> GND
	AUX3 8 -> +5V
*/

// parallel LCD Pins
// LCD pins: RS  RW  EN  D4 D5 D6 D7
// if not defined RW_PIN then RW needs to be connected to GND
#define LCD_RS  4
#define LCD_EN  5
#define LCD_D4  6
#define LCD_D5  7
#define LCD_D6  8
#define LCD_D7  9

LiquidCrystalFast lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

Encoder encoder(LE_ENCA, LE_ENCB);
Bounce buttonOk = Bounce();

#define STEP_PIN 12
#define DIR_PIN 11
#define ENABLE_PIN 10

#define ENDSTOP_MIN_PIN A2
#define ENDSTOP_MAX_PIN A3
#define PROBE_PIN A4

Axis lift(STEP_PIN,DIR_PIN,ENABLE_PIN,200,8,8,0.0,119.0,ENDSTOP_MIN_PIN,ENDSTOP_MAX_PIN,PROBE_PIN);

// The time in milliseconds, the display should refresh. The default value should be fine.
// default: 200
#define DISPLAY_REFRESH_INTERVAL_MS 200
// The time in milliseconds, the encoder steps should be read. The default value should be fine.
// default: 50
#define ENCODER_READ_INTERVAL_MS 50

bool buttonPressed;
long _lastEncoderPosition;
unsigned long _lastEncoderRead, _lastDisplayUpdate;

const char* axisStateText[] = {"None", "Go to Target", "Go to Home", "Go to Probe", "In Position", "Max!", "Min!"};
const char* homingStateText[] = {"None", "Backoff 1", "Move Fast", "Backoff 2", "Move slow", "Homed", "Error"};
const char* probingStateText[] = {"None", "Backoff 1", "Move Fast", "Backoff 2", "Move slow", "Probed", "Error"};

int readEncoder();

void setup (void)
{
  buttonOk.attach(BUTTON_PIN, INPUT_PULLUP);
  buttonOk.interval(5); // interval in ms

  // set up the LCD's number of rows and columns:
  lcd.begin(20, 4);
	lcd.setCursor(0, 0);
	lcd.print("RouterLift V0.01");
	lcd.setCursor(0, 1);
	lcd.print("Starting up...");
  lcd.setCursor(0, 2);

  Serial.begin(115200);
}  // end of setup

// main loop - wait for commands
void loop (void)
{
  lift.handle();
  buttonOk.update();

  if((lift.inPosition() || lift.isError()) && (millis() - _lastDisplayUpdate > DISPLAY_REFRESH_INTERVAL_MS))
  {
    lcd.setCursor(0, 0);
    lcd.print("Status:             ");
    lcd.setCursor(7, 0);
    lcd.print(axisStateText[lift.getState()]);

    lcd.setCursor(0, 1);
    lcd.print("         :          ");
    lcd.setCursor(0, 1);
    lcd.print(homingStateText[lift.getHomingState()]);
    lcd.setCursor(10, 1);
    lcd.print(probingStateText[lift.getProbingState()]);

    lcd.setCursor(0, 2);
    lcd.print("Soll:               ");
    lcd.setCursor(5, 2);
    lcd.print(lift.getTargetPosition());
    lcd.print("mm");

    lcd.setCursor(0, 3);
    lcd.print("Ist:                ");
    lcd.setCursor(5, 3);
    lcd.print(lift.getCurrentPosition());
    lcd.print("mm ");
    lcd.print(lift.getWorkoffset());
    lcd.print("mm");
    _lastDisplayUpdate = millis();
  }
  
  if(lift.isHomed())
  {
    lift.setTargetPosition(lift.getTargetPosition() + (readEncoder() * 0.01));
  }

  if(buttonOk.rose() && buttonOk.previousDuration() < 1000) lift.moveToTarget();
  else if (buttonOk.rose() && buttonOk.previousDuration() > 2000 && buttonOk.previousDuration() < 4999) lift.probing();
  else if (buttonOk.read() == LOW && buttonOk.currentDuration() > 5000) lift.homing();

}

// Returns the change in steps
int readEncoder()
{
    if (millis() - _lastEncoderRead > ENCODER_READ_INTERVAL_MS)
    {
        // Calculate encoder steps
        int16_t currentEncoderPosition = encoder.read() / ENC_STEPS;
        if (currentEncoderPosition != _lastEncoderPosition)
        {
            int encoderMove = currentEncoderPosition - _lastEncoderPosition;
            _lastEncoderPosition = currentEncoderPosition;

            int encoderAccelerated = 0;
            // Encoder acceleration
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
            return encoderAccelerated;
        }
        _lastEncoderRead = millis();
    }
    return 0;
}