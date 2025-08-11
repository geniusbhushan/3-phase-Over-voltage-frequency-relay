#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Analog inputs for ZMPT101B sensors
#define PHASE1 A0
#define PHASE2 A1
#define PHASE3 A2

// Relay and button pins
#define RELAY_PIN 8
#define MODE_BTN 2  // INT0
#define SET_BTN 3   // INT1

// Calibration and config
const float calibrationFactor = 600.0;
const int SAMPLES = 1000;
const int EEPROM_ADDR = 0;

int voltageLimit = 250;
const int limitMin = 200;
const int limitMax = 300;

// State variables
volatile bool mode = false;  // false = monitor, true = setting
volatile bool setPressed = false;
volatile unsigned long setPressTime = 0;
volatile unsigned long lastModeInterrupt = 0;

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(MODE_BTN, INPUT);
  digitalWrite(MODE_BTN, HIGH); // Enable pull-up manually
  pinMode(SET_BTN, INPUT_PULLUP);

  // Read limit from EEPROM
  voltageLimit = EEPROM.read(EEPROM_ADDR);
  if (voltageLimit < limitMin || voltageLimit > limitMax) voltageLimit = 250;

  // Force monitoring mode
  mode = false;

  // Show splash
  lcd.setCursor(0, 0);
  lcd.print("3-Phase Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
  lcd.clear();

  // Wait before enabling interrupts (avoid false trigger)
  delay(2000);

  // Now attach interrupts
  attachInterrupt(digitalPinToInterrupt(MODE_BTN), toggleMode, FALLING);
  attachInterrupt(digitalPinToInterrupt(SET_BTN), handleSetInterrupt, CHANGE);
}

void loop() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();

    float V1 = readVoltage(PHASE1);
    float V2 = readVoltage(PHASE2);
    float V3 = readVoltage(PHASE3);

    if (!mode) {
      // --- Monitoring mode ---
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("R:"); lcd.print((int)V1); lcd.print("V");
      lcd.print("   Y:"); lcd.print((int)V2);lcd.print("V");
      lcd.setCursor(0, 1);
      lcd.print("B:"); lcd.print((int)V3);lcd.print("V");
      lcd.print("   L:"); lcd.print(voltageLimit);lcd.print("V");

      if (V1 > voltageLimit || V2 > voltageLimit || V3 > voltageLimit) {
        digitalWrite(RELAY_PIN, LOW); // Trip
      } else {
        digitalWrite(RELAY_PIN, HIGH); // Normal
      }

    } else {
      // --- Setting mode ---
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Set Voltage Limit:");
      lcd.setCursor(0, 1);
      lcd.print(voltageLimit); lcd.print(" V");
    }
  }
}

float readVoltage(int pin) {
  unsigned long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    int val = analogRead(pin) - 512;
    sum += (long)val * val;
  }
  float rms = sqrt(sum / (float)SAMPLES);
  float voltage = (rms * 5.0 / 1023.0) * calibrationFactor;
  return voltage;
}

// Toggle monitor/setting mode (with debounce)
void toggleMode() {
  unsigned long now = millis();
  if (now - lastModeInterrupt > 300) {
    if (digitalRead(MODE_BTN) == LOW) {
      mode = !mode;
      lastModeInterrupt = now;
    }
  }
}

// Handle setting voltage limit
void handleSetInterrupt() {
  if (digitalRead(SET_BTN) == LOW) {
    setPressTime = millis();
    setPressed = true;
  } else {
    if (setPressed && mode) {
      unsigned long pressDuration = millis() - setPressTime;
      if (pressDuration < 1000) {
        // Short press: increase limit
        voltageLimit++;
        if (voltageLimit > limitMax) voltageLimit = limitMax;
      } else {
        // Long press: decrease limit
        voltageLimit--;
        if (voltageLimit < limitMin) voltageLimit = limitMin;
      }
      EEPROM.write(EEPROM_ADDR, voltageLimit);
    }
    setPressed = false;
  }
}
