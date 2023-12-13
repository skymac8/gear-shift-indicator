#include "BluetoothSerial.h"
#include "ELMduino.h"
#include "GearShiftIndicator.h"

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Debug print interval: can be changed if more debug output is desired.
const uint8_t interval = 500;  // ms

// Display constants.
const uint8_t kScreenWidth = 128;  // OLED display width,  in pixels
const uint8_t kScreenHeight = 32;  // OLED display height, in pixels
const uint8_t kTextSize = 4;       // OLED text size

#define WIRE Wire

Adafruit_SSD1306 display = Adafruit_SSD1306(kScreenWidth, kScreenHeight, &WIRE);

// MAC address of the ELM327 Bluetooth OBD-II device.
uint8_t elm327MacAddress[6] = { 0xD8, 0xDB, 0xB9, 0x7C, 0x96, 0x76 };
// For use with ELM327 Bluetooth connection.
ELM327 myELM327;
BluetoothSerial SerialBT;
#define ELM_PORT SerialBT
#define DEBUG_PORT Serial

// Used to query multiple PIDs without timing issues.
typedef enum { ENG_RPM,
               SPEED } obd_pid_states;
// Begin with querying the engine RPM.
obd_pid_states obd_state = ENG_RPM;

// RPM and MPH.
float rpm = 0;
float mph = 0;
// Gears.
int matchedGear = 0;
int recGear = 0;

/*
Modes in which gear ratio is obtained.

CLOSEST - for each computed gear ratio, this mode determines which forward gear is closest,
          i.e. finds the minimum |computed gear ratio - kNth| for k1st ... k5th spec gear ratios,
          and returns the "closest" gear based on this method. Otherwise, assume we are not in a
          forward gear.
OVERALL_TOLERANCE - for each computed gear ratio, apply a +/- kTolerance % tolerance. If any
                     k1st ... k5th spec gear ratio lies within this tolerance, return that gear.
                     Otherwise, assume we are not in a forward gear.
TIRE_TOLERANCE - apply a + kTireTolerance % tolerance to the ***cold-measured tire diameter*** only.
                 Based on the cold-measured tire diameter and this upper tire diameter limit obtained
                 via this tolerance, compute a lower and upper bound gear ratio. If any of k1st ... k5th
                 spec gear ratio lie within this range, return that gear.
*/            
typedef enum { CLOSEST, OVERALL_TOLERANCE, TIRE_TOLERANCE} compute_gear_mode;

// Prints text in the center of the connected display screen.
void displayCenter(String text) {
  int16_t x;
  int16_t y;
  uint16_t width;
  uint16_t height;
  // Get text bounds of the specific display.
  display.getTextBounds(text, 0, 0, &x, &y, &width, &height);
  // Position cursor and display text.
  display.clearDisplay();
  display.setCursor((kScreenWidth - width) / 2, (kScreenHeight - height) / 2);
  display.println(text);
  display.display();
}

// Prints gear information on the connected display screen.
// If in Economy Mode, show the current gear, along with a gear shift recommendation if applicable.
// Otherwise, just show the current gear.
void displayGears(int matchedGear, int recommendedGear, bool economyMode) {
  // If we're in neutral, reverse, have the clutch in,
  // or otherwise don't match any known gear ratio, show nothing on the display.
  if (matchedGear == 0) {
    displayCenter("-");
    return;
  } else if (matchedGear == recommendedGear) {
    displayCenter(String(matchedGear));
    return;
  } else if (economyMode) {
    String recommendation = String(matchedGear) + " > " + String(recommendedGear);
    displayCenter(recommendation);
    return;
  }
}

// Prints debugging output to the serial monitor: current RPM, MPH, actual gear, & recommended gear.
void printDebug(float rpm, float mph, int matchedGear, int recGear, compute_gear_mode mode) {
    Serial.println("---------- New data ----------");
    Serial.print("RPM: ");
    Serial.println(rpm);
    Serial.print("MPH: ");
    Serial.println(mph);

    Serial.print("Mode: ");
    switch (mode) {
      case CLOSEST: {
        Serial.println("Closest");
      } case OVERALL_TOLERANCE: {
        Serial.println("Overall Tolerance");
      } case TIRE_TOLERANCE: {
        Serial.println("Tire Tolerance");
      }
    }

    Serial.print("Actual gear: ");
    Serial.println(matchedGear);

    Serial.print("Recommended gear: ");
    Serial.println(recGear);
}

// Computes current gear ratio, current gear, and current recommended gear,
// based on the current mode. If debug output is enabled,
// print debug values to the Serial Monitor.
void compute(float rpm, float mph, bool debug, compute_gear_mode mode) {
  switch (mode) {
    case CLOSEST: {
      float ratio = gearRatio(rpm, mph);
      matchedGear = matchedGearBasedOnClosest(ratio);
      break;
    } case OVERALL_TOLERANCE: {
      float ratio = gearRatio(rpm, mph);
      matchedGear = matchedGearBasedOnOverallTolerance(ratio);
      break;
    } case TIRE_TOLERANCE: {
      float ratioLower = gearRatio_LowerTireTolerance(rpm, mph);
      float ratioUpper = gearRatio_UpperTireTolerance(rpm, mph);
      matchedGear = matchedGearBasedOnTireTolerance(ratioLower, ratioUpper);
      break;
    }
  }

  recGear = recommendation(matchedGear, rpm);

  // Print values to the Serial Monitor on an interval, so they are human-readable.
  // (Otherwise, they print so fast you can't read them.)
  if (debug) {
    uint32_t time = millis();
    if (time % interval == 0) {
      printDebug(rpm, mph, matchedGear, recGear, mode);
    }
  }
  return;
}

// Setup.
void setup() {
  Serial.println("Setup beginning...");
#if LED_BUILTIN
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
#endif

  DEBUG_PORT.begin(bps);
  SerialBT.setPin("1234");
  ELM_PORT.begin("GearShiftIndicator", true);

  if (!ELM_PORT.connect(elm327MacAddress)) {
    DEBUG_PORT.println("Couldn't connect to OBD scanner - Phase 1");
    while (1);
  }

  if (!myELM327.begin(ELM_PORT, false, timeout)) {
    Serial.println("Couldn't connect to OBD scanner - Phase 2");
    while (1);
  }
  // Successful Bluetooth connection to ELM327 OBD-II reader.
  Serial.println("Connected to ELM327");

  Serial.println("OLED FeatherWing test");
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Address 0x3C for 128x32
  // Successful display startup.
  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();

  Serial.println("Starting up...");

  // Text display tests
  display.setTextSize(kTextSize);
  display.setTextColor(SSD1306_WHITE);
  display.display();  // actually display all of the above
}

// Repeating loop.
void loop() {
  bool error = false;
  switch (obd_state) {
    case ENG_RPM: {
        float tempRPM = myELM327.rpm();

        if (myELM327.nb_rx_state == ELM_SUCCESS) {
          rpm = tempRPM;
          obd_state = SPEED;
        } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
          // myELM327.printError();
          error = true;
          obd_state = SPEED;
        }
      } case SPEED: {
        float tempMph = myELM327.mph();

        if (myELM327.nb_rx_state == ELM_SUCCESS) {
          mph = tempMph;
          obd_state = ENG_RPM;
        } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
          // myELM327.printError();
          error = true;
          obd_state = ENG_RPM;
        }
      }
  }
  compute_gear_mode mode = CLOSEST;
  // Using these values, compute the current gear (and recommendation, if applicable), and show on display.
  compute(rpm, mph, /*bool debug=*/ true, mode);
  // If Economy Mode is true, show gear shift recommendations. Otherwise, only show current gear.
  displayGears(matchedGear, recGear, /*bool economyMode=*/ true);
}