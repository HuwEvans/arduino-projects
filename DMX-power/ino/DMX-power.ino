/*
 * DMX Power Control - Arduino Mega 2560
 * 
 * This sketch controls 4 relays via DMX protocol using a CQrobot DMX Shield
 * with the DMXSerial library
 * DMX Start Address: 365
 * Relay Pins: 26-29
 * 
 * DMX Channel Logic:
 * - Channel 365: Relay 1 (Pin 26)
 * - Channel 366: Relay 2 (Pin 27)
 * - Channel 367: Relay 3 (Pin 28)
 * - Channel 368: Relay 4 (Pin 29)
 * 
 * Value Logic:
 * - < 128: Relay OFF
 * - >= 128: Relay ON
 */

// Configure DMXSerial to use Serial1 (RX on pin 19, TX on pin 18)
#define DMXSERIAL_USE_PORT1

#include <DMXSerial.h>

// ============================================================================
// PIN CONFIGURATION
// ============================================================================
const int RELAY_PINS[4] = {26, 27, 28, 29};
const int NUM_RELAYS = 4;

const int DIP_SWITCH_PIN_1 = 30;  // DIP Switch 1 - Override safety timeout
const int DIP_SWITCH_PIN_2 = 31;  // DIP Switch 2 - Reserved for future use

// ============================================================================
// DMX CONFIGURATION
// ============================================================================
const int DMX_START_ADDRESS = 365;
const int DMX_THRESHOLD = 128;  // Values >= 128 turn relay ON, < 128 turn OFF
const unsigned long DMX_TIMEOUT = 5 * 60 * 1000;  // 5 minutes in milliseconds

// DMXSerial will use Serial1 on Arduino Mega
// Serial1 RX on pin 19, TX on pin 18

// ============================================================================
// TIMING AND STATE VARIABLES
// ============================================================================
unsigned long lastDMXReceived = 0;  // Timestamp of last valid DMX frame
boolean dmxSignalActive = false;    // Flag indicating if DMX signal is currently active
byte lastChannelValue[4] = {0, 0, 0, 0};  // Track last channel values to detect changes

// Timing for relay update loop
unsigned long lastRelayUpdate = 0;
const unsigned long RELAY_UPDATE_INTERVAL = 50;  // milliseconds

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Initialize relay pins as outputs
  for (int i = 0; i < NUM_RELAYS; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);  // Start with relays OFF
  }
  
  // Initialize DIP switch pins as inputs
  pinMode(DIP_SWITCH_PIN_1, INPUT_PULLUP);
  pinMode(DIP_SWITCH_PIN_2, INPUT_PULLUP);
  
  // Initialize DMX timeout tracking
  lastDMXReceived = millis();
  lastRelayUpdate = millis();
  
  // Initialize Serial for debugging (optional)
  Serial.begin(115200);
  delay(100);
  Serial.println("DMX Power Control Initialized");
  Serial.println("Start Address: 365");
  Serial.println("Relays on pins 26-29");
  Serial.println("DIP Switch Override on pins 30-31");
  Serial.println("Using DMXSerial Library - Receiver Mode on Serial1");
  Serial.println("DMX Timeout: 5 minutes");
  Serial.println("Non-blocking architecture enabled");
  
  // Initialize DMXSerial in receiver mode on Serial1
  // For Serial1 on Mega: RX=19, TX=18
  DMXSerial.init(DMXReceiver);
  
  delay(100);
  Serial.println("DMX Receiver initialized on Serial1 and waiting for DMX signal...");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Check for DMX timeout (non-blocking)
  checkDMXTimeout();
  
  // Update relays at specified interval (non-blocking with timing check)
  unsigned long currentTime = millis();
  if (currentTime - lastRelayUpdate >= RELAY_UPDATE_INTERVAL) {
    lastRelayUpdate = currentTime;
    updateRelays();
  }
}

// ============================================================================
// RELAY CONTROL
// ============================================================================
void updateRelays() {
  // Update each relay based on corresponding DMX channel value
  boolean dataChanged = false;
  
  for (int i = 0; i < NUM_RELAYS; i++) {
    int dmxChannel = DMX_START_ADDRESS + i;
    byte dmxValue = DMXSerial.read(dmxChannel);
    
    // Determine relay state based on DMX value
    boolean relayState = (dmxValue >= DMX_THRESHOLD);
    digitalWrite(RELAY_PINS[i], relayState ? HIGH : LOW);
    
    // Check if any channel value has changed
    if (dmxValue != lastChannelValue[i]) {
      lastChannelValue[i] = dmxValue;
      dataChanged = true;
    }
    
    // Debug output (uncomment to enable)
    // Serial.print("Relay ");
    // Serial.print(i + 1);
    // Serial.print(" (Ch ");
    // Serial.print(dmxChannel);
    // Serial.print("): ");
    // Serial.print(dmxValue);
    // Serial.print(" -> ");
    // Serial.println(relayState ? "ON" : "OFF");
  }
  
  // If any DMX channel value changed, update the last received timestamp
  if (dataChanged) {
    lastDMXReceived = millis();
    dmxSignalActive = true;
  }
}

void turnOffAllRelays() {
  for (int i = 0; i < NUM_RELAYS; i++) {
    digitalWrite(RELAY_PINS[i], LOW);
  }
}

// ============================================================================
// DMX TIMEOUT HANDLING
// ============================================================================
void checkDMXTimeout() {
  unsigned long currentTime = millis();
  unsigned long timeSinceLastDMX = currentTime - lastDMXReceived;
  
  // Check if DIP switch 1 is ON (override safety - LOW when switch is ON with pullup)
  boolean safetyOverride = !digitalRead(DIP_SWITCH_PIN_1);
  
  // If DMX signal has been lost for 5 minutes AND safety override is not enabled
  if (timeSinceLastDMX > DMX_TIMEOUT && !safetyOverride && dmxSignalActive) {
    Serial.println("DMX timeout: No signal for 5 minutes. Turning off all relays.");
    turnOffAllRelays();
    dmxSignalActive = false;
  }
}
