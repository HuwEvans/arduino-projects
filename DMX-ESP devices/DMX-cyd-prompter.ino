// CYD DMX ArtNet Prompter
// Uses CYD (Cheap Yellow Display) with ESP32 to display set/ready/go cues via ArtNet DMX
// Install "GFX Library for Arduino" with the Library Manager
// Install "Artnet" with the Library Manager by Pierre Guilbert

#include <Arduino_GFX_Library.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>         // Install "ArduinoJson" with the Library Manager (tested with v6.x)
#include <ArtnetWifi.h>          // Install "Artnet" with the Library Manager by Pierre Guilbert

// ============================================
// Pin Configuration for CYD (ESP32-2432S028)
// ============================================
#define BL_PIN 21                    // Backlight pin (on some yellow displays this is 27)
#define BOOT_PIN 0                   // Boot button pin
#define BOOT_BUTTON_DEBOUCE_TIME 400 // Debounce time in milliseconds

// ============================================
// Configuration
// ============================================
#define CONFIG_FILE "/spiffs/config.json"

// DMX Channel Configuration (calculated from base address)
#define DMX_CH_CUESTATE 0          // Will be set to dmxAddress
#define DMX_CH_NAME_SELECT 1       // Will be set to dmxAddress + 1
#define DMX_CH_BACKLIGHT 2         // Will be set to dmxAddress + 2

uint8_t DMX_ADDRESS = 1;           // Base DMX address
uint8_t DMX_CH_CUESTATE_RUNTIME = 1;      // Channel 0: 0=off(black), 1-85=set(red), 86-170=ready(yellow), 171-255=go(green)
uint8_t DMX_CH_NAME_SELECT_RUNTIME = 2;   // Channel 1: 0=no name, 1-21=name0, 22-42=name1, 43-63=name2, etc.
uint8_t DMX_CH_BACKLIGHT_RUNTIME = 3;     // Channel 2: 0=full brightness, 255=off (inverted control)

// ArtNet Device Identification
#define ARTPOLL_PORT 6454
#define DEVICE_SHORT_NAME "CYD-PM"
#define DEVICE_LONG_NAME "CYD Prompter"
#define DEVICE_FIRMWARE_VERSION 0x0100  // Version 1.0

// Display SPI Speed
#define DISPLAY_SPI_SPEED 80000000L  // 80MHz

// Configuration structure
struct Config {
    char ssid[64];
    char password[64];
    uint8_t universe;
    uint8_t dmxAddress;          // Base DMX address (channels will be address, address+1, address+2)
    char names[12][32];  // 12 names, up to 31 characters each
};

Config config;

// ============================================
// Global Variables
// ============================================
ArtnetWifi artnet;
WebServer webServer(80);
WiFiUDP udp;
WiFiUDP pollListenerUdp;  // UDP instance for receiving Art-Net Polls (no bind, just receive)
uint8_t dmxBuffer[512] = {0};
unsigned long lastDmxUpdate = 0;
bool configMode = false;
unsigned long lastPollTime = 0;

// Display setup
Arduino_DataBus *bus = new Arduino_HWSPI(2 /* DC */, 15 /* CS */, 14 /* SCK */, 13 /* MOSI */, 12 /* MISO */);
Arduino_GFX *gfx = new Arduino_ILI9341(bus);

// State machine
enum PrompterState { STATE_WAITING_FOR_DMX, STATE_OFF, STATE_SET, STATE_READY, STATE_GO };
PrompterState currentState = STATE_WAITING_FOR_DMX;
PrompterState lastDisplayedState = STATE_WAITING_FOR_DMX;
unsigned long stateOffStartTime = 0;
bool dmxReceived = false;
uint8_t currentNameIndex = 0;  // Currently selected name (0-11)
bool showName = false;  // Whether to display the name (false when DMX name channel is 0)
uint8_t lastDisplayedNameIndex = 0;  // Last displayed name index
bool lastDisplayedShowName = false;  // Last displayed show name flag
uint8_t backlightDMXValue = 255;  // DMX backlight control value (0=bright, 255=off)

// Color definitions
#define COLOR_BLACK 0x0000
#define COLOR_RED 0xF800
#define COLOR_YELLOW 0xFFE0
#define COLOR_GREEN 0x07E0
#define COLOR_WHITE 0xFFFF

// ============================================
// Helper Functions
// ============================================
String getStateName(PrompterState state) {
    switch(state) {
        case STATE_WAITING_FOR_DMX: return "Waiting for DMX";
        case STATE_OFF: return "OFF";
        case STATE_SET: return "SET";
        case STATE_READY: return "READY";
        case STATE_GO: return "GO";
        default: return "Unknown";
    }
}

// ============================================
// Setup
// ============================================
void setup() {
    Serial.begin(115200);
    delay(2000);  // Wait for serial connection
    Serial.println("\n\n========================================");
    Serial.println("CYD DMX ArtNet Prompter");
    Serial.println("========================================");

    // Initialize display
    gfx->begin();
    gfx->setRotation(1);  // Set to landscape orientation
    gfx->fillScreen(COLOR_BLACK);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(10, 50);
    gfx->println("CYD Prompter");
    gfx->setCursor(10, 100);
    gfx->setTextSize(1);
    gfx->println("Initializing...");

    // Initialize backlight
    pinMode(BL_PIN, OUTPUT);
    analogWrite(BL_PIN, 255);  // Set to full brightness initially (0-255 PWM)

    // Initialize boot button
    pinMode(BOOT_PIN, INPUT);

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("SPIFFS Mount Failed");
        gfx->setCursor(10, 120);
        gfx->println("SPIFFS Mount Failed");
        delay(2000);
    }

    // Load configuration
    loadConfig();

    // Display boot screen with connection info
    displayBootScreen();

    // Connect to WiFi
    connectToWiFi();

    // Initialize ArtNet
    artnet.begin();
    artnet.setArtDmxCallback(onArtnetDMX);
    
    // Note: Art-Net Poll responses are handled via manual UDP listening
    // We listen for broadcasts without binding to avoid port conflicts
    
    // Bind pollListenerUdp to port 6454 to receive Art-Net Poll packets
    pollListenerUdp.begin(ARTPOLL_PORT);
    Serial.printf("UDP listener bound to port %d\n", ARTPOLL_PORT);
    
    Serial.println("\n=== Art-Net Discovery Debug Info ===");
    Serial.printf("Device Name: %s (%s)\n", DEVICE_SHORT_NAME, DEVICE_LONG_NAME);
    Serial.printf("WiFi IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Subnet Mask: %s\n", WiFi.subnetMask().toString().c_str());
    IPAddress broadcastIP = ~WiFi.subnetMask() | WiFi.gatewayIP();
    Serial.printf("Broadcast Address: %s\n", broadcastIP.toString().c_str());
    uint8_t mac[6];
    WiFi.macAddress(mac);
    Serial.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("Art-Net Universe: %d\n", config.universe);
    Serial.printf("DMX Base Address: %d (Channels: %d, %d, %d)\n", config.dmxAddress, 
                 DMX_CH_CUESTATE_RUNTIME, DMX_CH_NAME_SELECT_RUNTIME, DMX_CH_BACKLIGHT_RUNTIME);
    Serial.println("Device will announce itself every 5 seconds");
    Serial.println("Listening for Art-Net Poll requests on port 6454");
    Serial.println("==================================\n");

    // Initialize web server routes
    webServer.on("/", handleRoot);
    webServer.on("/api/names", handleGetNames);
    webServer.on("/api/dmx-config", HTTP_GET, handleGetDmxConfig);
    webServer.on("/api/dmx-config", HTTP_POST, handleSetDmxConfig);
    webServer.on("/api/names/update", handleUpdateName);
    webServer.begin();

    // Display waiting for DMX screen
    displayWaitingForDMX();

    Serial.println("Setup complete!");
    Serial.printf("Access web interface at: http://%s\n", WiFi.localIP().toString().c_str());
    delay(2000);
}

// ============================================
// Main Loop
// ============================================
void loop() {
    // Handle web server requests
    webServer.handleClient();

    // Listen for Art-Net Poll requests on broadcast (without binding to avoid conflicts)
    listenForArtNetPoll();
    
    // Periodic device announcement for QLC+ discovery (every 5 seconds)
    if (millis() - lastPollTime > 5000) {
        lastPollTime = millis();
        announceDevice();
    }
    
    // Process ArtNet data
    artnet.read();

    // Update display if state changed OR name changed
    if (currentState != lastDisplayedState || currentNameIndex != lastDisplayedNameIndex || showName != lastDisplayedShowName) {
        updateDisplay();
        lastDisplayedState = currentState;
        lastDisplayedNameIndex = currentNameIndex;
        lastDisplayedShowName = showName;

        // Track when we enter OFF state
        if (currentState == STATE_OFF) {
            stateOffStartTime = millis();
        } else if (currentState != STATE_WAITING_FOR_DMX) {
            // Turn backlight back on when leaving OFF state (but not from WAITING_FOR_DMX)
            // Use DMX backlight value (inverted: 0=full bright, 255=off)
            analogWrite(BL_PIN, 255 - backlightDMXValue);
        }
    }

    // Check if device has been OFF for more than 1 minutes (60000 ms)
    if (currentState == STATE_OFF && stateOffStartTime > 0) {
        if (millis() - stateOffStartTime > 60000) {
            analogWrite(BL_PIN, 0);  // Force backlight off - timer takes priority
        } else {
            // Timer hasn't expired yet, use DMX backlight control (inverted: 0=full bright, 255=off)
            analogWrite(BL_PIN, 255 - backlightDMXValue);
        }
    } else {
        // Not in OFF state or just entered OFF state, use DMX backlight control (inverted: 0=full bright, 255=off)
        analogWrite(BL_PIN, 255 - backlightDMXValue);
    }

    // Check boot button for configuration mode
    if (digitalRead(BOOT_PIN) == LOW) {
        delay(BOOT_BUTTON_DEBOUCE_TIME);
        if (digitalRead(BOOT_PIN) == LOW) {
            Serial.println("Boot button pressed - entering configuration mode");
            configMode = true;
            displayConfigMode();
        }
    }

    delay(50);  // Small delay to prevent overwhelming the processor
}

// ============================================
// Art-Net Poll/Discovery Handling
// ============================================
void listenForArtNetPoll() {
    // Try to parse a broadcast packet to detect polls
    int packetSize = pollListenerUdp.parsePacket();
    if (packetSize > 0) {
        Serial.printf("[POLL LISTEN] Received packet of %d bytes from %s:%d\n", packetSize, 
                     pollListenerUdp.remoteIP().toString().c_str(), pollListenerUdp.remotePort());
        
        uint8_t buffer[min(packetSize, 512)];
        int len = pollListenerUdp.read(buffer, min(packetSize, 512));
        
        Serial.print("[POLL LISTEN] Packet header (first 10 bytes): ");
        for (int i = 0; i < min(10, len); i++) {
            Serial.printf("%02X ", buffer[i]);
        }
        Serial.println();
        
        // Check for Art-Net header and Poll OpCode
        if (len >= 10 && 
            buffer[0] == 'A' && buffer[1] == 'r' && buffer[2] == 't' && 
            buffer[3] == '-' && buffer[4] == 'N' && buffer[5] == 'e' && 
            buffer[6] == 't' && buffer[7] == 0) {
            
            uint16_t opcode = buffer[8] | (buffer[9] << 8);
            Serial.printf("[POLL LISTEN] Valid Art-Net packet! OpCode: 0x%04X\n", opcode);
            
            // Poll = 0x2000, Poll Reply = 0x2100
            if (opcode == 0x2000) {
                Serial.printf("[POLL LISTEN] >>> ART-NET POLL RECEIVED from %s! Sending reply...\n", 
                             pollListenerUdp.remoteIP().toString().c_str());
                sendArtNetPollReply(pollListenerUdp.remoteIP(), pollListenerUdp.remotePort());
            } else {
                Serial.printf("[POLL LISTEN] OpCode 0x%04X is not a Poll (0x2000)\n", opcode);
            }
        } else {
            Serial.printf("[POLL LISTEN] Not a valid Art-Net packet or too short (%d bytes)\n", len);
        }
    }
}

void announceDevice() {
    // Send a broadcast announcement that this device is available
    // This helps QLC+ and other Art-Net controllers discover the device
    
    static unsigned long announceCount = 0;
    announceCount++;
    
    IPAddress broadcastIP = ~WiFi.subnetMask() | WiFi.gatewayIP();  // Calculate broadcast address
    
    Serial.printf("[ANNOUNCE #%lu] Broadcasting to %s:6454\n", announceCount, broadcastIP.toString().c_str());
    
    Serial.printf("[ANNOUNCE #%lu] Broadcasting device info to %s:6454...\n", 
                 announceCount, broadcastIP.toString().c_str());
    
    uint8_t announcement[239];
    memset(announcement, 0, sizeof(announcement));
    
    // Header: "Art-Net\0"
    announcement[0] = 'A'; announcement[1] = 'r'; announcement[2] = 't';
    announcement[3] = '-'; announcement[4] = 'N'; announcement[5] = 'e';
    announcement[6] = 't'; announcement[7] = 0;
    
    // OpCode: Poll Reply (0x2100)
    announcement[8] = 0x21;
    announcement[9] = 0x00;
    
    // IP Address
    uint32_t ipAddr = WiFi.localIP();
    announcement[10] = (ipAddr >> 0) & 0xFF;
    announcement[11] = (ipAddr >> 8) & 0xFF;
    announcement[12] = (ipAddr >> 16) & 0xFF;
    announcement[13] = (ipAddr >> 24) & 0xFF;
    
    // Port (6454 - little endian)
    announcement[14] = 0x36;
    announcement[15] = 0x19;
    
    // Version info
    announcement[16] = 0x00;
    announcement[17] = 0x0E;
    
    // Net Switch (high 4 bits of universe) and Sub Switch (low 4 bits of universe)
    announcement[18] = config.universe >> 4;   // Net = universe / 16
    announcement[19] = config.universe & 0x0F; // SubSwitch = universe % 16
    
    // Oem codes
    announcement[20] = 0x00;
    announcement[21] = 0x00;
    
    // UBEA Version
    announcement[22] = 0x00;
    
    // Status 1 - bit 0: UBEA present, bit 6: low/normal (not low addr mode)
    announcement[23] = 0x40;  // 0x40 = device supports 15-bit universe addressing
    
    // Short Name
    String shortName = DEVICE_SHORT_NAME;
    if (shortName.length() >= 18) shortName = shortName.substring(0, 17);
    for (int i = 0; i < shortName.length(); i++) {
        announcement[24 + i] = shortName[i];
    }
    
    // Long Name
    String longName = DEVICE_LONG_NAME;
    if (longName.length() >= 64) longName = longName.substring(0, 63);
    for (int i = 0; i < longName.length(); i++) {
        announcement[42 + i] = longName[i];
    }
    
    // Node Report
    String nodeReport = "Ready";
    if (nodeReport.length() >= 64) nodeReport = nodeReport.substring(0, 63);
    for (int i = 0; i < nodeReport.length(); i++) {
        announcement[106 + i] = nodeReport[i];
    }
    
    // Port types (output)
    announcement[170] = 0x80;  // Port 1: output
    
    // Good Output
    announcement[178] = 0x02;  // Port 1 active
    
    // DMX Universe
    announcement[182] = config.universe & 0xFF;
    
    // Style
    announcement[190] = 0x00;
    
    // MAC Address
    uint8_t mac[6];
    WiFi.macAddress(mac);
    for (int i = 0; i < 6; i++) {
        announcement[191 + i] = mac[i];
    }
    
    // Bind IP (0 = not bound)
    announcement[197] = 0x00;
    announcement[198] = 0x00;
    announcement[199] = 0x00;
    announcement[200] = 0x00;
    
    announcement[201] = 0x00;  // Bind Index
    announcement[202] = 0x08;  // Status 2 (bit 3: supports 15-bit universe addressing)
    
    // Send broadcast announcement
    pollListenerUdp.beginPacket(broadcastIP, ARTPOLL_PORT);
    int bytesWritten = pollListenerUdp.write(announcement, sizeof(announcement));
    int sendResult = pollListenerUdp.endPacket();
    
    Serial.printf("  - Bytes written: %d, Send result: %d\n", bytesWritten, sendResult);
    if (sendResult > 0) {
        Serial.println("  ✓ Announcement sent successfully");
    } else {
        Serial.println("  ✗ Announcement send FAILED!");
    }
}

void sendArtNetPollReply(IPAddress remoteIP, uint16_t remotePort) {
    // Send Art-Net Poll Reply to the controller that polled us
    
    uint8_t reply[239];
    memset(reply, 0, sizeof(reply));
    
    // Header: "Art-Net\0"
    reply[0] = 'A'; reply[1] = 'r'; reply[2] = 't';
    reply[3] = '-'; reply[4] = 'N'; reply[5] = 'e';
    reply[6] = 't'; reply[7] = 0;
    
    // OpCode: Poll Reply (0x2100 in little-endian)
    reply[8] = 0x21;
    reply[9] = 0x00;
    
    // IP Address (4 bytes)
    uint32_t ipAddr = WiFi.localIP();
    reply[10] = (ipAddr >> 0) & 0xFF;
    reply[11] = (ipAddr >> 8) & 0xFF;
    reply[12] = (ipAddr >> 16) & 0xFF;
    reply[13] = (ipAddr >> 24) & 0xFF;
    
    // Port (6454 - little endian)
    reply[14] = 0x36;  // 54
    reply[15] = 0x19;  // 25
    
    // Version info
    reply[16] = 0x00;
    reply[17] = 0x0E;
    
    // Net Switch (high 4 bits of universe) and Sub Switch (low 4 bits of universe)
    reply[18] = config.universe >> 4;   // Net = universe / 16
    reply[19] = config.universe & 0x0F; // SubSwitch = universe % 16
    
    // Oem and UBEA
    reply[20] = 0x00;
    reply[21] = 0x00;
    reply[22] = 0x00;
    
    // Status 1 - bit 0: UBEA present, bit 6: low/normal (not low addr mode)
    reply[23] = 0x40;  // 0x40 = device supports 15-bit universe addressing
    
    // Short Name (18 bytes)
    String shortName = DEVICE_SHORT_NAME;
    if (shortName.length() >= 18) shortName = shortName.substring(0, 17);
    for (int i = 0; i < shortName.length(); i++) {
        reply[24 + i] = shortName[i];
    }
    
    // Long Name (64 bytes)
    String longName = DEVICE_LONG_NAME;
    if (longName.length() >= 64) longName = longName.substring(0, 63);
    for (int i = 0; i < longName.length(); i++) {
        reply[42 + i] = longName[i];
    }
    
    // Node Report (64 bytes)
    String nodeReport = "Art-Net Prompter";
    if (nodeReport.length() >= 64) nodeReport = nodeReport.substring(0, 63);
    for (int i = 0; i < nodeReport.length(); i++) {
        reply[106 + i] = nodeReport[i];
    }
    
    // Port types: output on port 1
    reply[170] = 0x80;  // Bit 7: output, bits 6-0: DMX
    reply[171] = 0x00;
    reply[172] = 0x00;
    reply[173] = 0x00;
    
    // Good Input (status of input ports - we have none)
    reply[174] = 0x00;  // Port 1 has no input
    reply[175] = 0x00;  // Port 2
    reply[176] = 0x00;  // Port 3
    reply[177] = 0x00;  // Port 4
    
    // Good Output (status of output ports)
    reply[178] = 0x02;  // Port 1 transmitting DMX
    reply[179] = 0x00;
    reply[180] = 0x00;
    reply[181] = 0x00;
    
    // DMX In/Out (universe per port)
    reply[182] = config.universe & 0xFF;
    reply[183] = 0x00;
    reply[184] = 0x00;
    reply[185] = 0x00;
    
    // Macro/Remote In
    reply[186] = 0x00;
    reply[187] = 0x00;
    reply[188] = 0x00;
    reply[189] = 0x00;
    
    // Style (0 = standard DMX device)
    reply[190] = 0x00;
    
    // MAC Address
    uint8_t mac[6];
    WiFi.macAddress(mac);
    for (int i = 0; i < 6; i++) {
        reply[191 + i] = mac[i];
    }
    
    // Bind IP (0 = not bound to specific controller)
    reply[197] = 0x00;
    reply[198] = 0x00;
    reply[199] = 0x00;
    reply[200] = 0x00;
    
    // Bind Index
    reply[201] = 0x00;
    
    // Status 2 (bit 3: supports 15-bit universe addressing)
    reply[202] = 0x08;  // 0x08 = supports 15-bit universe addressing
    
    // Send unicast reply
    Serial.printf("[POLL REPLY] Sending response to %s:%d\n", remoteIP.toString().c_str(), remotePort);
    pollListenerUdp.beginPacket(remoteIP, remotePort);
    int bytesWritten = pollListenerUdp.write(reply, sizeof(reply));
    int sendResult = pollListenerUdp.endPacket();
    
    if (sendResult > 0) {
        Serial.printf("  ✓ Poll reply sent successfully (%d bytes)\n", bytesWritten);
    } else {
        Serial.println("  ✗ Poll reply send FAILED!");
    }
}



// ============================================
// ArtNet Callback - Called when DMX data is received
// ============================================
void onArtnetDMX(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data) {
    // Check if this is the universe we're listening on
    if (universe == config.universe) {
        // Copy DMX data to buffer
        for (int i = 0; i < length && i < 512; i++) {
            dmxBuffer[i] = data[i];
        }
        lastDmxUpdate = millis();

        // If this is the first DMX data received, transition from waiting state to OFF
        if (!dmxReceived) {
            dmxReceived = true;
            currentState = STATE_OFF;
            Serial.println("First DMX data received!");
        }

        // Update prompter state based on DMX channel (cue state)
        uint8_t cueValue = dmxBuffer[DMX_CH_CUESTATE_RUNTIME];

        // Determine state based on DMX value
        if (cueValue == 0) {
            currentState = STATE_OFF;
        } else if (cueValue >= 1 && cueValue <= 85) {
            currentState = STATE_SET;
        } else if (cueValue >= 86 && cueValue <= 170) {
            currentState = STATE_READY;
        } else if (cueValue >= 171 && cueValue <= 255) {
            currentState = STATE_GO;
        }

        // Update name selection based on name select channel
        uint8_t nameValue = dmxBuffer[DMX_CH_NAME_SELECT_RUNTIME];
        // 0 = no name shown, 1-21 = name0, 22-42 = name1, etc.
        if (nameValue == 0) {
            showName = false;
        } else {
            showName = true;
            // Map 1-255 to name indices 0-11 (each name gets ~21 values)
            currentNameIndex = min((uint8_t)11, (uint8_t)((nameValue - 1) / 21));
        }

        // Update backlight level based on backlight channel
        backlightDMXValue = dmxBuffer[DMX_CH_BACKLIGHT_RUNTIME];

        Serial.printf("DMX Universe: %d, CueChannel: %d, NameChannel: %d, BacklightChannel: %d, CueValue: %d, State: %d, NameIndex: %d, BacklightDMX: %d\n",
                     universe, DMX_CH_CUESTATE_RUNTIME + 1, DMX_CH_NAME_SELECT_RUNTIME + 1, DMX_CH_BACKLIGHT_RUNTIME + 1, cueValue, currentState, currentNameIndex, backlightDMXValue);
    }
}

// ============================================
// Display Functions
// ============================================
void displayBootScreen() {
    gfx->fillScreen(COLOR_BLACK);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(2);
    gfx->setCursor(20, 40);
    gfx->println("CYD");
    gfx->setCursor(10, 70);
    gfx->println("Prompter");

    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_YELLOW);
    gfx->setCursor(10, 120);
    gfx->println("Configuration:");
    gfx->setCursor(10, 140);
    gfx->printf("SSID: %s\n", config.ssid);
    gfx->setCursor(10, 155);
    gfx->printf("Universe: %d\n", config.universe);
    gfx->setCursor(10, 170);
    gfx->printf("DMX Address: %d\n", config.dmxAddress);

    gfx->setTextColor(COLOR_GREEN);
    gfx->setCursor(10, 200);
    gfx->println("Connecting to WiFi...");
}

void updateDisplay() {
    gfx->fillScreen(COLOR_BLACK);

    // Set colors and text based on state
    uint16_t backgroundColor = COLOR_BLACK;
    const char *stateText = "OFF";
    uint16_t textColor = COLOR_WHITE;

    switch (currentState) {
        case STATE_WAITING_FOR_DMX:
            backgroundColor = COLOR_BLACK;
            stateText = "Waiting for DMX";
            textColor = COLOR_WHITE;
            break;
        case STATE_OFF:
            backgroundColor = COLOR_BLACK;
            stateText = "OFF";
            textColor = COLOR_WHITE;
            break;
        case STATE_SET:
            backgroundColor = COLOR_RED;
            stateText = "READY";
            textColor = COLOR_WHITE;
            break;
        case STATE_READY:
            backgroundColor = COLOR_YELLOW;
            stateText = "SET";
            textColor = COLOR_BLACK;
            break;
        case STATE_GO:
            backgroundColor = COLOR_GREEN;
            stateText = "GO";
            textColor = COLOR_BLACK;
            break;
    }

    // Fill screen with state color
    gfx->fillScreen(backgroundColor);

    // Display name at top (only if enabled via DMX)
    if (showName) {
        gfx->setTextColor(textColor);
        gfx->setTextSize(2);
        gfx->setCursor(10, 10);
        gfx->println(config.names[currentNameIndex]);
    }

    // Display state text - centered, larger
    gfx->setTextColor(textColor);
    gfx->setTextSize(4);

    // Calculate text position for centering
    int16_t x1, y1;
    uint16_t w, h;
    gfx->getTextBounds((char *)stateText, 0, 0, &x1, &y1, &w, &h);
    int16_t xPos = (gfx->width() - w) / 2;
    int16_t yPos = (gfx->height() - h) / 2 + 30;

    gfx->setCursor(xPos, yPos);
    gfx->println(stateText);

    Serial.printf("Display updated - State: %s, ShowName: %d, Name: %s, Background: 0x%04X\n", stateText, showName, config.names[currentNameIndex], backgroundColor);
}

void displayConfigMode() {
    gfx->fillScreen(COLOR_BLACK);
    gfx->setTextColor(COLOR_RED);
    gfx->setTextSize(2);
    gfx->setCursor(10, 40);
    gfx->println("Config Mode");
    gfx->setTextSize(1);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setCursor(10, 90);
    gfx->println("Access web interface at:");
    gfx->setCursor(10, 110);
    gfx->println("http://192.168.1.100");
    gfx->setCursor(10, 130);
    gfx->println("(check serial for actual IP)");
}

void displayWaitingForDMX() {
    gfx->fillScreen(COLOR_BLACK);
    gfx->setTextColor(COLOR_WHITE);
    gfx->setTextSize(3);
    
    // Calculate text position for centering
    int16_t x1, y1;
    uint16_t w, h;
    const char *waitText = "Waiting for";
    gfx->getTextBounds((char *)waitText, 0, 0, &x1, &y1, &w, &h);
    int16_t xPos = (gfx->width() - w) / 2;
    
    gfx->setCursor(xPos, 80);
    gfx->println("Waiting for");
    
    const char *dmxText = "DMX data";
    gfx->getTextBounds((char *)dmxText, 0, 0, &x1, &y1, &w, &h);
    xPos = (gfx->width() - w) / 2;
    
    gfx->setCursor(xPos, 130);
    gfx->println("DMX data");
}

// ============================================
// Cleanup Functions
// ============================================
void closeUDP() {
    pollListenerUdp.stop();
}

// ============================================
// Configuration Functions
// ============================================
void loadConfig() {
    // Default configuration
    strcpy(config.ssid, "YourSSID");
    strcpy(config.password, "YourPassword");
    config.universe = 0;
    config.dmxAddress = 1;
    
    // Initialize default names
    for (int i = 0; i < 12; i++) {
        sprintf(config.names[i], "Name %d", i + 1);
    }

    // Try to load from SPIFFS
    if (SPIFFS.exists(CONFIG_FILE)) {
        File configFile = SPIFFS.open(CONFIG_FILE, "r");
        if (configFile) {
            StaticJsonDocument<1024> doc;
            DeserializationError error = deserializeJson(doc, configFile);
            configFile.close();

            if (!error) {
                strcpy(config.ssid, doc["ssid"] | "YourSSID");
                strcpy(config.password, doc["password"] | "YourPassword");
                config.universe = doc["universe"] | 0;
                config.dmxAddress = doc["dmxAddress"] | 1;
                
                // Load names
                if (doc.containsKey("names")) {
                    JsonArray namesArray = doc["names"];
                    for (int i = 0; i < 12 && i < namesArray.size(); i++) {
                        strcpy(config.names[i], namesArray[i] | "Name X");
                    }
                }
                Serial.println("Configuration loaded from SPIFFS");
            }
        }
    }

    // Calculate DMX channels from base address
    DMX_ADDRESS = config.dmxAddress;
    DMX_CH_CUESTATE_RUNTIME = DMX_ADDRESS - 1;      // Convert to 0-based index
    DMX_CH_NAME_SELECT_RUNTIME = DMX_ADDRESS;       // Next channel
    DMX_CH_BACKLIGHT_RUNTIME = DMX_ADDRESS + 1;     // Next channel

    Serial.printf("Config - SSID: %s, Universe: %d, DMX Address: %d (Channels: CueState=%d, NameSelect=%d, Backlight=%d)\n",
                 config.ssid, config.universe, DMX_ADDRESS, DMX_CH_CUESTATE_RUNTIME + 1, DMX_CH_NAME_SELECT_RUNTIME + 1, DMX_CH_BACKLIGHT_RUNTIME + 1);
}

void saveConfig() {
    StaticJsonDocument<1024> doc;
    doc["ssid"] = config.ssid;
    doc["password"] = config.password;
    doc["universe"] = config.universe;
    doc["dmxAddress"] = config.dmxAddress;
    
    // Save names array
    JsonArray namesArray = doc.createNestedArray("names");
    for (int i = 0; i < 12; i++) {
        namesArray.add(config.names[i]);
    }

    File configFile = SPIFFS.open(CONFIG_FILE, "w");
    if (configFile) {
        serializeJson(doc, configFile);
        configFile.close();
        Serial.println("Configuration saved to SPIFFS");
    }
}

// ============================================
// WiFi Functions
// ============================================
void connectToWiFi() {
    Serial.printf("Connecting to WiFi: %s\n", config.ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.ssid, config.password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.printf("IP address: %s\n", WiFi.localIP().toString().c_str());
        gfx->fillScreen(COLOR_BLACK);
        gfx->setTextColor(COLOR_GREEN);
        gfx->setTextSize(1);
        gfx->setCursor(10, 100);
        gfx->println("WiFi Connected!");
        gfx->setCursor(10, 120);
        gfx->println(WiFi.localIP().toString().c_str());
        delay(2000);
    } else {
        Serial.println("\nFailed to connect to WiFi");
        gfx->fillScreen(COLOR_BLACK);
        gfx->setTextColor(COLOR_RED);
        gfx->setTextSize(1);
        gfx->setCursor(10, 100);
        gfx->println("WiFi Connection Failed!");
        gfx->setCursor(10, 120);
        gfx->println("Check credentials");
        delay(2000);
    }
}

// ============================================
// Web Server Handlers
// ============================================
void handleRoot() {
    String html = "<!DOCTYPE html><html><head><title>CYD Prompter</title>";
    html += "<style>";
    html += "body { font-family: Arial; margin: 20px; }";
    html += ".container { max-width: 800px; margin: 0 auto; }";
    html += "h1 { color: #333; }";
    html += "h2 { color: #555; margin-top: 30px; border-bottom: 2px solid #ddd; padding-bottom: 10px; }";
    html += ".name-input { display: block; margin: 10px 0; padding: 8px; width: 300px; font-size: 16px; }";
    html += "button { padding: 10px 20px; font-size: 16px; background-color: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; margin: 5px 0; }";
    html += "button:hover { background-color: #45a049; }";
    html += ".status { margin: 20px 0; padding: 10px; background-color: #f0f0f0; border-radius: 4px; }";
    html += ".name-group { margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 4px; }";
    html += ".config-group { margin: 20px 0; padding: 15px; border: 1px solid #4CAF50; border-radius: 4px; background-color: #f0f8f0; }";
    html += ".config-input { display: block; margin: 10px 0; padding: 8px; width: 150px; font-size: 16px; }";
    html += ".config-label { font-weight: bold; margin-bottom: 5px; display: block; }";
    html += ".config-help { font-size: 12px; color: #666; margin-top: 5px; }";
    html += "</style></head><body>";
    html += "<div class=\"container\">";
    html += "<h1>CYD Prompter Configuration</h1>";
    html += "<div class=\"status\">";
    html += "<p><strong>Current State:</strong> <span id=\"state\">Loading...</span></p>";
    html += "<p><strong>Current Name:</strong> <span id=\"name\">Loading...</span></p>";
    html += "<p><strong>Backlight Level:</strong> <span id=\"backlight\">Loading...</span>/255</p>";
    html += "</div>";
    html += "<h2>DMX Channel Configuration</h2>";
    html += "<div class=\"config-group\">";
    html += "<label class=\"config-label\">Base DMX Address:</label>";
    html += "<input type=\"number\" id=\"dmxAddressInput\" min=\"1\" max=\"510\" class=\"config-input\" placeholder=\"1-510\"/>";
    html += "<div class=\"config-help\">Cue State: Address | Name Select: Address+1 | Backlight: Address+2</div>";
    html += "<button onclick=\"saveDmxConfig()\">Save DMX Configuration</button>";
    html += "</div>";
    html += "<h2>Edit Names (0-11)</h2>";
    html += "<div id=\"namesContainer\"></div>";
    html += "<button onclick=\"saveAllNames()\">Save All Names</button>";
    html += "</div>";
    html += "<script>";
    html += "function loadDmxConfig(){fetch('/api/dmx-config').then(r=>r.json()).then(d=>{document.getElementById('dmxAddressInput').value=d.dmxAddress||1;}).catch(e=>console.error('Load config error:',e));}";
    html += "function saveDmxConfig(){const addr=parseInt(document.getElementById('dmxAddressInput').value)||1;if(addr<1||addr>510){alert('DMX Address must be between 1 and 510');return;}fetch('/api/dmx-config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({dmxAddress:addr})}).then(r=>r.json()).then(d=>{alert('DMX configuration saved! Device settings updated.');}).catch(e=>{console.error('Save config error:',e);alert('Failed to save configuration');});}";
    html += "function loadNames(){fetch('/api/names').then(r=>r.json()).then(d=>{const c=document.getElementById('namesContainer');c.innerHTML='';d.names.forEach((n,i)=>{const v=document.createElement('div');v.className='name-group';v.innerHTML='<label><strong>Name '+i+':</strong></label><input type=\"text\" id=\"name'+i+'\" class=\"name-input\" value=\"'+n+'\" maxlength=\"31\">';c.appendChild(v);});}).catch(e=>console.error('Error:',e));}";
    html += "function saveAllNames(){const n=[];for(let i=0;i<12;i++){n.push(document.getElementById('name'+i).value);}fetch('/api/names/update',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({names:n})}).then(r=>r.json()).then(d=>{if(d.success){alert('Names saved successfully!');}else{alert('Error saving names: '+d.error);}}).catch(e=>{console.error('Error:',e);alert('Error saving names');});}";
    html += "function updateStatus(){fetch('/api/names').then(r=>r.json()).then(d=>{document.getElementById('state').textContent=d.state;document.getElementById('name').textContent=d.names[d.currentNameIndex];document.getElementById('backlight').textContent=d.backlightDMXValue;}).catch(e=>console.error('Error:',e));}";
    html += "window.onload=function(){loadDmxConfig();loadNames();updateStatus();setInterval(updateStatus,2000);};";
    html += "</script>";
    html += "</body></html>";
    webServer.send(200, "text/html", html);
}

void handleGetNames() {
    // Get current display state, names, and DMX values
    DynamicJsonDocument doc(2048);
    JsonArray namesArray = doc.createNestedArray("names");
    
    for (int i = 0; i < 12; i++) {
        namesArray.add(config.names[i]);
    }
    
    // Add current display state info
    doc["state"] = getStateName(currentState);
    doc["currentNameIndex"] = currentNameIndex;
    doc["backlightDMXValue"] = backlightDMXValue;
    
    String jsonString;
    serializeJson(doc, jsonString);
    webServer.send(200, "application/json", jsonString);
}

void handleGetDmxConfig() {
    // Get current DMX configuration
    DynamicJsonDocument doc(256);
    doc["dmxAddress"] = config.dmxAddress;
    doc["dmxChCuestate"] = DMX_CH_CUESTATE_RUNTIME;
    doc["dmxChNameSelect"] = DMX_CH_NAME_SELECT_RUNTIME;
    doc["dmxChBacklight"] = DMX_CH_BACKLIGHT_RUNTIME;
    
    String jsonString;
    serializeJson(doc, jsonString);
    webServer.send(200, "application/json", jsonString);
}

void handleSetDmxConfig() {
    // Update DMX address configuration
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}");
        return;
    }
    
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, webServer.arg("plain"));
    
    if (error) {
        webServer.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
        return;
    }
    
    uint8_t newAddress = doc["dmxAddress"];
    
    // Validate address (1-510 to ensure Address+2 <= 512)
    if (newAddress < 1 || newAddress > 510) {
        webServer.send(400, "application/json", "{\"success\":false,\"error\":\"DMX Address must be 1-510\"}");
        return;
    }
    
    // Update config and save
    config.dmxAddress = newAddress;
    saveConfig();
    
    // Recalculate runtime variables
    DMX_ADDRESS = config.dmxAddress;
    DMX_CH_CUESTATE_RUNTIME = DMX_ADDRESS;
    DMX_CH_NAME_SELECT_RUNTIME = DMX_ADDRESS + 1;
    DMX_CH_BACKLIGHT_RUNTIME = DMX_ADDRESS + 2;
    
    Serial.printf("DMX address updated to %d (Ch%d, Ch%d, Ch%d)\n", 
        newAddress, DMX_CH_CUESTATE_RUNTIME, DMX_CH_NAME_SELECT_RUNTIME, DMX_CH_BACKLIGHT_RUNTIME);
    
    webServer.send(200, "application/json", "{\"success\":true,\"dmxAddress\":" + String(newAddress) + "}");
}

void handleUpdateName() {
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}");
        return;
    }
    
    String body = webServer.arg("plain");
    StaticJsonDocument<1024> doc;
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
        webServer.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
        return;
    }
    
    if (!doc.containsKey("names")) {
        webServer.send(400, "application/json", "{\"success\":false,\"error\":\"Missing names array\"}");
        return;
    }
    
    JsonArray namesArray = doc["names"];
    for (int i = 0; i < 12 && i < namesArray.size(); i++) {
        strcpy(config.names[i], namesArray[i] | "Name X");
    }
    
    saveConfig();
    webServer.send(200, "application/json", "{\"success\":true}");
}
