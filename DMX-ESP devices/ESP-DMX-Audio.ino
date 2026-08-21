// ESP32 DFPlayer Mini MP3 Player - Art-Net DMX Control
// Uses: ESP32-WROOM DevKit1, DFRobot Mini MP3 Player Module (HW-247A)
// Library: DFRobotDFPlayerMini (install via Arduino IDE or PlatformIO)
// Control: Art-Net Protocol (DMX over Ethernet via UDP)

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <DFRobotDFPlayerMini.h>

// ==================== Configuration ====================
#define MP3_SERIAL Serial2
#define CONFIG_FILE "/config.json"
#define TRACKLIST_FILE "/tracklist.json"
#define LED_PIN 2  // Blue LED on GPIO 2 (change if using different pin)

// ==================== Art-Net Configuration ====================
const uint16_t ARTNET_PORT = 6454;
const uint8_t DMX_UNIVERSE = 0;  // Universe 0 (can be changed for multiple universes)
const size_t ARTNET_BUFFER_SIZE = 530;  // Art-Net packet max size

// DMX Channel Mapping (calculated from base address)
uint8_t DMX_ADDRESS = 1;        // Base DMX address
uint8_t DMX_CH_VOLUME = 1;      // Channel = Address (Volume: 0-255 → 0-30)
uint8_t DMX_CH_TRACK = 2;       // Channel = Address + 1 (Track: 0-255 → 1-255)
uint8_t DMX_CH_CONTROL = 3;     // Channel = Address + 2 (Control commands)
const uint8_t DMX_CH_STATUS = 4;      // Channel 4: Status feedback (output only)

// Art-Net packet structure offsets
const uint8_t ARTNET_ID_OFFSET = 0;    // "Art-Net" ID at offset 0
const uint8_t ARTNET_OPCODE_OFFSET = 8;  // OpCode at offset 8-9
const uint8_t ARTNET_UNIVERSE_OFFSET = 14;  // Universe at offset 14-15
const uint8_t ARTNET_LENGTH_OFFSET = 16;    // Data length at offset 16-17
const uint8_t ARTNET_DMX_OFFSET = 18;       // DMX data starts at offset 18

struct Config {
    char ssid[64];
    char password[64];
    uint8_t dmxAddress;  // Base DMX address (channels will be address, address+1, address+2)
};

Config config;
WebServer webServer(80);
DFRobotDFPlayerMini dfPlayer;
WiFiUDP udpServer;

// ==================== Art-Net UDP State ====================
uint8_t dmxBuffer[512] = {0};  // DMX512 data (512 channels)
uint8_t incomingPacket[ARTNET_BUFFER_SIZE];
size_t incomingPacketSize = 0;

// ==================== DFPlayer State ====================
enum PlayState { STOPPED, PLAYING, PAUSED };
PlayState playState = STOPPED;

int currentTrack = 0;
int totalTracks = 0;
uint8_t currentVolume = 15;  // 0-30

// ==================== DMX Control State ====================
uint8_t lastDmxVolume = 255;   // Initialize to invalid value to catch first change
uint8_t lastDmxTrack = 255;    // Initialize to invalid value to catch first change
uint8_t lastDmxControl = 255;  // Initialize to invalid value to catch first change
uint8_t selectedTrack = 1;     // Track selected via DMX CH2 (doesn't auto-play)
unsigned long lastDmxUpdate = 0;
const unsigned long DMX_TIMEOUT = 5000;  // Timeout if no DMX data received

// WiFi LED state tracking (to prevent flickering)
wl_status_t lastWiFiStatus = WL_DISCONNECTED;
unsigned long lastLEDUpdate = 0;

// Diagnostic counters/variables
unsigned long lastSerialActivity = 0;
unsigned long noResponseCount = 0;
unsigned long lastStatusQuery = 0;
const unsigned long STATUS_QUERY_INTERVAL = 2000;
bool fileCountDetected = false;  // Flag to detect file count once in loop
unsigned long fileCountDetectionTime = 0;  // Track when we first try to detect file count
bool dfPlayerBusy = false;  // Flag to prevent serial conflicts between control and status queries
unsigned long dfPlayerBusyTime = 0;  // Timeout for busy flag

// Art-Net status
bool artnetConnected = false;
unsigned long lastArtnetData = 0;

// DFPlayer command rate limiting
// Volume: 9600 baud UART can only sustain ~20 cmds/sec; DFPlayer needs ~50ms between volume commands
// Control: DFPlayer needs ~300ms after play/stop/pause before accepting another command
const unsigned long VOLUME_RATE_LIMIT_MS = 80;    // Max ~12 volume updates/sec
const unsigned long CONTROL_COOLDOWN_MS  = 350;   // Min gap between control commands
unsigned long lastVolumeCommandTime = 0;
unsigned long lastControlCommandTime = 0;
uint8_t pendingVolume = 255;   // Latest unmapped volume from DMX (255 = none pending)

void setup()
{
    Serial.begin(115200);
    delay(1000);  // Give serial time to initialize
    
    logMessage("\n\n=== ESP32 DFPlayer Mini MP3 Player ===");
    logMessage("Initializing systems...");
    
    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        logMessage("ERROR: SPIFFS failed");
    } else {
        logMessage("OK: SPIFFS initialized");
    }
    
    loadConfig();
    
    // Initialize LED pin
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    logMessage("OK: LED initialized on GPIO2");
    
    // Initialize MP3 module serial (UART2)
    logMessage("Initializing MP3 module serial (UART2)...");
    logMessage("  RX = GPIO16, TX = GPIO17");
    
    MP3_SERIAL.begin(9600, SERIAL_8N1, 16, 17);
    delay(1000);  // Wait for module to initialize
    
    logMessage("Initializing DFPlayer Mini library...");
    if (!dfPlayer.begin(MP3_SERIAL, true)) {
        logMessage("ERROR: DFPlayer initialization failed!");
        logMessage("  Check: (1) RX/TX connections (2) Module power (3) SD card presence");
    } else {
        logMessage("OK: DFPlayer initialized successfully");
        
        // Set initial volume (with longer delay for module to be ready)
        logMessage("Waiting for module initialization...");
        delay(1500);  // Give module more time to fully initialize
        
        logMessage("Setting initial volume to 15...");
        dfPlayer.volume(15);
        currentVolume = 15;
        delay(500);
        
        // File count will be read in loop() to avoid blocking setup
        logMessage("Will detect available tracks in main loop...");
    }
    
    logMessage("Setup complete. Starting WiFi and web server...");
    
    // Initialize WiFi and web server
    initWiFi();
    
    // Initialize Art-Net after WiFi is connected
    initArtNet();
    
    setupWebServer();
    
    logMessage("All systems ready!");
}

void loop()
{
    // Update WiFi LED status
    updateWiFiLED();
    
    // One-time file count detection (after module has initialized)
    if (!fileCountDetected && millis() > 5000) {  // Give module 5 seconds to init
        if (fileCountDetectionTime == 0) {
            fileCountDetectionTime = millis();
            logMessage("Detecting available tracks...");
            totalTracks = dfPlayer.readFileCounts();
        }
        
        // Check if we got a valid response
        if (totalTracks > 0) {
            logMessagef("OK: Detected %d MP3 files on SD card", totalTracks);
            fileCountDetected = true;
        } else if (millis() - fileCountDetectionTime > 3000) {
            // Timeout after 3 seconds - either no SD card or module not responding
            logMessage("WARNING: Could not detect file count - SD card may not be inserted");
            fileCountDetected = true;  // Mark as attempted
            totalTracks = 255;  // Default to max tracks possible
        }
    }
    
    // Handle web server
    webServer.handleClient();
    
    // Process one Art-Net UDP packet per loop (yield lets network stack refill between calls)
    if (receiveArtNetPacket()) {
        lastArtnetData = millis();
        if (!artnetConnected) {
            artnetConnected = true;
            logMessage("✓ Art-Net signal acquired");
        }
        
        // Read DMX channel values from buffer
        uint8_t newVolume = dmxBuffer[DMX_CH_VOLUME - 1];
        uint8_t newTrack = dmxBuffer[DMX_CH_TRACK - 1];
        uint8_t newControl = dmxBuffer[DMX_CH_CONTROL - 1];
        
        // DEBUG: Log all DMX channel values to verify they're being read
        static unsigned long lastDmxLog = 0;
        if (millis() - lastDmxLog > 5000) {  // Log every 5 seconds
            logMessagef("DMX RAW: Vol=%d, Track=%d, Control=%d | Last: Vol=%d, Track=%d, Control=%d", 
                newVolume, newTrack, newControl, lastDmxVolume, lastDmxTrack, lastDmxControl);
            logMessagef("  dmxBuffer[0]=%d, dmxBuffer[1]=%d, dmxBuffer[2]=%d (raw bytes)", 
                dmxBuffer[0], dmxBuffer[1], dmxBuffer[2]);
            lastDmxLog = millis();
        }
        
        // Control channel: detect zone change, then enforce cooldown before sending to DFPlayer.
        // This prevents rapid fader sweeps from queuing conflicting play/stop commands.
        if (newControl != lastDmxControl) {
            lastDmxControl = newControl;
            unsigned long now = millis();
            if (now - lastControlCommandTime >= CONTROL_COOLDOWN_MS) {
                logMessagef(">>> CONTROL CHANGE DETECTED! newControl=%d", newControl);
                dfPlayerBusy = true;
                dfPlayerBusyTime = now;
                processDmxControl(newControl);
                lastControlCommandTime = now;
                logMessagef("DMX Ch3: Control value changed to %d", newControl);
                dfPlayerBusy = false;
            } else {
                logMessagef("DMX Ch3: Control=%d ignored (cooldown %lums remaining)",
                    newControl, CONTROL_COOLDOWN_MS - (now - lastControlCommandTime));
            }
        }
        
        // Volume: track the latest value but only push to DFPlayer at a limited rate.
        // Flooding the DFPlayer serial port with 44 volume commands/sec overflows its buffer.
        if (newVolume != lastDmxVolume) {
            lastDmxVolume = newVolume;
            pendingVolume = newVolume;  // Always store latest; send when rate window opens
        }
        if (pendingVolume != 255 && (millis() - lastVolumeCommandTime) >= VOLUME_RATE_LIMIT_MS) {
            uint8_t mappedVolume = map(pendingVolume, 0, 255, 0, 30);
            mp3SetVolume(mappedVolume);
            logMessagef("DMX Ch1: Volume sent - raw: %d, mapped: %d/30", pendingVolume, mappedVolume);
            lastVolumeCommandTime = millis();
            pendingVolume = 255;  // Clear pending
        }
        
        // Track selection ONLY - does NOT trigger playback
        // Playback is triggered by Control Channel (CH3) PLAY command
        if (newTrack != lastDmxTrack) {
            lastDmxTrack = newTrack;
            if (newTrack > 0) {
                selectedTrack = newTrack;
                logMessagef("DMX Ch2: Track selected = %d (will play when CH3 PLAY command received)", newTrack);
            } else {
                logMessagef("DMX Ch2: Track value changed to %d (no action, track 0 is invalid)", newTrack);
            }
        }
    }
    
    // Flush any pending volume update (applied outside the packet block so it
    // sends even when no new Art-Net packet arrives in this loop iteration)
    if (pendingVolume != 255 && (millis() - lastVolumeCommandTime) >= VOLUME_RATE_LIMIT_MS) {
        uint8_t mappedVolume = map(pendingVolume, 0, 255, 0, 30);
        mp3SetVolume(mappedVolume);
        logMessagef("DMX Ch1: Volume sent (deferred) - raw: %d, mapped: %d/30", pendingVolume, mappedVolume);
        lastVolumeCommandTime = millis();
        pendingVolume = 255;
    }
    
    // Check for Art-Net timeout
    if (artnetConnected && (millis() - lastArtnetData) > DMX_TIMEOUT) {
        artnetConnected = false;
        logMessage("WARNING: Art-Net connection timeout - no DMX data received");
    }
    
    // Handle async DFPlayer events (non-blocking: only process if data is ready)
    if (!dfPlayerBusy && dfPlayer.available()) {
        uint16_t messages = dfPlayer.readType();
        uint16_t value = dfPlayer.read();
        
        switch (messages) {
            case TimeOut:
                logMessage("TIMEOUT: DFPlayer did not respond");
                break;
            case WrongStack:
                logMessage("ERROR: Wrong stack");
                break;
            case DFPlayerCardInserted:
                logMessage("INFO: SD card inserted");
                lastStatusQuery = 0;  // Force file count refresh
                break;
            case DFPlayerCardRemoved:
                logMessage("WARNING: SD card removed");
                totalTracks = 0;
                break;
            case DFPlayerPlayFinished:
                logMessage("INFO: Playback finished");
                playState = STOPPED;
                currentTrack = 0;
                break;
            case DFPlayerError:
                logMessagef("ERROR code: %d", value);
                break;
        }
    }
    
    yield();
}

// ==================== Logging System ====================
// Circular buffer for storing log messages (last 50 messages)
#define MAX_LOGS 50
struct LogEntry {
    unsigned long timestamp;
    String message;
};
LogEntry logs[MAX_LOGS];
uint16_t logIndex = 0;

void logMessage(const String& message) {
    logs[logIndex].timestamp = millis();
    logs[logIndex].message = message;
    logIndex = (logIndex + 1) % MAX_LOGS;
    Serial.println(message);
}

void logMessagef(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    // Avoid String allocation - log directly to serial
    logs[logIndex].timestamp = millis();
    logs[logIndex].message = String(buffer);  // Convert only once for buffer storage
    logIndex = (logIndex + 1) % MAX_LOGS;
    Serial.println(buffer);  // Print raw buffer to serial
}

// ==================== MP3 Control Functions ====================

void mp3Play()
{
    logMessage(">>> ACTION: Play");
    // Resume from current track if one is set, otherwise play track 1
    if (currentTrack > 0) {
        dfPlayer.playMp3Folder(currentTrack);
    } else {
        dfPlayer.playMp3Folder(1);
        currentTrack = 1;
    }
    playState = PLAYING;
}

void mp3Pause()
{
    logMessage(">>> ACTION: Pause");
    dfPlayer.pause();
    playState = PAUSED;
}

void mp3Stop()
{
    logMessage(">>> ACTION: Stop");
    dfPlayer.stop();
    playState = STOPPED;
    currentTrack = 0;
}

void mp3PlayTrack(int track)
{
    if (track < 1) track = 1;
    if (track > 255) track = 255;
    
    // Use playMp3Folder() so the DFPlayer selects by filename (001.mp3, 002.mp3...)
    // rather than by the order files were copied to the SD card.
    // Requires MP3 files to be inside a folder named "MP3" on the SD card root.
    logMessagef(">>> ACTION: Play track %d (file: MP3/%03d.mp3)", track, track);
    dfPlayer.playMp3Folder(track);
    currentTrack = track;
    playState = PLAYING;
}

void mp3SetVolume(uint8_t volume)
{
    // Clamp to 0-30 range
    if (volume > 30) volume = 30;
    
    logMessagef(">>> ACTION: Set volume to %d/30", volume);
    dfPlayer.volume(volume);
    currentVolume = volume;
}

// ==================== Configuration ====================
void loadConfig()
{
    logMessage("Loading config...");
    if (!SPIFFS.exists(CONFIG_FILE)) {
        strncpy(config.ssid, "HomeWifi", sizeof(config.ssid) - 1);
        strncpy(config.password, "password", sizeof(config.password) - 1);
        config.dmxAddress = 1;
        saveConfig();
    } else {
        File file = SPIFFS.open(CONFIG_FILE, "r");
        StaticJsonDocument<256> doc;
        deserializeJson(doc, file);
        strncpy(config.ssid, doc["ssid"] | "HomeWifi", sizeof(config.ssid) - 1);
        strncpy(config.password, doc["password"] | "password", sizeof(config.password) - 1);
        config.dmxAddress = doc["dmxAddress"] | 1;
        file.close();
    }
    // Calculate channel values from base address
    DMX_ADDRESS = config.dmxAddress;
    DMX_CH_VOLUME = DMX_ADDRESS;
    DMX_CH_TRACK = DMX_ADDRESS + 1;
    DMX_CH_CONTROL = DMX_ADDRESS + 2;
    
    logMessagef("SSID: %s", config.ssid);
    logMessagef("DMX Base Address: %d (Channels: Volume=%d, Track=%d, Control=%d)", DMX_ADDRESS, DMX_CH_VOLUME, DMX_CH_TRACK, DMX_CH_CONTROL);
}

void saveConfig()
{
    File file = SPIFFS.open(CONFIG_FILE, "w");
    StaticJsonDocument<256> doc;
    doc["ssid"] = config.ssid;
    doc["password"] = config.password;
    doc["dmxAddress"] = config.dmxAddress;
    serializeJson(doc, file);
    file.close();
    logMessage("Config saved");
}

// ==================== WiFi ====================
void initWiFi()
{
    logMessagef("Connecting to WiFi: %s", config.ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.ssid, config.password);
    
    int attempts = 20;
    while (WiFi.status() != WL_CONNECTED && attempts-- > 0) {
        delay(500);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        // Disable WiFi power-save mode - critical for Art-Net responsiveness.
        // Without this the radio sleeps between beacon intervals (~100ms),
        // causing it to miss the majority of 44Hz Art-Net packets.
        WiFi.setSleep(false);
        // Max TX power for best signal stability
        WiFi.setTxPower(WIFI_POWER_19_5dBm);
        digitalWrite(LED_PIN, HIGH);  // Turn ON LED when connected
        logMessagef("✓ WiFi connected! IP: %s (power-save disabled)", WiFi.localIP().toString().c_str());
    } else {
        digitalWrite(LED_PIN, LOW);  // Turn OFF LED when not connected
        logMessage("✗ WiFi connection failed - check credentials");
    }
}

void updateWiFiLED()
{
    // Only update LED if WiFi status has actually changed (reduces flickering)
    wl_status_t currentStatus = WiFi.status();
    
    if (currentStatus != lastWiFiStatus) {
        lastWiFiStatus = currentStatus;
        lastLEDUpdate = millis();
        
        if (currentStatus == WL_CONNECTED) {
            digitalWrite(LED_PIN, HIGH);  // LED ON when connected
            logMessage("LED: WiFi connected - turning ON");
        } else {
            digitalWrite(LED_PIN, LOW);   // LED OFF when disconnected
            logMessage("LED: WiFi disconnected - turning OFF");
        }
    }
}

// ==================== Art-Net DMX Control ====================

void initArtNet()
{
    logMessage("Initializing Art-Net receiver...");
    logMessagef("  Universe: %d, Port: %d", DMX_UNIVERSE, ARTNET_PORT);
    logMessagef("  IP: %s", WiFi.localIP().toString().c_str());
    
    if (!udpServer.begin(ARTNET_PORT)) {
        logMessage("ERROR: Failed to start UDP server on port 6454");
    } else {
        logMessage("✓ Art-Net UDP receiver initialized - listening on port 6454");
        logMessage("  DMX Channel Mapping:");
        logMessagef("    Ch %d: Volume (0-255 → 0-30)", DMX_CH_VOLUME);
        logMessagef("    Ch %d: Track (1-255)", DMX_CH_TRACK);
        logMessagef("    Ch %d: Control Commands", DMX_CH_CONTROL);
    }
}

bool receiveArtNetPacket()
{
    // Check if UDP data is available
    int packetSize = udpServer.parsePacket();
    if (packetSize == 0) {
        return false;  // No packet available
    }
    
    // Limit to buffer size
    if (packetSize > ARTNET_BUFFER_SIZE) {
        packetSize = ARTNET_BUFFER_SIZE;
    }
    
    // Read incoming data
    int bytesRead = udpServer.read(incomingPacket, packetSize);
    if (bytesRead < 36) {  // Minimum Art-Net packet size
        logMessagef("ArtNet: Packet too small (%d bytes)", bytesRead);
        return false;
    }
    
    // DEBUG: First packet or every 30 seconds, show raw header
    static unsigned long lastHeaderLog = 0;
    static bool firstPacket = true;
    if (firstPacket || (millis() - lastHeaderLog > 30000)) {
        logMessagef("ArtNet HEADER (first 18 bytes): %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
            incomingPacket[0], incomingPacket[1], incomingPacket[2], incomingPacket[3],
            incomingPacket[4], incomingPacket[5], incomingPacket[6], incomingPacket[7],
            incomingPacket[8], incomingPacket[9], incomingPacket[10], incomingPacket[11],
            incomingPacket[12], incomingPacket[13], incomingPacket[14], incomingPacket[15],
            incomingPacket[16], incomingPacket[17]);
        lastHeaderLog = millis();
        firstPacket = false;
    }
    
    // Validate Art-Net packet header
    // "Art-Net\0" at offset 0
    if (incomingPacket[0] != 'A' || incomingPacket[1] != 'r' || 
        incomingPacket[2] != 't' || incomingPacket[3] != '-' ||
        incomingPacket[4] != 'N' || incomingPacket[5] != 'e' || 
        incomingPacket[6] != 't' || incomingPacket[7] != 0x00) {
        logMessage("ArtNet: Invalid header");
        return false;  // Invalid Art-Net ID
    }
    
    // Check OpCode (should be 0x5000 = DMX data)
    uint16_t opCode = (incomingPacket[9] << 8) | incomingPacket[8];
    if (opCode != 0x5000) {
        logMessagef("ArtNet: Wrong OpCode (0x%04X, expected 0x5000)", opCode);
        return false;  // Not a DMX data packet
    }
    
    // Check Universe
    uint16_t universe = (incomingPacket[15] << 8) | incomingPacket[14];
    if (universe != DMX_UNIVERSE) {
        logMessagef("ArtNet: Wrong universe (%d, expected %d)", universe, DMX_UNIVERSE);
        return false;  // Wrong universe
    }
    
    // Extract DMX data length
    uint16_t dataLength = (incomingPacket[17] << 8) | incomingPacket[16];
    
    // Calculate the highest channel we need to read
    uint8_t maxChannelNeeded = DMX_CH_CONTROL;
    if (dataLength < maxChannelNeeded) {
        logMessagef("WARNING: Art-Net packet has dataLength=%d (less than %d needed for configured channels). Forcing to %d.", dataLength, maxChannelNeeded, maxChannelNeeded);
        dataLength = maxChannelNeeded;
    }
    if (dataLength > 512) {
        dataLength = 512;  // Cap at 512 channels
    }
    
    // Copy DMX data to buffer (starting at offset 18 in Art-Net packet)
    for (uint16_t i = 0; i < dataLength; i++) {
        dmxBuffer[i] = incomingPacket[ARTNET_DMX_OFFSET + i];
    }
    
    // DEBUG: Log detailed packet info every 10 seconds
    static unsigned long lastPacketLog = 0;
    if (millis() - lastPacketLog > 10000) {
        logMessagef("ArtNet PKT: Size=%d bytes, Universe=%d, DataLen=%d", bytesRead, universe, dataLength);
        logMessagef("  Packet Ch1=%d, Ch2=%d, Ch3=%d (offset 18-20 in packet)", 
            incomingPacket[ARTNET_DMX_OFFSET + 0], incomingPacket[ARTNET_DMX_OFFSET + 1], incomingPacket[ARTNET_DMX_OFFSET + 2]);
        logMessagef("  Buffer Ch1=%d, Ch2=%d, Ch3=%d (dmxBuffer[0-2])", 
            dmxBuffer[0], dmxBuffer[1], dmxBuffer[2]);
        lastPacketLog = millis();
    }
    
    return true;  // Valid Art-Net packet received
}

void processDmxControl(uint8_t controlValue)
{
    // DMX Control Command Values (Channel 3)
    // NOTE: Track selection happens via Channel 2 (selectedTrack variable)
    // Channel 3 only triggers ACTIONS
    //   0-50:     STOP
    //   51-100:   PLAY (plays track selected in CH2)
    //   101-150:  PAUSE
    //   151-200:  NEXT (increments current track)
    //   201-255:  PREVIOUS (decrements current track)
    
    if (controlValue <= 50) {
        logMessagef("DMX: STOP command received (value: %d)", controlValue);
        mp3Stop();
    }
    else if (controlValue <= 100) {
        logMessagef("DMX: PLAY command received (value: %d) - playing selected track %d", controlValue, selectedTrack);
        mp3PlayTrack(selectedTrack);  // Play the track selected via CH2
    }
    else if (controlValue <= 150) {
        logMessagef("DMX: PAUSE command received (value: %d)", controlValue);
        mp3Pause();
    }
    else if (controlValue <= 200) {
        logMessagef("DMX: NEXT track command received (value: %d)", controlValue);
        if (currentTrack < totalTracks) {
            mp3PlayTrack(currentTrack + 1);
            selectedTrack = currentTrack + 1;  // Update selectedTrack to match
        } else {
            mp3PlayTrack(1);  // Loop to first track
            selectedTrack = 1;
        }
    }
    else {
        logMessagef("DMX: PREVIOUS track command received (value: %d)", controlValue);
        if (currentTrack > 1) {
            mp3PlayTrack(currentTrack - 1);
            selectedTrack = currentTrack - 1;  // Update selectedTrack to match
        } else {
            mp3PlayTrack(totalTracks);  // Loop to last track
            selectedTrack = totalTracks;
        }
    }
}

// ==================== Web Server ====================

void setupWebServer()
{
    webServer.on("/", HTTP_GET, handleRoot);
    webServer.on("/api/status", HTTP_GET, handleStatusAPI);
    webServer.on("/api/play", HTTP_POST, handlePlayAPI);
    webServer.on("/api/pause", HTTP_POST, handlePauseAPI);
    webServer.on("/api/stop", HTTP_POST, handleStopAPI);
    webServer.on("/api/volume", HTTP_POST, handleVolumeAPI);
    webServer.on("/api/track", HTTP_POST, handleTrackAPI);
    webServer.on("/api/logs", HTTP_GET, handleLogsAPI);
    webServer.on("/api/config", HTTP_GET, handleGetConfigAPI);
    webServer.on("/api/config", HTTP_POST, handleSetConfigAPI);
    webServer.on("/api/tracklist", HTTP_GET, handleTracklistGetAPI);
    webServer.on("/api/tracklist", HTTP_POST, handleTracklistSetAPI);
    
    webServer.begin();
    logMessage("✓ Web server started on port 80");
}

void handleRoot()
{
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    html += "<title>DFPlayer MP3 Control</title>";
    html += "<style>";
    html += "* { margin: 0; padding: 0; box-sizing: border-box; }";
    html += "body { font-family: -apple-system, BlinkMacSystemFont, Segoe UI, Roboto, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; }";
    html += ".container { max-width: 500px; margin: 0 auto; background: white; border-radius: 12px; box-shadow: 0 10px 30px rgba(0,0,0,0.3); overflow: hidden; }";
    html += ".header { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 30px 20px; text-align: center; }";
    html += ".header h1 { font-size: 24px; margin-bottom: 5px; } .header p { font-size: 14px; opacity: 0.9; } .content { padding: 30px 20px; }";
    html += ".status-box { background: #f8f9fa; border-radius: 8px; padding: 20px; margin-bottom: 30px; border-left: 4px solid #667eea; }";
    html += ".status-item { margin: 10px 0; font-size: 14px; } .status-label { font-weight: 600; color: #333; } .status-value { color: #667eea; font-size: 18px; font-weight: bold; }";
    html += ".control-group { margin-bottom: 25px; } .control-label { font-weight: 600; margin-bottom: 10px; color: #333; }";
    html += ".button-group { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; margin-bottom: 15px; }";
    html += "button { padding: 12px; border: none; border-radius: 6px; font-size: 14px; font-weight: 600; cursor: pointer; transition: all 0.3s; }";
    html += ".btn-play { background: #51cf66; color: white; } .btn-play:hover { background: #40c057; }";
    html += ".btn-pause { background: #ffd43b; color: #333; } .btn-pause:hover { background: #ffec99; }";
    html += ".btn-stop { background: #ff8787; color: white; } .btn-stop:hover { background: #fa5252; }";
    html += "input[type=\"range\"] { width: 100%; height: 6px; border-radius: 3px; background: #e9ecef; outline: none; -webkit-appearance: none; }";
    html += "input[type=\"range\"]::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 18px; height: 18px; border-radius: 50%; background: #667eea; cursor: pointer; }";
    html += "input[type=\"range\"]::-moz-range-thumb { width: 18px; height: 18px; border-radius: 50%; background: #667eea; cursor: pointer; border: none; }";
    html += ".slider-label { display: flex; justify-content: space-between; margin-bottom: 8px; font-size: 14px; }";
    html += ".track-input { display: flex; gap: 10px; } .track-input input { flex: 1; padding: 10px; border: 1px solid #dee2e6; border-radius: 6px; font-size: 14px; }";
    html += ".track-input button { flex: 0 0 80px; background: #667eea; color: white; } .track-input button:hover { background: #5568d3; }";
    html += ".logs { background: #f8f9fa; border-radius: 8px; padding: 15px; max-height: 200px; overflow-y: auto; font-family: monospace; font-size: 12px; line-height: 1.4; color: #495057; }";
    html += ".log-item { margin: 5px 0; padding: 5px; border-left: 2px solid #dee2e6; padding-left: 10px; } .log-time { color: #667eea; font-weight: bold; }";
    html += ".tab-nav{display:flex;border-bottom:2px solid #dee2e6;background:#f8f9fa;} .tab-btn{flex:1;padding:13px;border:none;background:none;cursor:pointer;font-size:14px;font-weight:600;color:#888;border-bottom:3px solid transparent;margin-bottom:-2px;transition:all 0.2s;} .tab-btn:hover{color:#667eea;background:#f0f1ff;} .tab-active{color:#667eea !important;border-bottom-color:#667eea !important;background:white !important;}";
    html += ".track-row{display:flex;align-items:center;gap:8px;padding:8px 0;border-bottom:1px solid #f0f0f0;} .track-num{min-width:42px;font-weight:bold;color:#667eea;font-size:13px;text-align:right;padding-right:6px;font-family:monospace;} .track-filename{min-width:80px;color:#888;font-size:12px;font-family:monospace;} .track-name-input{flex:1;padding:5px 8px;border:1px solid #dee2e6;border-radius:4px;font-size:13px;} .track-play-btn{padding:5px 12px;background:#51cf66;color:white;border:none;border-radius:4px;cursor:pointer;font-size:13px;font-weight:bold;} .info-box{background:#fff8e1;border:1px solid #ffe082;border-radius:8px;padding:14px;font-size:12px;color:#795548;margin-bottom:16px;line-height:1.6;}";
    html += "</style></head><body><div class=\"container\"><div class=\"header\">";
    html += "<h1>Music DFPlayer MP3</h1><p>Web Control Interface</p></div>";
    html += "<div class=\"tab-nav\"><button id=\"tab-btn-control\" class=\"tab-btn tab-active\" onclick=\"showTab('control')\">&#9654; Control</button><button id=\"tab-btn-files\" class=\"tab-btn\" onclick=\"showTab('files')\">&#128192; Files</button></div>";
    html += "<div id=\"tab-control\" class=\"content\"><div class=\"status-box\">";
    html += "<div class=\"status-item\"><span class=\"status-label\">Status:</span><span class=\"status-value\" id=\"status\">-</span></div>";
    html += "<div class=\"status-item\"><span class=\"status-label\">Playing:</span><span class=\"status-value\" id=\"track\">-</span></div>";
    html += "<div class=\"status-item\"><span class=\"status-label\">Selected (CH2):</span><span class=\"status-value\" id=\"selectedTrack\">-</span></div>";
    html += "<div class=\"status-item\"><span class=\"status-label\">Volume:</span><span class=\"status-value\" id=\"volume\">-</span></div>";
    html += "<div class=\"status-item\"><span class=\"status-label\">WiFi:</span><span class=\"status-value\" id=\"wifi\">-</span></div>";
    html += "<div class=\"status-item\"><span class=\"status-label\">Art-Net:</span><span class=\"status-value\" id=\"artnet\">-</span></div>";
    html += "</div><div class=\"control-group\"><div class=\"control-label\">DMX Control Scheme</div><div style=\"background: #f8f9fa; border-radius: 8px; padding: 15px; font-size: 12px;\">";
    html += "<div style=\"margin-bottom: 8px;\"><strong>CH 1 (Volume):</strong> 0-255</div>";
    html += "<div style=\"margin-bottom: 8px;\"><strong>CH 2 (Track):</strong> 1-255 (selection only, no playback)</div>";
    html += "<div style=\"margin-bottom: 8px;\"><strong>CH 3 (Actions):</strong><br/>";
    html += "  0-50: STOP | 51-100: PLAY | 101-150: PAUSE<br/>";
    html += "  151-200: NEXT | 201-255: PREVIOUS</div>";
    html += "</div></div><div class=\"control-group\"><div class=\"control-label\">Playback</div><div class=\"button-group\">";
    html += "<button class=\"btn-play\" onclick=\"play()\">Play</button>";
    html += "<button class=\"btn-pause\" onclick=\"pause()\">Pause</button>";
    html += "<button class=\"btn-stop\" onclick=\"stop()\">Stop</button>";
    html += "</div></div><div class=\"control-group\"><div class=\"control-label\">DMX Channel Configuration</div><div style=\"background: #f8f9fa; border-radius: 8px; padding: 15px;\">";
    html += "<div style=\"margin-bottom: 12px;\"><label style=\"font-weight: 600; display: block; margin-bottom: 5px;\">Base DMX Address:</label>";
    html += "<input type=\"number\" id=\"dmxAddressInput\" min=\"1\" max=\"510\" style=\"width: 100%; padding: 8px; border: 1px solid #dee2e6; border-radius: 4px; font-size: 14px;\" placeholder=\"1-510\"/>";
    html += "<div style=\"font-size: 11px; color: #666; margin-top: 5px;\">Volume: Address | Track: Address+1 | Control: Address+2</div></div>";
    html += "<button onclick=\"saveDmxConfig()\" style=\"width: 100%; padding: 10px; background: #667eea; color: white; border: none; border-radius: 4px; font-weight: 600; cursor: pointer; transition: background 0.3s;\" onmouseover=\"this.style.background='#5568d3'\" onmouseout=\"this.style.background='#667eea'\">Save DMX Configuration</button>";
    html += "</div></div><div class=\"control-group\"><div class=\"slider-group\"><div class=\"slider-label\">";
    html += "<span>Volume</span><span id=\"volValue\">15/30</span></div>";
    html += "<input type=\"range\" id=\"volumeSlider\" min=\"0\" max=\"30\" value=\"15\" oninput=\"setVolume(this.value)\">";
    html += "</div></div><div class=\"control-group\"><div class=\"control-label\">Select Track</div><div class=\"track-input\">";
    html += "<input type=\"number\" id=\"trackNum\" min=\"1\" max=\"255\" value=\"1\" placeholder=\"Track #\">";
    html += "<button onclick=\"playTrack()\">Play</button></div></div>";
    html += "<div class=\"control-group\"><div class=\"control-label\">DMX Channels</div><div style=\"background: #f8f9fa; border-radius: 8px; padding: 15px; font-size: 13px;\">";
    html += "<div style=\"margin-bottom: 8px;\"><strong>Ch 1 (Volume):</strong> <span id=\"dmxCh1\" style=\"font-weight: bold; color: #667eea;\">-</span>/255</div>";
    html += "<div style=\"margin-bottom: 8px;\"><strong>Ch 2 (Track):</strong> <span id=\"dmxCh2\" style=\"font-weight: bold; color: #667eea;\">-</span>/255</div>";
    html += "<div style=\"margin-bottom: 12px;\"><strong>Ch 3 (Control):</strong> <span id=\"dmxCh3\" style=\"font-weight: bold; color: #667eea;\">-</span>/255 → <span id=\"dmxAction\" style=\"font-style: italic; color: #666;\"><em>-</em></span></div>";
    html += "<div><strong>Protocol:</strong> Art-Net (UDP Port 6454)</div>";
    html += "</div></div>";
    html += "<div class=\"control-group\"><div class=\"control-label\">System Log</div><div class=\"logs\" id=\"logs\"></div></div>";
    html += "</div>"; // end tab-control
    html += "<div id=\"tab-files\" class=\"content\" style=\"display:none\">";
    html += "<div class=\"control-group\">";
    html += "<div class=\"info-box\">";
    html += "<strong>&#128196; SD Card Files (MP3/ folder)</strong><br>";
    html += "MP3 files must be stored on the DFPlayer's SD card in a folder named <strong>MP3</strong>, ";
    html += "named <strong>001.mp3, 002.mp3, 003.mp3...</strong> etc.<br>";
    html += "The SD card is only accessible by physically removing it. ";
    html += "Use the names below to label each track for your reference.";
    html += "</div>";
    html += "<div style=\"display:flex;justify-content:space-between;align-items:center;margin-bottom:10px;\">";
    html += "<span class=\"control-label\">Track List</span>";
    html += "<div style=\"display:flex;gap:8px;\">";
    html += "<button onclick=\"loadTrackList()\" style=\"padding:6px 14px;background:#667eea;color:white;border:none;border-radius:4px;cursor:pointer;font-size:13px;font-weight:600;\">&#8635; Refresh</button>";
    html += "<button onclick=\"saveTrackNames()\" style=\"padding:6px 14px;background:#51cf66;color:white;border:none;border-radius:4px;cursor:pointer;font-size:13px;font-weight:600;\">&#128190; Save Names</button>";
    html += "</div></div>";
    html += "<div id=\"trackList\"><div style=\"color:#aaa;text-align:center;padding:30px;\">Click Refresh to load tracks</div></div>";
    html += "</div>";
    html += "</div>";
    html += "</div><script>";
    html += "function getControlAction(val){";
    html += "if(val==='-'||val===0||val==='0')return'Idle';";
    html += "const v=parseInt(val);";
    html += "if(v<=50)return'🛑 STOP';";
    html += "if(v<=100)return'▶️ PLAY';";
    html += "if(v<=150)return'⏸️ PAUSE';";
    html += "if(v<=200)return'⏭️ NEXT';";
    html += "return'⏮️ PREVIOUS';}";
    html += "function loadDmxConfig(){fetch('/api/config').then(r=>r.json()).then(d=>{document.getElementById('dmxAddressInput').value=d.dmxAddress||1;}).catch(e=>console.error('Load config error:',e));}";
    html += "function saveDmxConfig(){const addr=parseInt(document.getElementById('dmxAddressInput').value)||1;if(addr<1||addr>510){alert('DMX Address must be between 1 and 510');return;}fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({dmxAddress:addr})}).then(r=>r.json()).then(d=>{alert('DMX configuration saved! Device settings updated.');}).catch(e=>{console.error('Save config error:',e);alert('Failed to save configuration');});}";
    html += "function play(){fetch('/api/play',{method:'POST'}).then(()=>updateStatus());}";
    html += "function pause(){fetch('/api/pause',{method:'POST'}).then(()=>updateStatus());}";
    html += "function stop(){fetch('/api/stop',{method:'POST'}).then(()=>updateStatus());}";
    html += "function setVolume(val){document.getElementById('volValue').textContent=val+'/30';fetch('/api/volume',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({volume:parseInt(val)})}).then(()=>updateStatus());}";
    html += "function playTrack(){const track=parseInt(document.getElementById('trackNum').value);fetch('/api/track',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({track:track})}).then(()=>updateStatus());}";
    html += "function updateStatus(){fetch('/api/status').then(r=>r.json()).then(d=>{";
    html += "const states=['STOPPED','PLAYING','PAUSED'];";
    html += "document.getElementById('status').textContent=states[d.state]||'?';";
    html += "document.getElementById('track').textContent=d.track||'-';";
    html += "document.getElementById('selectedTrack').textContent=d.selectedTrack||'-';";
    html += "document.getElementById('volume').textContent=d.volume+'/30';";
    html += "document.getElementById('volumeSlider').value=d.volume;";
    html += "document.getElementById('volValue').textContent=d.volume+'/30';";
    html += "document.getElementById('wifi').textContent=d.wifi?'✓ Connected':'✗ Offline';";
    html += "document.getElementById('artnet').textContent=d.dmxConnected?'✓ Connected':'✗ Offline';";
    html += "document.getElementById('dmxCh1').textContent=d.dmxVolume||'0';";
    html += "document.getElementById('dmxCh2').textContent=d.dmxTrack||'0';";
    html += "document.getElementById('dmxCh3').textContent=d.dmxControl||'0';";
    html += "document.getElementById('dmxAction').textContent=getControlAction(d.dmxControl||'0');";
    html += "}).catch(e=>console.error('Status update error:',e));}";
    html += "function updateLogs(){fetch('/api/logs').then(r=>r.json()).then(d=>{";
    html += "const log=d.logs.map(log=>'<div class=\"log-item\"><span class=\"log-time\">'+new Date(log.t).toLocaleTimeString()+'</span> '+log.m+'</div>').reverse().join('');";
    html += "document.getElementById('logs').innerHTML=log||'<div class=\"log-item\">No logs...</div>';";
    html += "}).catch(e=>console.error('Log update error:',e));}";
    html += "loadDmxConfig();updateStatus();updateLogs();setInterval(updateStatus,1000);setInterval(updateLogs,2000);";
    html += "function showTab(t){document.getElementById('tab-control').style.display=t==='control'?'block':'none';document.getElementById('tab-files').style.display=t==='files'?'block':'none';document.getElementById('tab-btn-control').className='tab-btn'+(t==='control'?' tab-active':'');document.getElementById('tab-btn-files').className='tab-btn'+(t==='files'?' tab-active':'');if(t==='files')loadTrackList();}";
    html += "function escH(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;').replace(/\"/g,'&quot;');}";
    html += "function loadTrackList(){var el=document.getElementById('trackList');el.innerHTML='<div style=\"color:#aaa;text-align:center;padding:20px;\">Loading...</div>';fetch('/api/tracklist').then(function(r){return r.json();}).then(function(d){var tot=d.totalTracks||0;var names=d.names||{};if(tot===0){el.innerHTML='<div style=\"color:#aaa;text-align:center;padding:30px;\">No tracks detected. Check SD card and ensure files are in the MP3/ folder.</div>';return;}var h='<div style=\"font-size:11px;color:#aaa;display:flex;gap:8px;padding:4px 0 8px 0;border-bottom:2px solid #eee;margin-bottom:4px;\"><span style=\"min-width:42px;text-align:right;padding-right:6px;\">#</span><span style=\"min-width:80px;\">File</span><span style=\"flex:1;\">Friendly Name</span><span style=\"width:52px;\"></span></div>';for(var i=1;i<=tot;i++){var fname=String(i).padStart(3,'0')+'.mp3';h+='<div class=\"track-row\"><span class=\"track-num\">'+i+'</span><span class=\"track-filename\">'+fname+'</span><input class=\"track-name-input\" id=\"tname-'+i+'\" type=\"text\" value=\"'+escH(names[i]||'')+'\" placeholder=\"Track '+i+' name...\"><button class=\"track-play-btn\" onclick=\"playTrackNum('+i+')\">&#9654;</button></div>';}el.innerHTML=h;}).catch(function(){el.innerHTML='<div style=\"color:red;padding:10px;\">Error loading tracks.</div>';});}";
    html += "function saveTrackNames(){var names={};document.querySelectorAll('[id^=\"tname-\"]').forEach(function(el){var i=parseInt(el.id.replace('tname-',''));if(el.value.trim())names[i]=el.value.trim();});fetch('/api/tracklist',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({names:names})}).then(function(r){return r.json();}).then(function(){alert('Track names saved!');}).catch(function(){alert('Error saving names.');});}";
    html += "function playTrackNum(n){fetch('/api/track',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({track:n})}).then(function(){showTab('control');updateStatus();});}";
    html += "</script></body></html>";
    
    webServer.send(200, "text/html", html);
}

void handleStatusAPI()
{
    const uint8_t vol = dmxBuffer[DMX_CH_VOLUME - 1];
    const uint8_t trk = dmxBuffer[DMX_CH_TRACK - 1];
    const uint8_t ctrl = dmxBuffer[DMX_CH_CONTROL - 1];
    
    StaticJsonDocument<200> doc;
    doc["state"] = playState;
    doc["track"] = currentTrack;
    doc["selectedTrack"] = selectedTrack;
    doc["volume"] = currentVolume;
    doc["totalTracks"] = totalTracks;
    doc["wifi"] = (WiFi.status() == WL_CONNECTED);
    doc["dmxConnected"] = artnetConnected;
    doc["dmxVolume"] = vol;
    doc["dmxTrack"] = trk;
    doc["dmxControl"] = ctrl;
    
    String response;
    serializeJson(doc, response);
    
    // DEBUG: Log every status request showing what we're sending
    static unsigned long lastStatusLog = 0;
    if (millis() - lastStatusLog > 3000) {
        logMessagef("Status API: Sending dmxVolume=%d, dmxTrack=%d, dmxControl=%d (raw from buffer)", vol, trk, ctrl);
        logMessagef("  Response JSON length: %d bytes", response.length());
        lastStatusLog = millis();
    }
    
    webServer.send(200, "application/json", response);
}

void handlePlayAPI()
{
    mp3Play();
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handlePauseAPI()
{
    mp3Pause();
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleStopAPI()
{
    mp3Stop();
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleVolumeAPI()
{
    if (webServer.hasArg("plain")) {
        StaticJsonDocument<100> doc;
        deserializeJson(doc, webServer.arg("plain"));
        int vol = doc["volume"] | 15;
        mp3SetVolume(vol);
    }
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleTrackAPI()
{
    if (webServer.hasArg("plain")) {
        StaticJsonDocument<100> doc;
        deserializeJson(doc, webServer.arg("plain"));
        int track = doc["track"] | 1;
        mp3PlayTrack(track);
    }
    webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleLogsAPI()
{
    StaticJsonDocument<1024> doc;
    JsonArray logArray = doc.createNestedArray("logs");
    
    for (int i = 0; i < MAX_LOGS; i++) {
        if (logs[i].message.length() > 0) {
            JsonObject entry = logArray.createNestedObject();
            entry["t"] = logs[i].timestamp;
            entry["m"] = logs[i].message;
        }
    }
    
    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}

void handleGetConfigAPI()
{
    StaticJsonDocument<200> doc;
    doc["dmxAddress"] = DMX_ADDRESS;
    
    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}

void handleSetConfigAPI()
{
    if (webServer.hasArg("plain")) {
        StaticJsonDocument<200> doc;
        deserializeJson(doc, webServer.arg("plain"));
        
        uint8_t addr = doc["dmxAddress"] | DMX_ADDRESS;
        
        // Validate range (must leave room for address+2)
        if (addr < 1) addr = 1;
        if (addr > 510) addr = 510;
        
        // Update config
        config.dmxAddress = addr;
        saveConfig();
        
        // Update global variables
        DMX_ADDRESS = addr;
        DMX_CH_VOLUME = DMX_ADDRESS;
        DMX_CH_TRACK = DMX_ADDRESS + 1;
        DMX_CH_CONTROL = DMX_ADDRESS + 2;
        
        logMessagef("DMX Configuration Updated - Base Address: %d (Channels: Volume=%d, Track=%d, Control=%d)", DMX_ADDRESS, DMX_CH_VOLUME, DMX_CH_TRACK, DMX_CH_CONTROL);
        webServer.send(200, "application/json", "{\"ok\":true}");
    } else {
        webServer.send(400, "application/json", "{\"error\":\"Invalid request\"}");
    }
}

void handleTracklistGetAPI()
{
    DynamicJsonDocument doc(8192);
    doc["totalTracks"] = totalTracks;
    JsonObject names = doc.createNestedObject("names");

    if (SPIFFS.exists(TRACKLIST_FILE)) {
        File file = SPIFFS.open(TRACKLIST_FILE, "r");
        DynamicJsonDocument saved(8192);
        if (!deserializeJson(saved, file) && saved.containsKey("names")) {
            JsonObject savedNames = saved["names"].as<JsonObject>();
            for (JsonPair kv : savedNames) {
                names[kv.key().c_str()] = kv.value().as<const char*>();
            }
        }
        file.close();
    }

    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}

void handleTracklistSetAPI()
{
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"error\":\"No data\"}");
        return;
    }
    DynamicJsonDocument doc(8192);
    if (deserializeJson(doc, webServer.arg("plain"))) {
        webServer.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
        return;
    }
    File file = SPIFFS.open(TRACKLIST_FILE, "w");
    if (!file) {
        webServer.send(500, "application/json", "{\"error\":\"Write failed\"}");
        return;
    }
    serializeJson(doc, file);
    file.close();
    logMessage("Track names saved to SPIFFS");
    webServer.send(200, "application/json", "{\"ok\":true}");
}
