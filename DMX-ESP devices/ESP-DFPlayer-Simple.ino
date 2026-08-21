// ESP32 DFPlayer Mini MP3 Player - Simple Web Control
// Focus: Get MP3 playback working reliably with web UI
// No DMX/ArtNet - just core DFPlayer functionality

#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

// ==================== Configuration ====================
#define MP3_SERIAL Serial2
#define CONFIG_FILE "/config.json"

struct Config {
    char ssid[64];
    char password[64];
};

Config config;
WebServer webServer(80);

// ==================== DFPlayer State ====================
enum PlayState { STOPPED, PLAYING, PAUSED };
PlayState playState = STOPPED;

int currentTrack = 0;
int totalTracks = 0;
uint8_t currentVolume = 15;  // 0-30
uint8_t dfplayerStatus = 0;  // 0=stopped, 1=playing, 2=paused

// Serial response buffer
#define SERIAL_BUFFER_SIZE 9
uint8_t serialBuffer[SERIAL_BUFFER_SIZE] = {0};
uint8_t bufferIndex = 0;

// Query timing
unsigned long lastStatusQuery = 0;
unsigned long lastTrackQuery = 0;
const unsigned long STATUS_QUERY_INTERVAL = 1000;
const unsigned long TRACK_QUERY_INTERVAL = 3000;

// Logging
#define MAX_LOGS 50
struct LogEntry {
    unsigned long timestamp;
    String message;
};
LogEntry logs[MAX_LOGS];
uint16_t logIndex = 0;

// ==================== MP3 Commands ====================
const uint8_t MP3_CMD_PLAY = 0x03;
const uint8_t MP3_CMD_PAUSE = 0x04;
const uint8_t MP3_CMD_STOP = 0x16;
const uint8_t MP3_CMD_VOLUME = 0x06;
const uint8_t MP3_CMD_PLAYFILE = 0x08;
const uint8_t MP3_CMD_QUERY_TRACKS = 0x48;
const uint8_t MP3_CMD_QUERY_STATUS = 0x42;
const uint8_t MP3_CMD_QUERY_CURRENT_TRACK = 0x3F;
const uint8_t MP3_CMD_QUERY_VOLUME = 0x43;

// ==================== Setup ====================
void setup()
{
    Serial.begin(115200);
    delay(500);
    
    logMessage("\n=== ESP32 DFPlayer Web Control ===");
    
    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        logMessage("ERROR: SPIFFS failed");
    } else {
        logMessage("✓ SPIFFS initialized");
    }
    
    loadConfig();
    
    // Initialize MP3 module
    MP3_SERIAL.begin(9600, SERIAL_8N1, 16, 17);
    logMessage("✓ MP3 module serial initialized");
    
    delay(1500);  // Wait for SD card
    
    // Initialize MP3
    mp3SendCommand(MP3_CMD_VOLUME, 0x00, 15);  // Set volume to 15
    delay(200);
    mp3QueryTracks();  // Query total tracks
    delay(200);
    
    // Initialize WiFi
    initWiFi();
    setupWebServer();
}

// ==================== Main Loop ====================
void loop()
{
    // Handle web server
    webServer.handleClient();
    
    // Read MP3 responses
    handleMP3SerialRead();
    
    // Query status periodically
    if (millis() - lastStatusQuery > STATUS_QUERY_INTERVAL) {
        mp3QueryStatus();
        mp3QueryCurrentTrack();
        mp3QueryVolume();
        lastStatusQuery = millis();
    }
    
    // Query track count
    if (millis() - lastTrackQuery > TRACK_QUERY_INTERVAL && totalTracks == 0) {
        mp3QueryTracks();
        lastTrackQuery = millis();
    }
    
    yield();
}

// ==================== Logging ====================
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
    logs[logIndex].timestamp = millis();
    logs[logIndex].message = String(buffer);
    logIndex = (logIndex + 1) % MAX_LOGS;
    Serial.println(buffer);
}

// ==================== MP3 Control ====================
bool mp3SendCommand(uint8_t cmd, uint8_t param1, uint8_t param2)
{
    // Calculate checksum: -(0xFF + 0x06 + CMD + 0x00 + P1 + P2) & 0xFF
    uint8_t checksum = -(0xFF + 0x06 + cmd + 0x00 + param1 + param2) & 0xFF;
    
    logMessagef("SEND: CMD 0x%02X [p1=0x%02X p2=0x%02X] checksum=0x%02X", cmd, param1, param2, checksum);
    
    // Write 9-byte frame: 7E FF 06 [CMD] 00 [P1] [P2] [CHECKSUM] EF
    MP3_SERIAL.write(0x7E);
    MP3_SERIAL.write(0xFF);
    MP3_SERIAL.write(0x06);
    MP3_SERIAL.write(cmd);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(param1);
    MP3_SERIAL.write(param2);
    MP3_SERIAL.write(checksum);
    MP3_SERIAL.write(0xEF);
    MP3_SERIAL.flush();
    
    return true;
}

void mp3Play()
{
    if (playState != PLAYING) {
        logMessage(">>> PLAY");
        mp3SendCommand(MP3_CMD_PLAY, 0x00, 0x00);
        playState = PLAYING;
    }
}

void mp3Pause()
{
    if (playState != PAUSED) {
        logMessage(">>> PAUSE");
        mp3SendCommand(MP3_CMD_PAUSE, 0x00, 0x00);
        playState = PAUSED;
    }
}

void mp3Stop()
{
    if (playState != STOPPED) {
        logMessage(">>> STOP");
        mp3SendCommand(MP3_CMD_STOP, 0x00, 0x00);
        playState = STOPPED;
    }
}

void mp3SetVolume(uint8_t volume)
{
    if (volume > 30) volume = 30;
    if (currentVolume != volume) {
        logMessagef(">>> VOLUME: %d", volume);
        mp3SendCommand(MP3_CMD_VOLUME, 0x00, volume);
        currentVolume = volume;
    }
}

void mp3PlayTrack(int track)
{
    if (track < 1) track = 1;
    if (track > 255) track = 255;
    
    logMessagef(">>> PLAY TRACK: %d", track);
    mp3SendCommand(MP3_CMD_PLAYFILE, 0x00, track);
    currentTrack = track;
    playState = PLAYING;
}

void mp3QueryTracks()
{
    logMessage("QUERY: Total tracks");
    uint8_t checksum = -(0xFF + 0x06 + MP3_CMD_QUERY_TRACKS + 0x00 + 0x00 + 0x00) & 0xFF;
    MP3_SERIAL.write(0x7E);
    MP3_SERIAL.write(0xFF);
    MP3_SERIAL.write(0x06);
    MP3_SERIAL.write(MP3_CMD_QUERY_TRACKS);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(checksum);
    MP3_SERIAL.write(0xEF);
    MP3_SERIAL.flush();
}

void mp3QueryStatus()
{
    uint8_t checksum = -(0xFF + 0x06 + MP3_CMD_QUERY_STATUS + 0x00 + 0x00 + 0x00) & 0xFF;
    MP3_SERIAL.write(0x7E);
    MP3_SERIAL.write(0xFF);
    MP3_SERIAL.write(0x06);
    MP3_SERIAL.write(MP3_CMD_QUERY_STATUS);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(checksum);
    MP3_SERIAL.write(0xEF);
    MP3_SERIAL.flush();
}

void mp3QueryCurrentTrack()
{
    uint8_t checksum = -(0xFF + 0x06 + MP3_CMD_QUERY_CURRENT_TRACK + 0x00 + 0x00 + 0x00) & 0xFF;
    MP3_SERIAL.write(0x7E);
    MP3_SERIAL.write(0xFF);
    MP3_SERIAL.write(0x06);
    MP3_SERIAL.write(MP3_CMD_QUERY_CURRENT_TRACK);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(checksum);
    MP3_SERIAL.write(0xEF);
    MP3_SERIAL.flush();
}

void mp3QueryVolume()
{
    uint8_t checksum = -(0xFF + 0x06 + MP3_CMD_QUERY_VOLUME + 0x00 + 0x00 + 0x00) & 0xFF;
    MP3_SERIAL.write(0x7E);
    MP3_SERIAL.write(0xFF);
    MP3_SERIAL.write(0x06);
    MP3_SERIAL.write(MP3_CMD_QUERY_VOLUME);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(0x00);
    MP3_SERIAL.write(checksum);
    MP3_SERIAL.write(0xEF);
    MP3_SERIAL.flush();
}

// ==================== Serial Read ====================
void handleMP3SerialRead()
{
    while (MP3_SERIAL.available() > 0) {
        uint8_t byte = MP3_SERIAL.read();
        
        // Look for frame start (0x7E or 0x7C due to noise)
        if ((byte == 0x7E || byte == 0x7C) && bufferIndex < 2) {
            bufferIndex = 0;
            serialBuffer[bufferIndex++] = byte;
        } else if (bufferIndex > 0) {
            serialBuffer[bufferIndex++] = byte;
            
            // Check for end byte
            if (byte == 0xEF && bufferIndex >= 9) {
                processMP3Response();
                bufferIndex = 0;
            } else if (bufferIndex > 12) {
                bufferIndex = 0;
            }
        }
    }
}

void processMP3Response()
{
    // Validate frame
    if ((serialBuffer[0] != 0x7E && serialBuffer[0] != 0x7C) || serialBuffer[1] != 0xFF) {
        return;
    }
    
    uint8_t cmd = serialBuffer[3];
    
    logMessagef("RECV: Frame - CMD=0x%02X Data=[0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]",
        cmd, serialBuffer[1], serialBuffer[2], serialBuffer[3], serialBuffer[4], serialBuffer[5], serialBuffer[6]);
    
    // Track query response (0x41 or 0x48)
    if (cmd == 0x41 || cmd == 0x48) {
        int tracks = (serialBuffer[6] << 8) | serialBuffer[5];
        if (tracks > 0 && totalTracks != tracks) {
            totalTracks = tracks;
            logMessagef("✓ TRACKS DETECTED: %d", totalTracks);
        }
    }
    // Status response (0x42)
    else if (cmd == 0x42) {
        dfplayerStatus = serialBuffer[6];
        logMessagef("Status: %s", dfplayerStatus == 1 ? "PLAYING" : dfplayerStatus == 2 ? "PAUSED" : "STOPPED");
    }
    // Current track (0x3F)
    else if (cmd == 0x3F) {
        uint8_t track = serialBuffer[6];
        if (track > 0) {
            logMessagef("Current track: %d", track);
        }
    }
    // Volume (0x43)
    else if (cmd == 0x43) {
        logMessagef("Volume: %d/30", serialBuffer[6]);
    }
}

// ==================== Configuration ====================
void loadConfig()
{
    logMessage("Loading config...");
    if (!SPIFFS.exists(CONFIG_FILE)) {
        strncpy(config.ssid, "HomeWifi", sizeof(config.ssid) - 1);
        strncpy(config.password, "password", sizeof(config.password) - 1);
        saveConfig();
    } else {
        File file = SPIFFS.open(CONFIG_FILE, "r");
        StaticJsonDocument<256> doc;
        deserializeJson(doc, file);
        strncpy(config.ssid, doc["ssid"] | "HomeWifi", sizeof(config.ssid) - 1);
        strncpy(config.password, doc["password"] | "password", sizeof(config.password) - 1);
        file.close();
    }
    logMessagef("SSID: %s", config.ssid);
}

void saveConfig()
{
    File file = SPIFFS.open(CONFIG_FILE, "w");
    StaticJsonDocument<256> doc;
    doc["ssid"] = config.ssid;
    doc["password"] = config.password;
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
        logMessagef("✓ WiFi connected! IP: %s", WiFi.localIP().toString().c_str());
    } else {
        logMessage("✗ WiFi failed - starting AP");
        WiFi.mode(WIFI_AP);
        WiFi.softAP("ESP-DFPlayer", "12345678");
        logMessagef("AP IP: %s", WiFi.softAPIP().toString().c_str());
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
    
    webServer.begin();
    logMessage("Web server started on port 80");
}

void handleRoot()
{
    String html = R"(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>DFPlayer MP3 Control</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container {
            max-width: 500px;
            margin: 0 auto;
            background: white;
            border-radius: 12px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
            overflow: hidden;
        }
        .header {
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px 20px;
            text-align: center;
        }
        .header h1 { font-size: 24px; margin-bottom: 5px; }
        .header p { font-size: 14px; opacity: 0.9; }
        .content { padding: 30px 20px; }
        
        .status-box {
            background: #f8f9fa;
            border-radius: 8px;
            padding: 20px;
            margin-bottom: 30px;
            border-left: 4px solid #667eea;
        }
        .status-item { margin: 10px 0; font-size: 14px; }
        .status-label { font-weight: 600; color: #333; }
        .status-value { color: #667eea; font-size: 18px; font-weight: bold; }
        
        .control-group { margin-bottom: 25px; }
        .control-label { font-weight: 600; margin-bottom: 10px; color: #333; }
        
        .button-group {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 10px;
            margin-bottom: 15px;
        }
        button {
            padding: 12px;
            border: none;
            border-radius: 6px;
            font-size: 14px;
            font-weight: 600;
            cursor: pointer;
            transition: all 0.3s;
        }
        .btn-play { background: #51cf66; color: white; }
        .btn-play:hover { background: #40c057; }
        .btn-pause { background: #ffd43b; color: #333; }
        .btn-pause:hover { background: #ffec99; }
        .btn-stop { background: #ff8787; color: white; }
        .btn-stop:hover { background: #fa5252; }
        
        .slider-group {
            margin-bottom: 20px;
        }
        .slider-label {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            font-size: 14px;
        }
        input[type="range"] {
            width: 100%;
            height: 6px;
            border-radius: 3px;
            background: #e9ecef;
            outline: none;
            -webkit-appearance: none;
        }
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            appearance: none;
            width: 18px;
            height: 18px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
        }
        input[type="range"]::-moz-range-thumb {
            width: 18px;
            height: 18px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
            border: none;
        }
        
        .track-input {
            display: flex;
            gap: 10px;
        }
        .track-input input {
            flex: 1;
            padding: 10px;
            border: 1px solid #dee2e6;
            border-radius: 6px;
            font-size: 14px;
        }
        .track-input button {
            flex: 0 0 80px;
            background: #667eea;
            color: white;
        }
        .track-input button:hover { background: #5568d3; }
        
        .logs {
            background: #f8f9fa;
            border-radius: 8px;
            padding: 15px;
            max-height: 200px;
            overflow-y: auto;
            font-family: monospace;
            font-size: 12px;
            line-height: 1.4;
            color: #495057;
        }
        .log-item { margin: 5px 0; padding: 5px; border-left: 2px solid #dee2e6; padding-left: 10px; }
        .log-time { color: #667eea; font-weight: bold; }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🎵 DFPlayer MP3</h1>
            <p>Web Control Interface</p>
        </div>
        
        <div class="content">
            <!-- Status -->
            <div class="status-box">
                <div class="status-item">
                    <span class="status-label">Status:</span>
                    <span class="status-value" id="status">—</span>
                </div>
                <div class="status-item">
                    <span class="status-label">Track:</span>
                    <span class="status-value" id="track">—</span>
                </div>
                <div class="status-item">
                    <span class="status-label">Volume:</span>
                    <span class="status-value" id="volume">—</span>
                </div>
            </div>
            
            <!-- Transport Controls -->
            <div class="control-group">
                <div class="control-label">Playback</div>
                <div class="button-group">
                    <button class="btn-play" onclick="play()">▶ Play</button>
                    <button class="btn-pause" onclick="pause()">⏸ Pause</button>
                    <button class="btn-stop" onclick="stop()">⏹ Stop</button>
                </div>
            </div>
            
            <!-- Volume Control -->
            <div class="control-group">
                <div class="slider-group">
                    <div class="slider-label">
                        <span>Volume</span>
                        <span id="volValue">15/30</span>
                    </div>
                    <input type="range" id="volume" min="0" max="30" value="15" 
                           oninput="setVolume(this.value)">
                </div>
            </div>
            
            <!-- Track Selection -->
            <div class="control-group">
                <div class="control-label">Select Track</div>
                <div class="track-input">
                    <input type="number" id="trackNum" min="1" max="255" value="1" placeholder="Track #">
                    <button onclick="playTrack()">Play</button>
                </div>
            </div>
            
            <!-- Logs -->
            <div class="control-group">
                <div class="control-label">System Log</div>
                <div class="logs" id="logs"></div>
            </div>
        </div>
    </div>
    
    <script>
        function play() {
            fetch('/api/play', {method: 'POST'}).then(() => updateStatus());
        }
        function pause() {
            fetch('/api/pause', {method: 'POST'}).then(() => updateStatus());
        }
        function stop() {
            fetch('/api/stop', {method: 'POST'}).then(() => updateStatus());
        }
        function setVolume(val) {
            document.getElementById('volValue').textContent = val + '/30';
            fetch('/api/volume', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({volume: parseInt(val)})
            }).then(() => updateStatus());
        }
        function playTrack() {
            const track = parseInt(document.getElementById('trackNum').value);
            fetch('/api/track', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({track: track})
            }).then(() => updateStatus());
        }
        function updateStatus() {
            fetch('/api/status')
                .then(r => r.json())
                .then(d => {
                    const states = ['STOPPED', 'PLAYING', 'PAUSED'];
                    document.getElementById('status').textContent = states[d.state] || '?';
                    document.getElementById('track').textContent = d.track || '—';
                    document.getElementById('volume').value = d.volume;
                    document.getElementById('volValue').textContent = d.volume + '/30';
                });
        }
        function updateLogs() {
            fetch('/api/logs')
                .then(r => r.json())
                .then(d => {
                    const html = d.logs.map(log => 
                        `<div class="log-item"><span class="log-time">${log.t}</span> ${log.m}</div>`
                    ).reverse().join('');
                    document.getElementById('logs').innerHTML = html;
                });
        }
        updateStatus();
        updateLogs();
        setInterval(updateStatus, 2000);
        setInterval(updateLogs, 2000);
    </script>
</body>
</html>
    )";
    
    webServer.send(200, "text/html", html);
}

void handleStatusAPI()
{
    StaticJsonDocument<200> doc;
    doc["state"] = playState;
    doc["track"] = currentTrack;
    doc["volume"] = currentVolume;
    doc["totalTracks"] = totalTracks;
    
    String response;
    serializeJson(doc, response);
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
            entry["t"] = String(logs[i].timestamp);
            entry["m"] = logs[i].message;
        }
    }
    
    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}
