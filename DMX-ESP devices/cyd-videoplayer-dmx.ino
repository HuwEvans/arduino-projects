// Tutorial : https://youtu.be/jYcxUgxz9ks
// Use board "ESP32 Dev Module" (last tested on v3.2.0)
// DMX/ArtNet Control Integration

#include <Arduino_GFX_Library.h> // Install "GFX Library for Arduino" with the Library Manager (last tested on v1.6.0)
                                 // Install "JPEGDEC" with the Library Manager (last tested on v1.8.2)
#include "MjpegClass.h"          // Included in this project
#include "SD.h"                  // Included with the Espressif Arduino Core (last tested on v3.2.0)
#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>         // Install "ArduinoJson" with the Library Manager (tested with v6.x)
#include <ArtnetWifi.h>          // Install "Artnet" with the Library Manager by Pierre Guilbert

// Pins for the display
#define BL_PIN 21 // On some cheap yellow display model, BL pin is 27
#define SD_CS 5
#define SD_MISO 19
#define SD_MOSI 23
#define SD_SCK 18

#define BOOT_PIN 0                   // Boot pin
#define BOOT_BUTTON_DEBOUCE_TIME 400 // Debounce time when reading the boot button in milliseconds

// Configuration file
#define CONFIG_FILE "/spiffs/config.json"

// DMX Constants
#define DMX_CH_TRANSPORT 0      // Channel 0 (1-based would be 1)
#define DMX_CH_VIDEO 1          // Channel 1 (1-based would be 2)
#define DMX_CH_BRIGHTNESS 2     // Channel 2 (1-based would be 3)
#define DMX_CH_TRIGGER 3        // Channel 3 (1-based would be 4)
#define DMX_CH_LOOP 4           // Channel 4 (1-based would be 5)

// Configuration structure
struct Config {
    char ssid[64];
    char password[64];
    uint8_t universe;
    uint16_t dmxStartAddress;
};

Config config;

// DMX and WiFi
ArtnetWifi artnet;
WebServer webServer(80);
WiFiUDP udp;
uint8_t dmxBuffer[512] = {0};
unsigned long lastDmxUpdate = 0;
File uploadFile; // For handling file uploads

// ArtPoll Device Identification
#define ARTPOLL_PORT 6454
#define DEVICE_SHORT_NAME "CYD-VP"
#define DEVICE_LONG_NAME "CYD Video Player"
#define DEVICE_FIRMWARE_VERSION 0x0100  // Version 1.0

// Playback state
enum PlayState { STOPPED, PLAYING, PAUSED };
PlayState playState = STOPPED;
bool loopEnabled = true;
bool configMode = false;
uint8_t lastTriggerValue = 0;

// Some model of cheap Yellow display works only at 40Mhz
// #define DISPLAY_SPI_SPEED 40000000L // 40MHz 
#define DISPLAY_SPI_SPEED 80000000L // 80MHz 


#define SD_SPI_SPEED 80000000L      // 80Mhz

const char *MJPEG_FOLDER = "/mjpeg"; // Name of the mjpeg folder on the SD Card

// Storage for files to read on the SD card
#define MAX_FILES 20 // Maximum number of files, adjust as needed
String mjpegFileList[MAX_FILES];
uint32_t mjpegFileSizes[MAX_FILES] = {0}; // Store each GIF file's size in bytes
int mjpegCount = 0;
static int currentMjpegIndex = 0;
static File mjpegFile; // temp gif file holder

// Global variables for mjpeg
MjpegClass mjpeg;
int total_frames;
unsigned long total_read_video;
unsigned long total_decode_video;
unsigned long total_show_video;
unsigned long start_ms, curr_ms;
long output_buf_size, estimateBufferSize;
uint8_t *mjpeg_buf;
uint16_t *output_buf;

// Display global variables
Arduino_DataBus *bus = new Arduino_HWSPI(2 /* DC */, 15 /* CS */, 14 /* SCK */, 13 /* MOSI */, 12 /* MISO */);
Arduino_GFX *gfx = new Arduino_ILI9341(bus);

// SD Card reader is on a separate SPI
SPIClass sd_spi(VSPI);

// Interrupt to skip to the next mjpeg when the boot button is pressed
volatile bool skipRequested = false; // set in ISR, read in loop()
volatile uint32_t isrTick = 0;       // tick count captured in ISR
uint32_t lastPress = 0;              // used in main context for debounc
bool dmxSkipRequested = false;       // tracks if skip was triggered by DMX video change

void IRAM_ATTR onButtonPress()
{
    skipRequested = true;                 // flag handled in the playback loop
    isrTick = xTaskGetTickCountFromISR(); // safe, 1-tick resolution
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Initialize SPIFFS for configuration storage
    if (!SPIFFS.begin(true)) {
        Serial.println("ERROR: SPIFFS mount failed!");
    } else {
        Serial.println("SPIFFS mounted successfully");
    }

    // Load configuration
    loadConfig();

    // Set display backlight to High
    pinMode(BL_PIN, OUTPUT);
    analogWrite(BL_PIN, 160);
    
    // Display initialization
    Serial.println("Display initialization");
    if (!gfx->begin(DISPLAY_SPI_SPEED))
    {
        Serial.println("Display initialization failed!");
        while (true)
        {
            /* no need to continue */
        }
    }
    gfx->setRotation(0);
    gfx->fillScreen(RGB565_BLACK);
    Serial.printf("Screen size Width=%d,Height=%d\n", gfx->width(), gfx->height());

    // SD card initialization
    Serial.println("SD Card initialization");
    if (!SD.begin(SD_CS, sd_spi, SD_SPI_SPEED, "/sd"))
    {
        Serial.println("ERROR: File system mount failed!");
        while (true)
        {
            /* no need to continue */
        }
    }

    // Buffer allocation for mjpeg playing
    Serial.println("Buffer allocation");
    output_buf_size = gfx->width() * 4 * 2;
    output_buf = (uint16_t *)heap_caps_aligned_alloc(16, output_buf_size * sizeof(uint16_t), MALLOC_CAP_DMA);
    if (!output_buf)
    {
        Serial.println("output_buf aligned_alloc failed!");
        while (true)
        {
            /* no need to continue */
        }
    }
    estimateBufferSize = gfx->width() * gfx->height() * 2 / 5;
    mjpeg_buf = (uint8_t *)heap_caps_malloc(estimateBufferSize, MALLOC_CAP_8BIT);
    if (!mjpeg_buf)
    {
        Serial.println("mjpeg_buf allocation failed!");
        while (true)
        {
            /* no need to continue */
        }
    }

    loadMjpegFilesList(); // Load the list of mjpeg to play from the SD card

    // Set the boot button to skip the current mjpeg playing and go to the next
    pinMode(BOOT_PIN, INPUT);                        
    attachInterrupt(digitalPinToInterrupt(BOOT_PIN), // fast ISR
                    onButtonPress, FALLING);         // press == LOW

    // Initialize WiFi and ArtNet
    initWiFi();
    setupWebServer();
}

void loop()
{
    // Handle web server requests
    webServer.handleClient();

    // Update DMX values if WiFi is connected
    if (WiFi.status() == WL_CONNECTED) {
        // Read and process ArtNet packets (triggers callbacks internally)
        artnet.read();
        
        // Handle DMX control
        handleDMXControl();
        
        // Handle STOPPED state - black out screen and turn off brightness
        if (playState == STOPPED) {
            gfx->fillScreen(RGB565_BLACK);
            analogWrite(BL_PIN, 0);  // Turn off brightness
        }
        
        // Play video if playState is PLAYING
        if (playState == PLAYING) {
            analogWrite(BL_PIN, 160);  // Restore brightness when playing
            playSelectedMjpeg(currentMjpegIndex);
            // Only auto-increment if loop is enabled AND this wasn't a DMX-triggered skip
            if (loopEnabled && !dmxSkipRequested) {
                currentMjpegIndex++;
                if (currentMjpegIndex >= mjpegCount) {
                    currentMjpegIndex = 0;
                }
            }
            dmxSkipRequested = false;  // Reset flag for next iteration
        }
    } else if (!configMode) {
        // No WiFi and not in config mode - play videos normally
        if (playState == PLAYING || playState == STOPPED) {
            playState = PLAYING;
            analogWrite(BL_PIN, 160);  // Restore brightness when playing
            playSelectedMjpeg(currentMjpegIndex);
            if (loopEnabled) {
                currentMjpegIndex++;
                if (currentMjpegIndex >= mjpegCount) {
                    currentMjpegIndex = 0;
                }
            }
        }
    }
    
    delay(5); // Small delay to prevent watchdog issues
}

// Play the current mjpeg
void playSelectedMjpeg(int mjpegIndex)
{
    // Handle no videos available
    if (mjpegCount == 0) {
        Serial.println("No videos available - clearing screen");
        gfx->fillScreen(RGB565_BLACK);
        analogWrite(BL_PIN, 0);  // Turn off brightness
        playState = STOPPED;
        return;
    }
    
    // Bounds checking for invalid indices
    if (mjpegIndex < 0 || mjpegIndex >= mjpegCount) {
        Serial.printf("ERROR: Invalid video index %d (count: %d)\n", mjpegIndex, mjpegCount);
        gfx->fillScreen(RGB565_BLACK);
        analogWrite(BL_PIN, 0);  // Turn off brightness
        playState = STOPPED;
        return;
    }
    
    // Build the full path for the selected mjpeg
    String fullPath = String(MJPEG_FOLDER) + "/" + mjpegFileList[mjpegIndex];
    char mjpegFilename[128];
    fullPath.toCharArray(mjpegFilename, sizeof(mjpegFilename));

    Serial.printf("Playing %s\n", mjpegFilename);
    mjpegPlayFromSDCard(mjpegFilename);
}

// Callback function to draw a JPEG
int jpegDrawCallback(JPEGDRAW *pDraw)
{
    unsigned long s = millis();
    gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
    total_show_video += millis() - s;
    return 1;
}

// Play a mjpeg stored on the SD card
void mjpegPlayFromSDCard(char *mjpegFilename)
{
    Serial.printf("Opening %s\n", mjpegFilename);
    File mjpegFile = SD.open(mjpegFilename, "r");

    if (!mjpegFile || mjpegFile.isDirectory())
    {
        Serial.printf("ERROR: Failed to open %s file for reading\n", mjpegFilename);
    }
    else
    {
        Serial.println("MJPEG start");
        gfx->fillScreen(RGB565_BLACK);

        start_ms = millis();
        curr_ms = millis();
        total_frames = 0;
        total_read_video = 0;
        total_decode_video = 0;
        total_show_video = 0;

        mjpeg.setup(
            &mjpegFile, mjpeg_buf, jpegDrawCallback, true /* useBigEndian */,
            0 /* x */, 0 /* y */, gfx->width() /* widthLimit */, gfx->height() /* heightLimit */);

        // Read first buffer before loop to have valid frame for pause
        if (!mjpeg.readMjpegBuf()) {
            Serial.println("Failed to read first frame");
            mjpegFile.close();
            return;
        }

        bool videoEndedNaturally = false;
        while (!skipRequested && mjpegFile.available())
        {
            // Handle web server and ArtNet requests
            webServer.handleClient();
            
            // Process incoming ArtNet/DMX packets if WiFi is connected
            if (WiFi.status() == WL_CONNECTED) {
                artnet.read();
            }
            
            // Handle DMX control updates
            handleDMXControl();

            // Check if playstate changed to pause or stop
            if (playState == PAUSED) {
                // Draw current frame (don't advance) and wait
                mjpeg.drawJpg();
                delay(50);
                continue;  // Skip buffer reading, stay on same frame
            } else if (playState == STOPPED) {
                break; // Exit playback
            }

            // Draw current frame
            mjpeg.drawJpg();
            total_decode_video += millis() - curr_ms;
            curr_ms = millis();

            // Read next buffer from file for next iteration
            if (!mjpeg.readMjpegBuf()) {
                videoEndedNaturally = true;
                break;  // Exit if can't read
            }

            // Read video timing
            total_read_video += millis() - curr_ms;
            curr_ms = millis();
            total_frames++;
        }
        
        // If video ended naturally and loop is disabled, stop playback
        if (videoEndedNaturally && !loopEnabled) {
            playState = STOPPED;
        }
        
        /* We exited because the button was pressed or the video ended */
        if (skipRequested) // pressed?
        {
            uint32_t now = millis(); // safe here
            if (now - lastPress < BOOT_BUTTON_DEBOUCE_TIME)
            {
                // ignore if it was within the debounce time
            }
            else
            {
                lastPress = now;
            }
        }
        skipRequested = false;

        int time_used = millis() - start_ms;
        Serial.println(F("MJPEG end"));
        mjpegFile.close();
        skipRequested = false; // ready for next video
        float fps = 1000.0 * total_frames / time_used;
        total_decode_video -= total_show_video;
        Serial.printf("Total frames: %d\n", total_frames);
        Serial.printf("Time used: %d ms\n", time_used);
        Serial.printf("Average FPS: %0.1f\n", fps);
        Serial.printf("Read MJPEG: %lu ms (%0.1f %%)\n", total_read_video, 100.0 * total_read_video / time_used);
        Serial.printf("Decode video: %lu ms (%0.1f %%)\n", total_decode_video, 100.0 * total_decode_video / time_used);
        Serial.printf("Show video: %lu ms (%0.1f %%)\n", total_show_video, 100.0 * total_show_video / time_used);
        Serial.printf("Video size (wxh): %d×%d, scale factor=%d\n",mjpeg.getWidth(),mjpeg.getHeight(),mjpeg.getScale());
    }
}

// Read the mjpeg file list in the mjpeg folder of the SD card
void loadMjpegFilesList()
{
    File mjpegDir = SD.open(MJPEG_FOLDER);
    if (!mjpegDir)
    {
        Serial.printf("Failed to open %s folder\n", MJPEG_FOLDER);
        while (true)
        {
            /* code */
        }
    }
    mjpegCount = 0;
    while (true)
    {
        File file = mjpegDir.openNextFile();
        if (!file)
            break;
        if (!file.isDirectory())
        {
            String name = file.name();
            if (name.endsWith(".mjpeg"))
            {
                mjpegFileList[mjpegCount] = name;
                mjpegFileSizes[mjpegCount] = file.size(); // Save file size (in bytes)
                mjpegCount++;
                if (mjpegCount >= MAX_FILES)
                    break;
            }
        }
        file.close();
    }
    mjpegDir.close();
    Serial.printf("%d mjpeg files read\n", mjpegCount);
    // Optionally, print out each file's size for debugging:
    for (int i = 0; i < mjpegCount; i++)
    {
        Serial.printf("File %d: %s, Size: %lu bytes (%s)\n", i, mjpegFileList[i].c_str(), mjpegFileSizes[i],formatBytes(mjpegFileSizes[i]).c_str());
    }
}

// ==================== Configuration Management ====================

void loadConfig() {
    if (!SPIFFS.exists(CONFIG_FILE)) {
        // Create default config
        strncpy(config.ssid, "WiFiSSID", sizeof(config.ssid) - 1);
        strncpy(config.password, "WiFiPassword", sizeof(config.password) - 1);
        config.universe = 0;
        config.dmxStartAddress = 128;
        saveConfig();
        Serial.println("Created default config file");
        return;
    }

    File configFile = SPIFFS.open(CONFIG_FILE, "r");
    if (!configFile) {
        Serial.println("Failed to open config file");
        return;
    }

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, configFile);
    
    if (error) {
        Serial.println("Failed to parse config JSON");
        configFile.close();
        return;
    }

    strncpy(config.ssid, doc["ssid"] | "WiFiSSID", sizeof(config.ssid) - 1);
    strncpy(config.password, doc["password"] | "WiFiPassword", sizeof(config.password) - 1);
    config.universe = doc["universe"] | 0;
    config.dmxStartAddress = doc["dmxStartAddress"] | 128;

    configFile.close();
    Serial.printf("Config loaded: SSID=%s, Universe=%d, DMX Start=%d\n", config.ssid, config.universe, config.dmxStartAddress);
}

void saveConfig() {
    StaticJsonDocument<512> doc;
    doc["ssid"] = config.ssid;
    doc["password"] = config.password;
    doc["universe"] = config.universe;
    doc["dmxStartAddress"] = config.dmxStartAddress;

    File configFile = SPIFFS.open(CONFIG_FILE, "w");
    if (!configFile) {
        Serial.println("Failed to open config file for writing");
        return;
    }

    serializeJson(doc, configFile);
    configFile.close();
    Serial.println("Config saved");
}

// ==================== WiFi and ArtNet ====================

void initWiFi() {
    Serial.printf("Connecting to WiFi: %s\n", config.ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.ssid, config.password);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
        Serial.println("Starting ArtNet receiver...");
        
        // Initialize ArtNet - this binds to UDP port 6454 automatically
        artnet.begin();
        artnet.setArtDmxCallback(onDMXPacket);
        Serial.println("ArtNet initialized on UDP port 6454");
        
        configMode = false;
    } else {
        Serial.println("\nFailed to connect to WiFi");
        Serial.println("Starting in configuration mode...");
        startConfigMode();
    }
}

void startConfigMode() {
    configMode = true;
    WiFi.mode(WIFI_AP);
    WiFi.softAP("CYD-VideoPlayer", "12345678");
    Serial.printf("Access Point created. IP: %s\n", WiFi.softAPIP().toString().c_str());
}

void onDMXPacket(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t *data) {
    if (universe == config.universe) {
        memcpy(dmxBuffer, data, 512);
        lastDmxUpdate = millis();
        Serial.printf("DMX received: Universe %d, Sequence %d, Length %d\n", universe, sequence, length);
    }
}

void handleDMXControl() {
    // Only update if we've received recent DMX data
    if (millis() - lastDmxUpdate > 1000) {
        return; // Timeout - no recent DMX data
    }

    // Get DMX channel values (offset by dmxStartAddress, subtract 1 for 0-based indexing)
    // DMX channels are 1-indexed (1-512), so address 128 is array index 127
    uint16_t baseAddr = config.dmxStartAddress - 1;
    uint8_t ch1_transport = dmxBuffer[baseAddr + DMX_CH_TRANSPORT];
    uint8_t ch2_video = dmxBuffer[baseAddr + DMX_CH_VIDEO];
    uint8_t ch3_brightness = dmxBuffer[baseAddr + DMX_CH_BRIGHTNESS];
    uint8_t ch4_trigger = dmxBuffer[baseAddr + DMX_CH_TRIGGER];
    uint8_t ch5_loop = dmxBuffer[baseAddr + DMX_CH_LOOP];

    // Channel 1: Transport control (HIGHEST PRIORITY - apply immediately)
    if (ch1_transport < 20) {
        playState = STOPPED;
    } else if (ch1_transport < 50) {
        playState = PLAYING;
    } else if (ch1_transport < 80) {
        playState = PAUSED;
    }

    // Channel 3: Brightness control (0-255) - only when playing
    if (playState == PLAYING) {
        updateBrightness(ch3_brightness);
    }

    // Channel 5: Loop enable (0-127 off, 128-255 on)
    loopEnabled = (ch5_loop >= 128);

    // Channel 2: Video select (1-255 → video index 0-254)
    if (ch2_video > 0 && ch2_video <= mjpegCount) {
        int newIndex = ch2_video - 1;
        if (newIndex != currentMjpegIndex) {
            currentMjpegIndex = newIndex;
            // Trigger video change
            skipRequested = true;
            dmxSkipRequested = true;  // Mark this as a DMX-triggered skip
        }
    }

    // Channel 4: Trigger (rising edge = next video)
    if (ch4_trigger > 127 && lastTriggerValue <= 127) {
        // Rising edge detected
        skipRequested = true;
    }
    lastTriggerValue = ch4_trigger;
}

void updateBrightness(uint8_t level) {
    // Map 0-255 to brightness level
    analogWrite(BL_PIN, level);
}

// ==================== Web Server ====================

void setupWebServer() {
    webServer.on("/", HTTP_GET, handleRoot);
    webServer.on("/api/config", HTTP_GET, handleConfigGet);
    webServer.on("/api/config", HTTP_POST, handleConfigPost);
    webServer.on("/api/status", HTTP_GET, handleStatusGet);
    webServer.on("/api/dmx", HTTP_GET, handleDmxGet);
    webServer.on("/api/files", HTTP_GET, handleFilesGet);
    webServer.on("/api/upload", HTTP_POST, handleUpload, handleUploadFileData);
    webServer.on("/api/delete", HTTP_POST, handleFileDelete);
    
    webServer.begin();
    Serial.println("Web server started on port 80");
}

void handleRoot() {
    String html = "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
    html += "<title>CYD Video Player - Configuration</title>";
    html += "<style>";
    html += "body { font-family: Arial, sans-serif; margin: 20px; background: #f5f5f5; }";
    html += ".container { max-width: 600px; margin: 0 auto; background: white; padding: 20px; border-radius: 8px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }";
    html += "h1 { color: #333; }";
    html += "h2 { color: #333; font-size: 1.1em; margin-top: 25px; border-bottom: 2px solid #0099cc; padding-bottom: 10px; }";
    html += ".form-group { margin: 15px 0; }";
    html += "label { display: block; margin-bottom: 5px; color: #555; font-weight: bold; }";
    html += "input { width: 100%; padding: 8px; border: 1px solid #ddd; border-radius: 4px; box-sizing: border-box; }";
    html += "button { background: #007bff; color: white; padding: 10px 20px; border: none; border-radius: 4px; cursor: pointer; font-size: 16px; width: 100%; }";
    html += "button:hover { background: #0056b3; }";
    html += ".status { margin-top: 20px; padding: 10px; background: #e8f4f8; border-left: 4px solid #0099cc; border-radius: 4px; }";
    html += ".status-title { font-weight: bold; color: #0099cc; }";
    html += ".status-item { margin: 5px 0; }";
    html += ".now-playing { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; padding: 20px; border-radius: 8px; margin-bottom: 20px; }";
    html += ".now-playing-title { font-size: 0.9em; opacity: 0.9; margin-bottom: 10px; }";
    html += ".now-playing-video { font-size: 1.5em; font-weight: bold; margin-bottom: 8px; word-break: break-word; }";
    html += ".now-playing-status { display: flex; justify-content: space-between; align-items: center; font-size: 0.95em; }";
    html += ".play-state { background: rgba(255,255,255,0.2); padding: 4px 12px; border-radius: 20px; font-weight: bold; }";
    html += ".dmx-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; margin-top: 15px; }";
    html += ".dmx-channel { background: #f0f8ff; padding: 12px; border-radius: 4px; border-left: 4px solid #0099cc; }";
    html += ".dmx-label { font-weight: bold; color: #333; font-size: 0.9em; }";
    html += ".dmx-value { font-size: 1.5em; color: #0099cc; font-weight: bold; margin: 8px 0; }";
    html += ".dmx-status { font-size: 0.85em; color: #666; }";
    html += ".upload-area { border: 2px dashed #0099cc; border-radius: 8px; padding: 20px; text-align: center; background: #f9f9f9; cursor: pointer; margin-bottom: 15px; }";
    html += ".upload-area:hover { background: #f0f8ff; }";
    html += ".upload-area.dragover { background: #e8f4f8; border-color: #007bff; }";
    html += "#fileInput { display: none; }";
    html += ".file-list { margin-top: 15px; }";
    html += ".file-item { display: flex; gap: 10px; align-items: center; padding: 10px; background: #f9f9f9; border-radius: 4px; margin-bottom: 8px; border-left: 4px solid #28a745; }";
    html += ".file-name { width: 50%; font-family: monospace; font-size: 0.9em; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }";
    html += ".file-size { width: 40%; color: #666; font-size: 0.85em; }";
    html += ".delete-btn { width: 10%; background: #dc3545; padding: 4px 8px; font-size: 0.8em; border: none; border-radius: 4px; color: white; cursor: pointer; white-space: nowrap; text-align: center; }";
    html += ".delete-btn:hover { background: #c82333; }";
    html += ".progress-bar { width: 100%; height: 20px; background: #ddd; border-radius: 4px; overflow: hidden; margin: 10px 0; }";
    html += ".progress-fill { height: 100%; background: #007bff; width: 0%; transition: width 0.3s; }";
    html += ".message { padding: 10px; border-radius: 4px; margin-bottom: 10px; }";
    html += ".error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }";
    html += ".success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }";
    html += "</style></head><body>";
    html += "<div class=\"container\"><h1>CYD Video Player Configuration</h1>";
    html += "<div class=\"now-playing\">";
    html += "<div class=\"now-playing-title\">NOW PLAYING</div>";
    html += "<div class=\"now-playing-video\" id=\"nowPlayingVideo\">-</div>";
    html += "<div class=\"now-playing-status\"><div id=\"nowPlayingInfo\">-</div><div class=\"play-state\" id=\"playStateDisplay\">-</div></div>";
    html += "</div>";
    html += "<h2>File Management</h2>";
    html += "<div id=\"messages\"></div>";
    html += "<div class=\"upload-area\" id=\"uploadArea\" onclick=\"document.getElementById('fileInput').click()\">";
    html += "<p><strong>Click to select or drag & drop MJPEG files here</strong></p>";
    html += "<p style=\"font-size: 0.9em; color: #666;\">Max file size depends on available SD card space</p>";
    html += "</div>";
    html += "<input type=\"file\" id=\"fileInput\" accept=\".mjpeg\" multiple>";
    html += "<div id=\"uploadProgress\" style=\"display: none;\">";
    html += "<div class=\"progress-bar\"><div class=\"progress-fill\" id=\"progressFill\"></div></div>";
    html += "<p id=\"progressText\">Uploading...</p>";
    html += "</div>";
    html += "<h3>Video Files on Device</h3>";
    html += "<div class=\"file-list\" id=\"fileList\">Loading...</div>";
    html += "<h2>Device Settings</h2>";
    html += "<div class=\"form-group\"><label for=\"ssid\">WiFi SSID:</label><input type=\"text\" id=\"ssid\" placeholder=\"Network name\"></div>";
    html += "<div class=\"form-group\"><label for=\"password\">WiFi Password:</label><input type=\"password\" id=\"password\" placeholder=\"Network password\"></div>";
    html += "<div class=\"form-group\"><label for=\"universe\">DMX Universe:</label><input type=\"number\" id=\"universe\" min=\"0\" max=\"15\" value=\"0\"></div>";
    html += "<div class=\"form-group\"><label for=\"dmxStart\">DMX Start Address:</label><input type=\"number\" id=\"dmxStart\" min=\"0\" max=\"507\" value=\"128\"></div>";
    html += "<button onclick=\"saveConfig()\">Save Configuration</button>";
    html += "<div class=\"status\"><div class=\"status-title\">Device Status:</div>";
    html += "<div class=\"status-item\">WiFi: <span id=\"wifiStatus\">Checking...</span></div>";
    html += "<div class=\"status-item\">Videos: <span id=\"videoCount\">Loading...</span></div>";
    html += "<div class=\"status-item\">DMX Status: <span id=\"dmxStatus\">Waiting...</span></div></div>";
    html += "<h2>Live DMX Values</h2>";
    html += "<div class=\"dmx-grid\">";
    html += "<div class=\"dmx-channel\"><div class=\"dmx-label\">Ch1: Transport</div><div class=\"dmx-value\" id=\"dmx1\">-</div><div class=\"dmx-status\" id=\"dmx1s\"></div></div>";
    html += "<div class=\"dmx-channel\"><div class=\"dmx-label\">Ch2: Video Select</div><div class=\"dmx-value\" id=\"dmx2\">-</div><div class=\"dmx-status\" id=\"dmx2s\"></div></div>";
    html += "<div class=\"dmx-channel\"><div class=\"dmx-label\">Ch3: Brightness</div><div class=\"dmx-value\" id=\"dmx3\">-</div><div class=\"dmx-status\" id=\"dmx3s\" style=\"color: #0099cc;\"></div></div>";
    html += "<div class=\"dmx-channel\"><div class=\"dmx-label\">Ch4: Trigger</div><div class=\"dmx-value\" id=\"dmx4\">-</div><div class=\"dmx-status\" id=\"dmx4s\"></div></div>";
    html += "<div class=\"dmx-channel\"><div class=\"dmx-label\">Ch5: Loop Enable</div><div class=\"dmx-value\" id=\"dmx5\">-</div><div class=\"dmx-status\" id=\"dmx5s\"></div></div>";
    html += "<div class=\"dmx-channel\"><div class=\"dmx-label\">Last Update</div><div class=\"dmx-value\" id=\"dmxTime\" style=\"font-size: 0.9em;\">-</div><div class=\"dmx-status\" id=\"dmxTimestamp\"></div></div>";
    html += "</div></div>";
    html += "<script>";
    html += "function loadConfig(){fetch('/api/config').then(r=>r.json()).then(d=>{document.getElementById('ssid').value=d.ssid;document.getElementById('password').value=d.password;document.getElementById('universe').value=d.universe;document.getElementById('dmxStart').value=d.dmxStartAddress;});}";
    html += "function saveConfig(){const config={ssid:document.getElementById('ssid').value,password:document.getElementById('password').value,universe:parseInt(document.getElementById('universe').value),dmxStartAddress:parseInt(document.getElementById('dmxStart').value)};fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(config)}).then(r=>r.json()).then(d=>{alert(d.message);if(d.success)location.reload();});}";
    html += "function updateStatus(){fetch('/api/status').then(r=>r.json()).then(d=>{document.getElementById('wifiStatus').textContent=d.wifiConnected?'Connected':'Disconnected';document.getElementById('videoCount').textContent=d.videoCount+' videos';document.getElementById('dmxStatus').textContent=d.dmxRecent?'Active':'Idle';document.getElementById('nowPlayingVideo').textContent=d.currentVideoName||'No Video';document.getElementById('nowPlayingInfo').textContent=(d.currentVideo+1)+' of '+d.videoCount;document.getElementById('playStateDisplay').textContent=d.playState;});}";
    html += "function updateDMX(){fetch('/api/dmx').then(r=>r.json()).then(d=>{if(d.ch1!==undefined){document.getElementById('dmx1').textContent=d.ch1;const t1=['STOP','PLAY','PAUSE'];document.getElementById('dmx1s').textContent=d.ch1<20?t1[0]:(d.ch1<50?t1[1]:t1[2]);}\n";
    html += "if(d.ch2!==undefined){document.getElementById('dmx2').textContent=d.ch2;document.getElementById('dmx2s').textContent=d.ch2===0?'(disabled)':'video '+(d.ch2);}\n";
    html += "if(d.ch3!==undefined){document.getElementById('dmx3').textContent=d.ch3;document.getElementById('dmx3s').textContent=Math.round(d.ch3/255*100)+'%;'}\n";
    html += "if(d.ch4!==undefined){document.getElementById('dmx4').textContent=d.ch4;document.getElementById('dmx4s').textContent=d.ch4>127?'HIGH':'low';}\n";
    html += "if(d.ch5!==undefined){document.getElementById('dmx5').textContent=d.ch5;document.getElementById('dmx5s').textContent=d.ch5>=128?'ON':'off';}\n";
    html += "if(d.timestamp!==undefined){const date=new Date(d.timestamp);document.getElementById('dmxTime').textContent=date.toLocaleTimeString();document.getElementById('dmxTimestamp').textContent=d.age+'ms ago';}});}";
    html += "loadConfig();updateStatus();updateDMX();loadFiles();setInterval(()=>{updateStatus();updateDMX();},1000);";
    html += "const uploadArea=document.getElementById('uploadArea');const fileInput=document.getElementById('fileInput');const messages=document.getElementById('messages');";
    html += "uploadArea.addEventListener('dragover',(e)=>{e.preventDefault();uploadArea.classList.add('dragover');});";
    html += "uploadArea.addEventListener('dragleave',()=>{uploadArea.classList.remove('dragover');});";
    html += "uploadArea.addEventListener('drop',(e)=>{e.preventDefault();uploadArea.classList.remove('dragover');handleFiles(e.dataTransfer.files);});";
    html += "fileInput.addEventListener('change',(e)=>{handleFiles(e.target.files);});";
    html += "function handleFiles(files){if(files.length===0)return;for(let file of files){if(!file.name.endsWith('.mjpeg')){showMessage('Only .mjpeg files are supported','error');continue;}uploadFile(file);}}";
    html += "function uploadFile(file){const formData=new FormData();formData.append('file',file);document.getElementById('uploadProgress').style.display='block';const xhr=new XMLHttpRequest();xhr.upload.addEventListener('progress',(e)=>{if(e.lengthComputable){const percent=(e.loaded/e.total)*100;document.getElementById('progressFill').style.width=percent+'%';document.getElementById('progressText').textContent='Uploading: '+file.name+' ('+Math.round(percent)+'%)';}});";
    html += "xhr.addEventListener('load',()=>{if(xhr.status===200){showMessage('File uploaded: '+file.name,'success');loadFiles();}else{showMessage('Upload failed: '+file.name,'error');}document.getElementById('uploadProgress').style.display='none';fileInput.value='';});";
    html += "xhr.addEventListener('error',()=>{showMessage('Upload error','error');document.getElementById('uploadProgress').style.display='none';});";
    html += "xhr.open('POST','/api/upload');xhr.send(formData);}";
    html += "function showMessage(msg,type){const div=document.createElement('div');div.className='message '+type;div.textContent=msg;messages.innerHTML='';messages.appendChild(div);setTimeout(()=>{div.style.opacity='0';div.style.transition='opacity 0.5s';},3000);}";
    html += "function loadFiles(){fetch('/api/files').then(r=>r.json()).then(d=>{const list=document.getElementById('fileList');if(d.files.length===0){list.innerHTML='<p style=\\\"color: #666;\\\">No video files found</p>';return;}list.innerHTML=d.files.map(f=>'<div class=\\\"file-item\\\"><span class=\\\"file-name\\\">'+f.name+'</span><span class=\\\"file-size\\\">'+f.size+'</span><button class=\\\"delete-btn\\\" onclick=\\\"deleteFile(\\''+f.name+'\\');\\\">Delete</button></div>').join('');}); }";
    html += "function deleteFile(name){if(!confirm('Delete '+name+'?'))return;fetch('/api/delete',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({filename:name})}).then(r=>r.json()).then(d=>{if(d.success){showMessage('File deleted: '+name,'success');loadFiles();}else{showMessage('Delete failed: '+d.message,'error');}});}";
    html += "</script></body></html>";
    
    webServer.send(200, "text/html", html);
}

void handleConfigGet() {
    StaticJsonDocument<512> doc;
    doc["ssid"] = config.ssid;
    doc["password"] = config.password;
    doc["universe"] = config.universe;
    doc["dmxStartAddress"] = config.dmxStartAddress;
    
    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}

void handleConfigPost() {
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"success\": false, \"message\": \"No body\"}");
        return;
    }

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, webServer.arg("plain"));

    if (error) {
        webServer.send(400, "application/json", "{\"success\": false, \"message\": \"Invalid JSON\"}");
        return;
    }

    strncpy(config.ssid, doc["ssid"] | config.ssid, sizeof(config.ssid) - 1);
    strncpy(config.password, doc["password"] | config.password, sizeof(config.password) - 1);
    config.universe = doc["universe"] | config.universe;
    config.dmxStartAddress = doc["dmxStartAddress"] | config.dmxStartAddress;

    saveConfig();

    StaticJsonDocument<256> response;
    response["success"] = true;
    response["message"] = "Configuration saved. Device will reconnect...";
    
    String responseString;
    serializeJson(response, responseString);
    webServer.send(200, "application/json", responseString);

    // Reconnect WiFi after a short delay
    delay(1000);
    WiFi.disconnect();
    initWiFi();
}

void handleStatusGet() {
    StaticJsonDocument<256> doc;
    doc["wifiConnected"] = (WiFi.status() == WL_CONNECTED);
    doc["wifiSSID"] = config.ssid;
    doc["videoCount"] = mjpegCount;
    doc["currentVideo"] = currentMjpegIndex;
    doc["currentVideoName"] = (mjpegCount > currentMjpegIndex) ? mjpegFileList[currentMjpegIndex].c_str() : "None";
    doc["playState"] = (playState == PLAYING) ? "PLAYING" : (playState == PAUSED) ? "PAUSED" : "STOPPED";
    doc["dmxRecent"] = (millis() - lastDmxUpdate < 1000);
    doc["universe"] = config.universe;
    doc["dmxStartAddress"] = config.dmxStartAddress;
    
    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}

void handleDmxGet() {
    StaticJsonDocument<256> doc;
    
    // Get DMX channel values (subtract 1 for 0-based indexing)
    // DMX channels are 1-indexed (1-512), so address 128 is array index 127
    uint16_t baseAddr = config.dmxStartAddress - 1;
    uint8_t ch1 = dmxBuffer[baseAddr + DMX_CH_TRANSPORT];
    uint8_t ch2 = dmxBuffer[baseAddr + DMX_CH_VIDEO];
    uint8_t ch3 = dmxBuffer[baseAddr + DMX_CH_BRIGHTNESS];
    uint8_t ch4 = dmxBuffer[baseAddr + DMX_CH_TRIGGER];
    uint8_t ch5 = dmxBuffer[baseAddr + DMX_CH_LOOP];
    
    doc["ch1"] = ch1;
    doc["ch2"] = ch2;
    doc["ch3"] = ch3;
    doc["ch4"] = ch4;
    doc["ch5"] = ch5;
    doc["timestamp"] = lastDmxUpdate;
    doc["age"] = (long)(millis() - lastDmxUpdate);
    doc["dmxActive"] = (millis() - lastDmxUpdate < 1000);
    
    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}

void handleFilesGet() {
    StaticJsonDocument<512> doc;
    JsonArray files = doc.createNestedArray("files");
    
    File mjpegDir = SD.open(MJPEG_FOLDER);
    if (mjpegDir) {
        while (true) {
            File file = mjpegDir.openNextFile();
            if (!file) break;
            if (!file.isDirectory() && String(file.name()).endsWith(".mjpeg")) {
                JsonObject fileObj = files.createNestedObject();
                fileObj["name"] = file.name();
                fileObj["size"] = formatBytes(file.size());
            }
            file.close();
        }
        mjpegDir.close();
    }
    
    String response;
    serializeJson(doc, response);
    webServer.send(200, "application/json", response);
}

void handleUpload() {
    webServer.send(200);
}

void handleUploadFileData() {
    if (webServer.uri() != "/api/upload") return;
    
    HTTPUpload& upload = webServer.upload();
    
    if (upload.status == UPLOAD_FILE_START) {
        String filename = String(MJPEG_FOLDER) + "/" + upload.filename;
        Serial.printf("Upload start: %s\n", filename.c_str());
        
        //Remove path for filename
        if (filename.lastIndexOf("/") >= 0) {
            filename = filename.substring(filename.lastIndexOf("/") + 1);
        }
        filename = String(MJPEG_FOLDER) + "/" + filename;
        
        // Delete existing file if present
        if (SD.exists(filename)) {
            SD.remove(filename);
        }
        
        uploadFile = SD.open(filename, FILE_WRITE);
        if (!uploadFile) {
            Serial.printf("Failed to create file: %s\n", filename.c_str());
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadFile) {
            uploadFile.write(upload.buf, upload.currentSize);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (uploadFile) {
            uploadFile.close();
            Serial.printf("Upload complete: %s, Size: %d bytes\n", upload.filename.c_str(), upload.totalSize);
            loadMjpegFilesList(); // Refresh file list
        }
    }
}

void handleFileDelete() {
    if (!webServer.hasArg("plain")) {
        webServer.send(400, "application/json", "{\"success\": false, \"message\": \"No filename provided\"}");
        return;
    }
    
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, webServer.arg("plain"));
    
    if (error || !doc.containsKey("filename")) {
        webServer.send(400, "application/json", "{\"success\": false, \"message\": \"Invalid JSON\"}");
        return;
    }
    
    String filename = doc["filename"];
    String fullPath = String(MJPEG_FOLDER) + "/" + filename;
    
    // Safety check - only allow deletion from mjpeg folder
    if (!filename.endsWith(".mjpeg") || filename.indexOf("/") >= 0) {
        webServer.send(400, "application/json", "{\"success\": false, \"message\": \"Invalid filename\"}");
        return;
    }
    
    if (SD.remove(fullPath)) {
        Serial.printf("Deleted file: %s\n", filename.c_str());
        loadMjpegFilesList(); // Refresh file list
        webServer.send(200, "application/json", "{\"success\": true, \"message\": \"File deleted\"}");
    } else {
        webServer.send(400, "application/json", "{\"success\": false, \"message\": \"Failed to delete file\"}");
    }
}

// Function helper display sizes on the serial monitor
String formatBytes(size_t bytes)
{
    if (bytes < 1024)
    {
        return String(bytes) + " B";
    }
    else if (bytes < (1024 * 1024))
    {
        return String(bytes / 1024.0, 2) + " KB";
    }
    else
    {
        return String(bytes / 1024.0 / 1024.0, 2) + " MB";
    }
}