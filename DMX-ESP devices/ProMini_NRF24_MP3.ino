// Arduino Pro Mini + NRF24L01 + DFRobot Mini MP3 Player - Wireless DMX Remote
// Receives wireless control signals from DMX_NRF24_Hub and controls MP3 playback
// Reports device status back to hub via RDM (Remote Device Management) packets
//
// SETUP:
// 1. Install this sketch on Arduino Pro Mini (5V version recommended)
// 2. Locate THIS_NODE_ID define (currently 0-5) and set unique ID for each Pro Mini
// 3. Compile and upload the same sketch to multiple Pro Minis with different THIS_NODE_ID values
// 4. Connect NRF24L01 module and DFRobot Mini MP3 module as described below
//
// WIRELESS CONTROL:
// - Listens on NRF24L01 for control packets from hub on channel 76
// - Accepts broadcast packets (nodeID = 0xFF) from all 6 broadcast channels (Ch 123-127)
// - Accepts device-specific packets (nodeID = THIS_NODE_ID) from device channels (Ch 128+)
//
// CONTROL CHANNELS (per device):
//   Ch N+0: Transport (0-19=STOP, 20-49=PLAY, 50-79=PAUSE)
//   Ch N+1: Track Select (1-255)
//   Ch N+2: Volume (0-255)
//   Ch N+3: Trigger (0-127=low, 128-255=high/next)
//   Ch N+4: Loop Enable (0-127=off, 128-255=on)
// Where N = 123 for broadcast, 128 for Device 0, 133 for Device 1, etc.
//
// RDM (Remote Device Management):
// - Sends status packets periodically (every 2 seconds) to hub
// - Also sends status immediately after processing each command
// - Status packet includes: playState, currentTrack, currentVolume, packet count
// - Hub can track which devices are online/offline based on heartbeat
//
// HARDWARE CONNECTIONS:
// Pro Mini Pin 7  → NRF24L01 CE
// Pro Mini Pin 8  → NRF24L01 CSN
// Pro Mini SPI    → NRF24L01 MOSI/MISO/SCK
// Pro Mini Pin 10 → DFRobot MP3 RX (SoftwareSerial)
// Pro Mini Pin 11 → DFRobot MP3 TX (SoftwareSerial)
// Pro Mini GND    → DFRobot MP3 GND, NRF24L01 GND
// Pro Mini 5V     → DFRobot MP3 5V, NRF24L01 3.3V (use voltage regulator)

#include <SPI.h>
#include <RF24.h>
#include <SoftwareSerial.h>

// ==================== NRF24L01 Configuration ====================
// RF24(CE pin, CSN pin)
RF24 radio(7, 8);  // CE on pin 7, CSN on pin 8

// This remote's address (must match one in the hub)
// Change this value (0xC1, 0xC2, 0xC3, 0xC4, 0xC5, or 0xC6) for each remote device
const byte thisRemoteAddress[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC1};

// ==================== MP3 Module Serial Connection ====================
// RX pin on Pro Mini (0), TX pin on Pro Mini (1) - Hardware Serial
// MP3 Module: RX to GPIO 10 (SoftwareSerial), TX to GPIO 11
#define MP3_RX_PIN 10  // Pro Mini pin 10
#define MP3_TX_PIN 11  // Pro Mini pin 11
SoftwareSerial mp3Serial(MP3_RX_PIN, MP3_TX_PIN);  // RX, TX

// ==================== Control Packet Structure ====================
struct ControlPacket {
    uint8_t identifier[4];      // Authentication code: "MPTG" verifies hub origin
    uint8_t nodeID;             // 0-5 (which remote), 0xFF = broadcast
    uint8_t ch1_transport;      // 0-19=STOP, 20-49=PLAY, 50-79=PAUSE
    uint8_t ch2_track;          // Track select (1-255)
    uint8_t ch3_volume;         // Volume (0-255)
    uint8_t ch4_trigger;        // Trigger (0-127=low, 128-255=high)
    uint8_t ch5_loop;           // Loop enable (0-127=off, 128-255=on)
};

// RDM Status Response packet (sent back to hub)
struct RDMResponsePacket {
    uint8_t identifier[4];      // "MPTG"
    uint8_t nodeID;             // This remote's ID
    uint8_t rmdType;            // RDM type (0x01 = status)
    uint8_t playState;          // STOPPED=0, PLAYING=1, PAUSED=2
    uint8_t currentTrack;       // Current track
    uint8_t currentVolume;      // Current volume
    uint32_t packetCount;       // Packets received by remote
};

// Hub identifier for packet verification
const uint8_t hubIdentifier[4] = {'M', 'P', 'T', 'G'};

// ====== CRITICAL: Device Identification ======
// THIS_NODE_ID determines which DMX channel bank controls this device:
//   THIS_NODE_ID 0 → listens to Ch 128-132
//   THIS_NODE_ID 1 → listens to Ch 133-137
//   THIS_NODE_ID 2 → listens to Ch 138-142
//   THIS_NODE_ID 3 → listens to Ch 143-147
//   THIS_NODE_ID 4 → listens to Ch 148-152
//   THIS_NODE_ID 5 → listens to Ch 153-157
// All devices ALSO listen to broadcast channels 123-127
//
// DEPLOYMENT PROCEDURE:
// 1. Compile and upload this sketch to Pro Mini #1, keeping THIS_NODE_ID = 0
// 2. Edit THIS_NODE_ID to 1, recompile, upload to Pro Mini #2
// 3. Repeat for remaining devices (max 6 devices, THIS_NODE_ID 0-5)
// 4. Each sketch has same code; THIS_NODE_ID is only difference per device
// 5. Label each Pro Mini with its THIS_NODE_ID for reference
//
#define THIS_NODE_ID 0  // ← CHANGE THIS (0-5) FOR EACH DEVICE BEFORE UPLOAD

// Function to verify packet authenticity
bool isValidPacket(const ControlPacket &packet) {
    return (packet.identifier[0] == hubIdentifier[0] &&
            packet.identifier[1] == hubIdentifier[1] &&
            packet.identifier[2] == hubIdentifier[2] &&
            packet.identifier[3] == hubIdentifier[3]);
}

// ==================== MP3 Commands ====================
const uint8_t MP3_CMD_PLAY = 0x03;
const uint8_t MP3_CMD_PAUSE = 0x04;
const uint8_t MP3_CMD_VOLUME = 0x06;
const uint8_t MP3_CMD_NEXT = 0x01;
const uint8_t MP3_CMD_PREV = 0x02;
const uint8_t MP3_CMD_PLAYFILE = 0x08;

// ==================== Playback State ====================
enum PlayState { STOPPED, PLAYING, PAUSED };
PlayState playState = STOPPED;
bool loopEnabled = true;
uint8_t currentTrack = 1;
uint8_t currentVolume = 25;  // 0-30
uint8_t lastTriggerValue = 0;

// Wireless state
unsigned long lastPacketTime = 0;
uint32_t packetsReceived = 0;

void setup()
{
    // ====== Serial Debug Output ======
    // Connect at 115200 baud to see initialization messages:
    // Expected startup output:
    //   === Arduino Pro Mini NRF24L01 MP3 Remote ===
    //   Node ID: 0 (or 1-5 depending on THIS_NODE_ID)
    //   Initializing NRF24L01...
    //   NRF24L01 initialized | Remote Address: 0xC2C2C2C2C1
    //   MP3 module initialized
    //
    // If "ERROR: NRF24L01 initialization failed!" appears:
    //   - Check NRF24L01 wiring (CE=7, CSN=8, SPI pins correct)
    //   - Verify Pro Mini has 3.3V power to NRF24L01 via voltage regulator
    //   - Check power supply can provide enough current (>20mA during TX)
    
    Serial.begin(115200);  // Debug serial
    delay(1000);
    
    Serial.println("\n=== Arduino Pro Mini NRF24L01 MP3 Remote ===");
    Serial.print("Node ID: ");
    Serial.println(THIS_NODE_ID);
    
    // Initialize NRF24L01
    Serial.println("Initializing NRF24L01...");
    if (!radio.begin()) {
        Serial.println("ERROR: NRF24L01 initialization failed!");
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
        }
    }
    
    // Configure NRF24L01 (must match hub settings)
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setCRCLength(RF24_CRC_16);
    radio.setRetries(0, 0);  // Remote doesn't need to transmit ACKs
    radio.setChannel(76);
    radio.enableDynamicPayloads();
    
    // Open reading pipe on this remote's address
    radio.openReadingPipe(1, thisRemoteAddress);
    
    // Enable ACK payloads for RDM status responses
    radio.enableAckPayload();
    
    radio.startListening();  // Receive mode
    
    Serial.print("NRF24L01 initialized | Remote Address: 0x");
    for (int i = 0; i < 5; i++) {
        if (thisRemoteAddress[i] < 0x10) Serial.print("0");
        Serial.print(thisRemoteAddress[i], HEX);
    }
    Serial.println();
    
    // Initialize MP3 serial
    mp3Serial.begin(9600);
    delay(500);
    
    // Send initialization commands to MP3 module
    mp3SetVolume(currentVolume);
    delay(100);
    
    Serial.println("MP3 module initialized");
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    Serial.println("Listening for control packets and ready for RDM...\n");
}

void loop()
{
    // Check for incoming wireless packets
    if (radio.available()) {
        ControlPacket packet;
        
        if (radio.read(&packet, sizeof(ControlPacket))) {
            // Verify packet authenticity using "MPTG" identifier
            if (!isValidPacket(packet)) {
                Serial.println("RX: Invalid packet - wrong identifier (security check failed)");
                return;
            }
            
            // Check if this packet is for us (broadcast or our specific nodeID)
            if (packet.nodeID == 0xFF || packet.nodeID == THIS_NODE_ID) {
                lastPacketTime = millis();
                packetsReceived++;
                
                digitalWrite(LED_BUILTIN, HIGH);  // LED on for RX
                
                Serial.print("RX: [");
                Serial.print(packet.ch1_transport);
                Serial.print(", ");
                Serial.print(packet.ch2_track);
                Serial.print(", ");
                Serial.print(packet.ch3_volume);
                Serial.print(", ");
                Serial.print(packet.ch4_trigger);
                Serial.print(", ");
                Serial.print(packet.ch5_loop);
                Serial.print("] From Hub (ID: MPTG, Node: ");
                Serial.print(packet.nodeID);
                Serial.println(")");
                
                // Process control packet
                handleControlPacket(packet);
                
                // Send back RDM status response
                sendRDMStatus();
                
                delay(100);  // LED stays on briefly
                digitalWrite(LED_BUILTIN, LOW);
            }
        }
    }
    
    // Periodically send heartbeat RDM status to hub
    static unsigned long lastStatusUpdate = 0;
    if (millis() - lastStatusUpdate > 2000) {
        sendRDMStatus();
        lastStatusUpdate = millis();
    }
    
    // Check for RF timeout (no packets for >5 seconds)
    if (millis() - lastPacketTime > 5000 && playState != STOPPED) {
        // Optional: stop playback on RF timeout for safety
        // playState = STOPPED;
        // mp3Stop();
        // Serial.println("RF timeout - stopping playback");
    }
    
    delay(10);
}

void handleControlPacket(ControlPacket &packet)
{
    // Channel 1: Transport control (HIGHEST PRIORITY)
    if (packet.ch1_transport < 20) {
        if (playState != STOPPED) {
            playState = STOPPED;
            mp3Stop();
            Serial.println("  -> STOP");
        }
    } else if (packet.ch1_transport < 50) {
        if (playState != PLAYING) {
            playState = PLAYING;
            mp3Play();
            Serial.println("  -> PLAY");
        }
    } else if (packet.ch1_transport < 80) {
        if (playState != PAUSED) {
            playState = PAUSED;
            mp3Pause();
            Serial.println("  -> PAUSE");
        }
    }
    
    // Channel 3: Volume control (0-255 mapped to 0-30)
    uint8_t dmxVolume = (packet.ch3_volume / 255.0) * 30;
    if (dmxVolume != currentVolume) {
        mp3SetVolume(dmxVolume);
        Serial.print("  -> Volume: ");
        Serial.println(dmxVolume);
    }
    
    // Channel 5: Loop enable (0-127 off, 128-255 on)
    loopEnabled = (packet.ch5_loop >= 128);
    
    // Channel 2: Track select (1-255)
    if (packet.ch2_track > 0 && packet.ch2_track <= 255) {
        if (packet.ch2_track != currentTrack) {
            currentTrack = packet.ch2_track;
            mp3PlayTrack(currentTrack);
            Serial.print("  -> Track: ");
            Serial.println(currentTrack);
        }
    }
    
    // Channel 4: Trigger (rising edge = next track)
    if (packet.ch4_trigger > 127 && lastTriggerValue <= 127) {
        mp3Next();
        Serial.println("  -> Next Track");
    }
    lastTriggerValue = packet.ch4_trigger;
}

// ==================== MP3 Control Functions ====================

void sendRDMStatus()
{
    // Send RDM status response back to hub
    radio.stopListening();
    
    RDMResponsePacket response;
    response.identifier[0] = 'M';
    response.identifier[1] = 'P';
    response.identifier[2] = 'T';
    response.identifier[3] = 'G';
    response.nodeID = THIS_NODE_ID;
    response.rmdType = 0x01;  // Status response
    response.playState = playState;  // 0=STOPPED, 1=PLAYING, 2=PAUSED
    response.currentTrack = currentTrack;
    response.currentVolume = currentVolume;
    response.packetCount = packetsReceived;
    
    // Try to send status (hub should be listening)
    radio.openWritingPipe(0xC2C2C2C2C2LL);  // Hub listening address
    radio.write(&response, sizeof(response));
    
    radio.startListening();  // Return to listening mode
}

void mp3SendCommand(uint8_t cmd, uint8_t param1, uint8_t param2)
{
    // DFRobot Mini MP3 protocol: 7E FF 06 [CMD] [Param1] [Param2] EF
    uint8_t buffer[8] = {0x7E, 0xFF, 0x06, cmd, 0x00, param1, param2, 0xEF};
    for (int i = 0; i < 8; i++) {
        mp3Serial.write(buffer[i]);
    }
    delay(100);
}

void mp3Play()
{
    mp3SendCommand(MP3_CMD_PLAY, 0x00, 0x00);
    playState = PLAYING;
}

void mp3Pause()
{
    mp3SendCommand(MP3_CMD_PAUSE, 0x00, 0x00);
    playState = PAUSED;
}

void mp3Stop()
{
    mp3SendCommand(MP3_CMD_PAUSE, 0x00, 0x00);
    playState = STOPPED;
}

void mp3PlayTrack(int trackNum)
{
    // trackNum is 1-indexed
    if (trackNum < 1) trackNum = 1;
    if (trackNum > 255) trackNum = 255;
    
    currentTrack = trackNum;
    mp3SendCommand(MP3_CMD_PLAYFILE, 0x00, trackNum);
    playState = PLAYING;
}

void mp3SetVolume(uint8_t volume)
{
    // Volume range: 0-30
    if (volume > 30) volume = 30;
    currentVolume = volume;
    mp3SendCommand(MP3_CMD_VOLUME, 0x00, volume);
}

void mp3Next()
{
    mp3SendCommand(MP3_CMD_NEXT, 0x00, 0x00);
}

void mp3Prev()
{
    mp3SendCommand(MP3_CMD_PREV, 0x00, 0x00);
}
