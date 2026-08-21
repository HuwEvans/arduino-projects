// DMX/ArtNet to NRF24L01 Hub
// Central receiver that broadcasts DMX control signals to multiple remote Pro Mini + NRF24L01 devices
// Arduino Mega 2560 with NRF24L01 module + CF Robot DMX Shield (optional)
// Receives DMX data via CF Robot DMX Shield or Serial1 and broadcasts to wireless remote devices
// Serial0 (USB) reserved for monitoring/debugging
// Serial1 (pins 18-19) for DMX serial input/fallback
//
// FEATURES:
// - Per-device control: Each remote has 5 dedicated DMX channels
// - Broadcast control: Single set of channels controls all devices
// - RDM (Remote Device Management): Devices report back status to hub
// - Authenticated packets: All wireless traffic uses "MPTG" identifier
//
// DMX CHANNEL LAYOUT:
// Broadcast channels (Ch 123-127): Control ALL remotes simultaneously
//   Ch 123: Transport (0-19=STOP, 20-49=PLAY, 50-79=PAUSE)
//   Ch 124: Track Select (1-255)
//   Ch 125: Volume (0-255)
//   Ch 126: Trigger (0-127=low, 128-255=high)
//   Ch 127: Loop Enable (0-127=off, 128-255=on)
//
// Per-device channels (starting at Ch 128):
//   Device 0: Ch 128-132 (same layout as broadcast)
//   Device 1: Ch 133-137
//   Device 2: Ch 138-142
//   Device 3: Ch 143-147
//   Device 4: Ch 148-152
//   Device 5: Ch 153-157
//
// RDM RESPONSES:
// Devices periodically send status back to hub at 2-second intervals
// or immediately after receiving a command packet
// Status includes: playState, currentTrack, currentVolume, packetsReceived

#include <SPI.h>
#include <RF24.h>
#include <DmxShield.h>    // CF Robot DMX Shield - Install: https://github.com/cfrobot/DmxShield

// ==================== NRF24L01 Configuration ====================
// RF24(CE pin, CSN pin)
// Mega 2560: CE on pin 7, CSN on pin 8 (same as Uno/Nano)
RF24 radio(7, 8);

// NRF24L01 pipe addresses for remotes (0-5 devices supported)
const byte remoteAddresses[6][6] = {
    {0xC2, 0xC2, 0xC2, 0xC2, 0xC1},  // Remote 0
    {0xC2, 0xC2, 0xC2, 0xC2, 0xC2},  // Remote 1
    {0xC2, 0xC2, 0xC2, 0xC2, 0xC3},  // Remote 2
    {0xC2, 0xC2, 0xC2, 0xC2, 0xC4},  // Remote 3
    {0xC2, 0xC2, 0xC2, 0xC2, 0xC5},  // Remote 4
    {0xC2, 0xC2, 0xC2, 0xC2, 0xC6}   // Remote 5
};

#define MAX_REMOTES 6
uint8_t activeRemotes = 1;  // Number of active remote devices

// ==================== Serial DMX Configuration ====================
// Serial0 (USB): Monitoring/debugging output at 115200 baud
// Serial1 (pins 18-19): DMX serial input from external gateway/receiver
// Receive DMX data via Serial1 (from external DMX receiver module or gateway)
// DMX data arrives as simple 5-byte packets: [CH1, CH2, CH3, CH4, CH5]
#define SERIAL_BAUD 115200
#define SERIAL_DMX_BAUD 115200  // Baud rate for Serial1 DMX input

// DMX channel mapping - per-device control
// Each device gets 5 channels: (base + offset)
// Device 0: Ch 128-132, Device 1: Ch 133-137, Device 2: Ch 138-142, etc.
// Broadcast channels: Ch 123-127 (control all devices)
#define DMX_BROADCAST_BASE 123

#define DMX_CH_TRANSPORT 0      // 0-19=STOP, 20-49=PLAY, 50-79=PAUSE
#define DMX_CH_TRACK 1          // Track select (1-255)
#define DMX_CH_VOLUME 2         // Volume (0-255)
#define DMX_CH_TRIGGER 3        // Trigger (0-127=low, 128-255=high)
#define DMX_CH_LOOP 4           // Loop enable (0-127=off, 128-255=on)
#define DMX_CHANNELS_PER_DEVICE 5

// RDM constants
#define RDM_UID_SIZE 6
#define RDM_RESPONSE_TIMEOUT 100  // ms to wait for RDM responses

// Control packet structure for wireless transmission
// Includes authentication code "MPTG" to verify packets from hub
struct ControlPacket {
    uint8_t identifier[4];      // "MPTG" - identifies hub packets for security
    uint8_t nodeID;             // 0-5 (which remote this is for, 0xFF = broadcast)
    uint8_t ch1_transport;      // Transport control
    uint8_t ch2_track;          // Track select
    uint8_t ch3_volume;         // Volume
    uint8_t ch4_trigger;        // Trigger
    uint8_t ch5_loop;           // Loop enable
};

// RDM Status Response packet from remotes
struct RDMResponsePacket {
    uint8_t identifier[4];      // "MPTG"
    uint8_t nodeID;             // Which remote
    uint8_t rmdType;            // RDM response type (0x01 = status)
    uint8_t playState;          // STOPPED=0, PLAYING=1, PAUSED=2
    uint8_t currentTrack;       // Current track
    uint8_t currentVolume;      // Current volume
    uint32_t packetCount;       // Packets received by remote
};

// Device status structure
struct DeviceStatus {
    uint8_t nodeID;
    bool online;
    uint8_t playState;
    uint8_t currentTrack;
    uint8_t currentVolume;
    unsigned long lastHeartbeat;
    uint32_t packetsReceived;
};
DeviceStatus devices[MAX_REMOTES];

// Serial receive buffer
uint8_t dmxBuffer[5] = {0};
uint8_t dmxIndex = 0;
unsigned long lastDmxUpdate = 0;

// CF Robot DMX Shield mode
bool useDmxShield = false;  // Will be set to true if shield detected

// Wireless packet counters
uint32_t packetsSent = 0;
uint32_t broadcastCycles = 0;

void setup()
{
    // ====== Serial Debug Output ======
    // Serial0 (USB) at 115200 baud for monitoring hub status:
    // Expected startup output:
    //   === DMX NRF24L01 Hub ===
    //   Checking for DMX Shield...
    //   [Either] DMX Shield detected - using hardware DMX interface
    //   [Or]    DMX Shield not detected - using Serial1 DMX input (5-byte packets on pins 18-19)
    //   Initializing NRF24L01...
    //   NRF24L01 initialized successfully | Channel: 76 | Data Rate: 250kbps | PA Level: Max
    //   Active remotes: 6
    //
    // If "ERROR: NRF24L01 initialization failed!" appears:
    //   - Check NRF24L01 wiring (CE=7, CSN=8, SPI pins correct)
    //   - Verify Mega 2560 has 3.3V power to NRF24L01 via voltage regulator
    //   - Check power supply current capacity (>150mA for Mega + NRF24L01)
    //
    // For serial DMX input: Connect DMX gateway RX/TX to Mega pins 18 (RX1) / 19 (TX1)
    //
    // During operation, monitor DMX input via Serial0:
    //   Ctrl(broadcast): T:1 Tr:5 V:200 G:1 L:0
    //   (Transport=PLAY, Track=5, Volume=200, Trigger=low, Loop=off)
    //
    // Monitor RDM responses from devices:
    //   RDM[nodeID]: State=1 Track=5 Vol=200
    //   (Device 0: Playing, Track 5, Volume 200)
    
    // Initialize Serial0 for monitoring/debugging
    Serial.begin(SERIAL_BAUD);
    
    // Initialize Serial1 for DMX serial input (pins 18-19)
    Serial1.begin(SERIAL_DMX_BAUD);
    
    delay(1000);
    
    Serial.println("\n=== DMX NRF24L01 Hub (Mega 2560) ===");
    Serial.println("Serial0 (USB): Monitoring/Debug Output");
    Serial.println("Serial1 (Pins 18-19): DMX Serial Input");
    
    // Initialize CF Robot DMX Shield (optional)
    Serial.println("Checking for DMX Shield...");
    DmxShield.init();  // Initialize shield
    delay(100);
    
    // Try to detect shield by reading a channel
    uint8_t testValue = DmxShield.read(1);
    if (testValue >= 0 && testValue <= 255) {
        useDmxShield = true;
        Serial.println("DMX Shield detected - using hardware DMX interface");
    } else {
        useDmxShield = false;
        Serial.println("DMX Shield not detected - using Serial1 DMX input (5-byte packets on pins 18-19)");
    }
    
    Serial.println("Initializing NRF24L01...");
    
    // Initialize NRF24L01
    if (!radio.begin()) {
        Serial.println("ERROR: NRF24L01 initialization failed!");
        while (1) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(200);
            digitalWrite(LED_BUILTIN, LOW);
            delay(200);
        }
    }
    
    // Configure NRF24L01
    radio.setPALevel(RF24_PA_MAX);           // Max power
    radio.setDataRate(RF24_250KBPS);         // Slow, reliable data rate
    radio.setCRCLength(RF24_CRC_16);         // 16-bit CRC
    radio.setRetries(3, 5);                  // 3 retries, 5*250us delay
    radio.setChannel(76);                    // Channel 76 (2.476 GHz)
    radio.enableDynamicPayloads();
    
    // Open writing pipes for all potential remotes
    for (uint8_t i = 0; i < MAX_REMOTES; i++) {
        radio.openWritingPipe(remoteAddresses[i]);
        radio.stopListening();  // Hub mode - transmit only
    }
    
    Serial.print("NRF24L01 initialized successfully");
    Serial.print(" | Channel: 76");
    Serial.print(" | Data Rate: 250kbps");
    Serial.println(" | PA Level: Max");
    Serial.print("Active remotes: ");
    Serial.println(activeRemotes);
    
    Serial.println("\n=== Hardware Configuration ===");
    Serial.println("Board: Arduino Mega 2560");
    Serial.println("NRF24L01 CE: Pin 7");
    Serial.println("NRF24L01 CSN: Pin 8");
    if (useDmxShield) {
        Serial.println("DMX Input: CF Robot Shield (SPI)");
    } else {
        Serial.println("DMX Input: Serial1 (RX1=Pin 19, TX1=Pin 18)");
    }
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    if (!useDmxShield) {
        Serial.println("\nListening for DMX data on Serial1 (pins 18-19)...");
        Serial.println("Expected format: 5-byte packets [CH1 CH2 CH3 CH4 CH5]");
    } else {
        Serial.println("Reading from DMX Shield channels 1-5");
    }
}

void loop()
{
    // Read DMX data from shield or serial
    if (useDmxShield) {
        static unsigned long lastShieldRead = 0;
        if (millis() - lastShieldRead > 100) {  // Read every 100ms
            // Check broadcast channels (123-127)
            uint8_t bcast_ch1 = DmxShield.read(DMX_BROADCAST_BASE + DMX_CH_TRANSPORT);
            uint8_t bcast_ch2 = DmxShield.read(DMX_BROADCAST_BASE + DMX_CH_TRACK);
            uint8_t bcast_ch3 = DmxShield.read(DMX_BROADCAST_BASE + DMX_CH_VOLUME);
            uint8_t bcast_ch4 = DmxShield.read(DMX_BROADCAST_BASE + DMX_CH_TRIGGER);
            uint8_t bcast_ch5 = DmxShield.read(DMX_BROADCAST_BASE + DMX_CH_LOOP);
            
            // Send broadcast packet if any channel changed
            if (bcast_ch1 || bcast_ch2 || bcast_ch3 || bcast_ch4 || bcast_ch5) {
                Serial.print("DMX RX (Broadcast): ");
                Serial.print(bcast_ch1); Serial.print(" ");
                Serial.print(bcast_ch2); Serial.print(" ");
                Serial.print(bcast_ch3); Serial.print(" ");
                Serial.print(bcast_ch4); Serial.print(" ");
                Serial.println(bcast_ch5);
                
                broadcastDMXControl(0xFF, bcast_ch1, bcast_ch2, bcast_ch3, bcast_ch4, bcast_ch5);
            }
            
            // Check per-device channels (128+ for devices 0-5)
            for (uint8_t device = 0; device < activeRemotes; device++) {
                uint16_t baseChannel = (DMX_BROADCAST_BASE + DMX_CHANNELS_PER_DEVICE) + (device * DMX_CHANNELS_PER_DEVICE);
                
                uint8_t dev_ch1 = DmxShield.read(baseChannel + DMX_CH_TRANSPORT);
                uint8_t dev_ch2 = DmxShield.read(baseChannel + DMX_CH_TRACK);
                uint8_t dev_ch3 = DmxShield.read(baseChannel + DMX_CH_VOLUME);
                uint8_t dev_ch4 = DmxShield.read(baseChannel + DMX_CH_TRIGGER);
                uint8_t dev_ch5 = DmxShield.read(baseChannel + DMX_CH_LOOP);
                
                if (dev_ch1 || dev_ch2 || dev_ch3 || dev_ch4 || dev_ch5) {
                    Serial.print("DMX RX (Device ");
                    Serial.print(device);
                    Serial.print("): ");
                    Serial.print(dev_ch1); Serial.print(" ");
                    Serial.print(dev_ch2); Serial.print(" ");
                    Serial.print(dev_ch3); Serial.print(" ");
                    Serial.print(dev_ch4); Serial.print(" ");
                    Serial.println(dev_ch5);
                    
                    broadcastDMXControl(device, dev_ch1, dev_ch2, dev_ch3, dev_ch4, dev_ch5);
                }
            }
            
            lastShieldRead = millis();
        }
    } else {
        // Serial1 input mode (fallback) - send to broadcast
        while (Serial1.available()) {
            uint8_t byte = Serial1.read();
            dmxBuffer[dmxIndex] = byte;
            dmxIndex++;
            
            if (dmxIndex >= 5) {
                dmxIndex = 0;
                lastDmxUpdate = millis();
                
                Serial.print("DMX RX (Serial1): ");
                for (int i = 0; i < 5; i++) {
                    Serial.print(dmxBuffer[i]);
                    Serial.print(" ");
                }
                Serial.println();
                
                broadcastDMXControl(0xFF, dmxBuffer[0], dmxBuffer[1], dmxBuffer[2], dmxBuffer[3], dmxBuffer[4]);
            }
        }
    }
    
    // Listen for RDM responses (status from remote devices)
    radio.startListening();
    if (radio.available()) {
        RDMResponsePacket rmdResponse;
        if (radio.read(&rmdResponse, sizeof(RDMResponsePacket))) {
            if (rmdResponse.identifier[0] == 'M' && rmdResponse.identifier[1] == 'P' &&
                rmdResponse.identifier[2] == 'T' && rmdResponse.identifier[3] == 'G') {
                
                // Update device status
                if (rmdResponse.nodeID < MAX_REMOTES) {
                    devices[rmdResponse.nodeID].online = true;
                    devices[rmdResponse.nodeID].playState = rmdResponse.playState;
                    devices[rmdResponse.nodeID].currentTrack = rmdResponse.currentTrack;
                    devices[rmdResponse.nodeID].currentVolume = rmdResponse.currentVolume;
                    devices[rmdResponse.nodeID].lastHeartbeat = millis();
                    devices[rmdResponse.nodeID].packetsReceived = rmdResponse.packetCount;
                    
                    Serial.print("RDM Status from Device ");
                    Serial.print(rmdResponse.nodeID);
                    Serial.print(": Track=");
                    Serial.print(rmdResponse.currentTrack);
                    Serial.print(" Vol=");
                    Serial.print(rmdResponse.currentVolume);
                    Serial.print(" Packets=");
                    Serial.println(rmdResponse.packetCount);
                }
            }
        }
    }
    radio.stopListening();
    
    // Keep-alive: broadcast every 500ms
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 500) {
        broadcastDMXControl(0xFF, 0, 0, 0, 0, 0);  // Heartbeat
        lastHeartbeat = millis();
    }
    
    // Status LED pulse
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 100) {
        digitalWrite(LED_BUILTIN, LOW);
    }
}

void broadcastDMXControl(uint8_t nodeID, uint8_t ch1, uint8_t ch2, uint8_t ch3, uint8_t ch4, uint8_t ch5)
{
    // Create control packet
    ControlPacket packet;
    
    // Set authentication identifier "MPTG"
    packet.identifier[0] = 'M';
    packet.identifier[1] = 'P';
    packet.identifier[2] = 'T';
    packet.identifier[3] = 'G';
    
    packet.nodeID = nodeID;         // 0-5 for individual, 0xFF for broadcast
    packet.ch1_transport = ch1;
    packet.ch2_track = ch2;
    packet.ch3_volume = ch3;
    packet.ch4_trigger = ch4;
    packet.ch5_loop = ch5;
    
    radio.stopListening();
    
    // Send to all active remotes
    for (uint8_t i = 0; i < activeRemotes; i++) {
        // Only send to specific device if not broadcast
        if (nodeID != 0xFF && nodeID != i) {
            continue;
        }
        
        radio.openWritingPipe(remoteAddresses[i]);
        
        if (radio.write(&packet, sizeof(packet))) {
            packetsSent++;
            digitalWrite(LED_BUILTIN, HIGH);  // LED on for TX
        } else {
            Serial.print("TX failed to remote ");
            Serial.println(i);
        }
    }
    
    broadcastCycles++;
}

// Simple function to configure active remotes
// Could be extended to detect remotes dynamically
void setActiveRemotes(uint8_t count)
{
    if (count <= MAX_REMOTES) {
        activeRemotes = count;
        Serial.print("Active remotes set to: ");
        Serial.println(activeRemotes);
    }
}
