# RDM (Remote Device Management) Implementation Guide

## Overview

RDM is a feedback mechanism that allows wireless remotes to report their status back to the hub. This enables real-time monitoring of device state, online/offline status, and playback information.

---

## How RDM Works in This System

### Traditional DMX Flow
```
DMX Console → Hub → Wireless Link → Remotes
(one-way communication)
```

### DMX + RDM Flow
```
DMX Console → Hub ↔ Wireless Link ↔ Remotes
                ↑                      ↓
                └──────────────────────┘
                (bidirectional feedback)
```

### RDM Status Packet (8 bytes)

```cpp
struct RDMResponsePacket {
    uint8_t identifier[4];      // Bytes 0-3: "MPTG" (magic identifier)
    uint8_t nodeID;             // Byte 4: Device ID (0-5)
    uint8_t rmdType;            // Byte 5: 0x01 = status response
    uint8_t playState;          // Byte 6: 0=STOPPED, 1=PLAYING, 2=PAUSED
    uint8_t currentTrack;       // Byte 7: Current track number (0-255)
    uint8_t currentVolume;      // Byte 8: Current volume (0-30)
    uint32_t packetCount;       // Bytes 9-12: Total packets received (debug)
};
```

**Total: 13 bytes** (4 + 1 + 1 + 1 + 1 + 1 + 4)

---

## RDM Packet Transmission Schedule

### Automatic (Periodic Heartbeat)

Every 2 seconds, each remote sends an unsolicited status packet:

```cpp
// In ProMini_NRF24_MP3.ino loop():
static unsigned long lastStatusUpdate = 0;
if (millis() - lastStatusUpdate > 2000) {
    sendRDMStatus();  // Send heartbeat
    lastStatusUpdate = millis();
}
```

**Purpose**: Hub can detect if device is online/offline

### On-Demand (After Command Processing)

Immediately after processing a DMX command, remote sends status:

```cpp
// In ProMini_NRF24_MP3.ino loop():
if (radio.read(&packet, sizeof(ControlPacket))) {
    if (!isValidPacket(packet)) return;
    
    if (packet.nodeID == 0xFF || packet.nodeID == THIS_NODE_ID) {
        handleControlPacket(packet);  // Process command
        sendRDMStatus();              // Send status immediately
    }
}
```

**Purpose**: Console operator sees device respond immediately to commands

---

## Hub-Side RDM Handling

### Listening for RDM Responses

Hub periodically switches to receive mode and listens for status packets:

```cpp
// In DMX_NRF24_Hub.ino loop():
radio.startListening();     // Switch to RX mode
delay(10);                  // Brief listen window
if (radio.available()) {
    RDMResponsePacket rmdResponse;
    if (radio.read(&rmdResponse, sizeof(RDMResponsePacket))) {
        // Validate and process response
        if (rmdResponse.identifier[0] == 'M' && ...) {  // Check "MPTG"
            // Update device status
            uint8_t nodeID = rmdResponse.nodeID;
            devices[nodeID].playState = rmdResponse.playState;
            devices[nodeID].currentTrack = rmdResponse.currentTrack;
            devices[nodeID].currentVolume = rmdResponse.currentVolume;
            devices[nodeID].lastHeartbeat = millis();  // Reset timeout
            
            // Log to Serial Monitor
            Serial.print("RDM[");
            Serial.print(nodeID);
            Serial.print("]: State=");
            Serial.print(rmdResponse.playState);
            // ...
        }
    }
}
radio.stopListening();      // Switch back to TX mode
```

### Device Status Structure

Hub maintains status for each device:

```cpp
struct DeviceStatus {
    uint8_t nodeID;                     // Device identifier (0-5)
    bool online;                        // Calculated from heartbeat timeout
    uint8_t playState;                  // 0=STOPPED, 1=PLAYING, 2=PAUSED
    uint8_t currentTrack;               // Reported track number
    uint8_t currentVolume;              // Reported volume level
    unsigned long lastHeartbeat;        // Last RDM packet timestamp
    uint32_t packetsReceived;           // Total RDM packets from this device
};

DeviceStatus devices[MAX_REMOTES];  // Array of up to 6 devices
```

### Online/Offline Detection

Hub determines device status based on heartbeat timeout:

```cpp
// Check each device
for (uint8_t i = 0; i < activeRemotes; i++) {
    unsigned long timeSinceHeartbeat = millis() - devices[i].lastHeartbeat;
    if (timeSinceHeartbeat < 5000) {  // 5 second timeout
        devices[i].online = true;
    } else {
        devices[i].online = false;
    }
}
```

**Timeout**: 5 seconds
- If RDM packet arrives within 5 seconds: Device is ONLINE
- If no packet for 5 seconds: Device is OFFLINE

---

## Remote-Side RDM Transmission

### sendRDMStatus() Function

The remote constructs and sends status packet:

```cpp
void sendRDMStatus() {
    RDMResponsePacket response;
    
    // Set magic identifier for validation
    response.identifier[0] = 'M';
    response.identifier[1] = 'P';
    response.identifier[2] = 'T';
    response.identifier[3] = 'G';
    
    // Set device information
    response.nodeID = THIS_NODE_ID;
    response.rmdType = 0x01;           // Status response type
    response.playState = playState;    // Current playback state
    response.currentTrack = currentTrack;
    response.currentVolume = currentVolume;
    response.packetCount = packetsReceived;
    
    // Send to hub
    radio.stopListening();
    radio.openWritingPipe(0xC2C2C2C2C2LL);  // Hub address
    radio.write(&response, sizeof(response));
    radio.startListening();
}
```

### Status Variable Tracking

Remote maintains current state variables that get reported in RDM:

```cpp
// Global variables in ProMini_NRF24_MP3.ino
uint8_t playState = STOPPED;      // 0 = stopped, 1 = playing, 2 = paused
uint8_t currentTrack = 1;         // Track number (1-255)
uint8_t currentVolume = 15;       // Volume (0-30)
uint32_t packetsReceived = 0;     // Total RDM packets (for monitoring)

// Updated by handleControlPacket():
if (cmd == TRANSPORT_PLAY) {
    mp3Serial.write(MP3_PLAY);
    playState = PLAYING;
}
if (cmd == TRANSPORT_STOP) {
    mp3Serial.write(MP3_STOP);
    playState = STOPPED;
}
```

---

## RDM Monitoring & Debugging

### Serial Monitor Output

Hub Serial Monitor shows RDM reception in real-time:

```
RDM from Device 0: State=1 Track=5 Vol=20
RDM from Device 1: State=0 Track=3 Vol=15
RDM from Device 0: State=1 Track=5 Vol=20
RDM from Device 2: State=2 Track=7 Vol=18
```

### Interpreting Status

| State | Meaning |
|-------|---------|
| State=0 | STOPPED (not playing) |
| State=1 | PLAYING (currently playing track) |
| State=2 | PAUSED (playing but paused) |
| Track=0 | No track loaded |
| Track=1-255 | Current track number |
| Vol=0 | Muted |
| Vol=1-30 | Volume level |

### Debugging RDM Issues

**No RDM packets appearing**:
1. Check remote Serial Monitor shows "Node ID: X" at startup
2. Verify remote is powering up (wait 2+ seconds after power)
3. Check hub Serial Monitor shows NRF24L01 initialized
4. Try shorter distance (move remotes next to hub)
5. Verify hub is in listening mode (check `radio.startListening()` is called)

**RDM packets but state not updating**:
1. Send a DMX command (e.g., Play)
2. Check if remote plays the track
3. Check RDM shows playState = 1 (PLAYING)
4. If not, verify `playState` variable is being updated in `handleControlPacket()`

**Device shows offline**:
1. Verify RDM packets are being sent every 2 seconds
2. Check for timeout > 5 seconds
3. Verify wireless connection (may need to move closer)
4. Check power supply (brownout can cause intermittent connection)

---

## RDM Advanced Topics

### Adding More RDM Parameters

To extend RDM with additional data (e.g., battery level, firmware version):

1. Extend `RDMResponsePacket` struct:
```cpp
struct RDMResponsePacket {
    uint8_t identifier[4];
    uint8_t nodeID;
    uint8_t rmdType;
    uint8_t playState;
    uint8_t currentTrack;
    uint8_t currentVolume;
    uint8_t batteryLevel;       // NEW
    uint8_t firmwareVersion;    // NEW
    uint32_t packetCount;
};
```

2. Update `sendRDMStatus()` to populate new fields:
```cpp
response.batteryLevel = readBatteryVoltage();
response.firmwareVersion = FIRMWARE_VERSION;
```

3. Update hub to display/use new data:
```cpp
Serial.print("Battery: ");
Serial.println(rmdResponse.batteryLevel);
```

### RDM Parameter Discovery

Standard RDM allows controllers to query device capabilities. Current implementation is simplified:
- Controllers assume all devices support standard RDM
- No capability/property queries needed
- Hub just logs and displays what remotes send

To implement full RDM discovery protocol:
1. Add RDM command types (IDENTIFY, GET_DEVICE_INFO, etc.)
2. Implement command handlers on remote
3. Add query logic to hub
4. Extend RDMResponsePacket or add separate query/response structures

### RDM Packet Loss Handling

Current system assumes RDM is best-effort (may be lost):
- Hub tracks "last heartbeat" time
- Displays offline if 5+ seconds without packet
- No retry mechanism (no need for feedback acknowledgments)

To add packet loss detection:
1. Have hub send ACK for each RDM packet received
2. Remote counts sent/acknowledged packets
3. Remote reports packet loss percentage in RDM response
4. Hub can log or alert if loss exceeds threshold

---

## RDM Standard Compliance

This implementation is **RDM-inspired but not fully compliant**:

### What's Implemented
✅ Bidirectional communication (hub ↔ remote)  
✅ Device identification (nodeID)  
✅ Status reporting (playState, track, volume)  
✅ Periodic heartbeat  
✅ Packet validation (MPTG identifier)  

### What's Not Implemented
❌ Full RDM discovery protocol  
❌ Device enumeration/enumeration response  
❌ RDM commands (IDENTIFY, GET_DEVICE_INFO, etc.)  
❌ Maximum Response Time (MRT) handling  
❌ Complex RDM parameters (DMX personality, sensor data, etc.)  

### To Use Real RDM

If full RDM compliance is needed:
1. Implement RDM library (e.g., `rdmlib` on GitHub)
2. Remotes act as RDM devices
3. Hub acts as RDM controller/middleware
4. Full RDM protocol handling with querying and discovery
5. More complex, larger sketches, but full standard compatibility

---

## Performance Specifications

### RDM Timing

| Metric | Value |
|--------|-------|
| Periodic Heartbeat Interval | 2000 ms |
| On-Demand Response Latency | <50 ms (immediate) |
| Online/Offline Timeout | 5000 ms |
| Packet Size | 13 bytes |
| Wireless Data Rate | 250 kbps |
| Theoretical Bandwidth | ~19 packets/sec per remote |

### System Capacity

With 6 remotes:
- Hub listens for RDM: ~50 ms per read cycle
- Remotes send: 1 packet every 2 seconds (6 remotes = 3 packets/sec total)
- Hub can easily keep up (50ms RDM check + DMX reading + transmission)

### Scaling Beyond 6 Devices

RDM protocol is independent of device count:
- Add more remotes: increase `MAX_REMOTES` and `activeRemotes`
- Larger `DeviceStatus` array
- More listen cycles in hub loop
- Each additional device adds ~2% overhead (minimal impact)

---

## Troubleshooting RDM

### Slow RDM Response

**Symptom**: RDM packets arrive infrequently (> 5 sec intervals)

**Causes**:
1. Hub not calling `radio.startListening()` frequently
2. Hub loop blocked by other code
3. Wireless interference causing packet loss

**Solutions**:
1. Increase listen frequency in hub loop
2. Reduce blocking operations (delays, long serial reads)
3. Move remotes closer or change NRF24L01 channel

### RDM Packet Corruption

**Symptom**: RDM packets received but identifier check fails

**Causes**:
1. Wireless interference
2. Baud rate mismatch on serial
3. CRC disabled or misconfigured

**Solutions**:
1. Move closer to hub
2. Verify `radio.setCRCLength(RF24_CRC_16)` on both sides
3. Add packet counter to debug intermittent issues

### One Device Sending RDM, Others Not

**Symptom**: Device 0 shows RDM response, but Device 1-5 show offline

**Causes**:
1. Remotes powering up at different times
2. Some remotes have wrong Node ID
3. Wireless range issues for some devices

**Solutions**:
1. Power all remotes simultaneously
2. Check THIS_NODE_ID on each remote (should be 0-5 in sequence)
3. Test wireless range by moving remotes individually
4. Check power supply provides steady current for all modules

---

## RDM Quick Reference

### RDM Packet Structure
```
Byte 0-3:  "MPTG" identifier
Byte 4:    nodeID (0-5)
Byte 5:    rmdType (0x01 = status)
Byte 6:    playState (0=STOP, 1=PLAY, 2=PAUSE)
Byte 7:    currentTrack (1-255)
Byte 8:    currentVolume (0-30)
Byte 9-12: packetCount (debug)
```

### RDM Transmission Events
- **Periodic**: Every 2 seconds (heartbeat)
- **On-Demand**: Immediately after processing DMX command
- **Total**: ~0.5-1 packet per second per device (with both events)

### Hub Handling
- Listens during DMX read gaps
- Updates DeviceStatus array
- Logs to Serial Monitor
- Calculates online/offline based on 5-second timeout
- No storage or logging (RAM only)

### Key Variables (Remote)
```cpp
uint8_t playState;        // 0=STOPPED, 1=PLAYING, 2=PAUSED
uint8_t currentTrack;     // 1-255
uint8_t currentVolume;    // 0-30
uint32_t packetsReceived; // Debug counter
```

### Key Variables (Hub)
```cpp
DeviceStatus devices[6];  // Status for each remote
devices[i].online;        // true if heartbeat < 5sec
devices[i].playState;     // Current playback state
devices[i].lastHeartbeat; // Timestamp of last RDM packet
```

---

## Future RDM Enhancements

### Quick Wins (Easy to Add)
- Add battery voltage reporting
- Add firmware version in RDM
- Add error/status codes
- Add Wi-Fi signal strength (if WiFi future version)

### Medium Effort
- RDM device discovery (hub queries remotes for info)
- Support for device naming/labeling
- Store RDM history in EEPROM
- Report packet loss percentage

### Complex (Major Changes)
- Full RDM protocol compliance
- Web interface for RDM monitoring
- Statistics and analytics dashboard
- Integration with major RDM controllers (ETC, Chauvet, etc.)

---

**RDM Guide Version**: 1.0  
**Last Updated**: 2024  
**Compatibility**: DMX_NRF24_Hub.ino v2.0+, ProMini_NRF24_MP3.ino v2.0+
