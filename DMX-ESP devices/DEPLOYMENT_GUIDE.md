# DMX NRF24L01 Wireless System - Complete Deployment Guide

## System Overview

This is a two-tier wireless DMX control system:
- **Hub**: Receives DMX via hardware shield or serial, broadcasts to multiple remotes
- **Remotes**: Up to 6 wireless units, each controls MP3 playback independently or via broadcast

### Key Features

✅ **Per-Device Control**: Each device responds to its own dedicated 5-channel DMX bank  
✅ **Broadcast Control**: All devices respond to broadcast channels (Ch 123-127)  
✅ **RDM Feedback**: Devices report playback status back to hub (every 2 seconds + after commands)  
✅ **Authentication**: All packets verified with "MPTG" security identifier  
✅ **Hardware DMX Support**: Optional CF Robot DMX Shield with serial fallback  
✅ **Long Range**: 2.4GHz, 250kbps for reliable ~100m operation

---

## Hardware Requirements

### Hub (1 unit required)
- Arduino Mega 2560
- NRF24L01 wireless module (2.4GHz)
- **Optional**: CF Robot DMX Shield for hardware DMX input
- **Fallback**: USB to RS485 serial adapter connected to Serial1 (pins 18-19)
- 3.3V voltage regulator for NRF24L01

### Per Remote (up to 6 units)
- Arduino Pro Mini (5V version recommended)
- NRF24L01 wireless module (2.4GHz)
- DFRobot Mini MP3 player module
- microSD card with MP3 files
- 3.3V voltage regulator for NRF24L01

---

## Hardware Assembly

### NRF24L01 Module Wiring

**Hub (Mega 2560)**:
```
NRF24L01  →  Arduino Mega 2560
GND           GND (with capacitor to +3.3V)
VCC           3.3V (via voltage regulator, min 10µF capacitor)
CE            Pin 7
CSN           Pin 8
MOSI          Pin 51 (SPI)
MISO          Pin 50 (SPI)
SCK           Pin 52 (SPI)
```

**DMX Serial (Mega 2560 Serial1)**:
```
DMX Gateway  →  Arduino Mega 2560
GND             GND
RX (from PC)    Pin 19 (RX1 input)
TX (to PC)      Pin 18 (TX1 output)
```

**Remote (Pro Mini)**:
```
NRF24L01  →  Arduino Pro Mini
GND           GND
VCC           3.3V (via voltage regulator, min 10µF capacitor)
CE            Pin 7
CSN           Pin 8
MOSI          Pin 11 (SPI)
MISO          Pin 12 (SPI)
SCK           Pin 13 (SPI)
```

### DFRobot Mini MP3 Module Wiring (Pro Mini)

```
Mini MP3  →  Arduino Pro Mini
GND           GND
5V            5V
RX            Pin 10 (SoftwareSerial RX)
TX            Pin 11 (SoftwareSerial TX)
Busy (opt)    Pin 12
```

**Important**: Use voltage divider on MP3 RX pin if powering from 5V:
- 10kΩ + 20kΩ divider to step down Pro Mini 5V TX to ~3.3V for MP3 RX

### CF Robot DMX Shield (Optional, for Hub)

- Sits on top of Arduino Mega 2560 as shield
- No additional wiring needed beyond standard shield connectors
- XLR5 or XLR3 DMX input connector

---

## Software Installation

### Step 1: Install Required Libraries

**For Hub** (Arduino IDE → Sketch → Include Library → Manage Libraries):
- `RF24` by TMRh20 (NRF24L01 driver)
- `DmxShield` by CFRobot (for DMX hardware support) - *Install: https://github.com/cfrobot/DmxShield*

**For Remotes**:
- `RF24` by TMRh20

### Step 2: Upload Hub Sketch

1. Open `DMX_NRF24_Hub.ino`
2. Verify Arduino board selected is "Arduino Mega 2560"
3. Select correct COM port
4. Compile (Ctrl+R)
5. Upload (Ctrl+U)
6. Open Serial Monitor (115200 baud) to verify startup messages:

```
=== DMX NRF24L01 Hub (Mega 2560) ===
Serial0 (USB): Monitoring/Debug Output
Serial1 (Pins 18-19): DMX Serial Input
Checking for DMX Shield...
[DMX Shield detected] - or -
[DMX Shield not detected - using Serial1 DMX input (5-byte packets on pins 18-19)]
Initializing NRF24L01...
NRF24L01 initialized successfully | Channel: 76 | Data Rate: 250kbps | PA Level: Max
Active remotes: 6

=== Hardware Configuration ===
Board: Arduino Mega 2560
NRF24L01 CE: Pin 7
NRF24L01 CSN: Pin 8
DMX Input: [CF Robot Shield (SPI) or Serial1 (RX1=Pin 19, TX1=Pin 18)]
```

### Step 3: Upload Remote Sketches (One per Pro Mini)

**CRITICAL**: Each Pro Mini needs **THIS_NODE_ID** set to a unique value (0-5)

**For Device 0**:
1. Open `ProMini_NRF24_MP3.ino`
2. Find line: `#define THIS_NODE_ID 0`
3. Keep as `0` (or change for different device slot)
4. Select "Arduino Pro Mini" board, 5V, 16MHz
5. Compile and Upload
6. Open Serial Monitor to verify:

```
=== Arduino Pro Mini NRF24L01 MP3 Remote ===
Node ID: 0
Initializing NRF24L01...
NRF24L01 initialized | Remote Address: 0xC2C2C2C2C1
MP3 module initialized
```

**For Device 1, 2, 3, 4, 5**:
1. Edit line to: `#define THIS_NODE_ID 1` (etc.)
2. Compile and upload to next Pro Mini
3. **Label each Pro Mini with its device number** (0-5) for future reference

---

## DMX Channel Architecture

### Broadcast Channels (123-127)
Control **all devices** simultaneously:

| Channel | Function | Values |
|---------|----------|--------|
| 123 | Transport | 0-19=STOP, 20-49=PLAY, 50-79=PAUSE |
| 124 | Track Select | 1-255 (track number) |
| 125 | Volume | 0-255 (0=mute, 255=max) |
| 126 | Trigger | 0-127=low, 128-255=next/skip |
| 127 | Loop Enable | 0-127=off, 128-255=on |

### Per-Device Channels

**Device 0**: Channels 128-132 (same layout as broadcast)  
**Device 1**: Channels 133-137  
**Device 2**: Channels 138-142  
**Device 3**: Channels 143-147  
**Device 4**: Channels 148-152  
**Device 5**: Channels 153-157

Each device-specific bank has the same 5 channels as broadcast:
1. Transport (0-19=STOP, 20-49=PLAY, 50-79=PAUSE)
2. Track Select (1-255)
3. Volume (0-255)
4. Trigger (0-127=low, 128-255=high)
5. Loop Enable (0-127=off, 128-255=on)

### Example Usage (with Lighting Console)

**Play Track 5 on all devices**:
- Set Ch 123 = 40 (PLAY)
- Set Ch 124 = 5 (Track 5)

**Play Track 3 on Device 1 only**:
- Set Ch 133 = 40 (PLAY)
- Set Ch 134 = 3 (Track 3)

**Different audio on each device**:
- Ch 128-132: Device 0 → Track 1, Volume 200
- Ch 133-137: Device 1 → Track 5, Volume 180
- Ch 138-142: Device 2 → Track 2, Volume 220

---

## RDM (Remote Device Management) Status Monitoring

### What RDM Does

Devices report status back to hub every 2 seconds (or immediately after processing a command):
- **Play State**: STOPPED (0), PLAYING (1), PAUSED (2)
- **Current Track**: Track number (1-255)
- **Volume**: 0-30
- **Packet Count**: Total packets received (debug info)

### Hub Status Display

Open Serial Monitor at 115200 baud to see RDM status updates:

```
RDM from Device 0: State=1 Track=5 Vol=20  [Playing Track 5 at Volume 20]
RDM from Device 2: State=0 Track=0 Vol=0   [Stopped]
RDM from Device 1: State=2 Track=3 Vol=15  [Paused on Track 3]
```

### Online/Offline Status

Hub automatically tracks which devices are responding:
- Heartbeat timeout: 5 seconds (if no RDM packet in 5s, device shows as offline)
- Use Serial Monitor to monitor RDM responses
- If no status for 5+ seconds, device may have lost wireless connection or power

---

## Testing Procedures

### Initial Power-On Test

1. **Power hub first**, watch Serial Monitor for startup messages
2. **Power remotes one at a time**, verify each shows initialization and displays its Node ID
3. Remove power from serial terminal or DMX input
4. Verify each radio link is solid (~1-2 seconds after power, RDM packet should appear)

### Wireless Range Test

1. Keep hub and one remote powered and in close proximity (1 meter)
2. Send DMX commands (use Serial Monitor or lighting console)
3. Watch for RDM responses confirming packet receipt
4. Gradually increase distance and monitor packet loss
5. Typical range: 30-100m line-of-sight with NRF24L01

### Per-Device Control Test

1. Set Ch 128 = 40 (PLAY) on lighting console
2. Verify only Device 0 responds
3. Set Ch 133 = 40 (PLAY)
4. Verify only Device 1 responds
5. Set Ch 123 = 40 (PLAY)
6. Verify all devices respond

### Broadcast Control Test

1. Set Ch 123 = 40 (PLAY)
2. Verify all devices start playing
3. Set Ch 125 = 200 (Volume)
4. Verify all devices reach same volume
5. Watch Serial Monitor for RDM packets from all devices

---

## Troubleshooting

### "ERROR: NRF24L01 initialization failed!"

**Cause**: NRF24L01 not communicating with Arduino

**Solutions**:
- Check CE and CSN pin connections (Mega 2560: 7, 8)
- Verify 3.3V voltage regulator is providing stable 3.3V
- Add 10µF capacitor on NRF24L01 VCC to GND
- Try different NRF24L01 module (they can be defective)
- Check that SPI pins (50, 51, 52 on Mega 2560) are not used for other functions

### No RDM Responses Appearing

**Cause**: Wireless communication not working

**Solutions**:
- Verify both hub and remotes show "NRF24L01 initialized successfully"
- Check that Node ID on remote matches the device slot
- Power cycle hub and remotes (power hub first)
- Move remotes closer to hub (test at 1 meter range first)
- Check antenna orientation on NRF24L01 modules
- Verify remote is in listening mode (wait 2+ seconds after power)

### DMX Input Not Working (Hub)

**Cause**: Shield detection failed or Serial1 input not configured

**Solutions**:
- If using hardware shield: Verify shield is fully seated on Mega 2560
- If using serial DMX: Verify serial adapter is connected to Mega pins 18 (RX1) and 19 (TX1)
- Check Serial Monitor shows "DMX Shield detected" or "Using Serial1 DMX input"
- For serial input, test with 5-byte test packets from a PC terminal on the correct COM port

### Remotes Not Responding to Commands

**Cause**: Incorrect Node ID or channel mapping

**Solutions**:
- Verify THIS_NODE_ID on each Pro Mini matches its device slot (0-5)
- Check DMX console is sending to correct channel (128+ for device, 123-127 for broadcast)
- Watch Serial Monitor on hub to verify packets are being sent
- Watch Serial Monitor on remote to verify packet reception

### Only Some Remotes Responding

**Cause**: Wireless range or interference

**Solutions**:
- Move remotes closer to hub (2.4GHz signal can be blocked by metal, water, dense rooms)
- Check for WiFi interference (change NRF24L01 channel if needed - modify `radio.setChannel(76)`)
- Verify power supply has enough current for multiple NRF24L01 modules
- Use external antenna version of NRF24L01 for better range

### Serial Monitor Garbled Output

**Cause**: Wrong baud rate

**Solutions**:
- Hub: Set Serial Monitor to **115200 baud** (not 9600)
- Check Tools → Port for correct COM port
- Disconnect/reconnect USB cable

---

## Advanced Configuration

### Adding More Than 6 Devices

Current architecture supports up to 6 devices (Node ID 0-5). To extend:

1. In `DMX_NRF24_Hub.ino`, change `#define MAX_REMOTES 6` to higher value
2. Add entries to `remoteAddresses[MAX_REMOTES]` array with unique addresses
3. Extend DMX channel layout (each device requires 5 channels, starting at 128)
4. Increase `DeviceStatus devices[MAX_REMOTES]` array size
5. Recompile and test

### Changing Wireless Channel

To avoid WiFi interference:

1. In both hub and remote sketches, find `radio.setChannel(76)`
2. Change to different channel (0-125, in 2MHz increments)
3. **Use same channel on hub and all remotes**
4. Recompile and upload both

Recommended alternatives: 10, 40, 80, 110 (avoids standard WiFi channels)

### Enabling Encryption

Current system uses "MPTG" identifier for authentication. For true encryption:
1. NRF24L01 doesn't have built-in encryption
2. Could XOR payload with a key before transmission
3. Would require matching key on both hub and remotes
4. Adds complexity; "MPTG" is sufficient for amateur/venue use

---

## System Limitations & Future Enhancements

### Current Limitations
- Maximum 6 devices supported (architecture allows up to 32-48)
- No automatic device discovery (devices are hardcoded)
- RDM parameters limited to playback status
- No safety timeout (if hub stops sending, remotes continue last command)

### Future Enhancements (Not Implemented)
- Add more RDM parameters (battery level, firmware version, error codes)
- Web interface on hub to display device status in real-time
- Safety timeout: Auto-stop music if no packets received for >N seconds
- Device discovery: Hub automatically detects which remotes are online
- EEPROM storage of Node ID (program once, no USB uploads needed)
- Extend beyond 6 devices
- Add MIDI or Ethernet support to hub

---

## Files Overview

| File | Purpose | Upload To |
|------|---------|-----------|
| `DMX_NRF24_Hub.ino` | Central DMX receiver & broadcaster | Arduino Mega 2560 (1 unit) |
| `ProMini_NRF24_MP3.ino` | Wireless remote with MP3 control | Arduino Pro Mini (up to 6 units) |
| `ESP-DMX-Audio.ino` | ESP8266 alternative (separate project) | ESP8266 (optional) |
| `cyd-videoplayer-dmx.ino` | CYD video player (separate project) | ESP32 CYD (optional) |

---

## Support & Testing

### Verify System Components

**Test NRF24L01 module**:
- Use RF24 library examples to confirm module works
- Test communication between two Arduino boards with NRF24L01

**Test DMX Shield**:
- Use CF Robot examples if shield is present
- Verify shield can read DMX channels correctly

**Test MP3 Module**:
- Use DFRobot examples to confirm module responds to serial commands
- Test basic play/pause/track commands

### Expected Performance

| Parameter | Value |
|-----------|-------|
| Wireless Range | 30-100m line-of-sight |
| Data Rate | 250 kbps |
| Latency | ~20-50ms (one-way) |
| Update Rate | 100ms (DMX) or 2000ms (RDM) |
| Power per NRF24L01 | ~10-20mA (RX), ~30-50mA (TX) |
| Power per Hub | ~100-150mA |
| Power per Remote | ~80-120mA |

---

## License & Attribution

- RF24 library by TMRh20: https://github.com/nRF24/RF24
- DmxShield library by CFRobot: https://github.com/cfrobot/DmxShield
- Custom DMX/RDM integration: Original work for this project

---

## Quick Reference

### Hub Sketch Setup
```cpp
#define MAX_REMOTES 6          // Up to 6 wireless remotes
#define DMX_BROADCAST_BASE 123 // Broadcast channels
#define SERIAL_BAUD 115200     // Debug output
radio.setChannel(76);          // NRF24L01 Frequency
```

### Remote Sketch Setup
```cpp
#define THIS_NODE_ID 0  // Change to 0-5 for each device
// Device 0: Listens to Ch 128-132 + broadcast (123-127)
// Device 1: Listens to Ch 133-137 + broadcast (123-127)
// etc.
```

### DMX Channel Quick Lookup

| Function | Broadcast | Device 0 | Device 1 | Device 2 | Device 3 | Device 4 | Device 5 |
|----------|-----------|----------|----------|----------|----------|----------|----------|
| Transport | 123 | 128 | 133 | 138 | 143 | 148 | 153 |
| Track | 124 | 129 | 134 | 139 | 144 | 149 | 154 |
| Volume | 125 | 130 | 135 | 140 | 145 | 150 | 155 |
| Trigger | 126 | 131 | 136 | 141 | 146 | 151 | 156 |
| Loop | 127 | 132 | 137 | 142 | 147 | 152 | 157 |

---

**Document Version**: 1.0  
**Last Updated**: 2024  
**System Status**: Production Ready
