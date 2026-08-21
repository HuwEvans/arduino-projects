# ESP32 DMX/ArtNet Controlled MP3 Player - Setup & Usage Guide

## System Overview

The ESP32 DMX/ArtNet Audio Player is a wireless MP3 controller that receives DMX lighting control signals over ArtNet (UDP/WiFi) and converts them into playback commands for a DFRobot Mini MP3 player module.

### Key Features

✅ **ArtNet over WiFi**: Receives DMX via standard ArtNet protocol on port 6454  
✅ **Configurable Universe**: Supports universes 0-15  
✅ **5-Channel Control**: Transport, Track Select, Volume, Trigger, Loop Enable  
✅ **Non-Blocking Architecture**: Responsive command processing without delays  
✅ **WiFi Persistence**: Credentials stored in SPIFFS flash memory  
✅ **Web Configuration UI**: Easy setup via built-in HTTP web server  
✅ **AP Fallback Mode**: Automatically becomes access point if WiFi fails  
✅ **Per-Command Timing**: Independent throttling prevents command interference  

---

## Hardware Requirements

### ESP32 Setup (1 unit)
- **Microcontroller**: ESP32-WROOM DevKit1
- **MP3 Module**: DFRobot Mini MP3 Player
- **Storage**: microSD card with MP3 files (inserted into Mini MP3 module)
- **Power**: USB-C cable for ESP32 (5V/1A minimum) and separate 5V power for Mini MP3
- **Connection**: USB cable to laptop/network for initial setup

### Network Requirements
- **WiFi Router**: 2.4GHz WiFi network (5GHz not currently supported on this code, easily added)
- **ArtNet Controller**: DMX lighting console or software with ArtNet output (uses port 6454)
- **Same Network**: ESP32 and ArtNet controller must be on same WiFi network

---

## Hardware Assembly

### ESP32-WROOM DevKit1 Pinout

```
ESP32-WROOM Pin Layout (for reference):
                    USB
                     |
    GND  D35 D34 ... (many pins) ... EN RST 3.3V
    |                                |   |
    +------ SPI pins --------+       |   |
    |                        |       |   |
    GND 16(RX2) 17(TX2) ... other pins
```

### MP3 Module Wiring

**DFRobot Mini MP3 Player → ESP32-WROOM DevKit1**:

```
Mini MP3 Pin          ESP32 Connection     Purpose
─────────────────────────────────────────────────────
GND                   GND                  Ground
5V                    5V (separate PSU)    Power (do NOT use ESP32 5V!)
RX                    GPIO17 (TX2)         Serial RX from ESP32
TX                    GPIO16 (RX2)         Serial TX to ESP32
Busy (optional)       Not connected        Status feedback (optional)
```

**CRITICAL**: 
- Use **hardware UART Serial2** (GPIO16/17), NOT SoftwareSerial
- Power the Mini MP3 from a **separate 5V source**, NOT ESP32's 5V rail
- Check DFRobot module for correct pin labels (some modules have RX/TX swapped)

### wiring Diagram

```
┌──────────────────────────────────┐
│     ESP32-WROOM DevKit1          │
│                                  │
│  GPIO16 (RX2) ←── TX ── Mini MP3 │
│  GPIO17 (TX2) ──→ RX ── Module   │
│  GND ─────────────┬──→ GND        │
│                   │              │
│              (shared ground)     │
│                   │              │
│ 5V Rail (USB)    ├─ NOT used     │
│                   │              │
└──────────────────┼──────────────┘
                   │
     ┌─────────────┘
     │
     V
┌──────────────────────┐
│  Separate 5V Power   │  (e.g., USB power bank, PSU)
│  (recommended for    │
│   MP3 module only)   │
└──────────────────────┘
```

### microSD Card Preparation

1. Format SD card as FAT32
2. Copy MP3 files to root directory
3. Name files numerically: `001.mp3`, `002.mp3`, `003.mp3`, etc.
4. Insert SD card into Mini MP3 module
5. Device will report total track count on first boot

---

## Software Installation

### Step 1: Install ESP32 Board Support

1. **Arduino IDE** → **File** → **Preferences**
2. **Additional Board URLs**, paste:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Click OK, then **Tools** → **Board** → **Board Manager**
4. Search "esp32", install "ESP32 by Espressif Systems" (latest version)
5. Select **Tools** → **Board** → **ESP32-WROOM-32**

### Step 2: Install Required Libraries

Go to **Sketch** → **Include Library** → **Manage Libraries**, search and install:

1. **ArduinoJson** (by Benoit Blanchon, v6.x or newer)
2. **Artnet** (by Pierre Guilbert)
3. **WiFi.h** - included with ESP32 board package
4. **WebServer.h** - included with ESP32 board package
5. **SPIFFS.h** - included with ESP32 board package

### Step 3: Upload Sketch

1. **Tools** → **Board** → Select **ESP32-WROOM-32**
2. **Tools** → **Port** → Select ESP32 USB COM port
3. Open `ESP-DMX-Audio.ino`
4. **Sketch** → **Upload** (or Ctrl+U)
5. Wait for "Leaving..." message (device will reset)

### Step 4: Monitor Serial Output

1. **Tools** → **Serial Monitor** (115200 baud)
2. Should see startup messages like:
   ```
   === ESP32 MP3 Player with DMX Control ===
   Mounting SPIFFS file system...
   ✓ SPIFFS mounted successfully
   Connecting to WiFi: WiFiSSID (non-blocking)
   ✓ MP3 module initialized
   ```

---

## Initial Configuration

### Via Web Interface (Recommended)

1. **First Boot Behavior**:
   - Device tries to connect to WiFi with default SSID: `WiFiSSID`
   - If WiFi fails, device becomes **Access Point**

2. **Connect to Setup AP**:
   - Open WiFi settings on laptop/phone
   - Find network: **ESP-Audio-Player**
   - Password: **12345678**
   - Connect to it

3. **Open Configuration Page**:
   - Navigate to: `http://192.168.4.1` (when connected to ESP-Audio-Player AP)
   - Web UI loads with configuration form

4. **Configure Settings**:
   - **WiFi SSID**: Your home/venue WiFi network name
   - **WiFi Password**: Your WiFi password
   - **DMX Universe**: 0-15 (usually `0` for most DMX consoles)
   - **DMX Start Address**: First channel in universe (usually `1` or `128`)
   - Click **Save Configuration**

5. **Device Reboots and Connects**:
   - ESP32 restarts and connects to your WiFi network
   - Serial monitor shows: `✓ WiFi connected! IP: 192.168.x.x`
   - Device is now ready for DMX control

### Verify Connected to WiFi

1. Open browser to ESP32's IP address (from serial monitor): `http://192.168.x.x`
2. Configuration page should load
3. Status shows: **WiFi: Connected**
4. Live DMX Values section shows latest channel values

---

## DMX Channel Mapping

The device listens to **5 consecutive DMX channels** starting from the configured DMX Start Address.

| Channel | Purpose | Values | Effect |
|---------|---------|--------|--------|
| **1** (Transport) | Play/Pause/Stop | 0-255 | 0-19=Stop, 20-49=Play, 50-79=Pause, 80+=Pause |
| **2** (Track) | Track Selection | 0-255 | 1=Track1, 2=Track2, ... 255=Track255 |
| **3** (Volume) | Volume Control | 0-255 | 0=Silent, 128=Mid, 255=Max (MP3 hardware 0-30) |
| **4** (Trigger) | Next Track | 0-127/128-255 | Edge detection: rising edge = next track |
| **5** (Loop) | Loop Enable | 0-127=Off, 128-255=On | Enables track loop (firmware feature) |

### Example Configuration in DMX Console

If DMX Start Address = **128**:
- Universe 0, Channel 128 → Transport (Play/Pause/Stop)
- Universe 0, Channel 129 → Track Select
- Universe 0, Channel 130 → Volume
- Universe 0, Channel 131 → Trigger
- Universe 0, Channel 132 → Loop Enable

If DMX Start Address = **1**:
- Universe 0, Channel 1 → Transport
- Universe 0, Channel 2 → Track Select
- Universe 0, Channel 3 → Volume
- Universe 0, Channel 4 → Trigger
- Universe 0, Channel 5 → Loop Enable

---

## Operating the Device

### Basic Controls

**Transport Control (Play/Pause/Stop)**:
- Set DMX Channel 1 to:
  - **0-19**: Stop playback
  - **20-49**: Play current track
  - **50-79**: Pause playback
  - **80-255**: Pause playback

**Select Track**:
- Set DMX Channel 2 to track number (1-255)
- Device will immediately jump to and play that track
- Only changes if value differs from current track

**Volume Control**:
- Set DMX Channel 3 (0-255) → maps to MP3 hardware volume (0-30)
- 0 = silent, 255 = maximum
- Only sends command if value changes

**Next Track Trigger**:
- Toggle DMX Channel 4 from 0-127 to 128-255 (rising edge)
- Each toggle advances to next track
- Works independently of Track Select channel

**Loop Mode**:
- Set DMX Channel 5:
  - **0-127**: Loop disabled
  - **128-255**: Loop enabled (track repeats)

### Real-Time Monitoring

The web interface shows live DMX values:

1. Open browser to ESP32 IP: `http://192.168.x.x`
2. **Live DMX Values** section updates every 1 second
3. Shows each channel's current value and interpretation
4. **Last Update** timestamp indicates when last ArtNet packet was received

### Status Indicators

| Field | Meaning |
|-------|---------|
| **WiFi: Connected** | Device connected to WiFi, ready for ArtNet |
| **WiFi: Disconnected** | Not connected, or fell back to AP mode |
| **DMX Status: Active** | Receiving ArtNet packets (refreshed in last 1 second) |
| **DMX Status: Idle** | No ArtNet packets received yet or timeout |
| **Now Playing: Track N** | Current playback track number |
| **Play State** | PLAYING / PAUSED / STOPPED |

---

## Troubleshooting

### Device Won't Connect to WiFi

**Symptoms**: Serial monitor shows "." dots repeatedly, device enters AP mode

**Solutions**:
1. Verify WiFi SSID and password are correct in web UI
2. Check if WiFi network is 2.4GHz (5GHz not supported)
3. Try with AP closer to ESP32
4. Reset config: Delete SPIFFS and reload sketch (Tools → Erase All Flash)

### No DMX Values Appearing

**Symptoms**: Live DMX Values show "-" or timestamps don't update

**Checks**:
1. Verify device is connected to WiFi (Status → "WiFi: Connected")
2. Check DMX console is sending ArtNet packets to IP 255.255.255.255 (broadcast) or specific device IP
3. Verify DMX Universe setting matches console (usually 0)
4. Check firewall isn't blocking port 6454 (UDP)
5. Try different ArtNet console software to test connectivity

### MP3 Not Playing

**Symptoms**: DMX channels update, but no audio

**Checks**:
1. Verify Mini MP3 module powered correctly (separate 5V source)
2. Check GPIO16/GPIO17 connections (RX2/TX2)
3. Verify microSD card inserted and contains MP3 files
4. Monitor serial output for error messages
5. Try manual command via serial: Set transport to 20+ (play) manually

### Commands Taking Too Long

**Symptoms**: 1+ second delay between DMX change and MP3 response

**Causes & Fixes**:
- **WiFi congestion**: Move closer to AP or reduce interference
- **MP3 module latency**: Normal 30-100ms per command is expected
- **Multiple ArtNet packets**: Each packet queues one command (check console packet rate)

### Device Resets Unexpectedly

**Symptoms**: Serial monitor shows boot messages periodically

**Causes**:
- **Brown-out**: Weak power supply, use quality 5V/1A USB
- **Stack overflow**: Memory issue, check for serial output errors
- **Watchdog timeout**: Blocking code issue (should not occur with current build)

### Can't Access Web UI

**Symptoms**: Browser timeout or connection refused to `http://192.168.x.x`

**Solutions**:
1. Verify device has WiFi IP (from serial monitor)
2. Try pinging device: `ping 192.168.x.x`
3. Try accessing from same WiFi network (not cellular)
4. Check if port 80 is blocked by firewall
5. If fell back to AP mode, use: `http://192.168.4.1`

---

## Advanced Topics

### Changing DMX Universe

1. Open web UI: `http://192.168.x.x`
2. Under **Device Settings**, change **DMX Universe** (0-15)
3. Save and device reboots
4. Configure DMX console to match new universe

### Changing DMX Start Address

1. Open web UI to **Device Settings**
2. Change **DMX Start Address** (default is 128, valid 1-512)
3. Device calculates 5-channel block with new offset
4. Update DMX console patching to match

### Adjusting Command Timing

If commands queue too fast/slow, modify in code:
```cpp
const unsigned long CMD_DELAY_TRANSPORT = 30;  // Play/Pause/Stop (ms)
const unsigned long CMD_DELAY_TRACK = 30;      // Track selection (ms)
const unsigned long CMD_DELAY_VOLUME = 30;     // Volume control (ms)
const unsigned long CMD_DELAY_NEXT_PREV = 30;  // Next/Previous (ms)
```

Values are independent per command type, preventing cascading delays.

### Monitoring SPIFFS Configuration

1. Open serial monitor (115200 baud)
2. On boot, device lists SPIFFS contents:
   ```
   SPIFFS contents:
     - /config.json (xxx bytes)
   ```
3. Config is stored in `/config.json` (JSON format)

### Factory Reset

To erase all configuration:
1. **Arduino IDE** → **Tools** → **Erase All Flash**
2. Re-upload sketch to create fresh config

---

## Testing Checklist

✅ **Hardware**:
- [ ] ESP32 powers on via USB
- [ ] Mini MP3 powers on from separate 5V source
- [ ] No visible smoke or burning
- [ ] Serial monitor shows boot messages at 115200 baud

✅ **WiFi**:
- [ ] Device enters AP mode (`ESP-Audio-Player`)
- [ ] Can access web UI at `192.168.4.1`
- [ ] Can enter WiFi credentials and save

✅ **Connection**:
- [ ] Device connects to home WiFi
- [ ] Serial monitor shows IP address
- [ ] Can access web UI at device IP

✅ **DMX**:
- [ ] ArtNet console can send packets to device
- [ ] Web UI shows live DMX channel updates
- [ ] Timestamp refreshes each second

✅ **Audio**:
- [ ] Set transport to 20+ → MP3 plays
- [ ] Change volume → audio level changes
- [ ] Set track number → changes to different track
- [ ] Toggle trigger → advances to next track

---

## Support & Resources

- **ArtNet Protocol**: Open DMX over IP standard (port 6454)
- **DFRobot Mini MP3**: https://www.dfrobot.com/product-xxx.html
- **ESP32 Pinout**: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/
- **ArduinoJson**: https://arduinojson.org/
- **ArtNet Library**: https://github.com/rstephan/ArtnetWifi

---

**Last Updated**: April 2026  
**Firmware Version**: ESP-DMX-Audio v1.0 (Per-Command Timing)
