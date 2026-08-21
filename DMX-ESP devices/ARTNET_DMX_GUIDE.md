# Art-Net DMX Control Guide for ESP32 DFPlayer

## Overview
The ESP-DMX-Audio project now supports **Art-Net Protocol** for controlling the MP3 player via DMX channels over Ethernet. This allows professional DMX controllers and lighting consoles to command audio playback.

## Installation Requirements

### Libraries to Install
1. **DFRobotDFPlayerMini** - For MP3 control
2. **Art-Net** - For DMX-over-Ethernet
3. **ArduinoJson** - For configuration
4. **WiFi** - Built-in ESP32 library

#### Installation Steps:
**Arduino IDE:**
1. Sketch → Include Library → Manage Libraries
2. Search for `Art-Net` (by Christoph Guillermet)
3. Install the library
4. Repeat for `DFRobotDFPlayerMini` if not already installed

**PlatformIO:**
```ini
lib_deps = 
    DFRobotDFPlayerMini
    Art-Net
    ArduinoJson
```

## Network Setup

### ESP32 Configuration
The ESP32 must be connected to your network for Art-Net to work:

1. **WiFi SSID**: Update in `/config.json` or default "HomeWifi"
2. **WiFi Password**: Update credentials before uploading
3. **IP Address**: Assigned via DHCP (will be logged to serial monitor)

### Art-Net Configuration
- **Universe**: 0 (DMX Universe, can be changed in code)
- **Port**: UDP 6454 (standard Art-Net port)
- **Protocol**: Art-Net IV
- **Ethernet**: Via WiFi module in the ESP32

## DMX Channel Mapping

The following DMX channels control the MP3 player:

### Channel 1: Volume Control
- **Range**: 0-255 (8-bit)
- **Mapping**: 0-255 → 0-30 (DFPlayer volume scale)
- **Function**: Adjust playback volume
- **Example**: 
  - Value 0 = Muted (0/30)
  - Value 128 = Mid volume (~15/30)
  - Value 255 = Maximum (30/30)

### Channel 2: Track Selection
- **Range**: 0-255 (8-bit)
- **Mapping**: Direct (1-255 track numbers)
- **Function**: Select and play a specific MP3 track
- **Behavior**: Track 0 = no change, Tracks 1-255 = play that track
- **Example**:
  - Value 0 = No action
  - Value 1 = Play track 1
  - Value 50 = Play track 50
  - Value 255 = Play track 255

### Channel 3: Control Commands
- **Range**: 0-255 (8-bit)
- **Function**: Execute playback commands
- **Mapping**:
  | Value Range | Command | Action |
  |-------------|---------|--------|
  | 0-50 | STOP | Stop playback immediately |
  | 51-100 | PLAY | Resume or start playback |
  | 101-150 | PAUSE | Pause current track |
  | 151-200 | NEXT | Skip to next track (loops to track 1) |
  | 201-255 | PREVIOUS | Go to previous track (loops to last) |

- **Examples**:
  - Value 25 = STOP
  - Value 75 = PLAY
  - Value 125 = PAUSE
  - Value 175 = NEXT
  - Value 230 = PREVIOUS

### Channel 4: Status Feedback (Read-Only)
- **Not yet implemented** - Reserved for future status output

## DMX Console Configuration Examples

### Using QLC+ (Free DMX Software)
1. Open QLC+
2. Create a new Virtual Console
3. Add 3 faders:
   - Fader 1 → Universe 0, Channel 1 (Volume)
   - Fader 2 → Universe 0, Channel 2 (Track)
   - Fader 3 → Universe 0, Channel 3 (Control)
4. Set Art-Net output:
   - Preferences → Input/Output → Art-Net (add)
   - Set IP address to ESP32 IP (e.g., 192.168.1.100)
   - Click "Configure"
5. Start output and move faders

### Using a Physical DMX Console (e.g., ETC Eos, MA Lighting)
1. Configure Art-Net output to ESP32's IP address
2. Set universe output to Universe 0
3. Create cues or manually adjust channels:
   ```
   Channel 1 @ 128 → Medium volume
   Channel 2 @ 50 → Play track 50
   Channel 3 @ 75 → Play command
   ```

## Serial Monitor Output Examples

When Art-Net data is received, you'll see:
```
>>> ACTION: Play track 50
DMX: Volume changed to 15 (from channel 1 value 128)
DMX: Track changed to 50 (from channel 2 value 50)
DMX: PLAY command received (value: 75)
```

When no Art-Net data is received for 5 seconds:
```
WARNING: Art-Net connection timeout - no DMX data received
```

## Troubleshooting

### No Art-Net data received
1. **Check WiFi Connection**: Verify ESP32 is connected to WiFi (blue LED should be ON)
   - Open serial monitor to see the assigned IP address
2. **Check Network**: Ping the ESP32 from your DMX console computer
3. **Check Firewall**: Ensure UDP port 6454 is not blocked
4. **Verify Art-Net Configuration**: Confirm console is sending to correct IP/Universe

### Art-Net connected but commands not working
1. **Check DMX Channels**: Verify you're adjusting channels 1-3
2. **Check Value Ranges**: Ensure values are within 0-255
3. **Check Command Mappings**: Refer to Channel 3 value ranges above
4. **Serial Monitor**: Watch serial output for error messages

### Volume not changing
- Channel 1 is receiving but playback is paused/stopped
- Check current playback state on serial monitor

### Track won't play
- Verify track number exists on SD card (check total track count in logs)
- Ensure SD card is inserted and formatted with MP3 files
- Try manually setting a lower track number first

## Web UI Status

The web UI is still functional as a secondary control method but **DMX control takes priority**. You can use:
- **http://{ESP32_IP}/**: Web interface for manual control
- **http://{ESP32_IP}/api/status**: JSON status endpoint

Both UIs will show the current playback state updated by DMX commands.

## Typical DMX Lighting Console Workflow

### Setup
```
1. Upload sketch to ESP32
2. Power on ESP32, check blue LED (WiFi connected)
3. Note ESP32 IP address from serial monitor
4. Configure Art-Net in your DMX console to ESP32 IP
5. Select Universe 0
```

### Control
```
Adjust Channel 1 → Volume changes
Adjust Channel 2 → Track number changes
Adjust Channel 3 → Execute commands (Play/Stop/Pause/Next/Prev)
```

### Example Cue
```
Channel 1: 200 (volume ≈ 24/30)
Channel 2: 42  (track 42)
Channel 3: 75  (PLAY)
Result: Plays track 42 at volume 24
```

## Technical Details

### Frame Format
```
Art-Net DMX Frame:
- Universe: 0
- Data: 512 channels (only 1-3 used)
- Start Code: 0x00 (DMX)
- Refresh Rate: ~30-44Hz (typical for Art-Net)
```

### Timeout Behavior
- **No DMX data for 5 seconds**: ESP32 logs a timeout warning
- **Current state maintained**: The player does not stop; it continues playing
- **DMX reconnect**: Resumes control when data arrives again

## Future Enhancements

Possible future improvements:
- [ ] Channel 4: Status feedback (playing/stopped/paused)
- [ ] Channel 5-10: Preset buttons (store/recall cues)
- [ ] Channel 11+: Additional controls (EQ, effects, etc.)
- [ ] Multiple universes support
- [ ] sACN/E.131 protocol support

## Reference Links

- **Art-Net Library**: https://github.com/rstephan/ArtNet
- **DMX512 Standard**: https://en.wikipedia.org/wiki/DMX512
- **QLC+ Software**: https://www.qlcplus.org/
- **Art-Net Specification**: https://art-net.org.uk/

## Version Info
- **Updated**: April 2026
- **Sketch**: ESP-DMX-Audio.ino
- **Art-Net Support**: Yes
- **DMX Channels Used**: 3 (Channels 1-3)
- **Supported Universes**: 0 (configurable)
