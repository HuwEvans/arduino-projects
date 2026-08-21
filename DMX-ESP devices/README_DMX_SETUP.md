# CYD Video Player with ArtNet/DMX Control

This is an enhanced version of the CYD MJPEG video player with wireless ArtNet/DMX control support.

## Required Libraries

Install these libraries via the Arduino IDE Library Manager:

1. **GFX Library for Arduino** (v1.6.0 or later)
   - Used for display control
   
2. **JPEGDEC** (v1.8.2 or later)
   - JPEG decoding library
   
3. **ArduinoJson** (v6.x or later)
   - JSON library for configuration storage
   - Install: "ArduinoJson" by Benoit Blanchon
   
4. **Artnet** (latest version)
   - ArtNet/DMX WiFi library
   - Install: "Artnet" by Pierre Guilbert

## Hardware Setup

- **CYD Display** (Cheap Yellow Display) with ESP32
- **SD Card** with `/mjpeg/` folder containing `.mjpeg` video files
- **WiFi network** with ArtNet controller/sender

## Initial Configuration

### First Boot (No Config File)
1. Power on the device
2. It will create a default configuration file in SPIFFS
3. Default WiFi: tries to connect with placeholder credentials
4. If WiFi connection fails, device starts in **Configuration Mode**

### Accessing Configuration UI
- **URL**: `http://<device-ip>/` 
- **In AP Mode**: Connect to WiFi SSID "CYD-VideoPlayer" with password "12345678", then visit `http://192.168.4.1/`
- **In STA Mode**: Find device IP from your WiFi router and visit `http://<device-ip>/`

### Configuration Settings
- **WiFi SSID**: Your network name
- **WiFi Password**: Your network password
- **DMX Universe**: ArtNet universe (0-15, default: 0)
- **DMX Start Address**: Starting DMX channel address (0-507, default: 128)

## DMX Channel Mapping

After setting the DMX Start Address, the following channels are used:

| Channel | DMX Address | Function | Values |
|---------|-------------|----------|--------|
| 1 | Start+0 | **Transport Control** | 0–19 = Stop, 20–49 = Play, 50–79 = Pause |
| 2 | Start+1 | **Video Select** | 1–255 → Video index (1 = first video, max = video count) |
| 3 | Start+2 | **Brightness** | 0–255 (0 = off, 255 = full brightness) |
| 4 | Start+3 | **Trigger/Skip** | Rising edge (0→255) advances to next video |
| 5 | Start+4 | **Loop Enable** | 0–127 = Loop off, 128–255 = Loop on |

### Example Configuration
If DMX Start Address = 128:
- Channel 1 = DMX address 128 (Transport)
- Channel 2 = DMX address 129 (Video Select)
- Channel 3 = DMX address 130 (Brightness)
- Channel 4 = DMX address 131 (Trigger)
- Channel 5 = DMX address 132 (Loop Enable)

## Behavior & Control Logic

### Transport Control (Channel 1)
- **0–19 (Stop)**: Stops playback, displays current frame
- **20–49 (Play)**: Plays current video normally
- **50–79 (Pause)**: Freezes on current frame (does not stop)

### Video Selection (Channel 2)
- Values from 1 to number of videos loaded
- Example: If you have 5 videos, channel 2 value of 3 selects video 3
- Invalid values are ignored

### Brightness Control (Channel 3)
- Real-time brightness adjustment
- 0 = Backlight off
- 255 = Full brightness

### Trigger/Skip (Channel 4)
- Detects rising edge (transition from ≤127 to >127)
- Each rising edge advances to the next video
- Useful for manual skip control from lighting desk

### Loop Enable (Channel 5)
- 0–127: Auto-loop disabled (plays once then stops)
- 128–255: Auto-loop enabled (repeats from first video)
- Respects selected video from Channel 2 when looping

## Network Modes

### Station Mode (STA - Normal Operation)
- Device connects to WiFi network
- Receives ArtNet packets over network
- Responds to DMX control
- Web UI accessible from network

### Access Point Mode (AP - Configuration Mode)
- Device creates its own WiFi network
- SSID: "CYD-VideoPlayer"
- Password: "12345678"
- IP: 192.168.4.1
- Used when WiFi credentials are incorrect or not configured

### No WiFi Mode (Fallback)
- If WiFi is not available and not in configuration mode
- Device plays videos in auto-loop mode
- Boot button still works to advance videos
- Useful for standalone operation

## Troubleshooting

### Device won't connect to WiFi
1. Check SSID and password in configuration UI
2. Ensure device is within WiFi range
3. Check WiFi network is broadcasting (not hidden)
4. Device will automatically switch to AP mode if connection fails

### No DMX control
- Verify ArtNet controller is sending to correct Universe (default: 0)
- Check DMX Start Address setting matches your controller configuration
- Verify device has received at least one DMX packet (check web UI)
- Look at Serial Monitor for debug messages

### Configuration not saving
- Check SPIFFS is properly initialized (see Serial output)
- Try power cycle after saving
- Check if SPIFFS partition has enough space

### Video playback issues
- Ensure videos are in `/mjpeg/` folder on SD card
- Videos must be in `.mjpeg` format
- Check file names don't have special characters
- Verify SD card is properly inserted

## Serial Monitor Debug Output

Connect to Serial at 115200 baud to see:
- WiFi connection status
- ArtNet initialization
- DMX packet reception
- Video playback statistics
- Configuration changes

## Pins Configuration

- **Display DataBus**: GPIO 2/15/14/13/12
- **SD Card CS**: GPIO 5
- **SD Card SPI**: VSPI (19/23/18)
- **Backlight**: GPIO 21
- **Boot Button**: GPIO 0 (press to skip)

## Notes

- Pause state freezes the current frame and resumes from that frame
- Stop state returns to the first frame of the current video
- Loop setting affects auto-advance behavior only (manual control always works)
- Web server runs every 10ms to ensure responsive control
- DMX updates timeout after 1 second of no packets (reverts to local control)

