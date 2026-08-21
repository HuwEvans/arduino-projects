# Pro Mini Variant Deployment Guide

## Overview

Each Pro Mini remote needs a unique `THIS_NODE_ID` value (0-5) to respond to its dedicated DMX channel bank. This guide walks through creating, compiling, and deploying 6 different sketch variants.

---

## Quick Start (5-Minute Deployment)

### Prerequisites
- Arduino IDE installed with RF24 library
- 6 Arduino Pro Mini boards (5V version)
- 6 NRF24L01 modules with antennas
- 6 DFRobot Mini MP3 modules
- USB FTDI programmer or similar for Pro Mini
- Pro Mini labeling tape/stickers

### Step 1: Reference Sketch
Open `ProMini_NRF24_MP3.ino` in Arduino IDE (Base version with THIS_NODE_ID = 0)

### Step 2-7: Create Variants

Repeat 6 times (once per Pro Mini):

1. **For Device 0**: Leave THIS_NODE_ID = 0
2. **For Device 1**: Change line `#define THIS_NODE_ID 0` → `#define THIS_NODE_ID 1`
3. **For Device 2**: Change to `#define THIS_NODE_ID 2`
4. Set board: "Arduino Pro or Pro Mini"
5. Set processor: "ATmega328P (5V, 16MHz)"
6. Compile and upload to Pro Mini
7. **Label the device with "Device: 1"** (or whatever Node ID)
8. Repeat for Device 3, 4, 5

### Step 3: Verify Each Device

After uploading to each Pro Mini, open Serial Monitor (115200 baud):

**Expected output**:
```
=== Arduino Pro Mini NRF24L01 MP3 Remote ===
Node ID: 0    ← Should match the Device number
Initializing NRF24L01...
NRF24L01 initialized | Remote Address: 0xC2C2C2C2C1
MP3 module initialized
```

---

## Detailed Variant Creation

### Method 1: Arduino IDE (Easiest)

**Step 1: Open base sketch**
```
File → Open → ProMini_NRF24_MP3.ino
```

**Step 2: Make variant copies**

For each device (0-5):
1. Go to line with `#define THIS_NODE_ID 0`
2. Change the number (0 for Device 0, 1 for Device 1, etc.)
3. Use **File → Save As** to create variant:
   - Device 0: Keep original filename
   - Device 1: Save as `ProMini_NRF24_MP3_Device1.ino`
   - Device 2: Save as `ProMini_NRF24_MP3_Device2.ino`
   - etc.

**Step 3: Compile and upload**
1. Connect Pro Mini to FTDI programmer
2. Select correct board and port
3. Click Upload
4. Move to next device

### Method 2: Find & Replace (Faster for Bulk Updates)

If you need to modify all 6 copies at once:

1. Copy `ProMini_NRF24_MP3.ino` → `ProMini_NRF24_MP3_Device0.ino`
2. Open Device0, set THIS_NODE_ID to 0, save
3. Repeat for devices 1-5
4. Use **Find & Replace** in text editor:
   - Replace `THIS_NODE_ID 0` → `THIS_NODE_ID 1` in Device1 file
   - Replace `THIS_NODE_ID 0` → `THIS_NODE_ID 2` in Device2 file, etc.

### Method 3: Automated Script (If Comfortable with Scripting)

Create 6 variants automatically:

**PowerShell** (Windows):
```powershell
$baseContent = Get-Content "ProMini_NRF24_MP3.ino"
for ($i = 0; $i -lt 6; $i++) {
    $content = $baseContent -replace "#define THIS_NODE_ID 0", "#define THIS_NODE_ID $i"
    Set-Content -Path "ProMini_NRF24_MP3_Device$i.ino" -Value $content
}
```

**Bash** (Linux/Mac):
```bash
for i in {0..5}; do
    sed "s/#define THIS_NODE_ID 0/#define THIS_NODE_ID $i/" ProMini_NRF24_MP3.ino > ProMini_NRF24_MP3_Device$i.ino
done
```

---

## Pro Mini Configuration Reference

### Hardware Setup (Same for All Devices)

```
Pro Mini (5V)  →  NRF24L01
Pin 7             CE
Pin 8             CSN
Pin 11            MOSI
Pin 12            MISO
Pin 13            SCK
GND               GND
3.3V (via reg)    VCC

Pro Mini (5V)  →  DFRobot Mini MP3
Pin 10            RX (via voltage divider)
Pin 11            TX
GND               GND
5V                VCC
```

**CRITICAL**: Same hardware for all 6 units - only difference is THIS_NODE_ID in code

### Board Selection (Arduino IDE)

**For all 6 Pro Minis**:
- Board: `Arduino Pro or Pro Mini`
- Processor: `ATmega328P (5V, 16MHz)`
- Port: Select FTDI programmer COM port
- Programmer: `AVRISP mkII` (or appropriate for your programmer)

---

## DMX Channel Assignment Reference

Once all 6 devices are deployed with their respective THIS_NODE_ID values:

| Device Node ID | Listens To Channels | DMX Controls |
|---|---|---|
| 0 | 123-127 (broadcast) + 128-132 | Device 0 channels, plus broadcast |
| 1 | 123-127 (broadcast) + 133-137 | Device 1 channels, plus broadcast |
| 2 | 123-127 (broadcast) + 138-142 | Device 2 channels, plus broadcast |
| 3 | 123-127 (broadcast) + 143-147 | Device 3 channels, plus broadcast |
| 4 | 123-127 (broadcast) + 148-152 | Device 4 channels, plus broadcast |
| 5 | 123-127 (broadcast) + 153-157 | Device 5 channels, plus broadcast |

**Example**: If you upload with `THIS_NODE_ID 3`:
- Device listens to Ch 123-127 (broadcast to all)
- **AND** Ch 143-147 (Device 3 specific commands)

---

## Compilation Tips

### Reduce Sketch Size

If you encounter "Sketch too large" errors:

1. Disable unused features in sketch:
```cpp
// Disable RDM if not needed (not recommended):
// #define DISABLE_RDM

// Use minimal Serial output:
// Comment out debug prints
```

2. Compile with Size Optimization:
- Arduino IDE → File → Preferences
- Check "Show verbose output during compilation"
- Look for: `-Os` (already set by default)

3. Use ATmega328 without bootloader (advanced):
- Requires ISP programmer (USBASP, etc.)
- Can save ~3KB bootloader space

### Verify Compilation

Before uploading, verify sketch compiles:
1. Click **Verify** (checkmark button)
2. Should show: "Sketch uses XXX bytes of program storage space"
3. Should be less than 28KB for Pro Mini ATmega328

If over 28KB: Reduce features or upgrade to ATmega2560

---

## Physical Labeling & Identification

### Label Each Pro Mini

Use stickers or tape to clearly label:

```
Device 0
THIS_NODE_ID: 0
Ch: 128-132 (+ 123-127)
```

```
Device 1
THIS_NODE_ID: 1
Ch: 133-137 (+ 123-127)
```

Etc. for all 6 devices.

### Organize in Travel Case

- Use 6-slot organizer box
- Label each slot with device number
- Include USB cable, power supply, spare cables
- Keep this deployment guide with hardware

---

## Testing Each Variant

### Per-Device Testing Procedure

1. **Power Up Device 0**
   - Watch Serial Monitor: Should show "Node ID: 0"
   - Verify NRF24L01 initialized

2. **Send DMX to Device 0 Only**
   - Set Ch 128 = 40 (PLAY)
   - Wait for RDM response: "State=1"
   - Device 0 plays music
   - Other devices (if powered) do not respond

3. **Test Broadcast**
   - Set Ch 123 = 40 (PLAY)
   - All devices should play
   - All should send RDM responses

4. **Repeat for Each Device (1-5)**
   - Power up Device 1, test Ch 133-137
   - Power up Device 2, test Ch 138-142
   - etc.

### Automated Test Commands (DMX Console)

If you have a DMX console available:

```
Device 0 Test: Set Ch 128 = 50 (PLAY), Ch 129 = 1 (Track 1)
Device 1 Test: Set Ch 133 = 50 (PLAY), Ch 134 = 2 (Track 2)
Device 2 Test: Set Ch 138 = 50 (PLAY), Ch 135 = 3 (Track 3)
...
Broadcast Test: Set Ch 123 = 50 (PLAY), Ch 124 = 5 (Track 5)
```

Each should respond with music playing at that track.

---

## Troubleshooting Variant Issues

### "ERROR: NRF24L01 initialization failed" on Device X

**Cause**: Hardware issue with specific Pro Mini or NRF24L01

**Solutions**:
1. Check wiring on that specific device (pins 7, 8, 11-13)
2. Verify 3.3V voltage regulator is stable (use multimeter)
3. Try different NRF24L01 module (they can be defective)
4. Try uploading to different Pro Mini (may be bad ATmega328)

### Device X Responds to All Channels, Not Just Its Own

**Cause**: THIS_NODE_ID was not correctly changed

**Solution**:
1. Open the sketch file for that device
2. Find line: `#define THIS_NODE_ID 0`
3. Verify it's set to the correct number (0-5)
4. Recompile and re-upload

### Device X Doesn't Respond to Its Channels

**Cause**: Wrong THIS_NODE_ID or wrong DMX channel

**Solutions**:
1. Verify THIS_NODE_ID in sketch (0-5)
2. Verify DMX channel: 123-127 (broadcast) OR (128 + 5*nodeID) to (132 + 5*nodeID)
3. For Device 0: Ch 123-127 or 128-132
4. For Device 3: Ch 123-127 or 143-147
5. Test broadcast channels first (Ch 123 is easiest to test all devices)

---

## Production Deployment Checklist

- [ ] 6 Arduino Pro Mini boards ordered and tested
- [ ] 6 NRF24L01 modules with verified communication
- [ ] 6 DFRobot Mini MP3 modules working
- [ ] 6 sketches created with THIS_NODE_ID 0-5
- [ ] Each sketch compiled successfully (< 28KB)
- [ ] Each variant uploaded to correct Pro Mini
- [ ] Each Pro Mini labeled with device number
- [ ] Serial Monitor tested on each device (shows correct Node ID)
- [ ] Wireless range tested (minimum 5 meters)
- [ ] RDM feedback working (Serial Monitor on hub shows responses)
- [ ] Per-device DMX channels tested (123-127 broadcast, then 128+)
- [ ] All 6 devices tested together (no interference)
- [ ] Power supply verified (can handle 6 devices simultaneously)
- [ ] Mounting solution ready (enclosure, cables, connectors)
- [ ] Documentation printed and included with hardware

---

## Mass Deployment (If Building Multiple Systems)

If building more than one hub+6-remote system:

### Prepare Sketches Once
1. Create 6 sketch variants (Device 0-5)
2. Test variants on reference hardware
3. Save variants to cloud/backup

### Deploy to Multiple Sets
1. Use same sketches for all systems
2. Each system gets 1 hub + 6 remotes (all identical)
3. Hub sketch is same for all systems
4. Remote sketches are same THIS_NODE_ID variant across all systems

### Parts Inventory
```
Per System:
- 1 Arduino Mega 2560 (hub)
- 6 Arduino Pro Mini (remotes)
- 7 NRF24L01 modules (1 hub + 6 remotes)
- 6 DFRobot Mini MP3 modules
- 1 CF Robot DMX Shield (optional)
- USB power supplies (1 for hub, potentially 1 per remote)
- Wireless antennas (optional upgrades)
- Mounting hardware
```

---

## Quick Reference: THIS_NODE_ID Values

| Sketch File | THIS_NODE_ID | DMX Channels | Loaded To |
|---|---|---|---|
| ProMini_NRF24_MP3_Device0.ino | 0 | 128-132 + Broadcast | Pro Mini #0 |
| ProMini_NRF24_MP3_Device1.ino | 1 | 133-137 + Broadcast | Pro Mini #1 |
| ProMini_NRF24_MP3_Device2.ino | 2 | 138-142 + Broadcast | Pro Mini #2 |
| ProMini_NRF24_MP3_Device3.ino | 3 | 143-147 + Broadcast | Pro Mini #3 |
| ProMini_NRF24_MP3_Device4.ino | 4 | 148-152 + Broadcast | Pro Mini #4 |
| ProMini_NRF24_MP3_Device5.ino | 5 | 153-157 + Broadcast | Pro Mini #5 |

---

## Support & Debugging

### Serial Monitor Test Output

**Good (Device 0)**:
```
=== Arduino Pro Mini NRF24L01 MP3 Remote ===
Node ID: 0
Initializing NRF24L01...
NRF24L01 initialized | Remote Address: 0xC2C2C2C2C1
MP3 module initialized
```

**Bad (Wrong THIS_NODE_ID)**:
```
=== Arduino Pro Mini NRF24L01 MP3 Remote ===
Node ID: 0    ← Still shows 0 after recompiling!
```
→ Recompile with correct THIS_NODE_ID and upload again

**Bad (Hardware Issue)**:
```
=== Arduino Pro Mini NRF24L01 MP3 Remote ===
Node ID: 3
Initializing NRF24L01...
ERROR: NRF24L01 initialization failed!
```
→ Check NRF24L01 wiring, power supply, or try different module

---

## Advanced: Custom THIS_NODE_ID Scheme (Optional)

Instead of 0-5, you could use non-sequential IDs:

```cpp
// Example: Use device serial numbers as THIS_NODE_ID
#define THIS_NODE_ID 10  // "Device 10" in some numbering system
```

However, this requires:
1. Corresponding change in hub DMX channel mapping
2. Modification of hub sketch to use custom nodeID ranges
3. More complex deployment tracking

**Recommendation**: Stick with 0-5 (simplest, matches default channel layout)

---

**Variant Deployment Guide Version**: 1.0  
**Last Updated**: 2024  
**Compatibility**: ProMini_NRF24_MP3.ino v2.0+
