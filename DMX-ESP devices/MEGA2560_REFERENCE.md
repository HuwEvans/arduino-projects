# Arduino Mega 2560 Hub - Quick Reference

## Overview

The DMX NRF24L01 Hub has been upgraded to use the **Arduino Mega 2560** instead of Uno/Nano, enabling dual serial ports for simultaneous monitoring and DMX input.

---

## Key Differences from Uno/Nano

| Feature | Uno/Nano | Mega 2560 |
|---------|----------|-----------|
| Processor | ATmega328P | ATmega2560 |
| Flash Memory | 32KB | 256KB |
| RAM | 2KB | 8KB |
| Serial Ports | 1 (USB) | 4 (3 hardware + 1 USB) |
| SPI Pins | 11, 12, 13 | 50, 51, 52 |
| **NRF24L01 CE** | **Pin 9** | **Pin 7** |
| **NRF24L01 CSN** | **Pin 10** | **Pin 8** |
| **Serial0 (USB)** | Hardware UART | Hardware UART (monitoring) |
| **Serial1** | N/A | **Pins 18-19 (DMX input)** |
| **Serial2** | N/A | Pins 16-17 (available) |
| **Serial3** | N/A | Pins 14-15 (available) |

---

## Serial Port Configuration

### Serial0 (USB) - For Monitoring
```
Purpose: Debug output and status monitoring
Baud Rate: 115200
Hardware: Native USB via FTDI/CH340
Output: Status messages, RDM responses, debug info
Connect: USB cable to PC for monitoring
```

### Serial1 (Pins 18-19) - For DMX Input
```
Purpose: Receive DMX data from external gateway/receiver
Connect RX Line (data in): Pin 19 (RX1)
Connect TX Line (data out): Pin 18 (TX1)
Baud Rate: 115200 (configurable)
Packet Format: 5-byte packets [CH1, CH2, CH3, CH4, CH5]
Fallback When: CF Robot DMX Shield not detected
```

---

## Wiring Diagram

### NRF24L01 Module
```
                    Mega 2560
                   ┌────────┐
              GND -┤GND     │ 
             +3.3V-┤A0/GND  │
                   │        │
           CE  P7 -┤7       │
          CSN  P8 -┤8       │
                   │        │
         MOSI P51 -┤51      │
         MISO P50 -┤50      │
          SCK P52 -┤52      │
                   │        │
            NRF24 -┤GND     │
      (GND return) │        │
                   └────────┘
```

### Serial1 DMX Connection
```
              DMX Gateway
                ┌─────┐
            GND ┤GND  │
                │     │
         RX OUT ┤RX   │---→ Pin 19 (RX1) on Mega
         TX IN  ┤TX   │←--- Pin 18 (TX1) on Mega
                └─────┘
```

### Complete Hub Assembly
```
USB Cable (Monitoring)
     │
     ↓
 Mega 2560
     ├─→ Pin 7, 8, SPI (NRF24L01)
     └─→ Pin 18, 19 (Serial1 for DMX)
            │
            ↓
        Serial DMX Gateway
        (or CF Robot Shield)
            │
            ↓
        DMX Console
```

---

## Pin Assignment Summary

| Mega 2560 Pin | Function | Device | Notes |
|---|---|---|---|
| GND | Ground | All | Common ground required |
| 5V | Power | Mega only | Not used for external devices |
| 3.3V | Power | NRF24L01 via regulator | Via voltage regulator |
| 7 | NRF24L01 CE | Wireless module | Chip Enable |
| 8 | NRF24L01 CSN | Wireless module | Chip Select |
| 18 | Serial1 TX | DMX Gateway | To gateway RX (if Serial1 used) |
| 19 | Serial1 RX | DMX Gateway | From gateway TX (if Serial1 used) |
| 50 | SPI MISO | NRF24L01 | Shared SPI line |
| 51 | SPI MOSI | NRF24L01 | Shared SPI line |
| 52 | SPI SCK | NRF24L01 | Shared SPI line |

---

## Operational Modes

### Mode 1: Hardware DMX Shield (Preferred)
- CF Robot DMX Shield mounted on Mega 2560
- Directly reads XLR DMX input
- Serial1 unused (available for other devices)
- Baud rate irrelevant (hardware SPI interface)

**Startup Message**:
```
Checking for DMX Shield...
DMX Shield detected - using hardware DMX interface
```

### Mode 2: Serial1 DMX Gateway Input (Fallback)
- External DMX-to-Serial adapter on Serial1 (pins 18-19)
- Receives 5-byte DMX packets via serial
- Serial1 baud rate: 115200
- Useful if DMX Shield unavailable

**Startup Message**:
```
Checking for DMX Shield...
DMX Shield not detected - using Serial1 DMX input (5-byte packets on pins 18-19)
```

---

## Serial Monitor Output Examples

### Startup (Mega 2560)
```
=== DMX NRF24L01 Hub (Mega 2560) ===
Serial0 (USB): Monitoring/Debug Output
Serial1 (Pins 18-19): DMX Serial Input
Checking for DMX Shield...
DMX Shield detected - using hardware DMX interface
Initializing NRF24L01...
NRF24L01 initialized successfully | Channel: 76 | Data Rate: 250kbps | PA Level: Max
Active remotes: 6

=== Hardware Configuration ===
Board: Arduino Mega 2560
NRF24L01 CE: Pin 7
NRF24L01 CSN: Pin 8
DMX Input: CF Robot Shield (SPI)
```

### During Operation
```
DMX RX (Broadcast): 40 5 200 0 0
RDM Status from Device 0: State=1 Track=5 Vol=20
RDM Status from Device 1: State=0 Track=0 Vol=0
DMX RX (Device 2): 50 3 180 0 1
```

---

## Power Supply Recommendations

### Current Requirements
- Mega 2560 MCU: ~50-100 mA
- NRF24L01 (RX): ~10-20 mA
- NRF24L01 (TX): ~30-50 mA (peak)
- CF Robot DMX Shield (if used): ~10-20 mA

### Total Recommended: 200-300 mA at 5V

### Power Supply Specs
- **Input**: 9-12V DC (barrel jack)
- **Output**: 5V, minimum 500mA capacity
- **Backup**: USB power (5V, 500mA) if 5V supply unavailable
  - Mega 2560 can be powered via USB cable
  - Not recommended as primary power for wireless module

### Voltage Regulator (for NRF24L01)
- Input: 5V from Mega
- Output: 3.3V stable supply
- Capacity: 250-500 mA
- Decoupling: 10µF capacitor on output

---

## Compilation & Upload

### Arduino IDE Settings
```
Board: Arduino Mega 2560
Port: [Your COM port]
Processor: ATmega2560
Programmer: AVRISP mkII
Speed: 115200 baud
```

### Expected Compilation
```
Sketch uses 28,654 bytes of program storage space (11%)
Global variables use 1,598 bytes of dynamic memory (19%)
```

If sketch is larger than 32KB, you have plenty of room (256KB available).

---

## Connectivity Checklist

**Before First Power-On**:
- [ ] 5V power supply connected (barrel jack or USB)
- [ ] NRF24L01 module soldered/connected to Mega
- [ ] NRF24L01 decoupling capacitor (10µF) installed
- [ ] 3.3V voltage regulator connected
- [ ] Serial1 (pins 18-19) free and ready
- [ ] DMX Shield mounted (if using hardware shield)
- [ ] USB cable ready for Serial Monitor on Serial0

**After Upload**:
- [ ] Serial Monitor shows "Mega 2560" in startup message
- [ ] NRF24L01 initialization successful
- [ ] Serial1 baud rate confirmed (115200)
- [ ] DMX input mode detected (Shield or Serial)

---

## Troubleshooting Mega-Specific Issues

### Issue: "Serial1 not defined" compilation error
**Cause**: Board selection is wrong
**Solution**: Select "Arduino Mega 2560" in Tools → Board menu

### Issue: DMX data on Serial0 mixed with monitoring  
**Cause**: Using Serial instead of Serial1 for DMX input
**Solution**: Verify code uses Serial1.read(), Serial1.available()
**Status**: Already fixed in updated code

### Issue: NRF24L01 not responding (CE/CSN wrong pins)
**Expected Pins**:
- CE on Pin 7 (not Pin 9)
- CSN on Pin 8 (not Pin 10)
**Solution**: Check physical wiring matches code configuration

### Issue: Serial Monitor garbled output
**Cause**: Baud rate mismatch
**Solution**: Set Serial Monitor to 115200 baud

### Issue: Serial1 not receiving DMX data
**Possible Causes**:
- Wrong pins (should be 18-19)
- Baud rate mismatch (should be 115200)
- External adapter not connected
- External adapter sending to wrong serial port

**Debug**:
1. Verify `Serial1.begin(115200)` in setup()
2. Check DMX Gateway TX is connected to Pin 19 (RX1)
3. Add debug print: `if (Serial1.available()) Serial.println("Serial1 RX");`

---

## Advanced Configuration

### Using Different Serial Port for DMX
If you want to use Serial2 or Serial3 instead of Serial1:

1. Find this line in DMX_NRF24_Hub.ino:
```cpp
Serial1.begin(SERIAL_DMX_BAUD);
```

2. Change to Serial2 or Serial3:
```cpp
Serial2.begin(SERIAL_DMX_BAUD);  // Or Serial3
```

3. Update loop() code:
```cpp
while (Serial1.available()) {  // Change to Serial2 or Serial3
    uint8_t byte = Serial1.read();
```

**Serial Pin Mapping**:
- Serial1: Pins 18 (TX), 19 (RX)
- Serial2: Pins 16 (TX), 17 (RX)
- Serial3: Pins 14 (TX), 15 (RX)

### Changing NRF24L01 CE/CSN Pins
The code uses GPIO pins 7 and 8. To change:

1. Find this line:
```cpp
RF24 radio(7, 8);  // CE=7, CSN=8
```

2. Change to different pins (avoid hardware peripheral pins):
```cpp
RF24 radio(3, 4);  // CE=Pin 3, CSN=Pin 4 (example)
```

---

## Performance Notes

- **Mega 2560 Advantage**: 8x faster processor (16 MHz vs 8MHz Uno)
- **Mega 2560 Advantage**: 8x more RAM (8KB vs 1KB Uno) - allows larger buffers
- **Mega 2560 Advantage**: 4 serial ports - can monitor while receiving DMX
- **Practical Impact**: More stable wireless, less processor stalls, better RDM responsiveness

---

## Document Version
**Version**: 1.0  
**Field**: Mega 2560 updated system  
**Compatibility**: DMX_NRF24_Hub.ino v2.1+
