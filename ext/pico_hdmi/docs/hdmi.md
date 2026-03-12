# HDMI Implementation Guide

A low-level technical reference for implementing HDMI Audio and Video transmission. This document focuses on the fundamental signal requirements for basic but highly compatible HDMI output.

## Table of Contents

1. [Physical Layer & TMDS Encoding](#physical-layer--tmds-encoding)
2. [Video Timing Structure](#video-timing-structure)
3. [Data Island Packets](#data-island-packets)
4. [Audio Implementation](#audio-implementation)
5. [InfoFrames](#infoframes)
6. [DVI vs HDMI Mode](#dvi-vs-hdmi-mode)
7. [Compliance Requirements](#compliance-requirements)

---

## Physical Layer & TMDS Encoding

### TMDS Overview

HDMI uses **TMDS (Transition Minimized Differential Signaling)** for data transmission:

| Parameter         | Value                         |
| ----------------- | ----------------------------- |
| Data Channels     | 3 (RGB or YCbCr)              |
| Clock Channel     | 1                             |
| Bits per Symbol   | 10                            |
| Serializer Clock  | 5x pixel clock                |
| Pixel Clock Range | 25.2 MHz - 340 MHz (HDMI 1.4) |

### TMDS Encoding Algorithm

The encoder converts 8-bit video data to 10-bit symbols with DC balancing:

**Step 1: 8-bit to 9-bit conversion**

```
Count ones in input byte (N1D)
If N1D > 4 OR (N1D == 4 AND input[0] == 0):
    q_m = XNOR chain encoding
    q_m[8] = 0
Else:
    q_m = XOR chain encoding
    q_m[8] = 1
```

**Step 2: 9-bit to 10-bit DC balancing**

```
Count ones in q_m[7:0] (N1q_m07)
N0q_m07 = 8 - N1q_m07

If accumulator == 0 OR N1q_m07 == 4:
    q_out[9] = ~q_m[8]
    q_out[8] = q_m[8]
    If q_m[8] == 0:
        q_out[7:0] = ~q_m[7:0]
        accumulator += N0q_m07 - N1q_m07
    Else:
        q_out[7:0] = q_m[7:0]
        accumulator += N1q_m07 - N0q_m07
Else:
    ... (conditional inversion based on accumulator sign)
```

### Control Symbols

Used during blanking periods to encode sync signals:

| {VSYNC, HSYNC} | 10-bit Symbol  |
| -------------- | -------------- |
| 00             | `0b1101010100` |
| 01             | `0b0010101011` |
| 10             | `0b0101010100` |
| 11             | `0b1010101011` |

### TERC4 Encoding

4-bit to 10-bit encoding for Data Island periods:

| 4-bit Input | 10-bit Output  |
| ----------- | -------------- |
| 0000        | `0b1010011100` |
| 0001        | `0b1001100011` |
| 0010        | `0b1011100100` |
| 0011        | `0b1011100010` |
| 0100        | `0b0101110001` |
| 0101        | `0b0100011110` |
| 0110        | `0b0110001110` |
| 0111        | `0b0100111100` |
| 1000        | `0b1011001100` |
| 1001        | `0b0100111001` |
| 1010        | `0b0110011100` |
| 1011        | `0b1011000110` |
| 1100        | `0b1010001110` |
| 1101        | `0b1001110001` |
| 1110        | `0b0101100011` |
| 1111        | `0b1011000011` |

---

## Video Timing Structure

### Scanline Anatomy

A complete scanline consists of these regions:

```
|<-- Back Porch -->|<-- Active Video -->|<-- Front Porch -->|<-- Sync -->|
                   |                    |
                   +--------------------+
                   |   Visible Image    |
```

### Frame Period Diagram

```
        |<------------------ Total Horizontal Pixels ------------------>|
        |                                                                |
   +----+------------------+----------------------------------------+----+
   |    |                  |                                        |    |
   | BP |                  |            ACTIVE VIDEO                | FP | SYNC
   |    |                  |                                        |    |
   +----+------------------+----------------------------------------+----+
        |<-- Data Island ->|                                        |
        |    (HDMI only)   |

BP = Back Porch, FP = Front Porch
```

### Common Video Timings

#### 640x480p @ 60Hz (VIC 1)

| Parameter     | Value      |
| ------------- | ---------- |
| Pixel Clock   | 25.175 MHz |
| H Active      | 640        |
| H Front Porch | 16         |
| H Sync        | 96         |
| H Back Porch  | 48         |
| H Total       | 800        |
| V Active      | 480        |
| V Front Porch | 10         |
| V Sync        | 2          |
| V Back Porch  | 33         |
| V Total       | 525        |

#### 1280x720p @ 60Hz (VIC 4)

| Parameter     | Value     |
| ------------- | --------- |
| Pixel Clock   | 74.25 MHz |
| H Active      | 1280      |
| H Front Porch | 110       |
| H Sync        | 40        |
| H Back Porch  | 220       |
| H Total       | 1650      |
| V Active      | 720       |
| V Front Porch | 5         |
| V Sync        | 5         |
| V Back Porch  | 20        |
| V Total       | 750       |

#### 1920x1080p @ 60Hz (VIC 16)

| Parameter     | Value     |
| ------------- | --------- |
| Pixel Clock   | 148.5 MHz |
| H Active      | 1920      |
| H Front Porch | 88        |
| H Sync        | 44        |
| H Back Porch  | 148       |
| H Total       | 2200      |
| V Active      | 1080      |
| V Front Porch | 4         |
| V Sync        | 5         |
| V Back Porch  | 36        |
| V Total       | 1125      |

### Mode Periods

HDMI defines distinct transmission modes:

| Mode | Name               | Description                                |
| ---- | ------------------ | ------------------------------------------ |
| 0    | Control Period     | Blanking intervals, transmits sync signals |
| 1    | Video Data Period  | Active pixels, DC-balanced TMDS encoding   |
| 2    | Video Guard Band   | 2 pixels before video data                 |
| 3    | Data Island Period | Auxiliary data (audio, InfoFrames)         |

### Guard Bands and Preambles

**Video Data Period Sequence:**

```
Control -> Preamble (8 px) -> Guard (2 px) -> Video Data
```

**Data Island Period Sequence:**

```
Control -> Preamble (8 px) -> Guard (2 px) -> Packets (32 px each) -> Guard (2 px)
```

**Video Guard Band Symbols (Table 5-5):**

| Channel   | Symbol         |
| --------- | -------------- |
| Channel 0 | `0b1011001100` |
| Channel 1 | `0b0100110011` |
| Channel 2 | `0b1011001100` |

**Data Island Guard Band Symbols (Table 5-6):**

| Channel   | Symbol                    |
| --------- | ------------------------- |
| Channel 0 | TERC4 encoded (see below) |
| Channel 1 | `0b0100110011`            |
| Channel 2 | `0b0100110011`            |

Channel 0 during Data Island Guard Band uses TERC4 encoding based on HSYNC/VSYNC:

| {VSYNC, HSYNC} | TERC4 Input | 10-bit Symbol  |
| -------------- | ----------- | -------------- |
| 00             | 0xC (1100)  | `0b1010001110` |
| 01             | 0xD (1101)  | `0b1001110001` |
| 10             | 0xE (1110)  | `0b0101100011` |
| 11             | 0xF (1111)  | `0b1011000011` |

---

## Data Island Packets

### Packet Structure

Each packet occupies **32 pixel clocks** and contains:

```
|<----------------------- 32 Pixel Clocks ----------------------->|
+--------+--------+--------+--------+--------+--------+...+--------+
| Header | Header | Header | HEC    | Sub0   | Sub0   |   | Sub3   |
| Byte 0 | Byte 1 | Byte 2 |        | Byte 0 | Byte 1 |   | Parity |
+--------+--------+--------+--------+--------+--------+...+--------+
```

**Header (4 bytes):**

- Byte 0: Packet Type
- Byte 1: HB1 (varies by packet type)
- Byte 2: HB2 (varies by packet type)
- Byte 3: BCH ECC parity

**Subpackets (4 x 8 bytes each):**

- 7 data bytes
- 1 BCH ECC parity byte

### BCH Error Correction

BCH(32,24) for header, BCH(64,56) for subpackets.

**Generator polynomial:** x^8 + x^4 + x^3 + x^2 + 1

```c
uint8_t compute_bch_parity(uint8_t *data, int len) {
    uint8_t parity = 0;
    for (int i = 0; i < len; i++) {
        parity ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (parity & 0x80)
                parity = (parity << 1) ^ 0x1D;  // x^8+x^4+x^3+x^2+1
            else
                parity <<= 1;
        }
    }
    return parity;
}
```

### Packet Types

| Type | Name                       | Purpose                           |
| ---- | -------------------------- | --------------------------------- |
| 0x00 | Null Packet                | Padding for data island periods   |
| 0x01 | Audio Clock Regeneration   | Audio/video clock synchronization |
| 0x02 | Audio Sample Packet        | L-PCM audio samples               |
| 0x82 | AVI InfoFrame              | Video format description          |
| 0x83 | Source Product Description | Device identification             |
| 0x84 | Audio InfoFrame            | Audio format description          |

---

## Audio Implementation

### Audio Clock Regeneration (ACR)

The receiver recovers the audio clock from the video clock using N and CTS values:

```
f_audio = (N / CTS) × (f_TMDS / 128)
```

**Recommended N Values:**

| Audio Rate | 25.2/50.4 MHz | 27 MHz | 54 MHz | 74.25 MHz | 148.5 MHz |
| ---------- | ------------- | ------ | ------ | --------- | --------- |
| 32 kHz     | 4096          | 4096   | 4096   | 4096      | 4096      |
| 44.1 kHz   | 6272          | 6272   | 6272   | 6272      | 6272      |
| 48 kHz     | 6144          | 6144   | 6144   | 6144      | 6144      |
| 88.2 kHz   | 12544         | 12544  | 12544  | 12544     | 12544     |
| 96 kHz     | 12288         | 12288  | 12288  | 12288     | 12288     |
| 176.4 kHz  | 25088         | 25088  | 25088  | 25088     | 25088     |
| 192 kHz    | 24576         | 24576  | 24576  | 24576     | 24576     |

**CTS Calculation:**

```
CTS = (f_TMDS × N) / (128 × f_audio)
```

**ACR Packet Structure:**

```
Header:
  Byte 0: 0x01 (Packet Type)
  Byte 1: 0x00
  Byte 2: 0x00

Subpacket 0:
  Byte 0: 0x00 (reserved)
  Bytes 1-3: CTS[19:0] (20 bits, MSB first)
  Bytes 4-6: N[19:0] (20 bits, MSB first)
  Byte 7: 0x00 (reserved)
```

### Audio Sample Packet

Carries IEC 60958 formatted audio samples.

**Header:**

```
Byte 0: 0x02 (Packet Type)
Byte 1: Sample Present flags
        Bit 0: SP0 present
        Bit 1: SP1 present
        Bit 2: SP2 present
        Bit 3: SP3 present
        Bit 4: Layout (0=2ch, 1=8ch)
Byte 2: B3|B2|B1|B0 (frame start flags per subpacket)
```

**Subpacket Layout (Stereo):**

```
Byte 0: Left sample [7:0]
Byte 1: Left sample [15:8]
Byte 2: Left sample [23:16]
Byte 3: Right sample [7:0]
Byte 4: Right sample [15:8]
Byte 5: Right sample [23:16]
Byte 6: [V_R|U_R|C_R|P_R | V_L|U_L|C_L|P_L]
        V = Validity, U = User data, C = Channel status, P = Parity
```

### IEC 60958 Channel Status

192-bit block transmitted over 192 audio frames (1 bit per frame):

**Byte 0:**

- Bit 0: Consumer use (0)
- Bit 1: Audio/Data (0 = PCM)
- Bit 2: Copyright (0 = copyrighted)
- Bits 3-5: Pre-emphasis
- Bits 6-7: Mode

**Byte 1:** Category code (0x00 = general)

**Byte 2:** Source/Channel number

**Byte 3:**

- Bits 0-3: Sampling frequency
  - 0000: 44.1 kHz
  - 0010: 48 kHz
  - 0011: 32 kHz
- Bits 4-5: Clock accuracy

**Byte 4:**

- Bits 0-3: Word length
  - 0010: 16 bits
  - 1010: 24 bits

---

## InfoFrames

### AVI InfoFrame (0x82)

Describes video format, colorimetry, and aspect ratio.

```
Header:
  Byte 0: 0x82 (Type)
  Byte 1: 0x02 (Version)
  Byte 2: 0x0D (Length = 13 bytes)

Payload:
  Byte 0: Checksum
  Byte 1: [Y1:Y0|A0|B1:B0|S1:S0]
          Y = Color space (00=RGB, 01=YCbCr422, 10=YCbCr444)
          A = Active info present
          B = Bar info
          S = Scan info
  Byte 2: [C1:C0|M1:M0|R3:R0]
          C = Colorimetry
          M = Picture aspect ratio
          R = Active format aspect ratio
  Byte 3: [ITC|EC2:EC0|Q1:Q0|SC1:SC0]
          Q = RGB quantization (00=default, 01=limited, 10=full)
  Byte 4: VIC (Video Identification Code)
  Byte 5: [YQ1:YQ0|CN1:CN0|PR3:PR0]
          PR = Pixel repetition
  Bytes 6-13: Bar info (if B != 00)
```

**Common VIC Values:**

| VIC | Resolution | Refresh |
| --- | ---------- | ------- |
| 1   | 640x480p   | 60 Hz   |
| 2   | 720x480p   | 60 Hz   |
| 4   | 1280x720p  | 60 Hz   |
| 16  | 1920x1080p | 60 Hz   |
| 19  | 1280x720p  | 50 Hz   |
| 31  | 1920x1080p | 50 Hz   |
| 34  | 1920x1080p | 30 Hz   |

### Audio InfoFrame (0x84)

Describes audio format and channel configuration.

```
Header:
  Byte 0: 0x84 (Type)
  Byte 1: 0x01 (Version)
  Byte 2: 0x0A (Length = 10 bytes)

Payload:
  Byte 0: Checksum
  Byte 1: [CT3:CT0|CC2:CC0]
          CT = Coding Type (0000=PCM)
          CC = Channel Count (000=2ch - 1)
  Byte 2: [SF2:SF0|SS1:SS0]
          SF = Sampling Frequency (000=refer to stream)
          SS = Sample Size (000=refer to stream)
  Byte 3: 0x00 (Format dependent)
  Byte 4: CA (Channel Allocation, 0x00 for stereo)
  Byte 5: [DM_INH|LSV3:LSV0]
          DM_INH = Downmix inhibit
          LSV = Level shift value
  Bytes 6-10: Reserved (0x00)
```

**Checksum Calculation:**

```
Checksum = 256 - ((Type + Version + Length + Sum(Payload)) & 0xFF)
```

### Source Product Description InfoFrame (0x83)

Identifies the source device.

```
Header:
  Byte 0: 0x83 (Type)
  Byte 1: 0x01 (Version)
  Byte 2: 0x19 (Length = 25 bytes)

Payload:
  Byte 0: Checksum
  Bytes 1-8: Vendor Name (ASCII, null-padded)
  Bytes 9-24: Product Description (ASCII, null-padded)
  Byte 25: Source Device Information
           0x00 = Unknown
           0x01 = Digital STB
           0x02 = DVD Player
           0x03 = D-VHS
           0x04 = HDD Videorecorder
           0x05 = DVC
           0x06 = DSC
           0x07 = Video CD
           0x08 = Game
           0x09 = PC General
```

---

## DVI vs HDMI Mode

### Key Differences

| Feature      | DVI Mode | HDMI Mode                  |
| ------------ | -------- | -------------------------- |
| Audio        | No       | Yes                        |
| InfoFrames   | No       | Yes                        |
| Data Islands | No       | Yes                        |
| Guard Bands  | No       | Yes                        |
| Preambles    | No       | Yes                        |
| Min Blanking | Minimal  | ~58 pixels per data island |

### Mode Detection

Sinks detect HDMI mode by looking for Data Island preambles during blanking. If only control symbols are seen, DVI mode is assumed.

### Blanking Requirements

**DVI Mode:**

- Only control symbols during blanking
- Minimum blanking determined by sync timing only

**HDMI Mode (per Data Island):**

- Preamble: 8 pixels
- Leading Guard Band: 2 pixels
- Packet(s): 32 pixels each
- Trailing Guard Band: 2 pixels
- Minimum: 44 pixels for one packet

---

## Compliance Requirements

### Minimum HDMI 1.4 Compliance

**Video:**

- Support at least 640x480p @ 60Hz (VIC 1)
- Proper TMDS encoding with DC balance
- Valid sync timing
- AVI InfoFrame transmission

**Audio:**

- 2-channel L-PCM
- 32 kHz, 44.1 kHz, and 48 kHz sample rates
- 16-bit minimum sample depth
- Audio Clock Regeneration packets
- Audio Sample packets
- Audio InfoFrame transmission

### InfoFrame Transmission Requirements

| InfoFrame | Minimum Rate                                |
| --------- | ------------------------------------------- |
| AVI       | Once per 2 video fields                     |
| Audio     | Once per 2 video fields                     |
| ACR       | At least once per audio block (192 samples) |
| SPD       | Once per 2 video fields (optional)          |

### Electrical Requirements

| Parameter            | Min | Typ | Max        | Unit |
| -------------------- | --- | --- | ---------- | ---- |
| Differential Voltage | 400 | -   | 600        | mV   |
| Rise/Fall Time       | -   | -   | 0.4 × Tbit | -    |
| Jitter               | -   | -   | 0.25       | UI   |
| Source Termination   | 45  | 50  | 55         | Ohm  |

---

## Implementation

### Transmitter State Machine

1. Implement TMDS encoder with DC balance tracking
2. Implement TERC4 encoder for data islands
3. Generate control symbols during blanking
4. Generate proper preambles (video and data island)
5. Generate guard bands
6. Implement 10-to-1 serializer at 5x pixel clock

### Packet Generation

1. Implement BCH ECC calculator
2. Generate ACR packets with correct N/CTS
3. Generate Audio Sample packets with IEC 60958 framing
4. Generate AVI InfoFrame with correct VIC
5. Generate Audio InfoFrame
6. Fill remaining data island time with Null packets

### Timing

1. Generate accurate pixel clock for target resolution
2. Generate correct H/V sync timing
3. Schedule data island periods during horizontal blanking
4. Maintain audio sample timing aligned to video

---

## References

- HDMI Specification 1.3a (primary source for this document)
- HDMI Specification 1.4b (hdmi.org)
- CEA-861-D: Video Timing and InfoFrame Definitions
- IEC 60958: Digital Audio Interface
- hdl-util/hdmi (GitHub): FPGA reference implementation
