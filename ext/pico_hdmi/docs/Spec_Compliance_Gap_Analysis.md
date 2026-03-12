# HDMI 1.4 Spec Compliance Gap Analysis

Analysis of current implementation against HDMI specification requirements.

## Recent Fixes

### Video Preamble & Guard Band (2026-01-31)

**Issue**: Missing Video Preamble (8 pixels) and Video Guard Band (2 pixels) before active video data in HDMI mode.

**Impact**: Sync failures on strict displays (Acer XB271HU, Samsung Q80 TV).

**Fix**: Commit `83d9692` added proper Video Preamble (CH0=sync, CH1=CTRL_01, CH2=CTRL_00) and Video Guard Band (CH0=0x2CC, CH1=0x133, CH2=0x2CC) per HDMI 1.3a Section 5.2.2.

**Status**: ✓ Fixed - Acer XB271HU now works. Samsung Q80 TV still not syncing (further investigation needed).

---

## Summary

| Category | Status | Priority |
|----------|--------|----------|
| TMDS Encoding | Complete | - |
| Control Symbols | Complete | - |
| Video Preamble | Complete | - |
| Video Guard Band | Complete | - |
| Data Island Guard Band | Complete | - |
| Packet Structure & ECC | Complete | - |
| ACR Packet | Complete | - |
| Audio Sample Packet | Partial | High |
| AVI InfoFrame | Partial | Medium |
| Audio InfoFrame | Complete | - |
| General Control Packet | Missing | Medium |
| Source Product Description | Missing | Low |
| IEC 60958 Channel Status | Missing | High |

---

## High Priority Gaps

### 1. IEC 60958 Channel Status (C bit)

**Current**: The C (channel status) bit in audio samples is always 0.

**Required**: The 192-bit channel status block should be transmitted one bit per audio frame, cycling every 192 frames.

**Spec Reference**: HDMI 1.3a Section 7.1, IEC 60958

**Channel Status Block (Consumer Format)**:
```
Byte 0: [Mode|Emphasis|Copyright|Audio/Data|Consumer]
        Bit 0: 0 = Consumer
        Bit 1: 0 = L-PCM
        Bit 2: Copyright (0 = copyrighted)
        Bits 3-5: Pre-emphasis
        Bits 6-7: Mode

Byte 1: Category code (0x00 = general)

Byte 2: Source/Channel number

Byte 3: [Clock accuracy | Sampling frequency]
        Bits 0-3: Sampling frequency
          0000 = 44.1 kHz
          0010 = 48 kHz
          0011 = 32 kHz
        Bits 4-5: Clock accuracy

Byte 4: [Reserved | Word length]
        Bits 0-3: Word length
          0010 = 16 bits
          1010 = 24 bits

Bytes 5-23: Reserved (zeros)
```

**Impact**: Some displays/AVRs may not correctly identify audio format.

**Fix**:
```c
// Add to hstx_packet.c
static uint8_t channel_status[24] = {
    0x00,  // Consumer, PCM, no copyright, no pre-emphasis
    0x00,  // Category: general
    0x00,  // Source/channel
    0x02,  // 48kHz (bits 0-3 = 0010)
    0x0B,  // 24-bit max word length, 16-bit sample
    // ... rest zeros
};

// In audio sample encoding, set C bit from channel_status
bool c_bit = (channel_status[frame_count / 8] >> (frame_count % 8)) & 1;
d[6] |= (c_bit << 2) | (c_bit << 6);  // C_L and C_R
```

---

### 2. Audio Sample Byte Layout Verification

**Current**:
```c
d[0] = 0x00;              // Padding
d[1] = left & 0xFF;       // Left LSB
d[2] = (left >> 8) & 0xFF; // Left MSB
```

**Spec**: For 16-bit samples in 24-bit field, samples should be MSB-justified (bits 23:8), with LSB padding (bits 7:0).

**Current layout analysis**:
- Byte 0 (bits 7:0): 0x00 padding ✓
- Byte 1 (bits 15:8): sample[7:0]
- Byte 2 (bits 23:16): sample[15:8]

This places 16-bit sample in bits [23:8] which is **correct**.

**Status**: ✓ Already correct

---

## Medium Priority Gaps

### 3. General Control Packet (GCP) - Type 0x03

**Current**: Not implemented.

**Required**: Used for:
- **AVMUTE**: Mute audio/video during mode changes
- **Color Depth (CD)**: Indicate 24/30/36/48-bit color
- **Pixel Packing Phase (PP)**: For deep color
- **Default_Phase**: Phase alignment

**Spec Reference**: HDMI 1.3a Section 5.3.6

**Packet Structure**:
```
Header:
  HB0 = 0x03 (packet type)
  HB1 = 0x00
  HB2 = 0x00

Subpacket 0:
  SB0: [Set_AVMUTE | Clear_AVMUTE | CD3:CD0 | PP3:PP0]
       Bit 0: Set_AVMUTE
       Bit 4: Clear_AVMUTE
       Bits 1-3: Reserved
       Bits 5-7: CD (Color Depth)
  SB1: [Default_Phase | Reserved]
  SB2-6: Reserved (0x00)
```

**Impact**: No AVMUTE capability, receivers may not properly handle mode switches.

**Recommended Implementation**:
```c
void hstx_packet_set_gcp(hstx_packet_t *packet, bool set_avmute, bool clear_avmute) {
    hstx_packet_init(packet);
    packet->header[0] = 0x03;

    uint8_t sb0 = 0;
    if (set_avmute) sb0 |= 0x01;
    if (clear_avmute) sb0 |= 0x10;
    sb0 |= (0x04 << 4);  // CD = 4 (24-bit color)

    packet->subpacket[0][0] = sb0;
    // All subpackets identical for GCP
    for (int i = 1; i < 4; i++)
        memcpy(packet->subpacket[i], packet->subpacket[0], 7);

    compute_all_parity(packet);
}
```

---

### 4. AVI InfoFrame Completeness

**Current**:
```c
packet->subpacket[0][1] = 0x00;  // Y=RGB, A=0, B=0, S=0
packet->subpacket[0][2] = 0x08;  // ITC=1, rest=0
packet->subpacket[0][3] = 0x00;
packet->subpacket[0][4] = vic;
packet->subpacket[0][5] = 0x00;
```

**Recommended Improvements**:

```c
void hstx_packet_set_avi_infoframe(hstx_packet_t *packet, uint8_t vic,
                                    bool rgb, uint8_t aspect_ratio) {
    hstx_packet_init(packet);
    packet->header[0] = 0x82;
    packet->header[1] = 0x02;
    packet->header[2] = 0x0D;  // Length = 13

    // Byte 1: Y1:Y0 | A0 | B1:B0 | S1:S0
    uint8_t y = rgb ? 0x00 : 0x20;  // RGB or YCbCr444
    packet->subpacket[0][1] = y | 0x10;  // A=1 (active format valid)

    // Byte 2: C1:C0 | M1:M0 | R3:R0
    // M: 01=4:3, 10=16:9
    uint8_t m = (aspect_ratio == 16) ? 0x20 : 0x10;
    packet->subpacket[0][2] = m | 0x08;  // R=8 (same as coded)

    // Byte 3: ITC | EC2:EC0 | Q1:Q0 | SC1:SC0
    packet->subpacket[0][3] = 0x00;  // Q=0 (default range)

    // Byte 4: VIC
    packet->subpacket[0][4] = vic;

    // Byte 5: YQ1:YQ0 | CN1:CN0 | PR3:PR0
    packet->subpacket[0][5] = 0x00;  // No pixel repetition

    compute_infoframe_checksum(packet);
    compute_all_parity(packet);
}
```

---

## Low Priority Gaps

### 5. Source Product Description InfoFrame (0x83)

**Current**: Not implemented.

**Purpose**: Allows TVs to display source device name (e.g., "HDMI 1: Pico HDMI").

**Spec Reference**: HDMI 1.3a Section 8.2.3 (references CEA-861)

**Packet Structure**:
```
Header:
  HB0 = 0x83 (type)
  HB1 = 0x01 (version)
  HB2 = 0x19 (length = 25)

Payload:
  PB0: Checksum
  PB1-8: Vendor Name (8 bytes, ASCII, null-padded)
  PB9-24: Product Description (16 bytes, ASCII, null-padded)
  PB25: Source Device Information
        0x00 = Unknown
        0x01 = Digital STB
        0x08 = Game
        0x09 = PC General
```

**Recommended Implementation**:
```c
void hstx_packet_set_spd_infoframe(hstx_packet_t *packet,
                                    const char *vendor,
                                    const char *product,
                                    uint8_t device_type) {
    hstx_packet_init(packet);
    packet->header[0] = 0x83;
    packet->header[1] = 0x01;
    packet->header[2] = 0x19;  // 25 bytes

    // PB1-8: Vendor (spans subpackets)
    for (int i = 0; i < 8 && vendor[i]; i++)
        packet->subpacket[0][1 + i] = vendor[i];

    // PB9-24: Product
    // ... (handle subpacket boundaries)

    // PB25: Device type
    packet->subpacket[3][4] = device_type;

    compute_infoframe_checksum(packet);
    compute_all_parity(packet);
}
```

---

## Implementation Priority

1. **IEC 60958 Channel Status** - High impact on audio compatibility
2. **General Control Packet** - Needed for proper AVMUTE during init
3. **AVI InfoFrame enhancements** - Better display compatibility
4. **SPD InfoFrame** - Nice to have for user experience

---

## Testing Recommendations

After each improvement:
- [ ] Test on consumer TV (Samsung, LG, Sony)
- [ ] Test on Morph4K analyzer
- [ ] Test on AVR/soundbar for audio
- [ ] Verify no regressions in DVI mode
