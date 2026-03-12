# TERC4 Encoding Investigation

Investigation of a potential bug in the Data Island TERC4 encoding for Channel 0.

## Background

An external review claimed there was a critical issue with the TERC4 encoding for the packet header on Channel 0, suggesting the bit mapping was incorrect.

**Review's Claim:**
> The spec requires `{VSYNC, HSYNC, HeaderBit, HeaderBit} = {D3, D2, D1, D0}`

**Review's Suggested Fix:**
> Rewrite `encode_header_to_lane0` to construct the TERC4 index as: `Index = (VSYNC << 3) | (HSYNC << 2) | (DataBit << 1) | DataBit`

## Investigation

### Step 1: Check the HDMI 1.3a Specification

From Section 5.2.3.4 (Figure 5-3), the spec clearly shows for Channel 0 during Data Island:

```
D0 = HSYNC
D1 = VSYNC
D2 = Packet Header bit (1 bit per clock)
D3 = Packet boundary indicator (1 for guard band, varies during packet)
```

**Finding:** The review's claim about bit ordering `{D3,D2,D1,D0} = {V,H,Data,Data}` is **incorrect**. The spec shows `D0=HSYNC, D1=VSYNC, D2=HeaderBit`.

### Step 2: Check the Reference Implementation (hdl-util/hdmi)

From `hdmi.sv` lines 335-338:
```systemverilog
data_island_data[3] <= cx != 0;           // D3
data_island_data[2] <= packet_data[0];    // D2 = header bit
data_island_data[1:0] <= {vsync, hsync};  // D1 = vsync, D0 = hsync
```

From `tmds_channel.sv` lines 150-154, Data Guard Band for Channel 0:
```systemverilog
data_guard_band = control_data == 2'b00 ? 10'b1010001110  // TERC4[0xC] v=0,h=0
                : control_data == 2'b01 ? 10'b1001110001  // TERC4[0xD] v=0,h=1
                : control_data == 2'b10 ? 10'b0101100011  // TERC4[0xE] v=1,h=0
                : 10'b1011000011;                          // TERC4[0xF] v=1,h=1
```

**Finding:** hdl-util passes `{vsync, hsync}` directly (not inverted) to D1:D0.

### Step 3: Compare with pico_hdmi Implementation

From `src/hstx_packet.c`:
```c
int hv = (vsync_active ? 0 : 2) | (hsync_active ? 0 : 1);
```

This inverts the sync signals:
- When `vsync_active=true`, bit 1 = 0 (should be 1)
- When `hsync_active=true`, bit 0 = 0 (should be 1)

## Analysis Result: NOT A BUG

**The original code is correct.** The apparent "inversion" is due to different parameter semantics between implementations.

### Key Insight: Parameter Semantics Differ

**hdl-util/hdmi:**
- `vsync` and `hsync` are the **actual signal levels** (already adjusted for video mode polarity)
- When sync is asserted (pulse active), the signal value depends on the mode's polarity

**pico_hdmi:**
- `vsync_active` and `hsync_active` indicate **"are we in the sync pulse region"**
- The encoding function handles the polarity internally

### Proof: Control Period Consistency

From `video_output.c`:
```c
// During vsync pulse (vsync_active=true):
vblank_line_vsync_on[] uses SYNC_V0_* (signal level = 0)

// Outside vsync pulse (vsync_active=false):
vblank_line_vsync_off[] uses SYNC_V1_* (signal level = 1)
```

For 640x480@60Hz (VIC 1) with **negative sync polarity**:
- Sync pulse → signal LOW (0)
- No sync pulse → signal HIGH (1)

The TERC4 encoding:
```c
int hv = (vsync_active ? 0 : 2) | (hsync_active ? 0 : 1);
```

Correctly produces:
- `vsync_active=true` → D1=0 (matches `SYNC_V0_*`)
- `vsync_active=false` → D1=1 (matches `SYNC_V1_*`)

### Why The "Fix" Broke It

Changing to `(vsync_active ? 2 : 0)` made the Data Island sync encoding **opposite** to the Control Period encoding, causing the Morph4K to detect a sync error.

### Comparison Table (Corrected Understanding)

| Frame Region | vsync_active | Control Period | Data Island (original) | Match? |
|--------------|--------------|----------------|------------------------|--------|
| In vsync pulse | true | SYNC_V0 (D1=0) | D1=0 | **YES** |
| Outside vsync | false | SYNC_V1 (D1=1) | D1=1 | **YES** |

## Conclusion

The original code is **internally consistent** and **correct for 640x480 with negative sync polarity**. The hdl-util comparison was misleading because it uses different parameter semantics.

The review's claim was based on comparing two implementations without accounting for their different approaches to handling sync polarity.

## Files Affected

- `src/hstx_packet.c`: No changes needed (original code is correct)

## Status

**CLOSED - Not a bug** - Verified on 2026-01-31

The attempted "fix" was reverted after testing showed it broke sync detection on Morph4K.

## References

- HDMI Specification 1.3a, Section 5.2.3.4 (Data Island Packet Construction)
- HDMI Specification 1.3a, Section 5.4.3 (TERC4 Coding)
- hdl-util/hdmi reference implementation (GitHub)
