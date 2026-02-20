#define DAC_I2C_ADDR 0b0011000

// Without MCLK, the TLV320DAC3101 can derive its internal clocks from BCLK, but this requires careful PLL configuration. With MCLK, the clocking is more straightforward and flexible. For simplicity and better compatibility.
// MCLK use is cleaner and more robust, need additional pin tho but that's fine.
// Without MCLK, headphone detect doesn't work for some reason.
#define USE_MCLK 1
#define DAC_BCLK_FS_RATIO 32  // valid values: 32 or 64

uint16_t current_volume_level = 0;
bool is_muted = false;
bool headphones_present = false;
bool prev_headphones_present = false;
static uint8_t *vol_lut = NULL;
void set_volume(uint8_t l_volume, uint8_t r_volume);
void dac_i2c_write(uint8_t page, uint8_t reg, uint8_t data);
void dac_i2c_read(uint8_t page, uint8_t reg, uint8_t *data, size_t length);

#define ADC_MIN_CLIP   4     // below this → treat as 0
#define ADC_MAX_CLIP   4050   // above this → treat as full scale
#define DAC_MAX_VOL    100 // 127 is max
#define MUTE_THRESH     1
#define UNMUTE_THRESH   12
#define USE_LINEAR_VOLUME_SCALING 0
#define RAMPED_AUDIO_EXPONENT 0.4f // if this is changed you need to change the unmute threshold.

// Human audible reaction time is ~160ms, so we'll stay under this value for headphone detection polling.
// We'll stay well under for volume polling too for smoother volume changes.
// Who knows what kind of drugs people will be on to reduce that reaction time.
#define VOL_POLL_US        20000   // 20ms = 50Hz
#define HP_POLL_US         100000  // 100ms = 10Hz

#define VOL_LUT_SIZE 256


void detect_headphones() {
    uint8_t data;
    dac_i2c_read(0, 0x43, &data, 1);
    headphones_present = (data & 0x60) != 0;
}
void mute_dac() {
    dac_i2c_write(0, 0x40, 0x0C); // Mute DAC
}
void unmute_dac() {
    dac_i2c_write(0, 0x40, 0x00); // Unmute DAC
}

void mute_drivers() {
    dac_i2c_write(1, 0x28, 0x00); // HP_L Driver muted
    dac_i2c_write(1, 0x29, 0x00); // HP_R Driver muted
    dac_i2c_write(1, 0x2A, 0x00); // SP_L Driver muted
    dac_i2c_write(1, 0x2B, 0x00); // SP_R Driver muted
}

void unmute_drivers() {
    dac_i2c_write(1, 0x28, 0x06); // HP_L Driver unmuted.. no 3d effect because it sounds bad
    dac_i2c_write(1, 0x29, 0x06); // HP_R Driver unmuted
    // bits 3-4 control gain. 00 = 6dB, 01 = 12dB, 10 = 18dB, 11 = 24dB
    dac_i2c_write(1, 0x2A, 0b00000100); // SP_L Driver unmuted with 6dB
    dac_i2c_write(1, 0x2B, 0b00000100); // SP_R Driver unmuted with 6dB
}

void power_on_drivers() {
    if(headphones_present) {
        dac_i2c_write(1, 0x1F, 0b11010100); // Headphone drivers powered on
    } else {
        dac_i2c_write(1, 0x20, 0b11000110); // Class-D Amplifier powered on
    }
}
void power_off_drivers() {
    if(headphones_present) {
        dac_i2c_write(1, 0x1F, 0x00); // Headphone drivers powered off
    } else {
        dac_i2c_write(1, 0x20, 0x00); // Class-D Amplifier powered off
    }
}

void read_volume() {
    static uint64_t next_vol_us = 0;
    static uint64_t next_hp_us  = 0;
    uint64_t now = time_us_64();

    // Volume reading every 10ms
    if ((int64_t)(now - next_vol_us) >= 0) {
        next_vol_us = now + VOL_POLL_US;

        adc_select_input(GPIO_AUD_POT_ADC - 40); 
        uint16_t adc_val = read_adc(); // 0–4095

        // Clip edges
        if (adc_val < ADC_MIN_CLIP) adc_val = 0;
        else if (adc_val > ADC_MAX_CLIP) adc_val = ADC_MAX_CLIP;

#if USE_LINEAR_VOLUME_SCALING
        // Scale to 0–DAC_MAX_VOL
        uint8_t dac_vol = (uint8_t)((adc_val * DAC_MAX_VOL) / ADC_MAX_CLIP);
#else
        // uint8_t idx = (uint8_t)(adc_val >> 4); // 12-bit -> 8-bit
        uint8_t idx = (uint8_t)((adc_val * (VOL_LUT_SIZE - 1)) / ADC_MAX_CLIP);
        uint8_t dac_vol = vol_lut[idx];
#endif

        if (abs((int)dac_vol - (int)current_volume_level) > 1) {
            current_volume_level = dac_vol;
            set_volume(current_volume_level, current_volume_level);
        }
    }

    // Headphone detection every 50ms
    if ((int64_t)(now - next_hp_us) >= 0) {
        next_hp_us = now + HP_POLL_US;
        
        detect_headphones();
    }
    
    // --- Mute logic with one-time calls and hysteresis ---
    // don't turn off the drivers when headphones in because pop is audible and class ab headphone amp
    // doesn't consume any idle power due to 100uF dc blocking caps in hp path..
    if (current_volume_level <= MUTE_THRESH && !is_muted) {
        mute_dac();
        if (!headphones_present) power_off_drivers();
        is_muted = true;
    } 
    else if (current_volume_level >= UNMUTE_THRESH && is_muted) {
        power_on_drivers();
        unmute_dac();
        is_muted = false;
    }

    // if it ain't broke don't fix it... that's why I'm not tryna touch my mute and headphone logic
    // gains would also be very minimal.
    if (headphones_present && !prev_headphones_present) {
        dac_i2c_write(1, 0x20, 0x00); // Class-D Amplifier powered off
    } else if (!headphones_present && prev_headphones_present && !is_muted) {
        // we don't want to turn on the amp if we're muted.. we'll let the unmute logic handle that
        dac_i2c_write(1, 0x20, 0b11000110); // Class-D Amplifier powered on
    }
    // Update the previous state for next check
    prev_headphones_present = headphones_present;
}

bool volume_lut_init(void) {
    if (vol_lut) {
        return true; // already initialized
    }

    vol_lut = (uint8_t *)malloc(VOL_LUT_SIZE * sizeof(uint8_t));
    if (!vol_lut) {
        return false; // allocation failed
    }

    for (int i = 0; i < VOL_LUT_SIZE; i++) {
        float x = (float)i / (float)(VOL_LUT_SIZE - 1);
        float y = powf(x, RAMPED_AUDIO_EXPONENT) * (float)DAC_MAX_VOL;

        int v = (int)(y + 0.5f); // round
        if (v < 0) v = 0;
        if (v > DAC_MAX_VOL) v = DAC_MAX_VOL;

        vol_lut[i] = (uint8_t)v;
    }

    return true;
}


uint8_t current_page = 0;
void dac_i2c_page_select(uint8_t page) {
    if (current_page == page) return;

    uint8_t page_buffer[2] = {0x00, page};
    i2c_write_blocking(DAC_I2C_PORT, DAC_I2C_ADDR, page_buffer, 2, false);
    sleep_us(50);
    current_page = page;
}

void dac_i2c_write(uint8_t page, uint8_t reg, uint8_t data) {
    dac_i2c_page_select(page);

    uint8_t data_buffer[2] = {reg, data};
    i2c_write_blocking(DAC_I2C_PORT, DAC_I2C_ADDR, data_buffer, 2, false);
}

void dac_i2c_read(uint8_t page, uint8_t reg, uint8_t *data, size_t length) {
    dac_i2c_page_select(page);

    i2c_write_blocking(DAC_I2C_PORT, DAC_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(DAC_I2C_PORT, DAC_I2C_ADDR, data, length, false);
}

void set_speaker_gain(uint8_t gain_l, uint8_t gain_r) {
    dac_i2c_write(1, 0x26, 0x80 | (127 - (gain_l & 0x7f)));
    dac_i2c_write(1, 0x27, 0x80 | (127 - (gain_r & 0x7f)));
}
void set_headphone_gain(uint8_t gain_l, uint8_t gain_r) {
    dac_i2c_write(1, 0x24, 0x80 | ((gain_l & 0x7f) - 127));
    dac_i2c_write(1, 0x25, 0x80 | ((gain_r & 0x7f) - 127));
}
void set_gain(uint8_t gain) {
    set_headphone_gain(gain, gain);
    set_speaker_gain(gain, gain);
}

void set_volume(uint8_t l_volume, uint8_t r_volume) {
    float lRange = -63.5f + ((l_volume / 127.0f) * 87.5f); // scale 0-127 to -63.5..24 dB
    float rRange = -63.5f + ((r_volume / 127.0f) * 87.5f);

    int lInt = (int)floorf(lRange);
    int rInt = (int)floorf(rRange);

    if ((lRange - lInt) >= 0.5f) lInt = (lInt << 1) | 1;
    else lInt = lInt << 1;

    if ((rRange - rInt) >= 0.5f) rInt = (rInt << 1) | 1;
    else rInt = rInt << 1;

    dac_i2c_write(0, 0x41, (uint8_t)lInt);
    dac_i2c_write(0, 0x42, (uint8_t)rInt);
}

void set_3d(uint8_t msb_3d, uint8_t lsb_3d) {
    dac_i2c_write(8, 0x40, msb_3d);
    dac_i2c_write(8, 0x41, lsb_3d);
}

// MCLK-less clocking for TLV320DAC3101:
// PLL_CLKIN = BCLK, CODEC_CLKIN = PLL_CLK
// PLL multiplies BCLK by 32 (P=1, R=1, J=32, D=0).
// Fs = (32 * BCLK) / (NDAC * MDAC * DOSR).
// Use DAC_BCLK_FS_RATIO=32 for 16-bit stereo I2S (common), or 64 for 32-bit-slot I2S.

static void dac3101_clock_from_bclk_autotrack(void)
{
    // Muxing: PLL_CLKIN=BCLK (D3-D2=01), CODEC_CLKIN=PLL_CLK (D1-D0=11)
    // => 0b0000_0111 = 0x07  (see reg 0x04 table)
    dac_i2c_write(0, 0x04, 0x07);  // :contentReference[oaicite:5]{index=5}

    // Program PLL multipliers first (J, D), then power up PLL (datasheet sequence)
    // J = 32
    dac_i2c_write(0, 0x06, 32);    // :contentReference[oaicite:6]{index=6}

    // D = 0 (write reg 0x07 then 0x08 immediately)
    dac_i2c_write(0, 0x07, 0x00);  // D[13:8]
    dac_i2c_write(0, 0x08, 0x00);  // D[7:0]  :contentReference[oaicite:7]{index=7}

    // PLL Power up, P=1, R=1
    // bit7=1 (PLL on)
    // P=1 => bits6-4 = 001
    // R=1 => bits3-0 = 0001
    // => 0b1001_0001 = 0x91
    dac_i2c_write(0, 0x05, 0x91);  // :contentReference[oaicite:8]{index=8}

    sleep_ms(12); // PLL_CLK typically available ~10ms after powering PLL :contentReference[oaicite:9]{index=9}

    // Now set the clock dividers to match Fs = BCLK/DAC_BCLK_FS_RATIO
    // MDAC = 2 (powered)  -> 0x80 | 2 = 0x82
    // DOSR = 128          -> 0x00, 0x80
    // NDAC depends on selected BCLK ratio:
    //   ratio 32: NDAC=4  -> 0x84
    //   ratio 64: NDAC=8  -> 0x88
#if (DAC_BCLK_FS_RATIO == 32)
    const uint8_t ndac_reg = 0x84;
#elif (DAC_BCLK_FS_RATIO == 64)
    const uint8_t ndac_reg = 0x88;
#else
#error "DAC_BCLK_FS_RATIO must be 32 or 64"
#endif

    // MDAC = 2 (powered)  -> 0x80 | 2 = 0x82
    // DOSR = 128          -> 0x00, 0x80
    dac_i2c_write(0, 0x0B, ndac_reg);
    dac_i2c_write(0, 0x0C, 0x82);  // :contentReference[oaicite:11]{index=11}
    dac_i2c_write(0, 0x0D, 0x00);
    dac_i2c_write(0, 0x0E, 0x80);  // :contentReference[oaicite:12]{index=12}
}

void setup_dac() {
    dac_i2c_write(0, 0x01, 0x01); // Soft-reset the DAC
    sleep_ms(20); // Wait for reset to complete

#if USE_MCLK
    dac_i2c_write(0, 0x0b, 0x81); // nDAC_VAL
    dac_i2c_write(0, 0x0c, 0x82); // mDAC_VAL
    dac_i2c_write(0, 0x0d, 0x00); // DOSR default
    dac_i2c_write(0, 0x0e, 0x80); // DOSR default
#else
    dac3101_clock_from_bclk_autotrack();
#endif

    dac_i2c_write(0, 0x1b, 0b00000000); // Interface Control: I2S, 16-bit

    dac_i2c_write(0, 0x3c, 0x0b); // DAC Processing Block Selection 25
    dac_i2c_write(8, 0x01, 0x04); // DAC coefficient 

    dac_i2c_write(0, 0x74, 0x00); // VOL/MICDETECT SAR ADC
    dac_i2c_write(0, 0x43, 0x80); // Headset detection enabled

    set_3d(0x10, 0x00); // Enable 3D sound with slight depth

    // --- Power up phase ---
    // Power on DAC and mute
    dac_i2c_write(1, 0x23, 0x44); // DAC_L & R mix routing
    dac_i2c_write(0, 0x3F, 0xD4); // DAC data path set up, l and r dac powered on
    set_volume(0x00, 0x00); // set DAC volume to minimum
    mute_dac(); // DAC muted

    // Power on the drivers
    dac_i2c_write(1, 0x1f, 0b00010100); // Headphone drivers off, Vout set to 1.65V
    dac_i2c_write(1, 0x21, 0x46); // Headphone pop reduction - 600ms driver power on time, driver ramp
    dac_i2c_write(1, 0x22, 0x70); // Speaker pop reduction - 30.5ms power up wait time
    dac_i2c_write(1, 0x1F, 0b11010100); // Headphone drivers powered on
    dac_i2c_write(1, 0x20, 0b11000110); // Class-D Amplifier powered on

    // Wait for drivers to stabilize and ramp up
    sleep_ms(100); 

    // Unmute drivers and dacs after power up ramp
    unmute_drivers();

    // set gain
    set_gain(0x7F); // set to max amp gain
    unmute_dac(); // Unmute DAC
}


void check_dac() {
    uint8_t data;
    dac_i2c_read(0, 0x25, &data, 1);
    printf("DAC Status Register: 0x%02X (binary: 0b", data);
    for (int i = 7; i >= 0; i--) {
        printf("%d", (data >> i) & 1);
    }
    printf(")\n");

    dac_i2c_read(1, 0x20, &data, 1);
    printf("DAC Status Register: 0x%02X (binary: 0b\n", data);

    dac_i2c_read(0, 0x1B, &data, 1);
    printf("\nDAC Status Register: 0x%02X (binary: 0b\n", data);
}

bool dac_advanced_block = true; // true = 25, false = 7
static inline void dac_select_processing_block(uint8_t block)
{
    // Optional: stop audible artifacts
    bool was_muted = is_muted;
    if (!was_muted) mute_dac();

    // Optional: small settle so mute takes effect
    sleep_us(500);

    // Select processing block
    dac_i2c_write(0, 0x3C, block);

    // Give the DSP time to latch (a few hundred us is usually enough)
    sleep_us(500);

    if (!was_muted) unmute_dac();
}

static inline void alternate_eq(void)
{
    dac_advanced_block = !dac_advanced_block;
    dac_select_processing_block(dac_advanced_block ? 0x0B : 0x07);
}
