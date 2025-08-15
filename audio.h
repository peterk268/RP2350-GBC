#define DAC_I2C_ADDR 0b0011000

uint16_t current_volume_level = 0;
bool is_muted = false;
void set_volume(uint8_t l_volume, uint8_t r_volume);

// void read_volume(i2s_config_t *i2s_config) {

//     adc_select_input(GPIO_AUD_POT_ADC - 40); 
//     uint16_t volume_adc_value = read_adc();

//     // Scale to 0–31 steps
//     uint16_t volume_level = volume_adc_value / (4096 / 32);

//     if (abs(current_volume_level - volume_level) > 1) {
//         current_volume_level = volume_level;
//         uint8_t dac_volume = (volume_level * 0x7F) / 31; // Map to DAC range 0–127
//         set_volume(dac_volume, dac_volume);
//     }

//     #warning "come back to this with the i2c hp detect"
//     // Scale the volume based on headphone detect voltage
//     // if (hp_detect_voltage < 3.0f) { // Check if voltage is less than 3V (headphones present)
//     //     i2s_volume(i2s_config, (volume_level) / 2.0f); // Reduced volume scaling
//     // } else {
//         // i2s_volume(i2s_config, volume_level); // Full volume
//     // }
// }
void read_volume() {
    adc_select_input(GPIO_AUD_POT_ADC - 40); 
    uint16_t adc_val = read_adc(); // 0–4095

    // Linear scaling to DAC 0–127
    uint8_t dac_vol = (uint8_t)((adc_val * 127) / 4095);

    if (abs((int)dac_vol - (int)current_volume_level) > 1) {
        current_volume_level = dac_vol;
        set_volume(current_volume_level, current_volume_level);
    }

    // Mute logic
    if (current_volume_level == 0 && !is_muted) is_muted = true;
    else if (current_volume_level > 0 && is_muted) is_muted = false;
}


uint8_t current_page = 0;
void dac_i2c_page_select(uint8_t page) {
    if (current_page == page) return; // No need to change page if already set

    uint8_t page_buffer[2] = {0x00, page ? 0x01 : 0x00};
    // Write the page select register (stop = true so DAC latches it)
    i2c_write_blocking(DAC_I2C_PORT, DAC_I2C_ADDR, page_buffer, 2, false);
    sleep_us(50); // let DAC latch
    current_page = page; // Update current page state
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
    dac_i2c_write(1, 0x24, 0x80 | (127 - (gain_l & 0x7f)));
    dac_i2c_write(1, 0x25, 0x80 | (127 - (gain_r & 0x7f)));
}
void set_gain(uint8_t gain) {
    set_headphone_gain(gain, gain);
    set_speaker_gain(gain, gain);
}

// void set_volume(uint8_t l_volume, uint8_t r_volume) {
//     float lRange = -63.5 + ((l_volume / 255.0) * 87.5);
//     float rRange = -63.5 + ((r_volume / 255.0) * 87.5);

//     int8_t lInt = (int)lRange;
//     int8_t rInt = (int)rRange;


//     if ((lRange - lInt) >= 0.5) {
//         lInt <<= 1;
//         lInt |= 1;
//     } else {
//         lInt <<= 1;
//     }

//     if ((rRange - rInt) >= 0.5) {
//         rInt <<= 1;
//         rInt |= 1;
//     } else {
//         rInt <<= 1;
//     }

//     dac_i2c_write(0, 0x41, lInt);
//     dac_i2c_write(0, 0x42, rInt);
// }
// TLV320DAC3101 register encoding
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



void setup_dac() {
    dac_i2c_write(0, 0x01, 0x01); // Soft-reset the DAC
    sleep_ms(20); // Wait for reset to complete

    dac_i2c_write(0, 0x0b, 0x81); 
    dac_i2c_write(0, 0x0c, 0x82); 
    dac_i2c_write(0, 0x0d, 0x00);
    dac_i2c_write(0, 0x0e, 0x80);
    dac_i2c_write(0, 0x1b, 0b00000000);

    dac_i2c_write(0, 0x3c, 0x0b); // Block 25
    dac_i2c_write(8, 0x01, 0x04);

    dac_i2c_write(0, 0x74, 0x00);

    dac_i2c_write(1, 0x1f, 0b00010100);
    dac_i2c_write(1, 0x21, 0x4e);
    dac_i2c_write(1, 0x23, 0x44);
    dac_i2c_write(1, 0x28, 0x06);
    dac_i2c_write(1, 0x29, 0x06);
    dac_i2c_write(1, 0x2A, 0x1C);
    dac_i2c_write(1, 0x2B, 0x1C);
    dac_i2c_write(1, 0x1F, 0b11010100);
    dac_i2c_write(1, 0x20, 0b11000110);

    // set gain
    set_gain(0x7F); // set to max amp gain
    dac_i2c_write(0, 0x40, 0x00); // unmute

    dac_i2c_write(0, 0x3F, 0xD4);
    // dac_i2c_write(0, 0x41, 0xD4);
    // dac_i2c_write(0, 0x42, 0xD4);
    set_volume(0x00, 0x00); // set volume to minimum
    dac_i2c_write(0, 0x40, 0x00);
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