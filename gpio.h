// MARK: - Ports
#define SD_SPI spi0
#define LCD_SPI spi1

#define IOX_I2C_PORT i2c1
#define BAT_MONITOR_I2C_PORT IOX_I2C_PORT


//MARK: - GPIO Pin Definitions

// Misc.
#define GPIO_QSPI_nCS1    0
#define GPIO_PWR_LED      1

// SD SPI
#define GPIO_SD_SCK       2
#define GPIO_SD_MOSI      3
#define GPIO_SD_MISO      4

#define GPIO_SW_OUT       5

// IOX & BAT Monitor I2C
#define GPIO_I2C_SDA      6
#define GPIO_I2C_SCL      7

// LCD SPI & B_SELECT
#define GPIO_LCD_MISO     8
#define GPIO_B_SELECT     9
#define GPIO_LCD_SCK      10
#define GPIO_LCD_MOSI     11

// DVI
#define GPIO_DVI_D2P      12
#define GPIO_DVI_D2N      13
#define GPIO_DVI_CLKP     14
#define GPIO_DVI_CLKN     15
#define GPIO_DVI_D1P      16
#define GPIO_DVI_D1N      17
#define GPIO_DVI_D0P      18
#define GPIO_DVI_D0N      19

#define GPIO_BUTTON_LED   20

// I2S
#define GPIO_I2S_BCLK     21
#define GPIO_I2S_DIN      22
#define GPIO_I2S_LRCLK    23

#define GPIO_LCD_LED      24

// DPI
#define GPIO_DPI_B0       25
#define GPIO_DPI_B1       26
#define GPIO_DPI_B2       27
#define GPIO_DPI_B3       28
#define GPIO_DPI_B4       29
#define GPIO_DPI_G0       30
#define GPIO_DPI_G1       31
#define GPIO_DPI_G2       32
#define GPIO_DPI_G3       33
#define GPIO_DPI_G4       34
#define GPIO_DPI_G5       35
#define GPIO_DPI_R0       36
#define GPIO_DPI_R1       37
#define GPIO_DPI_R2       38
#define GPIO_DPI_R3       39
#define GPIO_DPI_R4       40
#define GPIO_DPI_VSYNC    41
#define GPIO_DPI_HSYNC    42
#define GPIO_DPI_PCLK     43
#define GPIO_DPI_DEN      44

// Additional Pins
#warning "Gonna change vbat adc to iox interrupt at some point"
#define GPIO_VBAT_ADC     45 
#define GPIO_nHP_DETECT   46
#define GPIO_AUD_POT_ADC  47


// MARK: - IOX Pin definitions
#define IOX_B_A         (0 + 48)
#define IOX_B_B         (1 + 48)
#define IOX_B_UP        (2 + 48)
#define IOX_B_DOWN      (3 + 48)
#define IOX_B_LEFT      (4 + 48)
#define IOX_B_RIGHT     (5 + 48)
#define IOX_B_START     (6 + 48)

#define IOX_DVI_DETECT  (7 + 48)

#define IOX_SD_nCS      (10 + 48)
#define IOX_TFT_nCS     (11 + 48)
#define IOX_IPS_nCS     (12 + 48)
#define IOX_LCD_RST     (13 + 48)
#define IOX_n3V3_MCU_EN (14 + 48)

#warning "Changing cc detect to speaker cut off instead"
#define IOX_nCC_DETECT  (15 + 48)

#define IOX_nBT_AUDIO   (16 + 48)
#define IOX_nBT_PAIR    (17 + 48)

// MARK: - Boolean variables for IO Expander pins
bool b_a_state = 0;
bool b_b_state = 0;
bool b_up_state = 0;
bool b_down_state = 0;
bool b_left_state = 0;
bool b_right_state = 0;
bool b_start_state = 0;
bool dvi_detect_state = 0;
bool sd_ncs_state = 0;
bool tft_ncs_state = 0;
bool ips_ncs_state = 0;
bool lcd_rst_state = 0;
bool n3v3_mcu_en_state = 0;
bool ncc_detect_state = 0;
bool nbt_audio_state = 0;
bool nbt_pair_state = 0;

// MARK: - IOX REGS
#define IOX_I2C_ADDR 0x20
const uint8_t IOX_INPUT_0_REG = 0x00;
const uint8_t IOX_INPUT_1_REG = 0x01;
const uint8_t IOX_OUTPUT_0_REG = 0x02;
const uint8_t IOX_OUTPUT_1_REG = 0x03;
const uint8_t IOX_CONFIG_0_REG = 0x06;
const uint8_t IOX_CONFIG_1_REG = 0x07;


// MARK: - Functions declarations
void config_iox_port(bool port, uint8_t config_data);
void config_iox_ports();
void read_io_expander_states(int8_t port);
void write_io_expander_states(bool port, uint8_t data);
void write_iox_port1(int8_t new_nbt_pair_state, 
                     int8_t new_nbt_audio_state, 
                     int8_t new_ncc_detect_state, 
                     int8_t new_n3v3_mcu_en_state, 
                     int8_t new_lcd_rst_state, 
                     int8_t new_ips_ncs_state, 
                     int8_t new_tft_ncs_state, 
                     int8_t new_sd_ncs_state);
bool gpio_read(uint8_t gpio_num);
bool iox_state_lookup(uint8_t gpio_num);
void iox_state_set(uint8_t gpio_num, bool value);
uint8_t get_iox_port_states(bool port);


void config_iox_port(bool port, uint8_t config_data) {
    uint8_t reg = (port == 0) ? IOX_CONFIG_0_REG : IOX_CONFIG_1_REG;
    uint8_t write_buffer[2] = {reg, config_data};
    // Write register address and data in one I2C transaction
    i2c_write_blocking(IOX_I2C_PORT, IOX_I2C_ADDR, write_buffer, 2, false);
}

// Configures pins as I/O and sets default values.
void config_iox_ports() {
    // 1 input, 0 output
    // Port 0 has all inputs.
    config_iox_port(0, 0b11111111);
    #warning "Change cc detect config to output when changed to speaker control"
    // Port 1 has all outputs except nCC_DETECT
    config_iox_port(1, 0b00100000);

    // Set up IOX outputs with default values.
    write_iox_port1(1, 1, 0, 1, 0, 1, 1, 1);
}

// Ports 0 & 1
// Set port = 2 to read both.
void read_io_expander_states(int8_t port) {    
    // Recursion to read both ports with port = 2
    if (port > 2 || port < 0) return;
    if (port == 2) {
        read_io_expander_states(port - 1);
    }

    uint8_t read_buffer = 0;
    uint8_t reg = (port == 0) ? IOX_INPUT_0_REG : IOX_INPUT_1_REG;
    
    // Start I2C transmission to the slave device
    i2c_write_blocking(IOX_I2C_PORT, IOX_I2C_ADDR, &reg, 1, true);

    // Read 1 byte of data from the input register
    i2c_read_blocking(IOX_I2C_PORT, IOX_I2C_ADDR, &read_buffer, 1, false);

    // Output the state of the pins (MSB first)
    printf("Pin states: 0x%02X\n", read_buffer);

    // Decode individual pins based on port value
    if (port == 0) {
        b_a_state       = read_buffer & (1 << (IOX_B_A - 48));
        b_b_state       = read_buffer & (1 << (IOX_B_B - 48));
        b_up_state      = read_buffer & (1 << (IOX_B_UP - 48));
        b_down_state    = read_buffer & (1 << (IOX_B_DOWN - 48));
        b_left_state    = read_buffer & (1 << (IOX_B_LEFT - 48));
        b_right_state   = read_buffer & (1 << (IOX_B_RIGHT - 48));
        b_start_state   = read_buffer & (1 << (IOX_B_START - 48));
        dvi_detect_state = read_buffer & (1 << (IOX_DVI_DETECT - 48));
    } else {
        sd_ncs_state    = read_buffer & (1 << (IOX_SD_nCS - 10 - 48));
        tft_ncs_state   = read_buffer & (1 << (IOX_TFT_nCS - 10 - 48));
        ips_ncs_state   = read_buffer & (1 << (IOX_IPS_nCS - 10 - 48));
        lcd_rst_state   = read_buffer & (1 << (IOX_LCD_RST - 10 - 48));
        n3v3_mcu_en_state = read_buffer & (1 << (IOX_n3V3_MCU_EN - 10 - 48));
        ncc_detect_state = read_buffer & (1 << (IOX_nCC_DETECT - 10 - 48));
        nbt_audio_state = read_buffer & (1 << (IOX_nBT_AUDIO - 10 - 48));
        nbt_pair_state  = read_buffer & (1 << (IOX_nBT_PAIR - 10 - 48));
    }

}

void write_io_expander_states(bool port, uint8_t data) {
    const uint8_t reg = (port == 0) ? IOX_OUTPUT_0_REG : IOX_OUTPUT_1_REG;
    uint8_t write_buffer[2] = {reg, data};

    // Write register address and data in one I2C transaction
    i2c_write_blocking(IOX_I2C_PORT, IOX_I2C_ADDR, write_buffer, 2, false);
}

#define NO_UPDATE -1
void write_iox_port0(int8_t new_dvi_detect_state, 
                     int8_t new_b_start_state, 
                     int8_t new_b_right_state, 
                     int8_t new_b_left_state, 
                     int8_t new_b_down_state, 
                     int8_t new_b_up_state, 
                     int8_t new_b_b_state, 
                     int8_t new_b_a_state) {
    if (new_dvi_detect_state != NO_UPDATE) {
        dvi_detect_state = new_dvi_detect_state;
    }
    if (new_b_start_state != NO_UPDATE) {
        b_start_state = new_b_start_state;
    }
    if (new_b_right_state != NO_UPDATE) {
        b_right_state = new_b_right_state;
    }
    if (new_b_left_state != NO_UPDATE) {
        b_left_state = new_b_left_state;
    }
    if (new_b_down_state != NO_UPDATE) {
        b_down_state = new_b_down_state;
    }
    if (new_b_up_state != NO_UPDATE) {
        b_up_state = new_b_up_state;
    }
    if (new_b_b_state != NO_UPDATE) {
        b_b_state = new_b_b_state;
    }
    if (new_b_a_state != NO_UPDATE) {
        b_a_state = new_b_a_state;
    }

    uint8_t states = get_iox_port_states(0);
    write_io_expander_states(0, states);
}

void write_iox_port1(int8_t new_nbt_pair_state, 
                     int8_t new_nbt_audio_state, 
                     int8_t new_ncc_detect_state, 
                     int8_t new_n3v3_mcu_en_state, 
                     int8_t new_lcd_rst_state, 
                     int8_t new_ips_ncs_state, 
                     int8_t new_tft_ncs_state, 
                     int8_t new_sd_ncs_state) {
    if (new_nbt_pair_state != NO_UPDATE) {
        nbt_pair_state = new_nbt_pair_state;
    }
    if (new_nbt_audio_state != NO_UPDATE) {
        nbt_audio_state = new_nbt_audio_state;
    }
    if (new_ncc_detect_state != NO_UPDATE) {
        ncc_detect_state = new_ncc_detect_state;
    }
    if (new_n3v3_mcu_en_state != NO_UPDATE) {
        n3v3_mcu_en_state = new_n3v3_mcu_en_state;
    }
    if (new_lcd_rst_state != NO_UPDATE) {
        lcd_rst_state = new_lcd_rst_state;
    }
    if (new_ips_ncs_state != NO_UPDATE) {
        ips_ncs_state = new_ips_ncs_state;
    }
    if (new_tft_ncs_state != NO_UPDATE) {
        tft_ncs_state = new_tft_ncs_state;
    }
    if (new_sd_ncs_state != NO_UPDATE) {
        sd_ncs_state = new_sd_ncs_state;
    }

    uint8_t states = get_iox_port_states(1); // Get the combined state for port 1
    write_io_expander_states(1, states);    // Write the states to the IO expander
}

bool gpio_read(uint8_t gpio_num) {
    bool state;
    if (gpio_num <= 47) {
        state = gpio_get(gpio_num);
    } else {
        // For IOX pins, return the corresponding state variable
        state = iox_state_lookup(gpio_num);
    }
    return state;
}

void gpio_write(uint8_t gpio_num, bool value) {
    if (gpio_num <= 47) {
        gpio_put(gpio_num, value);
    } else {
        // Calculate the pin number relative to the IOX port (pins 48 to 55 for Port 0, pins 58 to 65 for Port 1)
        uint8_t pin = gpio_num - 48;
        
        // Determine the port: Port 0 for pins 48-55, Port 1 for pins 58-65
        bool port = (pin >= 10) ? 1 : 0;  // Port 0 for pins 48-55, Port 1 for pins 58-65

        iox_state_set(gpio_num, value);

        // Get the current state of the specified port
        uint8_t states = get_iox_port_states(port);

        // Write the updated states to the IO expander for the correct port
        write_io_expander_states(port, states);
    }
}

bool iox_state_lookup(uint8_t gpio_num) {
    switch (gpio_num) {
        case IOX_B_A:
            return b_a_state;
        case IOX_B_B:
            return b_b_state;
        case IOX_B_UP:
            return b_up_state;
        case IOX_B_DOWN:
            return b_down_state;
        case IOX_B_LEFT:
            return b_left_state;
        case IOX_B_RIGHT:
            return b_right_state;
        case IOX_B_START:
            return b_start_state;
        case IOX_DVI_DETECT:
            return dvi_detect_state;

        case IOX_SD_nCS:
            return sd_ncs_state;
        case IOX_TFT_nCS:
            return tft_ncs_state;
        case IOX_IPS_nCS:
            return ips_ncs_state;
        case IOX_LCD_RST:
            return lcd_rst_state;
        case IOX_n3V3_MCU_EN:
            return n3v3_mcu_en_state;
        case IOX_nCC_DETECT:
            return ncc_detect_state;
        case IOX_nBT_AUDIO:
            return nbt_audio_state;
        case IOX_nBT_PAIR:
            return nbt_pair_state;
        default:
            // Invalid IOX pin number
            printf("Invalid GPIO");
            return false;  // Default return value for invalid input
    }
}

void iox_state_set(uint8_t gpio_num, bool value) {
    switch (gpio_num) {
        case IOX_B_A:
            b_a_state = value;
            break;
        case IOX_B_B:
            b_b_state = value;
            break;
        case IOX_B_UP:
            b_up_state = value;
            break;
        case IOX_B_DOWN:
            b_down_state = value;
            break;
        case IOX_B_LEFT:
            b_left_state = value;
            break;
        case IOX_B_RIGHT:
            b_right_state = value;
            break;
        case IOX_B_START:
            b_start_state = value;
            break;
        case IOX_DVI_DETECT:
            dvi_detect_state = value;
            break;
        case IOX_SD_nCS:
            sd_ncs_state = value;
            break;
        case IOX_TFT_nCS:
            tft_ncs_state = value;
            break;
        case IOX_IPS_nCS:
            ips_ncs_state = value;
            break;
        case IOX_LCD_RST:
            lcd_rst_state = value;
            break;
        case IOX_n3V3_MCU_EN:
            n3v3_mcu_en_state = value;
            break;
        case IOX_nCC_DETECT:
            ncc_detect_state = value;
            break;
        case IOX_nBT_AUDIO:
            nbt_audio_state = value;
            break;
        case IOX_nBT_PAIR:
            nbt_pair_state = value;
            break;
        default:
            printf("Invalid GPIO pin\n");
            break;
    }
}

uint8_t get_iox_port_states(bool port) {
    if (port == 0) {
        return (dvi_detect_state << 7) | 
               (b_start_state << 6) | 
               (b_right_state << 5) | 
               (b_left_state << 4) | 
               (b_down_state << 3) | 
               (b_up_state << 2) | 
               (b_b_state << 1) | 
               (b_a_state << 0);
    } else {
        return (nbt_pair_state << 7) | 
               (nbt_audio_state << 6) | 
               (ncc_detect_state << 5) | 
               (n3v3_mcu_en_state << 4) | 
               (lcd_rst_state << 3) | 
               (ips_ncs_state << 2) | 
               (tft_ncs_state << 1) | 
               (sd_ncs_state << 0);
    }
}