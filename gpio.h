// MARK: - Ports
#define SD_SPI               spi0
#define LCD_SPI              spi0
#define ESP_SPI              spi0

#define IOX_I2C_PORT         i2c1
#define BAT_MONITOR_I2C_PORT i2c1
#define IMU_I2C_PORT         i2c1
#define RTC_I2C_PORT         i2c1
#define DAC_I2C_PORT         i2c1

#define I2S_PIO              pio1
#define I2S_PIO_GPIO         GPIO_FUNC_PIO1

// MARK: - GPIO Pin Definitions

// Misc.
#define GPIO_QSPI_CS1        0  // PSRAM CS1
// PWR HOLD gets moved to gpio 1 and imu int to iox and sd card detect removed
#define GPIO_PWR_HOLD        1

// SD, LCD, ESP SPI (SPI0 Shared)
#define GPIO_SPI0_SCK        2
#define GPIO_SPI0_MOSI       3
#define GPIO_SPI0_MISO       4

#define GPIO_SW_OUT          5

// IOX & I2C Peripherals (I2C1 Shared)
#define GPIO_I2C1_SDA        6
#define GPIO_I2C1_SCL        7

// ESP32 UART
#define GPIO_ESP_UART1_TX    8
#define GPIO_ESP_UART1_RX    9

#define GPIO_B_SELECT        10
#define GPIO_IOX_nINT        11

// DVI
#define GPIO_DVI_D2P         12
#define GPIO_DVI_D2N         13
#define GPIO_DVI_CLKP        14
#define GPIO_DVI_CLKN        15
#define GPIO_DVI_D1P         16
#define GPIO_DVI_D1N         17
#define GPIO_DVI_D0P         18
#define GPIO_DVI_D0N         19

// I2S
#define GPIO_I2S_MCLK        20
#define GPIO_I2S_BCLK        21
#define GPIO_I2S_LRCLK       22
#define GPIO_I2S_DIN         23

#define GPIO_LCD_LED         24

// DPI
#define GPIO_DPI_B0          25
#define GPIO_DPI_B1          26
#define GPIO_DPI_B2          27
#define GPIO_DPI_B3          28
#define GPIO_DPI_B4          29
#define GPIO_DPI_G0          30
#define GPIO_DPI_G1          31
#define GPIO_DPI_G2          32
#define GPIO_DPI_G3          33
#define GPIO_DPI_G4          34
#define GPIO_DPI_G5          35
#define GPIO_DPI_R0          36
#define GPIO_DPI_R1          37
#define GPIO_DPI_R2          38
#define GPIO_DPI_R3          39
#define GPIO_DPI_R4          40
#define GPIO_DPI_HSYNC       41
#define GPIO_DPI_VSYNC       42
#define GPIO_DPI_PCLK        43
#define GPIO_DPI_DEN         44

// Additional Pins
#define GPIO_PWR_LED         45
#define GPIO_BUTTON_LED      46
#define GPIO_AUD_POT_ADC     47

// MARK: - IOX Pin Definitions
#define IOX_B_A              (0  + 48)
#define IOX_B_B              (1  + 48)
#define IOX_B_UP             (2  + 48)
#define IOX_B_DOWN           (3  + 48)
#define IOX_B_LEFT           (4  + 48)
#define IOX_B_RIGHT          (5  + 48)
#define IOX_B_START          (6  + 48)
#define IOX_DVI_DETECT       (7  + 48)

#define IOX_SD_CS            (10 + 48)
#define IOX_LCD_nCS          (11 + 48)
#define IOX_ESP_nCS          (12 + 48)
#define IOX_LCD_nRST         (13 + 48)
#define IOX_ESP_EN           (14 + 48)
#define IOX_AUDIO_EN         (15 + 48)
#define IOX_SD_nEN           (16 + 48)
#define IOX_CAMERA_nCS       (17 + 48)


void set_up_select() {
    gpio_set_function(GPIO_B_SELECT, GPIO_FUNC_SIO);
    gpio_set_dir(GPIO_B_SELECT, false);
    gpio_pull_up(GPIO_B_SELECT);
    sleep_ms(1);

    // If the Select button is pressed during power-on, we should go into bootloader mode.
    if (!gpio_get(GPIO_B_SELECT)) {
        // Parameters: gpio_activity_pin_mask, disable_interface_mask
        reset_usb_boot(0, 0);
    }
}

void gpio_write(uint8_t gpio_num, bool value);

void setup_hold_power() {
    printf("Setting up power hold on GPIO %d\n", GPIO_PWR_HOLD);
    gpio_set_function(GPIO_PWR_HOLD, GPIO_FUNC_SIO);
	gpio_set_dir(GPIO_PWR_HOLD, true);
    gpio_write(GPIO_PWR_HOLD, false);

    gpio_init(GPIO_SW_OUT);
    gpio_set_dir(GPIO_SW_OUT, GPIO_IN);
}

void hold_power() {
    gpio_write(GPIO_PWR_HOLD, true);
}
void release_power() {
    gpio_write(GPIO_PWR_HOLD, false);
}

void test_hold_power() {
    hold_power();
    sleep_ms(5000);
    release_power();
}

#define I2C_RECOVERY_CLOCKS 9

static void i2c_recover_bus(uint sda_pin, uint scl_pin) {
    gpio_set_function(sda_pin, GPIO_FUNC_SIO);
    gpio_set_function(scl_pin, GPIO_FUNC_SIO);
    gpio_pull_down(sda_pin);
    gpio_pull_down(scl_pin);
    gpio_set_dir(sda_pin, true);
    gpio_set_dir(scl_pin, true);
    gpio_put(sda_pin, 0);
    gpio_put(scl_pin, 0);
    sleep_ms(1);
    gpio_put(sda_pin, 1);
    gpio_put(scl_pin, 1);
    sleep_ms(1);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Send 9 clock pulses to free the bus
    for (int i = 0; i < I2C_RECOVERY_CLOCKS; i++) {
        gpio_put(scl_pin, 0);
        sleep_us(5);
        gpio_put(scl_pin, 1);
        sleep_us(5);
    }

    // Generate a STOP condition
    gpio_put(sda_pin, 0);
    sleep_us(5);
    gpio_put(scl_pin, 1);
    sleep_us(5);
    gpio_put(sda_pin, 1);
    sleep_us(5);

    // Restore I²C function
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
}

void init_i2c() {
#if I2C_HAS_TROUBLES
    i2c_recover_bus(GPIO_I2C1_SDA, GPIO_I2C1_SCL);
#endif

    gpio_set_function(GPIO_I2C1_SDA, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_I2C1_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(GPIO_I2C1_SDA);
    gpio_pull_up(GPIO_I2C1_SCL);

    sleep_us(100);

    i2c_init(IOX_I2C_PORT, 400 * 1000); // 400 kHz

    sleep_ms(1);
}

// Waits until the I2C bus is idle and optionally checks if a device ACKs.
// Returns true if ready, false if timeout.
bool i2c_wait_ready(i2c_inst_t *i2c, uint8_t dev_addr, uint32_t timeout_ms) {
    absolute_time_t start_time = get_absolute_time();
    
    while (true) {
        // 1. Check if master is active
        if (!(i2c_get_hw(i2c)->status & I2C_IC_STATUS_MST_ACTIVITY_BITS)) {
            // Bus is idle, now check device if address is provided
            if (dev_addr != 0xFF) {
                int ret = i2c_write_blocking(i2c, dev_addr, NULL, 0, false);
                if (ret >= 0) {
                    printf("I2C device 0x%02X is ready\n", dev_addr);
                    return true; // Device responded
                }
            } else {
                printf("I2C bus idle\n");
                return true; // Bus idle, no device check requested
            }
        }

        // Timeout
        if (absolute_time_diff_us(start_time, get_absolute_time()) > timeout_ms * 1000) {
            printf("I2C wait timeout\n");
            return false;
        }

        sleep_ms(1); // Small delay to avoid tight loop
    }
}

// MARK: - Boolean variables for IO Expander pins
bool b_a_state = 0;
bool b_b_state = 0;
bool b_up_state = 0;
bool b_down_state = 0;
bool b_left_state = 0;
bool b_right_state = 0;
bool b_start_state = 0;
bool dvi_detect_state = 0;

bool sd_cs_state = 0;
bool lcd_ncs_state = 1;
bool esp_ncs_state = 1;
bool lcd_rst_state = 0;
bool esp_en_state = 0;
bool audio_en_state = 0;
bool sd_nen_state = 1;
bool camera_ncs_state = 0;

// MARK: - IOX REGS
#define IOX_I2C_ADDR 0x20
const uint8_t IOX_INPUT_0_REG = 0x00;
const uint8_t IOX_INPUT_1_REG = 0x01;
const uint8_t IOX_OUTPUT_0_REG = 0x02;
const uint8_t IOX_OUTPUT_1_REG = 0x03;
const uint8_t IOX_POLARITY_0_REG = 0x04;
const uint8_t IOX_POLARITY_1_REG = 0x05;
const uint8_t IOX_CONFIG_0_REG = 0x06;
const uint8_t IOX_CONFIG_1_REG = 0x07;

// MARK: - Function declarations
void config_iox_port(bool port, uint8_t config_data);
void config_iox_ports();
void read_io_expander_states(int8_t port);
void write_io_polarity(bool port, uint8_t polarity_data);
void write_io_expander_states(bool port, uint8_t data);
void write_iox_port1(int8_t new_sd_cs_state, int8_t new_lcd_ncs_state, int8_t new_esp_ncs_state, int8_t new_lcd_rst_state, int8_t new_esp_en_state, int8_t new_audio_en_state, int8_t new_sd_nen_state, int8_t new_camera_ncs_state);
bool gpio_read(uint8_t gpio_num);
bool iox_state_lookup(uint8_t gpio_num);
void iox_state_set(uint8_t gpio_num, bool value);
uint8_t get_iox_port_states(bool port);

void config_iox_port(bool port, uint8_t config_data) {
    uint8_t reg = (port == 0) ? IOX_CONFIG_0_REG : IOX_CONFIG_1_REG;
    uint8_t write_buffer[2] = {reg, config_data};
    i2c_write_blocking(IOX_I2C_PORT, IOX_I2C_ADDR, write_buffer, 2, false);
}

// Configures pins as I/O and sets default values.
void config_iox_ports() {
    gpio_init(GPIO_IOX_nINT);
    gpio_set_dir(GPIO_IOX_nINT, GPIO_IN);

    // Port 0: all inputs (buttons and DVI detect)
    config_iox_port(0, 0b11111111);
    // Port 1: outputs (SD, LCD, ESP, Audio control signals) & Camera nCS Input
    config_iox_port(1, 0b00000000);

    // Initialize IOX outputs to defaults
    write_iox_port1(0, 1, 1, 0, 0, 0, 1, 1);

    sleep_ms(1);
    // clear any interrupt states
    read_io_expander_states(2);
}

// port: 0 or 1, or 2 for both
void read_io_expander_states(int8_t port) {
    if (port > 2 || port < 0) return;
    if (port == 2) {
        read_io_expander_states(0);
        read_io_expander_states(1);
        return;
    }

    uint8_t read_buffer = 0;
    uint8_t reg = (port == 0) ? IOX_INPUT_0_REG : IOX_INPUT_1_REG;

    i2c_write_blocking(IOX_I2C_PORT, IOX_I2C_ADDR, &reg, 1, true);
    i2c_read_blocking(IOX_I2C_PORT, IOX_I2C_ADDR, &read_buffer, 1, false);

    if (port == 0) {
        b_a_state        = read_buffer & (1 << (IOX_B_A - 48));
        b_b_state        = read_buffer & (1 << (IOX_B_B - 48));
        b_up_state       = read_buffer & (1 << (IOX_B_UP - 48));
        b_down_state     = read_buffer & (1 << (IOX_B_DOWN - 48));
        b_left_state     = read_buffer & (1 << (IOX_B_LEFT - 48));
        b_right_state    = read_buffer & (1 << (IOX_B_RIGHT - 48));
        b_start_state    = read_buffer & (1 << (IOX_B_START - 48));
        dvi_detect_state = read_buffer & (1 << (IOX_DVI_DETECT - 48));
    } else {
        sd_cs_state   = read_buffer & (1 << (IOX_SD_CS - 48));
        lcd_ncs_state  = read_buffer & (1 << (IOX_LCD_nCS - 48));
        esp_ncs_state  = read_buffer & (1 << (IOX_ESP_nCS - 48));
        lcd_rst_state  = read_buffer & (1 << (IOX_LCD_nRST - 48));
        esp_en_state   = read_buffer & (1 << (IOX_ESP_EN - 48));
        audio_en_state = read_buffer & (1 << (IOX_AUDIO_EN - 48));
        sd_nen_state   = read_buffer & (1 << (IOX_SD_nEN - 48));
        camera_ncs_state = read_buffer & (1 << (IOX_CAMERA_nCS - 48));
    }
}

void write_io_polarity(bool port, uint8_t polarity_data) {
    uint8_t reg = (port == 0) ? IOX_POLARITY_0_REG : IOX_POLARITY_1_REG;
    uint8_t write_buffer[2] = {reg, polarity_data};
    i2c_write_blocking(IOX_I2C_PORT, IOX_I2C_ADDR, write_buffer, 2, false);
}

void write_io_expander_states(bool port, uint8_t data) {
    const uint8_t reg = (port == 0) ? IOX_OUTPUT_0_REG : IOX_OUTPUT_1_REG;
    uint8_t write_buffer[2] = {reg, data};
    i2c_write_blocking(IOX_I2C_PORT, IOX_I2C_ADDR, write_buffer, 2, false);
}

#define NO_UPDATE -1
void write_iox_port0(int8_t new_dvi_detect_state, int8_t new_b_start_state, int8_t new_b_right_state, int8_t new_b_left_state, int8_t new_b_down_state, int8_t new_b_up_state, int8_t new_b_b_state, int8_t new_b_a_state) {
    if (new_dvi_detect_state != NO_UPDATE) dvi_detect_state = new_dvi_detect_state;
    if (new_b_start_state != NO_UPDATE) b_start_state = new_b_start_state;
    if (new_b_right_state != NO_UPDATE) b_right_state = new_b_right_state;
    if (new_b_left_state != NO_UPDATE) b_left_state = new_b_left_state;
    if (new_b_down_state != NO_UPDATE) b_down_state = new_b_down_state;
    if (new_b_up_state != NO_UPDATE) b_up_state = new_b_up_state;
    if (new_b_b_state != NO_UPDATE) b_b_state = new_b_b_state;
    if (new_b_a_state != NO_UPDATE) b_a_state = new_b_a_state;

    uint8_t states = get_iox_port_states(0);
    write_io_expander_states(0, states);
}

void write_iox_port1(int8_t new_sd_cs_state, int8_t new_lcd_ncs_state, int8_t new_esp_ncs_state, int8_t new_lcd_rst_state, int8_t new_esp_en_state, int8_t new_audio_en_state, int8_t new_sd_nen_state, int8_t new_camera_ncs_state) {
    if (new_sd_cs_state != NO_UPDATE) sd_cs_state = new_sd_cs_state;
    if (new_lcd_ncs_state != NO_UPDATE) lcd_ncs_state = new_lcd_ncs_state;
    if (new_esp_ncs_state != NO_UPDATE) esp_ncs_state = new_esp_ncs_state;
    if (new_lcd_rst_state != NO_UPDATE) lcd_rst_state = new_lcd_rst_state;
    if (new_esp_en_state != NO_UPDATE) esp_en_state = new_esp_en_state;
    if (new_audio_en_state != NO_UPDATE) audio_en_state = new_audio_en_state;
    if (new_sd_nen_state != NO_UPDATE) sd_nen_state = new_sd_nen_state;
    if (new_camera_ncs_state != NO_UPDATE) camera_ncs_state = new_camera_ncs_state;

    uint8_t states = get_iox_port_states(1);
    write_io_expander_states(1, states);
}

bool gpio_read(uint8_t gpio_num) {
    if (gpio_num < 48) {
        return gpio_get(gpio_num);
    } else {
        return iox_state_lookup(gpio_num);
    }
}

void gpio_write(uint8_t gpio_num, bool value) {
    if (gpio_num < 48) {
        gpio_put(gpio_num, value);
    } else {
        iox_state_set(gpio_num, value);
        uint8_t pin = gpio_num - 48;
        bool port = (pin >= 10);  // pins 48-57 port0, 58+ port1
        uint8_t states = get_iox_port_states(port);
        write_io_expander_states(port, states);
    }
}

bool iox_state_lookup(uint8_t gpio_num) {
    switch (gpio_num) {
        case IOX_B_A:        return b_a_state;
        case IOX_B_B:        return b_b_state;
        case IOX_B_UP:       return b_up_state;
        case IOX_B_DOWN:     return b_down_state;
        case IOX_B_LEFT:     return b_left_state;
        case IOX_B_RIGHT:    return b_right_state;
        case IOX_B_START:    return b_start_state;
        case IOX_DVI_DETECT: return dvi_detect_state;

        case IOX_SD_CS:     return sd_cs_state;
        case IOX_LCD_nCS:    return lcd_ncs_state;
        case IOX_ESP_nCS:    return esp_ncs_state;
        case IOX_LCD_nRST:   return lcd_rst_state;
        case IOX_ESP_EN:     return esp_en_state;
        case IOX_AUDIO_EN:   return audio_en_state;
        case IOX_SD_nEN:     return sd_nen_state;
        case IOX_CAMERA_nCS: return camera_ncs_state;

        default:
            printf("Invalid GPIO pin\n");
            return false;
    }
}

void iox_state_set(uint8_t gpio_num, bool value) {
    switch (gpio_num) {
        case IOX_B_A:        b_a_state = value; break;
        case IOX_B_B:        b_b_state = value; break;
        case IOX_B_UP:       b_up_state = value; break;
        case IOX_B_DOWN:     b_down_state = value; break;
        case IOX_B_LEFT:     b_left_state = value; break;
        case IOX_B_RIGHT:    b_right_state = value; break;
        case IOX_B_START:    b_start_state = value; break;
        case IOX_DVI_DETECT: dvi_detect_state = value; break;

        case IOX_SD_CS:     sd_cs_state = value; break;
        case IOX_LCD_nCS:    lcd_ncs_state = value; break;
        case IOX_ESP_nCS:    esp_ncs_state = value; break;
        case IOX_LCD_nRST:   lcd_rst_state = value; break;
        case IOX_ESP_EN:     esp_en_state = value; break;
        case IOX_AUDIO_EN:   audio_en_state = value; break;
        case IOX_SD_nEN:     sd_nen_state = value; break;
        case IOX_CAMERA_nCS: camera_ncs_state = value; break;

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
        return (camera_ncs_state << 7) |
               (sd_nen_state << 6) |
               (audio_en_state << 5) |
               (esp_en_state << 4) |
               (lcd_rst_state << 3) |
               (esp_ncs_state << 2) |
               (lcd_ncs_state << 1) |
               (sd_cs_state << 0);
    }
}

// MARK: - ADC
// Function to initialize the ADC
void init_adc(uint8_t adc_gpio) {
    // Initialize ADC hardware
    adc_init();

    // Map GPIO 40-47 to ADC inputs 0-7
    switch (adc_gpio) {
        case 40: adc_gpio_init(adc_gpio); adc_select_input(0); break;  // GPIO 40 corresponds to ADC0
        case 41: adc_gpio_init(adc_gpio); adc_select_input(1); break;  // GPIO 41 corresponds to ADC1
        case 42: adc_gpio_init(adc_gpio); adc_select_input(2); break;  // GPIO 42 corresponds to ADC2
        case 43: adc_gpio_init(adc_gpio); adc_select_input(3); break;  // GPIO 43 corresponds to ADC3
        case 44: adc_gpio_init(adc_gpio); adc_select_input(4); break;  // GPIO 44 corresponds to ADC4
        case 45: adc_gpio_init(adc_gpio); adc_select_input(5); break;  // GPIO 45 corresponds to ADC5
        case 46: adc_gpio_init(adc_gpio); adc_select_input(6); break;  // GPIO 46 corresponds to ADC6
        case 47: adc_gpio_init(adc_gpio); adc_select_input(7); break;  // GPIO 47 corresponds to ADC7
        default:
            printf("Invalid ADC GPIO. Supported GPIOs: 40, 41, 42, 43, 44, 45, 46, 47\n");
            return;
    }
}

// Function to read ADC value
uint16_t read_adc() {
    return adc_read(); // Reads raw 12-bit ADC value (0-4095)
}

// Build a bitmask of "pressed" buttons on the IO expander.
// Assumes your IOX buttons are active-low and readable via gpio_read(IOX_B_...).
// Add/remove buttons to match your design.
static inline uint32_t iox_pressed_mask_now(void) {
    uint32_t m = 0;

    // Example mapping (edit to your actual IOX button defines)
    // "pressed" = !gpio_read(...)
    if (!gpio_read(IOX_B_A))        m |= (1u << 0);
    if (!gpio_read(IOX_B_B))        m |= (1u << 1);
    if (!gpio_read(IOX_B_UP))       m |= (1u << 2);
    if (!gpio_read(IOX_B_DOWN))     m |= (1u << 3);
    if (!gpio_read(IOX_B_LEFT))     m |= (1u << 4);
    if (!gpio_read(IOX_B_RIGHT))    m |= (1u << 5);
    if (!gpio_read(IOX_B_START))    m |= (1u << 6);
    if (!gpio_read(IOX_DVI_DETECT)) m |= (1u << 7);

    return m;
}
