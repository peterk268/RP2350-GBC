// MARK: - Monitor

#define BAT_MONITOR_I2C_ADDR 0b1010101

void sleep_device();


// Address in SRAM to store a flag
#define REBOOT_FLAG_ADDRESS ((uint32_t *)0x20041FFC) // Last SRAM address

// Function to read a 16-bit register
uint16_t read_register(uint8_t reg) {
    uint8_t buffer[2] = {0};
    
    // Write the register address
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, &reg, 1, true);
    
    // Read 2 bytes from the register
    i2c_read_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, buffer, 2, false);
    
    // Combine high and low bytes into a 16-bit value
    return (buffer[1] << 8) | buffer[0];
}

uint8_t read_register_8(uint8_t reg) {
    uint8_t val;

    // Write the register address (no stop)
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, &reg, 1, true);

    // Read 1 byte from the register
    i2c_read_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, &val, 1, false);

    return val;
}


uint16_t read_voltage_mV() {
    return read_register(0x04);
}

uint16_t get_remaining_bat_capacity_mAh() {
    return read_register(0x0C);
}

uint16_t get_full_bat_capacity_mAh() {
    return read_register(0x0E);
}

int16_t get_average_current_mA() {
    return read_register(0x10);
}

uint16_t get_bat_charge_percent() {
    return read_register(0x1C);
}

// bool is_charging() {
//     get_average_current_mA() < 0;
// }

bool is_charging(int16_t current_mA) {
    return current_mA <= 0;
}
void shutdown_peripherals(bool keep_i2c);

// --- Configurable parameters ---
#define LOW_POWER_THRESHOLD      20  // Warning threshold
#define CRITICAL_SHUTDOWN_THRESH 5   // Shutdown threshold
#define RECOVERY_THRESHOLD       7   // Recovery threshold
                
void write_cart_ram_file(struct gb_s *gb);

void process_bat_percent() {
    uint16_t percent = get_bat_charge_percent();
    int16_t current_mA = get_average_current_mA();

    printf("Battery Percent: %d, rem_cap: %d, full_cap: %d, current: %d, voltage: %d\n",
           percent, get_remaining_bat_capacity_mAh(), get_full_bat_capacity_mAh(), current_mA, read_voltage_mV());

    // --- Low-power warning ---
    if (percent <= LOW_POWER_THRESHOLD) {
        if (!low_power) {
            low_power = true;
            remove_pwr_led_flash();
            setup_pwr_led_flash(40); // Slow pulse for warning
        }

        // --- Critical shutdown ---
        if (percent <= CRITICAL_SHUTDOWN_THRESH && !is_charging(current_mA)) {
            if (!low_power_shutdown) {
                low_power_shutdown = true;
                // main loop handles saving
                // faster pulse seems to mess up w timings
                // remove_pwr_led_flash();
                // setup_pwr_led_flash(15); // Fast pulse for critical.. has watchdog
            }
        }

    } else {
        // Battery above low-power warning
        if (low_power) {
            low_power = false;
            remove_pwr_led_flash();
        }
    }
    // Battery recovering after critical shutdown
    if (low_power_shutdown && (percent > RECOVERY_THRESHOLD || is_charging(current_mA))) {
        low_power_shutdown = false;
        watchdog_reboot(0, 0, 0); // Safe reset
        #warning "in the future find a way to allow the game to be resumed"
    }

}

// Timer callback function
bool battery_timer_callback(repeating_timer_t *rt) {
    battery_task_flag = true;
    return true; // Return true to keep the timer running
}

void subcommand_control(uint16_t subcommand) {
    // Control command register 0x00 & 0x01 but incremental write used so only 0x00
    // plus 0xXXXX subcommand.
    uint8_t data[3] = {0x00, (uint8_t)(subcommand >> 8), (uint8_t)(subcommand & 0x00FF)};
    // Write the register address
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, data, 3, false);
}

void enable_bat_monitor_shutdown() {
    subcommand_control(0x001B);
}
void disable_bat_monitor_shutdown() {
    // needs to set GPOUT pin from low to high for 200µs to exit shutdown
    // no way of disabling shutdown after enabling it...

    // Unless if we hard reset?
}

void reset_bat_monitor() {
    subcommand_control(0x0041);
}
void soft_reset_bat_monitor() {
    subcommand_control(0x0042);
}

void change_bat_chem_to_lipo() {
    // Chem B = 4.2V, ID = 1202
    // Control Data = 0x0031
    subcommand_control(0x0031);

    // Soft reset to update
    soft_reset_bat_monitor();
}

void set_design_capacity(uint16_t cap_mah) {
    // --- Step -2: Compute what we would write ---
    uint8_t new_cap_lsb = cap_mah & 0xFF;
    uint8_t new_cap_msb = (cap_mah >> 8) & 0xFF;
    uint16_t design_energy = (uint16_t)(cap_mah * 37 / 10);
    uint8_t energy_lsb = design_energy & 0xFF;
    uint8_t energy_msb = (design_energy >> 8) & 0xFF;

    // --- Step -1: Read current values ---
    uint8_t cur_cap_msb = read_register_8(0x46);
    uint8_t cur_cap_lsb = read_register_8(0x47);
    uint8_t cur_energy_msb = read_register_8(0x48);
    uint8_t cur_energy_lsb = read_register_8(0x49);

    // Compute what checksum would be based on current values
    uint8_t cur_csum = read_register_8(0x60);
    uint16_t temp_sum = cur_cap_msb + cur_cap_lsb + cur_energy_msb + cur_energy_lsb;
    uint8_t expected_csum = 255 - ((255 - cur_csum - temp_sum + (new_cap_msb + new_cap_lsb + energy_msb + energy_lsb)) & 0xFF);

    // --- Step 0: Check if everything matches ---
    if (cur_cap_msb == new_cap_msb && cur_cap_lsb == new_cap_lsb &&
        cur_energy_msb == energy_msb && cur_energy_lsb == energy_lsb &&
        cur_csum == expected_csum) {
        printf("Design capacity and energy already set. Skipping write.\n");
        return;
    }


    uint8_t data[2];

    // --- Step 1: Enter CFGUPDATE mode ---
    subcommand_control(0x0013);  // SET_CFGUPDATE
    printf("Entered CFGUPDATE mode.\n");
    sleep_ms(1100);

    // --- Step 2: Enable Block Data Memory control ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, (uint8_t[]){0x61, 0x00}, 2, false);
    printf("Block data memory control enabled.\n");

    // --- Step 3: Select State subclass (0x52) ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, (uint8_t[]){0x3E, 0x52}, 2, false);
    printf("State subclass 0x52 selected.\n");

    // --- Step 4: Set block offset to 0 ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, (uint8_t[]){0x3F, 0x00}, 2, false);
    printf("Block offset set to 0.\n");

    // --- Step 5: Read old checksum and old capacity/energy bytes ---
    uint8_t old_csum   = read_register_8(0x60);
    uint8_t old_cap_msb = read_register_8(0x46);
    uint8_t old_cap_lsb = read_register_8(0x47);
    uint8_t old_energy_msb = read_register_8(0x48);
    uint8_t old_energy_lsb = read_register_8(0x49);

    printf("Old checksum: 0x%02X\n", old_csum);
    printf("Old Capacity: MSB=0x%02X LSB=0x%02X\n", old_cap_msb, old_cap_lsb);
    printf("Old Energy: MSB=0x%02X LSB=0x%02X\n", old_energy_msb, old_energy_lsb);

    // --- Step 6: Write new Design Capacity ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, (uint8_t[]){0x46, new_cap_msb}, 2, false);
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, (uint8_t[]){0x47, new_cap_lsb}, 2, false);
    printf("Wrote new Capacity: MSB=0x%02X LSB=0x%02X\n", new_cap_msb, new_cap_lsb);

    // --- Step 7: Write new Design Energy ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, (uint8_t[]){0x48, energy_msb}, 2, false);
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, (uint8_t[]){0x49, energy_lsb}, 2, false);
    printf("Wrote new Energy: MSB=0x%02X LSB=0x%02X\n", energy_msb, energy_lsb);

    // --- Step 8: Compute new checksum (update with capacity and energy) ---
    uint16_t temp_old_sum = old_cap_lsb + old_cap_msb + old_energy_lsb + old_energy_msb;
    uint16_t temp_new_sum = new_cap_lsb + new_cap_msb + energy_lsb + energy_msb;
    uint8_t new_csum = 255 - ((255 - old_csum - temp_old_sum + temp_new_sum) & 0xFF);

    // --- Step 9: Write new checksum ---
    uint8_t buf[2] = {0x60, new_csum};
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, buf, 2, false);
    printf("Wrote new checksum: 0x%02X\n", new_csum);

    // --- Step 10: Wait for commit ---
    sleep_ms(200);

    // --- Step 11: Soft reset to apply changes ---
    soft_reset_bat_monitor();
    printf("Soft reset sent.\n");
    // Give the gauge time to reboot
    sleep_us(100);

    // --- Step 11: Confirm values ---
    uint8_t check_cap_msb = read_register_8(0x46);
    uint8_t check_cap_lsb = read_register_8(0x47);
    uint8_t check_energy_msb = read_register_8(0x48);
    uint8_t check_energy_lsb = read_register_8(0x49);
    uint8_t check_csum = read_register_8(0x60);

    printf("Confirm Capacity: MSB=0x%02X LSB=0x%02X\n", check_cap_msb, check_cap_lsb);
    printf("Confirm Energy: MSB=0x%02X LSB=0x%02X\n", check_energy_msb, check_energy_lsb);
    printf("Confirm Checksum: 0x%02X\n", check_csum);
}

void config_battery_monitor() {
    // reset_bat_monitor();
    set_design_capacity(1500);
    change_bat_chem_to_lipo();

    // enable bat monitor shutdown when we shutdown peripherals.
}

// MARK: - Sleep
static powman_power_state off_state;
static powman_power_state on_state;

// Initialise everything
void powman_example_init(uint64_t abs_time_ms) {
    // start powman and set the time if not already.
    initialize_rtc(abs_time_ms);

    // Allow power down when debugger connected
    powman_set_debug_power_request_ignored(true);

    // Power states
    powman_power_state P1_7 = POWMAN_POWER_STATE_NONE;

    powman_power_state P0_3 = POWMAN_POWER_STATE_NONE;
    P0_3 = powman_power_state_with_domain_on(P0_3, POWMAN_POWER_DOMAIN_SWITCHED_CORE);
    P0_3 = powman_power_state_with_domain_on(P0_3, POWMAN_POWER_DOMAIN_XIP_CACHE);

    off_state = P1_7;
    on_state = P0_3;
}

// Initiate power off
static int powman_example_off(void) {
    // Get ready to power off
    stdio_flush();

    // Set power states
    bool valid_state = powman_configure_wakeup_state(off_state, on_state);
    if (!valid_state) {
        return PICO_ERROR_INVALID_STATE;
    }

    // reboot to main
    powman_hw->boot[0] = 0;
    powman_hw->boot[1] = 0;
    powman_hw->boot[2] = 0;
    powman_hw->boot[3] = 0;

    uart_default_tx_wait_blocking();
    sleep_run_from_lposc();
    uart_default_tx_wait_blocking();

    // sleep_us(10);

    // Switch to required power state
    int rc = powman_set_power_state(off_state);
    if (rc != PICO_OK) {
        return rc;
    }

    // Power down
    while (true) __wfi();
}

void shutdown_peripherals(bool keep_i2c) {
    if (!keep_i2c) decrease_pwr_brightness(MAX_BRIGHTNESS);
    sleep_ms(1);
    decrease_lcd_brightness(MAX_BRIGHTNESS);
    sleep_ms(1);
    decrease_button_brightness(MAX_BRIGHTNESS);
    sleep_ms(1);
    deconfig_leds(keep_i2c);

    sleep_ms(1);                            // Allow PWM/LED/LCD registers to settle

    config_iox_ports();

    if (!keep_i2c) {
        remove_pwr_led_flash();
        cancel_repeating_timer(&battery_timer);
        enable_bat_monitor_shutdown();
        i2c_deinit(IOX_I2C_PORT);
    }

    sd_card_t *pSD=sd_get_by_num(0);
    f_unmount(pSD->pcName);

    spi_deinit(LCD_SPI);
    if (LCD_SPI != SD_SPI)
        spi_deinit(SD_SPI);

    pio_sm_set_enabled(I2S_PIO, 0, false);
    pio_sm_set_enabled(pio0, 0, false);
    for (uint dma_channel = 0; dma_channel < NUM_DMA_CHANNELS; dma_channel++) dma_channel_abort(dma_channel); // Reset DMA

    for (uint8_t i = 0; i<48; i++) {
        if (i != GPIO_SW_OUT && 
            (!keep_i2c || (i != GPIO_I2C1_SCL && i != GPIO_I2C1_SDA && i != GPIO_PWR_LED)))
            gpio_deinit(i);
    }
}

#warning "sleep device currently unstable"
void sleep_device() {
    // #if ENABLE_SDCARD				
    // write_cart_ram_file(gbc);
    // #endif	

    // sleep_ms(1);

    // Check if we're rebooting and executing specific code
    if (*REBOOT_FLAG_ADDRESS == 0xDEADBEEF) {
        *REBOOT_FLAG_ADDRESS = 0; // Clear the flag
        //and proceed to sleep
    } else {
        // Normal execution
        shutdown_peripherals(false);
        *REBOOT_FLAG_ADDRESS = 0xDEADBEEF; // Set the flag
        watchdog_reboot(0, SRAM_END, 0);  // Trigger the reboot
    }

    // sleep_ms(500);
    // // Check switch state
    // if (gpio_read(GPIO_SW_OUT)) {
    //     watchdog_reboot(0, 0, 0); // Reboot if switch toggled
    // }

    // sleep_us(10);
    powman_example_init(RTC_DEFAULT_VALUE);

    powman_enable_gpio_wakeup(0, GPIO_SW_OUT, false, true);
    // sleep_us(10);
    powman_example_off();
}

// Function to execute on interrupt
void gpio_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_LEVEL_LOW) {
        // Handle the falling edge interrupt
        // printf("GPIO %d went LOW\n", gpio);
        // Sleep device
        sleep_device();
    }
}

void setup_switch_sleep() {
    gpio_init(GPIO_SW_OUT);
    gpio_set_dir(GPIO_SW_OUT, GPIO_IN);
    sleep_us(10);
    // Set up the interrupt
    gpio_set_irq_enabled_with_callback(GPIO_SW_OUT, GPIO_IRQ_LEVEL_LOW, true, &gpio_callback);
}