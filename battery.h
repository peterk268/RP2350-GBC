// MARK: - Monitor

#define BAT_MONITOR_I2C_ADDR 0b1010101

// Function to read a 16-bit register
uint16_t read_register(uint8_t reg) {
    uint8_t buffer[2] = {0};
    
    // Write the register address
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, &reg, 1, true);
    
    // Read 2 bytes from the register
    i2c_read_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, buffer, 2, false);
    
    // Combine high and low bytes into a 16-bit value
    return (buffer[0] << 8) | buffer[1];
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

uint16_t get_average_current_mA() {
    return read_register(0x10);
}

uint16_t get_bat_charge_percent() {
    return read_register(0x1C);
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

void change_bat_chem_to_lipo() {
    // Chem B = 4.2V, ID = 1202
    // Control Data = 0x0031
    subcommand_control(0x0031);

    // Soft reset to update
    subcommand_control(0x0042);
}

void config_battery_monitor() {
    change_bat_chem_to_lipo();
    enable_bat_monitor_shutdown();
}

// MARK: - Sleep
static powman_power_state off_state;
static powman_power_state on_state;

// Initialise everything
void powman_example_init(uint64_t abs_time_ms) {
    // start powman and set the time
    powman_timer_start();
    powman_timer_set_ms(abs_time_ms);

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

    // Switch to required power state
    int rc = powman_set_power_state(off_state);
    if (rc != PICO_OK) {
        return rc;
    }

    // Power down
    while (true) __wfi();
}

void sleep_device() {
    iox_state_set(IOX_n3V3_MCU_EN, false);
    
    #warning "Save Game"
    powman_example_init(1704067200000);

    powman_example_off();
    powman_enable_gpio_wakeup(0, GPIO_SW_OUT, false, true);

}

// Function to execute on interrupt
void gpio_callback(uint gpio, uint32_t events) {
    if (events & GPIO_IRQ_EDGE_FALL) {
        // Handle the falling edge interrupt
        printf("GPIO %d went LOW\n", gpio);
        // Sleep device
        sleep_device();
    }
}

void setup_switch_sleep() {
    gpio_init(GPIO_SW_OUT);
    gpio_set_dir(GPIO_SW_OUT, GPIO_IN);
    gpio_pull_up(GPIO_SW_OUT);

    // Set up the interrupt
    gpio_set_irq_enabled_with_callback(GPIO_SW_OUT, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
}