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
    return current_mA >= 0;
}
void shutdown_peripherals(bool keep_i2c);

// --- Configurable parameters ---
#define LOW_POWER_THRESHOLD      20  // Warning threshold
#define CRITICAL_SHUTDOWN_THRESH 5   // Shutdown threshold
#define RECOVERY_THRESHOLD       7   // Recovery threshold
#define BATTERY_VOLTAGE_SHUTDOWN_mV 3700 // Voltage threshold for shutdown 

void write_cart_ram_file(struct gb_s *gb, bool hold_sd_busy);

// Estimate SoC from voltage (1S LiPo)
static uint16_t estimate_soc_from_voltage(uint16_t mV) {
    if (mV >= 4200) return 100;
    if (mV >= 4100) return 95;
    if (mV >= 4000) return 90;
    if (mV >= 3900) return 80;
    if (mV >= 3800) return 70;
    if (mV >= 3700) return 60;
    if (mV >= 3600) return 50;
    if (mV >= 3500) return 40;
    if (mV >= 3400) return 30;
    if (mV >= 3300) return 20;
    if (mV >= 3200) return 10;
    return 0;
}

bool gauge_is_learning(uint16_t voltage_mV, uint16_t reported_percent) {
    const uint16_t learning_voltage_threshold = 4100; // e.g., 4.1V
    const uint16_t zero_learning_voltage_threshold = 3100; // e.g., 3.1V

    // If gauge reports 100% but voltage is below threshold, it's likely still learning
    return (reported_percent == 100 && voltage_mV < learning_voltage_threshold) ||
           (reported_percent == 0 && voltage_mV > zero_learning_voltage_threshold);
}

uint16_t get_bat_charge_percent_with_learning(uint16_t voltage_mV, uint16_t reported_percent) {
    // Now that we fixed the cc gain issue, the battery monitor has gotten very accurate with percentages and has no need for this
#if BAT_HAS_PERCENT_ISSUES
    if (gauge_is_learning(voltage_mV, reported_percent)) {
        uint16_t percent = estimate_soc_from_voltage(voltage_mV);
        printf("Gauge learning detected. Estimated SoC: %d%%\n", percent);
        return percent;
    }
    return reported_percent;
#else
    return reported_percent;
#endif
}

void process_bat_percent() {
    uint16_t percent = get_bat_charge_percent();
    int16_t current_mA = get_average_current_mA();
    uint16_t voltage_mV = read_voltage_mV();

    printf("Battery Percent: %d, rem_cap: %d, full_cap: %d, current: %d, voltage: %d\n",
           percent, get_remaining_bat_capacity_mAh(), get_full_bat_capacity_mAh(), current_mA, voltage_mV);

    percent = get_bat_charge_percent_with_learning(voltage_mV, percent);

    // --- Low-power warning ---
    if (percent <= LOW_POWER_THRESHOLD) {
        if (percent <= LOW_POWER_THRESHOLD / 2) {
            // 10% and below fast flash
            step = 20;
        } else if (percent <= (LOW_POWER_THRESHOLD * 3 / 4)) {
            // 15-10% medium flash
            step = 10;
        } else {
            // 20-15% slow flash
            step = 5;
        }
        if (!low_power) {
            low_power = true;
            remove_pwr_led_flash();
            setup_pwr_led_flash(40); // Slow pulse for warning
        }

        // --- Critical shutdown ---
        if (percent <= CRITICAL_SHUTDOWN_THRESH && !is_charging(current_mA) && voltage_mV <= BATTERY_VOLTAGE_SHUTDOWN_mV) {
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
    // Write two bytes (LSB first) to Control() register 0x00
    uint8_t data[3] = {0x00, (uint8_t)(subcommand & 0xFF), (uint8_t)(subcommand >> 8)};
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

bool bq_is_sealed = false; 
void bq_unseal() {
    bq_is_sealed = false;
    if (!bq_is_sealed) {
        printf("Device already unsealed.\n");
        return;
    }
    subcommand_control(0x8000);
    subcommand_control(0x8000);
    printf("Device unsealed.\n");
    sleep_ms(5);

    subcommand_control(0xFFFF);
    subcommand_control(0xFFFF);
    sleep_ms(5);
    printf("Device in FULL ACCESS.\n");
}

void bq_seal() {
    bq_is_sealed = true;
    if (bq_is_sealed) {
        printf("Device already sealed.\n");
        return;
    }
    // === Seal the device ===
    subcommand_control(0x0020); // SEAL
    printf("Device sealed.\n");
    sleep_ms(10);
}

bool in_config = false; 
void enter_config() {
    if (in_config) {
        printf("Already in CONFIG UPDATE mode.\n");
        return;
    }
    watchdog_disable();

    // 0. UNSEAL + FULL ACCESS REQUIRED
    bq_unseal();

    // 1. SET_CFGUPDATE
    subcommand_control(0x0013);
    printf("SET_CFGUPDATE sent.\n");

    // 2. Poll Flags() until CFGUPMODE is active (bit4)
    sleep_ms(50);
    for (int i = 0; i < 50; i++) {
        uint16_t flags = read_register(0x06); // Flags()
        if (flags & (1 << 4)) {               // CFGUPMODE = bit 4
            printf("CFGUPMODE active.\n");
            in_config = true;
            break;
        }
        sleep_ms(20);
    }

    watchdog_enable(WATCHDOG_STARTUP_TIMEOUT_MS, true);

    return;
}

bool bq_needs_to_save = false;
void save_config() {
    if (!in_config) {
        printf("Not in CONFIG UPDATE mode. Cannot save config.\n");
        return;
    }

    sleep_ms(100);

    // SOFT RESET
    soft_reset_bat_monitor();
    printf("Soft reset sent.\n");

    // Poll Flags() until CFGUPMODE is inactive (bit4)
    sleep_ms(50);
    for (int i = 0; i < 50; i++) {
        uint16_t flags = read_register(0x06); // Flags()
        if (!(flags & (1 << 4))) {               // CFGUPMODE = bit 4
            printf("CFGUPMODE inactive.\n");
            in_config = false;
            break;
        }
        sleep_ms(20);
    }

    bq_seal();

    bq_needs_to_save = false;
}

void fix_cc_gain_sign_positive(void) {
    printf("\n--- Fixing CC_Gain Sign Bit (TI Direct Method) ---\n");

    // 3. Select Calibration subclass
    i2c_write_blocking(
        BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
        (uint8_t[]){0x3E, 0x69, 0x00}, 3, false
    );
    printf("Calibration subclass 0x69 selected.\n");

    // 4. Read CC_Gain sign byte (direct)
    uint8_t sign = read_register_8(0x45);
    printf("Current sign: 0x%02X\n", sign);

    // 5. Read checksum (direct)
    uint8_t csum = read_register_8(0x60);
    printf("Current checksum: 0x%02X\n", csum);

    if ((sign & 0x80) == 0) {
        printf("Sign already positive.\n");
        return;
    }

    enter_config();

    uint8_t new_sign = sign & 0x7F;
    uint8_t new_csum = csum ^ 0x80;

    // 6. Write new sign (direct)
    i2c_write_blocking(
        BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
        (uint8_t[]){0x45, new_sign}, 2, false
    );
    printf("Wrote new sign byte: 0x%02X\n", new_sign);

    // 7. Write new checksum (direct)
    i2c_write_blocking(
        BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
        (uint8_t[]){0x60, new_csum}, 2, false
    );
    printf("Wrote new checksum: 0x%02X\n", new_csum);

    sleep_ms(10);

    bq_needs_to_save = true; 
}

void verify_cc_gain_sign() {
    // Verify
    uint8_t verify = read_register_8(0x45);
    printf("Verify sign byte: 0x%02X\n", verify);

    if ((verify & 0x80) == 0)
        printf("✅ CC_Gain sign fixed.\n");
    else
        printf("❌ CC_Gain sign still negative.\n");
}

const uint16_t new_chem_id = 0x1202; // ChemID 1202 (LiPo)
void change_bat_chem_to_lipo() {
    printf("\n--- Changing Chemistry to ChemID 0x%04X ---\n", new_chem_id);

    // === Step 1: Read current ChemID ===
    subcommand_control(0x0008);  // Read ChemID
    sleep_ms(10);
    uint16_t current_chem = read_register(0x00);
    printf("Current ChemID: 0x%04X\n", current_chem);

    if (current_chem == new_chem_id) {
        printf("Already set. Skipping chemistry update.\n");
        return;
    }

    enter_config();

    // === Step 4: Set CHEM ID to 0x1202 LiPo ===
    subcommand_control(0x0031); // SET_CHEM_ID

    sleep_ms(10);

    bq_needs_to_save = true; 
}

void verify_chem() {
    // === Step 6: Verify new ChemID ===
    subcommand_control(0x0008);
    sleep_ms(500);
    uint16_t verify = read_register(0x00);
    printf("New ChemID readback: 0x%04X\n", verify);

    if (verify == new_chem_id) {
        printf("Chemistry successfully changed to 0x%04X.\n", verify);
    } else {
        printf("ChemID mismatch. Expected 0x%04X but got 0x%04X.\n", new_chem_id, verify);
    }

}

void set_design_capacity(uint16_t cap_mah) {
    printf("\n--- Setting Design Capacity ---\n");

    // --- Step 1: Enable Block Data Control ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x61, 0x00}, 2, false);
    printf("Block data control enabled.\n");

    // --- Step 2: Select State subclass (0x52) ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x3E, 0x52}, 2, false);
    printf("Selected subclass 0x52.\n");

    // --- Step 3: Select block 0 ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x3F, 0x00}, 2, false);
    printf("Block 0 selected.\n");

    // --- Step 4: Read CURRENT values NOW (correct window) ---
    uint8_t cur_cap_msb    = read_register_8(0x46);
    uint8_t cur_cap_lsb    = read_register_8(0x47);
    uint8_t cur_energy_msb = read_register_8(0x48);
    uint8_t cur_energy_lsb = read_register_8(0x49);
    uint8_t cur_csum       = read_register_8(0x60);

    printf("Current Capacity: MSB=%02X LSB=%02X\n", cur_cap_msb, cur_cap_lsb);
    printf("Current Energy:   MSB=%02X LSB=%02X\n", cur_energy_msb, cur_energy_lsb);
    printf("Current checksum: 0x%02X\n", cur_csum);

    // --- Step 5: Compute new target values ---
    uint8_t new_cap_msb = (cap_mah >> 8) & 0xFF;
    uint8_t new_cap_lsb = cap_mah & 0xFF;

    uint16_t design_energy = (uint16_t)(cap_mah * 37 / 10);
    uint8_t new_eng_msb = (design_energy >> 8) & 0xFF;
    uint8_t new_eng_lsb = design_energy & 0xFF;

    // --- Step 6: Compute what checksum SHOULD BE ---
    uint16_t sum_old = cur_cap_msb + cur_cap_lsb + cur_energy_msb + cur_energy_lsb;
    uint16_t sum_new = new_cap_msb + new_cap_lsb + new_eng_msb + new_eng_lsb;

    uint8_t expected_csum =
        255 - ((255 - cur_csum - sum_old + sum_new) & 0xFF);

    // ✅ --- Step 7: Skip check — NOW it finally works ---
    if (cur_cap_msb == new_cap_msb &&
        cur_cap_lsb == new_cap_lsb &&
        cur_energy_msb == new_eng_msb &&
        cur_energy_lsb == new_eng_lsb &&
        cur_csum == expected_csum)
    {
        printf("Design capacity and energy already set. Skipping write.\n");
        return;
    }

    enter_config();

    printf("Updating capacity/energy values...\n");

    // --- Step 8: Write NEW capacity ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x46, new_cap_msb}, 2, false);
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x47, new_cap_lsb}, 2, false);

    printf("Wrote Capacity: MSB=%02X LSB=%02X\n", new_cap_msb, new_cap_lsb);

    // --- Step 9: Write NEW energy ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x48, new_eng_msb}, 2, false);
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x49, new_eng_lsb}, 2, false);

    printf("Wrote Energy:   MSB=%02X LSB=%02X\n", new_eng_msb, new_eng_lsb);

    // --- Step 10: Compute NEW checksum (TI incremental method) ---
    uint8_t new_csum =
        255 - ((255 - cur_csum - sum_old + sum_new) & 0xFF);

    // --- Step 11: Write NEW checksum ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x60, new_csum}, 2, false);

    printf("Wrote new checksum: 0x%02X\n", new_csum);

    printf("Design capacity update staged (waiting for finalize).\n");

    sleep_ms(10);

    bq_needs_to_save = true; 
}

void verify_design_capacity() {
    // --- Step 12: Confirm values ---
    uint8_t check_cap_msb = read_register_8(0x46);
    uint8_t check_cap_lsb = read_register_8(0x47);
    uint8_t check_energy_msb = read_register_8(0x48);
    uint8_t check_energy_lsb = read_register_8(0x49);
    uint8_t check_csum = read_register_8(0x60);

    printf("Confirm Capacity: MSB=0x%02X LSB=0x%02X\n", check_cap_msb, check_cap_lsb);
    printf("Confirm Energy: MSB=0x%02X LSB=0x%02X\n", check_energy_msb, check_energy_lsb);
    printf("Confirm Checksum: 0x%02X\n", check_csum);
}

// Sleep Current is in Data Memory: State subclass (0x52), offset 23, type I2 (signed 16-bit), units mA
// Offset 23 is inside block 0 (offsets 0..31), so bytes live at 0x40 + 23 = 0x57 (LSB) and 0x58 (MSB)

void set_sleep_current_mA(int16_t sleep_current_mA) {
    if (sleep_current_mA < 0) sleep_current_mA = -sleep_current_mA; // parameter is magnitude threshold
    if (sleep_current_mA > 1000) sleep_current_mA = 1000;

    printf("\n--- Setting BQ27427 Sleep Current to %dmA ---\n", sleep_current_mA);

    // --- Step 1: Enable Block Data Control ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x61, 0x00}, 2, false);

    // --- Step 2: Select State subclass (0x52) ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x3E, 0x52}, 2, false);

    // --- Step 3: Select block 0 (offsets 0..31) ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x3F, 0x00}, 2, false);

    // Sleep Current @ offset 23 => 0x57/0x58
    const uint8_t REG_LSB = 0x57;
    const uint8_t REG_MSB = 0x58;

    uint8_t old_lsb  = read_register_8(REG_LSB);
    uint8_t old_msb  = read_register_8(REG_MSB);
    uint8_t old_csum = read_register_8(0x60);

    int16_t old_val = (int16_t)((old_msb << 8) | old_lsb);
    printf("Current Sleep Current: %dmA (raw 0x%04X)\n", old_val, (uint16_t)old_val);
    printf("Current checksum: 0x%02X\n", old_csum);

    uint8_t new_lsb = (uint8_t)(sleep_current_mA & 0xFF);
    uint8_t new_msb = (uint8_t)((sleep_current_mA >> 8) & 0xFF);

    // If already set, skip
    if (old_lsb == new_lsb && old_msb == new_msb) {
        printf("Sleep Current already set. Skipping write.\n");
        return;
    }

    // You already manage unseal + cfgupdate inside enter_config()
    enter_config();

    // --- Write new value ---
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){REG_LSB, new_lsb}, 2, false);
    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){REG_MSB, new_msb}, 2, false);

    // --- Incremental checksum update (same method you used) ---
    uint8_t new_csum =
        255 - ((255 - old_csum - old_lsb - old_msb + new_lsb + new_msb) & 0xFF);

    i2c_write_blocking(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR,
                       (uint8_t[]){0x60, new_csum}, 2, false);

    printf("Wrote Sleep Current: %dmA (LSB=0x%02X MSB=0x%02X)\n", sleep_current_mA, new_lsb, new_msb);
    printf("Wrote new checksum: 0x%02X\n", new_csum);

    sleep_ms(10);
    bq_needs_to_save = true;
}

void config_battery_monitor() {
    printf("Configuring Battery Monitor...\n");

    // You must unseal in order to read values:
    bq_unseal();

    set_design_capacity(1500);
    watchdog_update();
    change_bat_chem_to_lipo();
    watchdog_update();
    fix_cc_gain_sign_positive();

    if (bq_needs_to_save) {
        watchdog_disable();
        save_config();
#if BAT_MONITOR_DEBUG
        verify_cc_gain_sign();
        verify_chem();
        verify_design_capacity();
#endif
        watchdog_enable(WATCHDOG_STARTUP_TIMEOUT_MS, true);
    }

    bq_seal();
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
    // in_game_save_auto_state();
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

void shutdown_screen(uint32_t duration_ms) {
    watchdog_update();

    lv_deinit();

    if (lvgl_fb) memset(lvgl_fb, 0, lvgl_fb_bytes);

    lv_init();

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, lv_buf1, NULL, DISP_HOR_RES * LV_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_VER_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;

    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    (void)disp;

    // Create a container to hold UI
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_center(cont);
    lv_obj_set_style_bg_color(cont, lv_color_hex(0), 0);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0), 0);

    lv_obj_t *title = lv_label_create(cont);
	lv_label_set_text(title, "Battery low\nPico Pal shutting down...");
	lv_obj_set_style_text_font(title, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
	lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

    lv_tick_inc(1);
    lv_timer_handler();

    __atomic_store_n(&show_gui, true, __ATOMIC_RELEASE);

    watchdog_update();
    sleep_ms(duration_ms);
    watchdog_update();
}


#define USE_IOX_TO_WAKEUP 1

static volatile bool g_wake_button = false;
#if USE_IOX_TO_WAKEUP
static volatile bool g_wake_iox    = false;
#endif
static volatile bool g_wake_swlow  = false;
static volatile bool g_wake_tick   = false;

// One callback for GPIO_B_SELECT, GPIO_SW_OUT, and IOX nINT
static void gpio_wake_cb(uint gpio, uint32_t events) {
    (void)events;

    if (gpio == GPIO_B_SELECT) {
        g_wake_button = true;   // active-low press -> falling edge
    } else if (gpio == GPIO_SW_OUT) {
        g_wake_swlow  = true;   // falling edge means switch went low
    } 
#if USE_IOX_TO_WAKEUP
    else if (gpio == GPIO_IOX_nINT) {
        g_wake_iox    = true;   // IO expander interrupt asserted (active-low)
    }
#endif
}

// TIMER alarm 0 IRQ: every 50s
static void __isr alarm0_irq(void) {
    // clear alarm0 interrupt
    hw_clear_bits(&timer_hw->intr, 1u << 0);

    g_wake_tick = true;

    // re-arm for +50s
    uint32_t now = timer_hw->timerawl;
    timer_hw->alarm[0] = now + 50u * 1000u * 1000u; // 50 seconds in us
}

static void tick_timer_start_50s(void) {
    uint32_t now = timer_hw->timerawl;
    timer_hw->alarm[0] = now + 50u * 1000u * 1000u;

    hw_set_bits(&timer_hw->inte, 1u << 0);
    irq_set_exclusive_handler(TIMER0_IRQ_0, alarm0_irq);
    irq_set_enabled(TIMER0_IRQ_0, true);
}

static void tick_timer_stop(void) {
    irq_set_enabled(TIMER0_IRQ_0, false);
    hw_clear_bits(&timer_hw->inte, 1u << 0);
    hw_clear_bits(&timer_hw->intr, 1u << 0);
}


void light_sleep_loop(void) {
    g_wake_button = false;
#if USE_IOX_TO_WAKEUP
    g_wake_iox    = false;
#endif
    g_wake_swlow  = false;
    g_wake_tick   = true;   // run maintenance once immediately (optional)

    // remove power hold since we already saved everything prior to going to bed
    release_power();

    // Disable watchdog because max timer is 16.7s
    watchdog_disable();

    // Enable button wake (active-low)
    gpio_set_irq_enabled_with_callback(GPIO_B_SELECT,
                                       GPIO_IRQ_EDGE_FALL,
                                       true,
                                       &gpio_wake_cb);

    // Enable switch wake ONLY on going low
    gpio_set_irq_enabled(GPIO_SW_OUT, GPIO_IRQ_EDGE_FALL, true);

#if USE_IOX_TO_WAKEUP
    // Enable IO expander interrupt wake (active-low nINT)
    gpio_set_irq_enabled(GPIO_IOX_nINT, GPIO_IRQ_EDGE_FALL, true);

    // ------------------------------------------------------------
    // IMPORTANT: snapshot currently-held IOX buttons BEFORE sleeping
    // so that RELEASE events don't wake us up.
    //
    // Also ACK/refresh once here so nINT isn't already low when we
    // go into __wfi().
    // ------------------------------------------------------------
    read_io_expander_states(0);
    uint32_t held_at_sleep = iox_pressed_mask_now();
#endif

    tick_timer_start_50s();

    // Sleep as long as switch is high and we’re not shutting down and button not pressed
    while (gpio_read(GPIO_SW_OUT) && !low_power_shutdown && !g_wake_button) {

        // If switch went low, break (while condition will also fail on next iteration)
        if (g_wake_swlow) {
            break;
        }

#if USE_IOX_TO_WAKEUP
        // If IOX interrupt fired, ACK it by reading the expander to release nINT
        if (g_wake_iox) {
            // This should clear the expander's interrupt latch / deassert nINT
            read_io_expander_states(0);

            // ------------------------------------------------------------
            // Only wake if a *new press* happened.
            // - If user was holding A at sleep entry, then letting go causes
            //   an interrupt edge -> we do NOT want to wake.
            //
            // "pressed mask" is ACTIVE-LOW (pressed = 1 in the bitmask).
            // new_presses = pressed_now AND NOT held_before
            // ------------------------------------------------------------
            uint32_t pressed_now = iox_pressed_mask_now();
            uint32_t new_presses = pressed_now & ~held_at_sleep;

            // Update baseline so future interrupts behave correctly
            held_at_sleep = pressed_now;

            // Clear flag AFTER ACK+read to avoid missing back-to-back events
            g_wake_iox = false;

            // If a new press occurred, wake the system
            if (new_presses) {
                break;
            }

            // Otherwise ignore (it was a release or already-held button activity)
        }
#endif

        // 50s maintenance tick to check on battery for shutdown (monitor updates every 48s in sleep)
        if (g_wake_tick) {
            g_wake_tick = false;

            watchdog_update();

            // If you're chasing low current, consider a quiet version here (no printf)
            process_bat_percent();
        }

        // Wait for interrupt: button, iox nINT, switch-low, or timer tick
        __wfi();
        tight_loop_contents();
    }

    // Restore your normal watchdog timeout
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    // Restore power hold now that we have watchdog back
    hold_power();

    tick_timer_stop();

    // Disable IRQs
    gpio_set_irq_enabled(GPIO_B_SELECT, GPIO_IRQ_EDGE_FALL, false);
#if USE_IOX_TO_WAKEUP
    gpio_set_irq_enabled(GPIO_IOX_nINT, GPIO_IRQ_EDGE_FALL, false);
#endif
    gpio_set_irq_enabled(GPIO_SW_OUT,   GPIO_IRQ_EDGE_FALL, false);
}

uint8_t saved_button_brightness = 0;

bool lcd_is_on() {
    return iox_state_lookup(IOX_LCD_nRST) != 0;
}

// adding a bool for shutdown core1 because when doing mp3 we get some issues shutting down core1 
// that I haven't fully investigated yet, so this allows us to skip that step for now when doing mp3
// shutdowns until I can figure out the root cause.
// SHOULD ALWAYS BE THE FIRST STEP BEFORE DOING A SD READ/WRITE AND THE LAST STEP AFTER DOING A SD READ/WRITE to avoid resource contention.
void shutdown_lcd(bool button_leds_off, bool shutdown_core1) {
    if (!lcd_is_on()) return;

    // Shut off display and LEDs
    gpio_write(IOX_LCD_nRST, 0);
    set_sd_busy(true);
    if (button_leds_off) {
        saved_button_brightness = button_led_duty_cycle;
        decrease_button_brightness(MAX_BRIGHTNESS);
    }

    wait_for_core1_parked(10 * 1000);

    scanvideo_timing_enable(false);

    if (shutdown_core1)
        multicore_reset_core1();
}

void start_lcd(bool button_leds_restore, bool start_core1) {
    if (lcd_is_on()) return;

    // start up core1 and lcd
    lcd_power_on_reset();
    init_spi_lcd();
    gpio_write(IOX_LCD_nCS, 0);
    lcd_config();
    gpio_write(IOX_LCD_nCS, 1);

    sleep_ms(1);
    // Give SPI pins back to SPI controller
    gpio_set_function(GPIO_SPI0_SCK, GPIO_FUNC_SPI);
    gpio_set_function(GPIO_SPI0_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(GPIO_SPI0_MISO, GPIO_FUNC_SPI);

    __atomic_store_n(&core1_parked, false, __ATOMIC_RELEASE);

    scanvideo_timing_enable(true);

    // LEDs
    set_sd_busy(false);
    if (button_leds_restore) increase_button_brightness(saved_button_brightness);

    if (start_core1)
        multicore_launch_core1(main_core1);
}

static inline void imu_sleep(void);
static inline void imu_wake(void);

void sleep_and_shutdown_peripherals() {
    // lcd nrst going low
    // audio en going low
    // sd nen going high
    // core1 shutdown
    // wfi on core0 for iox or select button

    shutdown_lcd(true, true);

    gpio_write(IOX_AUDIO_EN, 0);

    imu_sleep();

    hyper_underclock_cpu(true); // goes to 20MHz

    sleep_ms(100);

    light_sleep_loop();
}

void wakeup_and_start_peripherals() {
    // clear iox int
    read_io_expander_states(0);

    imu_wake();

    // DAC powered back on (needs time since LDO starts up too)
    gpio_write(IOX_AUDIO_EN, 1);

    // CPU Clock
    underclock_cpu(false); // takes us back to 300MHz and steps voltage back up properly.
    if (run_mode == MODE_POWERSAVE) underclock_cpu(true); // will take us to 180MHz

    start_lcd(true, true);

    // Audio again
    setup_dac();
    // Set current volume level to 0 to start reading the pot again because
    // read_volume checks for a change in pot value to set the volume.
    current_volume_level = 0;
    read_volume();
}