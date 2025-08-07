#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0

// Duty Cycle Values
#warning "Make these flash variables"
uint8_t lcd_led_duty_cycle = MAX_BRIGHTNESS/2;    
uint8_t pwr_led_duty_cycle = MAX_BRIGHTNESS/8;  
uint8_t button_led_duty_cycle = MIN_BRIGHTNESS; 

// Generalized LED Configuration Function
void config_led(uint8_t gpio_num, uint8_t duty_cycle, bool is_active_low) {
    gpio_set_function(gpio_num, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_num);
    uint channel = pwm_gpio_to_channel(gpio_num);
    pwm_set_wrap(slice_num, 255);
    uint8_t actual_duty_cycle = is_active_low ? 255 - duty_cycle : duty_cycle;
    pwm_set_chan_level(slice_num, channel, actual_duty_cycle);
    pwm_set_enabled(slice_num, true);
}

void deconfig_led(uint8_t gpio_num) {
    uint slice_num = pwm_gpio_to_slice_num(gpio_num);
    pwm_set_enabled(slice_num, false);
}
void deconfig_leds() {
    deconfig_led(GPIO_BUTTON_LED);
    deconfig_led(GPIO_LCD_LED);
    deconfig_led(GPIO_PWR_LED);
}

// Function to set up a fast blinking LED with PWM
void setup_fast_blink(uint8_t gpio_num, uint16_t blink_period_ms, bool is_active_low) {
    // Configure the GPIO for PWM
    gpio_set_function(gpio_num, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(gpio_num);
    uint channel = pwm_gpio_to_channel(gpio_num);

    // Set PWM wrap for smooth transitions
    uint16_t wrap = 255; // Max value for 8-bit resolution
    pwm_set_wrap(slice_num, wrap);

    // Calculate duty cycles
    uint8_t on_duty_cycle = is_active_low ? 0 : wrap;
    uint8_t off_duty_cycle = is_active_low ? wrap : 0;

    // Set PWM configuration
    pwm_set_chan_level(slice_num, channel, on_duty_cycle);
    pwm_set_enabled(slice_num, true);

    // Configure the PWM frequency to achieve the desired blink period
    uint32_t clock_divider = (125000000 * blink_period_ms) / (wrap * 1000);
    pwm_set_clkdiv(slice_num, clock_divider);
}

// Function to Adjust Brightness
void adjust_brightness(uint8_t gpio_num, uint8_t *current_duty_cycle, uint8_t step, bool increase, bool is_active_low) {
    uint8_t new_duty_cycle = increase
                             ? (*current_duty_cycle + step > MAX_BRIGHTNESS ? MAX_BRIGHTNESS : *current_duty_cycle + step)
                             : (*current_duty_cycle < step ? MIN_BRIGHTNESS : *current_duty_cycle - step);

    uint slice_num = pwm_gpio_to_slice_num(gpio_num);
    uint channel = pwm_gpio_to_channel(gpio_num);
    uint8_t actual_duty_cycle = is_active_low ? 255 - new_duty_cycle : new_duty_cycle;
    pwm_set_chan_level(slice_num, channel, actual_duty_cycle);

    *current_duty_cycle = new_duty_cycle;
}

// LED Configuration Wrapper
void config_leds() {
    config_led(GPIO_LCD_LED, lcd_led_duty_cycle, false);    // Active High
    config_led(GPIO_PWR_LED, pwr_led_duty_cycle, false);    // Active High
    config_led(GPIO_BUTTON_LED, button_led_duty_cycle, false); // Active High
}

void increase_lcd_brightness(uint8_t step) {
    adjust_brightness(GPIO_LCD_LED, &lcd_led_duty_cycle, step, true, false);
}
void decrease_lcd_brightness(uint8_t step) {
    adjust_brightness(GPIO_LCD_LED, &lcd_led_duty_cycle, step, false, false); 
}

void increase_pwr_brightness(uint8_t step) {
    adjust_brightness(GPIO_PWR_LED, &pwr_led_duty_cycle, step, true, false);
}
void decrease_pwr_brightness(uint8_t step) {
    adjust_brightness(GPIO_PWR_LED, &pwr_led_duty_cycle, step, false, false);
}

void increase_button_brightness(uint8_t step) {
    adjust_brightness(GPIO_BUTTON_LED, &button_led_duty_cycle, step, true, false);
}
void decrease_button_brightness(uint8_t step) {
    adjust_brightness(GPIO_BUTTON_LED, &button_led_duty_cycle, step, false, false);
}

repeating_timer_t pwr_led_timer;
bool pwr_led_on = false;

bool pwr_led_timer_callback(repeating_timer_t *rt) {
    if (pwr_led_on) {
        increase_pwr_brightness(128);
    } else {
        decrease_pwr_brightness(255);
    }
    pwr_led_on = !pwr_led_on;
    return true; // Return true to keep the timer running
}

void setup_pwr_led_flash() {
    config_led(GPIO_PWR_LED, pwr_led_duty_cycle, false);    // Active High

    // deconfig_led(GPIO_PWR_LED);
    // // gpio_deinit(GPIO_PWR_LED);

    // sleep_ms(1);

    // // gpio_init(GPIO_PWR_LED);
    // gpio_set_function(GPIO_PWR_LED, GPIO_FUNC_SIO);
    // gpio_set_dir(GPIO_PWR_LED, true);
    // sleep_ms(1);
    sleep_ms(1);

    if (!add_repeating_timer_ms(250, pwr_led_timer_callback, NULL, &pwr_led_timer)) {
        printf("Failed to add repeating timer\n");
    }
}

void remove_pwr_led_flash() {
    cancel_repeating_timer(&pwr_led_timer);
}