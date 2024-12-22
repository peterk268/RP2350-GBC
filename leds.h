#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0

// Duty Cycle Values
#warning "Make these flash variables"
uint8_t lcd_led_duty_cycle = MAX_BRIGHTNESS/2;    // Active High
uint8_t pwr_led_duty_cycle = MAX_BRIGHTNESS/8;    // Active High
uint8_t button_led_duty_cycle = MIN_BRIGHTNESS; // Active Low

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
    #warning "fix pmos"
    // config_led(GPIO_BUTTON_LED, button_led_duty_cycle, true); // Active Low
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
    adjust_brightness(GPIO_BUTTON_LED, &button_led_duty_cycle, step, true, true);
}
void decrease_button_brightness(uint8_t step) {
    adjust_brightness(GPIO_BUTTON_LED, &button_led_duty_cycle, step, false, true);
}
