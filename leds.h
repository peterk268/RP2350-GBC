#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0

#define HIGHEST_BRIGHTNESS_LEVEL 230
#define LOWEST_BRIGHTNESS_LEVEL 0

// Duty Cycle Values
uint8_t lcd_led_duty_cycle = MAX_BRIGHTNESS/8;   
uint8_t pwr_led_duty_cycle = MAX_BRIGHTNESS/8;  
uint8_t button_led_duty_cycle = MAX_BRIGHTNESS/8; 

// perceptual brightness curve 
static const uint8_t brightness_levels[16] = {
     LOWEST_BRIGHTNESS_LEVEL,  6,  8, 10, 14, 19, 25, 33,
    43, 55, 70, 90,115,145,185,HIGHEST_BRIGHTNESS_LEVEL
};

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
void deconfig_leds(bool keep_pwr) {
    deconfig_led(GPIO_BUTTON_LED);
    deconfig_led(GPIO_LCD_LED);
    if (!keep_pwr) deconfig_led(GPIO_PWR_LED);
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

uint8_t increase_clamp(uint8_t val, uint8_t step) {
    if (val > 255 - step) return 255;
    return val + step;
}

uint8_t decrease_clamp(uint8_t val, uint8_t step) {
    if (val < step) return 0;
    return val - step;
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

uint8_t get_next_brightness_level(uint8_t current, bool increase) {
    int idx = 0;

    // Find the nearest lower/equal level
    for (int i = 0; i < 16; i++) {
        if (current <= brightness_levels[i]) {
            idx = i;
            break;
        }
        idx = 15; // default to top if above all
    }

    if (increase) {
        if (idx < 15) idx++;
    } else {
        if (idx > 0) idx--;
    }

    return brightness_levels[idx];
}

// Generic helper for any LED
void step_brightness(uint8_t gpio, uint8_t *duty, bool increase, bool is_active_low) {
    uint8_t target = get_next_brightness_level(*duty, increase);
    *duty = target;
    uint slice = pwm_gpio_to_slice_num(gpio);
    uint channel = pwm_gpio_to_channel(gpio);
    uint8_t actual = is_active_low ? 255 - target : target;
    pwm_set_chan_level(slice, channel, actual);
}

void step_lcd_brightness(bool increase) {
    step_brightness(GPIO_LCD_LED, &lcd_led_duty_cycle, increase, false);
}

void step_pwr_brightness(bool increase) {
    step_brightness(GPIO_PWR_LED, &pwr_led_duty_cycle, increase, false);
}


static const uint8_t button_brightness_levels[6] = {0, 13, 23, 55, 130, 230};

uint8_t get_next_button_brightness(uint8_t current, bool increase) {
    int idx = 0;

    // Find the closest brightness level
    for (int i = 0; i < 6; i++) {
        if (current <= button_brightness_levels[i]) {
            idx = i;
            break;
        }
        idx = 5; // max level
    }

    // Step up or down
    if (increase) {
        if (idx < 5) idx++;
    } else {
        if (idx > 0) idx--;
    }

    return button_brightness_levels[idx];
}

void step_button_brightness(bool increase) {
    uint8_t target = get_next_button_brightness(button_led_duty_cycle, increase);
    button_led_duty_cycle = target;

    uint slice = pwm_gpio_to_slice_num(GPIO_BUTTON_LED);
    uint channel = pwm_gpio_to_channel(GPIO_BUTTON_LED);
    pwm_set_chan_level(slice, channel, target);
}


// Globals
bool increasing = true;
uint8_t step = 5; // Amount to change per tick

bool pwr_led_timer_callback(repeating_timer_t *rt) {
    if (increasing) {
        increase_pwr_brightness(step);
        if (pwr_led_duty_cycle >= MAX_BRIGHTNESS) {
            pwr_led_duty_cycle = MAX_BRIGHTNESS;
            increasing = false;
        }
    } else {
        decrease_pwr_brightness(step);
        if (pwr_led_duty_cycle <= MIN_BRIGHTNESS) {
            pwr_led_duty_cycle = MIN_BRIGHTNESS;
            increasing = true;
        }
    }

    // watchdog_update();
    return true; // keep running
}

uint8_t prev_pwr_led_duty_cycle;  

void setup_pwr_led_flash(uint32_t interval_ms) {
#if !TIE_PWR_LED_TO_LCD
    prev_pwr_led_duty_cycle = pwr_led_duty_cycle; // Save current brightness
#endif
    if (!add_repeating_timer_ms(interval_ms, pwr_led_timer_callback, NULL, &pwr_led_timer)) {
        printf("Failed to add repeating timer\n");
    }
}

void remove_pwr_led_flash() {
    cancel_repeating_timer(&pwr_led_timer);
    decrease_pwr_brightness(MAX_BRIGHTNESS); // Turn off LED
    pwr_led_duty_cycle = TIE_PWR_LED_TO_LCD ? lcd_led_duty_cycle : prev_pwr_led_duty_cycle; // Restore brightness
    increase_pwr_brightness(pwr_led_duty_cycle); // Apply restored brightness
}


// LED Fade in on start up here
// Target brightness for each LED at startup
uint8_t lcd_target_brightness    = (MAX_BRIGHTNESS/8);
uint8_t pwr_target_brightness    = (MAX_BRIGHTNESS/8);
uint8_t button_target_brightness = (MAX_BRIGHTNESS/8);

static repeating_timer_t led_ramp_timer;
volatile bool led_ramp_done = false; // flag you can check in main()

#define BRIGHTNESS_STEP  1
#define RAMP_INTERVAL_MS 25

typedef struct {
    uint gpio;
    uint8_t *duty;
    uint8_t *target;
    bool is_active_low;
} led_ramp_t;

static led_ramp_t led_ramps[] = {
    { GPIO_LCD_LED,    &lcd_led_duty_cycle,    &lcd_target_brightness,    false },
    { GPIO_PWR_LED,    &pwr_led_duty_cycle,    &pwr_target_brightness,    false },
    { GPIO_BUTTON_LED, &button_led_duty_cycle, &button_target_brightness, false },
};

bool led_ramp_timer_callback(repeating_timer_t *rt) {
    bool all_done = true;

    for (size_t i = 0; i < count_of(led_ramps); ++i) {
        led_ramp_t *l = &led_ramps[i];
        uint8_t duty   = *(l->duty);
        uint8_t target = *(l->target);

        if (duty < target) {
            // Progress of current fade (0–1)
            float progress = (float)duty / (float)target;

            // Desired total fade duration scales slightly with brightness
            // Lower brightness = shorter fade, but never instant
            float min_fade_ms = 300.0f;  // fade time for very dim targets
            float max_fade_ms = 900.0f;  // fade time for max brightness
            float fade_time_ms = min_fade_ms + (max_fade_ms - min_fade_ms) * (target / 255.0f);

            // Calculate how many timer ticks we’ll run in total
            float total_ticks = fade_time_ms / (float)RAMP_INTERVAL_MS;

            // Smooth easing function: starts slow, speeds up, then slows near the end
            float eased = progress * progress * (3 - 2 * progress); // smoothstep(0,1,progress)

            // Dynamic step size so that we finish near target within fade_time_ms
            float step_f = ((float)target / total_ticks) * (0.6f + 0.8f * eased);

            uint8_t step = (uint8_t)(step_f < 1 ? 1 : step_f);

            adjust_brightness(l->gpio, l->duty, step, true, l->is_active_low);

            if (*(l->duty) < target)
                all_done = false;
        }
    }

    if (all_done) {
        led_ramp_done = true;
        return false;
    }

    return true;
}

void fade_in_leds_startup(void) {
    // start from zero
    lcd_led_duty_cycle = MIN_BRIGHTNESS;
    pwr_led_duty_cycle = MIN_BRIGHTNESS;
    button_led_duty_cycle = MIN_BRIGHTNESS;

    config_leds(); // configure PWM channels at 0

    if (!add_repeating_timer_ms(RAMP_INTERVAL_MS, led_ramp_timer_callback, NULL, &led_ramp_timer)) {
        printf("Failed to add led ramp timer\n");
    }
}

#define BRIGHTNESS_STEP_DOWN  5
#define FADE_INTERVAL_MS 1
// Power-down fade uses the same ramp structure, just decreases
bool led_fadeout_timer_callback(repeating_timer_t *rt) {
    bool all_done = true;
    for (size_t i = 0; i < sizeof(led_ramps)/sizeof(led_ramps[0]); ++i) {
        led_ramp_t *l = &led_ramps[i];
        if (*(l->duty) > MIN_BRIGHTNESS) {
            adjust_brightness(l->gpio, l->duty, BRIGHTNESS_STEP_DOWN, false, l->is_active_low); // decrease
            if (*(l->duty) > MIN_BRIGHTNESS) all_done = false;
        }
    }

    if (all_done) {
        led_ramp_done = true;       // reuse the same flag
        deconfig_leds(false);       // optional: turn off LEDs
        return false;               // stop timer
    }
    return true; // keep running
}

void fade_out_leds_powerdown(void) {
    led_ramp_done = false;          // reuse flag

    if (!add_repeating_timer_ms(FADE_INTERVAL_MS, led_fadeout_timer_callback, NULL, &led_ramp_timer)) {
        printf("Failed to add led fadeout timer\n");
    }
}
