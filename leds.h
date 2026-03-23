#define MAX_BRIGHTNESS 255
#define MIN_BRIGHTNESS 0

#define HIGHEST_BRIGHTNESS_LEVEL 230
#define LOWEST_BRIGHTNESS_LEVEL 0

#define DEFAULT_LED_DC 26

// Duty Cycle Values
uint8_t lcd_led_duty_cycle = DEFAULT_LED_DC;   
uint8_t pwr_led_duty_cycle = DEFAULT_LED_DC;  
uint8_t button_led_duty_cycle = DEFAULT_LED_DC; 

// perceptual brightness curve 
static const uint8_t brightness_levels[16] = {
     LOWEST_BRIGHTNESS_LEVEL,  6,  8, 10, 13, 19, DEFAULT_LED_DC, 36,
    44, 57, 74, 91,116,146,190,HIGHEST_BRIGHTNESS_LEVEL
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

// Set the LCD backlight PWM directly without modifying lcd_led_duty_cycle.
// Use this for BFI strobe so the saved brightness value stays correct.
static inline void lcd_set_pwm_direct(uint8_t level) {
    pwm_set_chan_level(pwm_gpio_to_slice_num(GPIO_LCD_LED),
                      pwm_gpio_to_channel(GPIO_LCD_LED), level);
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


static const uint8_t button_brightness_levels[6] = {LOWEST_BRIGHTNESS_LEVEL, 13, DEFAULT_LED_DC, 57, 132, HIGHEST_BRIGHTNESS_LEVEL};

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

// ================================================================
// Heartbeat pulse animation for PWR LED (two quick beats + pause)
// Uses the same style: increase_pwr_brightness(), decrease_pwr_brightness(),
// pwr_led_duty_cycle, MIN_BRIGHTNESS, MAX_BRIGHTNESS, step, repeating_timer_t.
// ================================================================

// This one is really nice for low power

// Heartbeat state machine
typedef enum {
    HB_RISE_1,
    HB_FALL_1,
    HB_GAP_1,     // short gap after first beat
    HB_RISE_2,
    HB_FALL_2,
    HB_PAUSE      // longer pause before repeating
} hb_state_t;

static hb_state_t hb_state = HB_RISE_1;

// Counters for timing gaps/pauses (measured in timer ticks)
static uint16_t hb_ticks_left = 0;

// Tuning knobs (in ticks). With interval_ms=10:
//  HB_GAP_1=6  -> 60ms
//  HB_PAUSE=40 -> 400ms
static uint16_t hb_gap1_ticks   = 6;
static uint16_t hb_pause_ticks  = 40;

// Beat amplitudes (duty targets)
static uint8_t hb_peak1 = 0;  // set on start
static uint8_t hb_peak2 = 0;  // set on start
static uint8_t hb_base  = 0;  // MIN_BRIGHTNESS typically

// Speed per phase (duty delta per tick)
static uint8_t hb_step_rise1 = 10;
static uint8_t hb_step_fall1 = 14;
static uint8_t hb_step_rise2 = 18; // second beat is snappier
static uint8_t hb_step_fall2 = 16;

// Optional: independent direction flag (kept similar to your style)
static bool hb_increasing = true;

bool pwr_led_heartbeat_timer_callback(repeating_timer_t *rt) {
    // Handle "wait" phases (gap/pause) without touching brightness
    if (hb_state == HB_GAP_1 || hb_state == HB_PAUSE) {
        if (hb_ticks_left > 0) hb_ticks_left--;
        if (hb_ticks_left == 0) {
            hb_state = (hb_state == HB_GAP_1) ? HB_RISE_2 : HB_RISE_1;
            hb_increasing = true;
        }
        return true;
    }

    switch (hb_state) {
        case HB_RISE_1:
            hb_increasing = true;
            increase_pwr_brightness(hb_step_rise1);
            if (pwr_led_duty_cycle >= hb_peak1) {
                pwr_led_duty_cycle = hb_peak1;
                hb_state = HB_FALL_1;
                hb_increasing = false;
            }
            break;

        case HB_FALL_1:
            hb_increasing = false;
            decrease_pwr_brightness(hb_step_fall1);
            if (pwr_led_duty_cycle <= hb_base) {
                pwr_led_duty_cycle = hb_base;
                hb_state = HB_GAP_1;
                hb_ticks_left = hb_gap1_ticks;
            }
            break;

        case HB_RISE_2:
            hb_increasing = true;
            increase_pwr_brightness(hb_step_rise2);
            if (pwr_led_duty_cycle >= hb_peak2) {
                pwr_led_duty_cycle = hb_peak2;
                hb_state = HB_FALL_2;
                hb_increasing = false;
            }
            break;

        case HB_FALL_2:
            hb_increasing = false;
            decrease_pwr_brightness(hb_step_fall2);
            if (pwr_led_duty_cycle <= hb_base) {
                pwr_led_duty_cycle = hb_base;
                hb_state = HB_PAUSE;
                hb_ticks_left = hb_pause_ticks;
            }
            break;

        default:
            hb_state = HB_RISE_1;
            hb_increasing = true;
            break;
    }

    return true;
}

// You can reuse your existing prev_pwr_led_duty_cycle and timer handle.
// Use either 10ms, 12ms (slow), or 5ms (fast)
void setup_pwr_led_heartbeat(uint32_t interval_ms) {
#if !TIE_PWR_LED_TO_LCD
    prev_pwr_led_duty_cycle = pwr_led_duty_cycle; // Save current brightness
#endif

    // Base is your minimum (or you can set it to current brightness if you want)
    hb_base = MIN_BRIGHTNESS;

    // Two peaks: first is stronger, second is a bit smaller (classic heartbeat feel)
    hb_peak1 = (uint8_t)(MAX_BRIGHTNESS);
    hb_peak2 = (uint8_t)((uint16_t)MAX_BRIGHTNESS * 70 / 100); // ~70% of max

    // Reset state machine
    hb_state = HB_RISE_1;
    hb_increasing = true;
    hb_ticks_left = 0;

    // Force starting point at base (optional)
    if (pwr_led_duty_cycle > hb_base) {
        decrease_pwr_brightness(MAX_BRIGHTNESS);
        pwr_led_duty_cycle = hb_base;
        increase_pwr_brightness(pwr_led_duty_cycle);
    }

    if (!add_repeating_timer_ms(interval_ms, pwr_led_heartbeat_timer_callback, NULL, &pwr_led_timer)) {
        printf("Failed to add repeating timer\n");
    }
}

// ================================================================
// Simple "double pulse": bright -> dim -> bright quickly -> dim (repeat)
// interval_ms controls the feel (10ms is a good default)
// ================================================================
// Faster pulse than the heartbeat.. could be used for critical low power

typedef enum {
    PULSE_RISE1,
    PULSE_FALL1,
    PULSE_RISE2,
    PULSE_FALL2,
    PULSE_PAUSE
} pulse_state_t;

static pulse_state_t pulse_state = PULSE_RISE1;
static uint16_t pulse_wait_ticks = 0;

// Tuning (in timer ticks)
static uint16_t pulse_pause_ticks = 20;   // pause after the double pulse
static uint8_t  pulse_peak1_pct   = 100;  // first peak strength (% of MAX)
static uint8_t  pulse_peak2_pct   = 70;   // second peak strength (% of MAX)

// Speeds (duty per tick)
static uint8_t pulse_step1_up   = 10;
static uint8_t pulse_step1_down = 14;
static uint8_t pulse_step2_up   = 18;     // quicker second pulse up
static uint8_t pulse_step2_down = 16;

static uint8_t pulse_base;   // usually MIN_BRIGHTNESS
static uint8_t pulse_peak1;  // computed
static uint8_t pulse_peak2;  // computed

bool pwr_led_simple_pulse_timer_callback(repeating_timer_t *rt) {
    if (pulse_state == PULSE_PAUSE) {
        if (pulse_wait_ticks) pulse_wait_ticks--;
        if (!pulse_wait_ticks) pulse_state = PULSE_RISE1;
        return true;
    }

    switch (pulse_state) {
        case PULSE_RISE1:
            increase_pwr_brightness(pulse_step1_up);
            if (pwr_led_duty_cycle >= pulse_peak1) {
                pwr_led_duty_cycle = pulse_peak1;
                pulse_state = PULSE_FALL1;
            }
            break;

        case PULSE_FALL1:
            decrease_pwr_brightness(pulse_step1_down);
            if (pwr_led_duty_cycle <= pulse_base) {
                pwr_led_duty_cycle = pulse_base;
                pulse_state = PULSE_RISE2;
            }
            break;

        case PULSE_RISE2:
            increase_pwr_brightness(pulse_step2_up);
            if (pwr_led_duty_cycle >= pulse_peak2) {
                pwr_led_duty_cycle = pulse_peak2;
                pulse_state = PULSE_FALL2;
            }
            break;

        case PULSE_FALL2:
            decrease_pwr_brightness(pulse_step2_down);
            if (pwr_led_duty_cycle <= pulse_base) {
                pwr_led_duty_cycle = pulse_base;
                pulse_state = PULSE_PAUSE;
                pulse_wait_ticks = pulse_pause_ticks;
            }
            break;

        default:
            pulse_state = PULSE_RISE1;
            break;
    }

    return true;
}

void setup_pwr_led_simple_pulse(uint32_t interval_ms) {
#if !TIE_PWR_LED_TO_LCD
    prev_pwr_led_duty_cycle = pwr_led_duty_cycle; // Save current brightness
#endif

    pulse_base  = MIN_BRIGHTNESS;
    pulse_peak1 = (uint8_t)((uint16_t)MAX_BRIGHTNESS * pulse_peak1_pct / 100);
    pulse_peak2 = (uint8_t)((uint16_t)MAX_BRIGHTNESS * pulse_peak2_pct / 100);

    // Start from base (optional but makes it consistent)
    decrease_pwr_brightness(MAX_BRIGHTNESS);
    pwr_led_duty_cycle = pulse_base;
    increase_pwr_brightness(pwr_led_duty_cycle);

    pulse_state = PULSE_RISE1;
    pulse_wait_ticks = 0;

    if (!add_repeating_timer_ms(interval_ms, pwr_led_simple_pulse_timer_callback, NULL, &pwr_led_timer)) {
        printf("Failed to add repeating timer\n");
    }
}

// Bright -> Dim -> (quick) Bright -> Dim, repeat.
// Never turns fully off (dim is clamped above MIN_BRIGHTNESS).
// This ones mid.. not as complex as the other 2
bool pwr_led_double_pulse_timer_callback(repeating_timer_t *rt) {
    // 0: rise1->bright, 1: fall1->dim, 2: rise2 quick->bright, 3: fall2->dim
    static uint8_t phase = 0;

    // Pick your two levels (edit these 2 numbers only)
    const uint8_t BRIGHT = MAX_BRIGHTNESS/2;              // peak level
    const uint8_t DIM    = 15;        // never off

    // Step sizes (second rise is quicker)
    const uint8_t STEP_SLOW = step;                     // use your global step
    const uint8_t STEP_FAST = (uint8_t)(step * 2);      // quick pulse

    switch (phase) {
        case 0: // up to BRIGHT
            increase_pwr_brightness(STEP_SLOW);
            if (pwr_led_duty_cycle >= BRIGHT) {
                pwr_led_duty_cycle = BRIGHT;
                phase = 1;
            }
            break;

        case 1: // down to DIM
            decrease_pwr_brightness(STEP_SLOW);
            if (pwr_led_duty_cycle <= DIM) {
                pwr_led_duty_cycle = DIM;
                phase = 2;
            }
            break;

        case 2: // quick up to BRIGHT
            increase_pwr_brightness(STEP_FAST);
            if (pwr_led_duty_cycle >= BRIGHT) {
                pwr_led_duty_cycle = BRIGHT;
                phase = 3;
            }
            break;

        default: // 3: down to DIM
            decrease_pwr_brightness(STEP_SLOW);
            if (pwr_led_duty_cycle <= DIM) {
                pwr_led_duty_cycle = DIM;
                phase = 0;
            }
            break;
    }

    return true;
}

void setup_pwr_led_double_pulse(uint32_t interval_ms) {
#if !TIE_PWR_LED_TO_LCD
    prev_pwr_led_duty_cycle = pwr_led_duty_cycle;
#endif
    if (!add_repeating_timer_ms(interval_ms, pwr_led_double_pulse_timer_callback, NULL, &pwr_led_timer)) {
        printf("Failed to add repeating timer\n");
    }
}

// LED Fade in on start up here
// Target brightness for each LED at startup
uint8_t lcd_target_brightness    = DEFAULT_LED_DC;
uint8_t pwr_target_brightness    = DEFAULT_LED_DC;
uint8_t button_target_brightness = DEFAULT_LED_DC;

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

            uint8_t remaining = (uint8_t)(target - duty);
            if (step > remaining) step = remaining;   // prevents overshoot

            adjust_brightness(l->gpio, l->duty, step, true, l->is_active_low);

            if (*(l->duty) < target) all_done = false;
            else *(l->duty) = target; // hard clamp
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
