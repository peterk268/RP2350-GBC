// ================================================================
// Sunrise Alarm Clock
// ================================================================
// Entry: run_alarm_clock() — triggered when UP is held at boot.
//
// UI:  Black bg, warm amber text.  L/R selects field (hour/min/ampm),
//      U/D changes value.  START saves alarm and enters sleep.
//
// Sleep: mirrors sleep_and_shutdown_peripherals() — full shutdown
//        (Core 1/DPI off) — but uses a 60s RTC-check loop instead
//        of light_sleep_loop().
//
// Wake: wakeup_and_start_peripherals() (reused verbatim), then
//       alarm_lvgl_init() to take over the display.
//
// Sunrise (15-min window before alarm, all phases compound):
//   0–7 min   LCD color deep-red→amber→warm-white→turquoise + backlight ramp
//   7–10 min  button LED ramp
//   10–15 min ambient audio from /alarm/dawn.mp3 on SD (Core 0, DMA-paced)
//   15 min+   sustain until power-off
// ================================================================

// ── Constants ───────────────────────────────────────────────────

#define ALARM_MAGIC          0xA1A4C10Cu
#define ALARM_AUDIO_DIR      "/alarm"
#define ALARM_AUDIO_FILE     "/alarm/dawn.mp3"

#define ALARM_DEBUG 0
#if ALARM_DEBUG
// Dawn starts this many minutes before the alarm time
#define ALARM_DAWN_WINDOW_MIN  4

// Phase durations in seconds (from start of dawn window)
#define ALARM_PHASE1_END_S   (2  * 60)  // 420s: LCD color + backlight
#define ALARM_PHASE2_END_S   (3 * 60)  // 600s: button LEDs added
#define ALARM_PHASE3_END_S   (4 * 60)  // 900s: audio added (= alarm time)
#else 
// Dawn starts this many minutes before the alarm time
#define ALARM_DAWN_WINDOW_MIN  15

// Phase durations in seconds (from start of dawn window)
#define ALARM_PHASE1_END_S   (7  * 60)  // 420s: LCD color + backlight
#define ALARM_PHASE2_END_S   (10 * 60)  // 600s: button LEDs added
#define ALARM_PHASE3_END_S   (15 * 60)  // 900s: audio added (= alarm time)
#endif
// Warm amber for UI text / sleep screen
#define ALARM_COLOR_AMBER    0xFF8C00u
// Color for selected field
#define ALARM_COLOR_SELECT   0x33CC66u
// Color for unselected fields
#define ALARM_COLOR_DIM      0x666666u
// Max brightness presets for LEDs during alarm
#define ALARM_BRIGHTNESS_LOW    26u
#define ALARM_BRIGHTNESS_MEDIUM 116u
#define ALARM_BRIGHTNESS_HIGH   230u
#define ALARM_MAX_BRIGHTNESS ALARM_BRIGHTNESS_MEDIUM  // default

// ── Data Structures ──────────────────────────────────────────────

typedef struct {
    uint32_t magic;
    uint8_t  hour;    // 0-23 in 24h; 0xFF = never set
    uint8_t  minute;  // 0-59
} alarm_settings_t;

typedef enum {
    ALARM_BRIGHTNESS_PRESET_LOW = 0,
    ALARM_BRIGHTNESS_PRESET_MEDIUM,
    ALARM_BRIGHTNESS_PRESET_HIGH,
    ALARM_BRIGHTNESS_PRESET_COUNT
} alarm_brightness_preset_t;

typedef enum {
    ALARM_FIELD_HOUR = 0,
    ALARM_FIELD_MINUTE,
    ALARM_FIELD_AMPM,
    ALARM_FIELD_BRIGHTNESS,
    ALARM_FIELD_COUNT
} alarm_field_t;

// ── Globals ───────────────────────────────────────────────────────

static volatile bool g_alarm_tick = false;
static alarm_brightness_preset_t g_alarm_brightness_preset = ALARM_BRIGHTNESS_PRESET_MEDIUM;

// ── Settings persistence ─────────────────────────────────────────

static bool alarm_load(alarm_settings_t *out) {
    out->magic  = 0;
    out->hour   = 0xFF;  // sentinel: not set
    out->minute = 0;

    FATFS *fs = (FATFS *)calloc(1, sizeof(FATFS));
    FIL   *f  = (FIL *)calloc(1, sizeof(FIL));
    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr;
    bool mounted = false;
    bool ok = false;

    if (!fs || !f || !pSD) goto done;
    fr = f_mount(fs, pSD->pcName, 1);
    if (fr != FR_OK) goto done;
    mounted = true;

    fr = f_open(f, ALARM_FILE_PATH, FA_READ);
    if (fr != FR_OK) goto done;

    UINT br;
    alarm_settings_t tmp;
    fr = f_read(f, &tmp, sizeof(tmp), &br);
    f_close(f);

    if (fr != FR_OK || br != sizeof(tmp) || tmp.magic != ALARM_MAGIC) goto done;
    *out = tmp;
    ok = true;

done:
    if (mounted) f_mount(NULL, pSD->pcName, 0);
    if (f) free(f);
    if (fs) free(fs);
    return ok;
}

static bool alarm_save(const alarm_settings_t *s) {
    hold_power();
    shutdown_lcd(false, false);

    FATFS *fs = (FATFS *)calloc(1, sizeof(FATFS));
    FIL   *f  = (FIL *)calloc(1, sizeof(FIL));
    FRESULT fr;
    bool ok = false;
    bool mounted = false;

    sd_card_t *pSD = sd_get_by_num(0);
    if (!fs || !f || !pSD) goto done;

    fr = f_mount(fs, pSD->pcName, 1);
    if (fr != FR_OK) goto done;
    mounted = true;

    // Create /settings dir if needed
    f_mkdir(SETTINGS_DIR);

    fr = f_open(f, ALARM_FILE_PATH, FA_WRITE | FA_CREATE_ALWAYS);
    if (fr != FR_OK) goto done;

    UINT bw;
    fr = f_write(f, s, sizeof(*s), &bw);
    f_close(f);
    ok = (fr == FR_OK && bw == sizeof(*s));

done:
    if (mounted) f_mount(NULL, pSD->pcName, 0);
    if (f) free(f);
    if (fs) free(fs);
    start_lcd(false, false);
    release_power();
    return ok;
}

// ── LVGL Initialisation ───────────────────────────────────────────
// Mirrors play_mp3_stream() lines 2839-2863 exactly.

static void alarm_lvgl_init(void) {
    lv_init();

    lvgl_free_buffers();
    if (free_fb) { free(free_fb); free_fb = NULL; }
    lvgl_fb = front_fb->data;
    lv_buf1 = (lv_color_t *)write_fb->data;

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, lv_buf1, NULL, DISP_HOR_RES * LV_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = DISP_HOR_RES;
    disp_drv.ver_res  = DISP_VER_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
}

// ── Solid Color Fill ──────────────────────────────────────────────
// Direct write to lvgl_fb; Core 1 scans it via show_gui=true.
// Uses RGB888 → RGB565 conversion.

static void alarm_fill_screen_rgb(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t c = (uint16_t)(((uint16_t)(r >> 3) << 11) |
                             ((uint16_t)(g >> 2) << 5)  |
                              (uint16_t)(b >> 3));
    for (int y = 0; y < LCD_HEIGHT; y++) {
        for (int x = 0; x < LCD_WIDTH; x++) {
            lvgl_fb[y][x] = c;
        }
    }
}

// ── Alarm UI ──────────────────────────────────────────────────────

// Convert 24h hour to display 12h (1-12)
static uint8_t to_12h(uint8_t h24) {
    uint8_t h = h24 % 12;
    return (h == 0) ? 12 : h;
}

static bool is_pm(uint8_t h24) { return h24 >= 12; }

// Convert 12h display + ampm → 24h
static uint8_t to_24h(uint8_t h12, bool pm) {
    if (!pm)  return (h12 == 12) ? 0  : h12;
    else      return (h12 == 12) ? 12 : (uint8_t)(h12 + 12);
}

// Convert brightness preset to actual value
static uint8_t brightness_preset_to_value(alarm_brightness_preset_t preset) {
    switch (preset) {
        case ALARM_BRIGHTNESS_PRESET_LOW:    return ALARM_BRIGHTNESS_LOW;
        case ALARM_BRIGHTNESS_PRESET_MEDIUM: return ALARM_BRIGHTNESS_MEDIUM;
        case ALARM_BRIGHTNESS_PRESET_HIGH:   return ALARM_BRIGHTNESS_HIGH;
        default:                             return ALARM_BRIGHTNESS_MEDIUM;
    }
}

// Get preset name string
static const char* brightness_preset_name(alarm_brightness_preset_t preset) {
    switch (preset) {
        case ALARM_BRIGHTNESS_PRESET_LOW:    return "Low";
        case ALARM_BRIGHTNESS_PRESET_MEDIUM: return "Medium";
        case ALARM_BRIGHTNESS_PRESET_HIGH:   return "High";
        default:                             return "?";
    }
}

typedef struct {
    lv_obj_t *hour_lbl;
    lv_obj_t *min_lbl;
    lv_obj_t *ampm_lbl;
    lv_obj_t *brightness_lbl;
    lv_obj_t *cur_time_lbl;
} alarm_ui_t;

static alarm_ui_t alarm_create_ui(const alarm_settings_t *s,
                                   alarm_field_t field,
                                   const rtc_time_t *now)
{
    alarm_ui_t ui = {0};

    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_clean(scr);

    // Title
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "SUNRISE ALARM");
    lv_obj_set_style_text_color(title, lv_color_hex(ALARM_COLOR_AMBER), 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    // Current RTC time (read-only)
    ui.cur_time_lbl = lv_label_create(scr);
    if (now) {
        lv_label_set_text_fmt(ui.cur_time_lbl, "Now: %02u:%02u %s",
            to_12h(now->hours), now->minutes,
            is_pm(now->hours) ? "PM" : "AM");
    } else {
        lv_label_set_text(ui.cur_time_lbl, "Now: --:-- --");
    }
    lv_obj_set_style_text_color(ui.cur_time_lbl, lv_color_hex(ALARM_COLOR_DIM), 0);
    lv_obj_align(ui.cur_time_lbl, LV_ALIGN_TOP_MID, 0, 30);

    // Separator
    lv_obj_t *sep = lv_label_create(scr);
    lv_label_set_text(sep, "Wake Up At:");
    lv_obj_set_style_text_color(sep, lv_color_hex(ALARM_COLOR_AMBER), 0);
    lv_obj_align(sep, LV_ALIGN_CENTER, 0, -28);

    // Hour field
    ui.hour_lbl = lv_label_create(scr);
    if (s->hour == 0xFF) lv_label_set_text(ui.hour_lbl, "--");
    else lv_label_set_text_fmt(ui.hour_lbl, "%02u", to_12h(s->hour));
    lv_obj_set_style_text_color(ui.hour_lbl,
        lv_color_hex(field == ALARM_FIELD_HOUR ? ALARM_COLOR_SELECT : ALARM_COLOR_AMBER), 0);
    lv_obj_align(ui.hour_lbl, LV_ALIGN_CENTER, -28, 0);

    // Colon
    lv_obj_t *colon = lv_label_create(scr);
    lv_label_set_text(colon, ":");
    lv_obj_set_style_text_color(colon, lv_color_hex(ALARM_COLOR_AMBER), 0);
    lv_obj_align(colon, LV_ALIGN_CENTER, -8, 0);

    // Minute field
    ui.min_lbl = lv_label_create(scr);
    if (s->hour == 0xFF) lv_label_set_text(ui.min_lbl, "--");
    else lv_label_set_text_fmt(ui.min_lbl, "%02u", s->minute);
    lv_obj_set_style_text_color(ui.min_lbl,
        lv_color_hex(field == ALARM_FIELD_MINUTE ? ALARM_COLOR_SELECT : ALARM_COLOR_AMBER), 0);
    lv_obj_align(ui.min_lbl, LV_ALIGN_CENTER, 8, 0);

    // AM/PM field
    ui.ampm_lbl = lv_label_create(scr);
    if (s->hour == 0xFF) lv_label_set_text(ui.ampm_lbl, "--");
    else lv_label_set_text(ui.ampm_lbl, is_pm(s->hour) ? "PM" : "AM");
    lv_obj_set_style_text_color(ui.ampm_lbl,
        lv_color_hex(field == ALARM_FIELD_AMPM ? ALARM_COLOR_SELECT : ALARM_COLOR_AMBER), 0);
    lv_obj_align(ui.ampm_lbl, LV_ALIGN_CENTER, 32, 0);

    lv_obj_t *note = lv_label_create(scr);
    lv_label_set_text(note, "Dawn starts 15 min before");
    lv_obj_set_style_text_color(note, lv_color_hex(ALARM_COLOR_DIM), 0);
    lv_obj_align(note, LV_ALIGN_CENTER, 0, 40);

    // Brightness field
    ui.brightness_lbl = lv_label_create(scr);
    lv_label_set_text_fmt(ui.brightness_lbl, "Brightness: %s",
        brightness_preset_name(g_alarm_brightness_preset));
    lv_obj_set_style_text_color(ui.brightness_lbl,
        lv_color_hex(field == ALARM_FIELD_BRIGHTNESS ? ALARM_COLOR_SELECT : ALARM_COLOR_AMBER), 0);
    lv_obj_align(ui.brightness_lbl, LV_ALIGN_CENTER, 0, 22);

    lv_obj_t *hint2 = lv_label_create(scr);
    lv_label_set_text(hint2, "START: set alarm & sleep");
    lv_obj_set_style_text_color(hint2, lv_color_hex(ALARM_COLOR_DIM), 0);
    lv_obj_align(hint2, LV_ALIGN_BOTTOM_MID, 0, -8);

    lv_tick_inc(1);
    lv_timer_handler();
    __atomic_store_n(&show_gui, true, __ATOMIC_RELEASE);

    return ui;
}

static void alarm_update_ui_fields(alarm_ui_t *ui,
                                    const alarm_settings_t *s,
                                    alarm_field_t field,
                                    const rtc_time_t *now)
{
    if (s->hour == 0xFF) {
        lv_label_set_text(ui->hour_lbl, "--");
        lv_label_set_text(ui->min_lbl,  "--");
        lv_label_set_text(ui->ampm_lbl, "--");
    } else {
        lv_label_set_text_fmt(ui->hour_lbl, "%02u", to_12h(s->hour));
        lv_label_set_text_fmt(ui->min_lbl,  "%02u", s->minute);
        lv_label_set_text(ui->ampm_lbl, is_pm(s->hour) ? "PM" : "AM");
    }

    lv_label_set_text_fmt(ui->brightness_lbl, "Brightness: %s",
        brightness_preset_name(g_alarm_brightness_preset));

    lv_obj_set_style_text_color(ui->hour_lbl,
        lv_color_hex(field == ALARM_FIELD_HOUR   ? ALARM_COLOR_SELECT : ALARM_COLOR_AMBER), 0);
    lv_obj_set_style_text_color(ui->min_lbl,
        lv_color_hex(field == ALARM_FIELD_MINUTE  ? ALARM_COLOR_SELECT : ALARM_COLOR_AMBER), 0);
    lv_obj_set_style_text_color(ui->ampm_lbl,
        lv_color_hex(field == ALARM_FIELD_AMPM    ? ALARM_COLOR_SELECT : ALARM_COLOR_AMBER), 0);
    lv_obj_set_style_text_color(ui->brightness_lbl,
        lv_color_hex(field == ALARM_FIELD_BRIGHTNESS ? ALARM_COLOR_SELECT : ALARM_COLOR_AMBER), 0);

    if (now && ui->cur_time_lbl) {
        lv_label_set_text_fmt(ui->cur_time_lbl, "Now: %02u:%02u %s",
            to_12h(now->hours), now->minutes,
            is_pm(now->hours) ? "PM" : "AM");
    }
}

// ── 60-second Alarm Tick Timer ────────────────────────────────────
// Uses timer_hw->alarm[0] + TIMER0_IRQ_0 (same slot as tick_timer_start_50s
// in battery.h — mutually exclusive by boot path).

static void __isr alarm_tick_irq(void) {
    hw_clear_bits(&timer_hw->intr, 1u << 0);
    g_alarm_tick = true;
    // Re-arm for +60s
    timer_hw->alarm[0] = timer_hw->timerawl + 60u * 1000u * 1000u;
}

static void alarm_tick_start_60s(void) {
    g_alarm_tick = false;
    timer_hw->alarm[0] = timer_hw->timerawl + 60u * 1000u * 1000u;
    hw_set_bits(&timer_hw->inte, 1u << 0);
    irq_set_exclusive_handler(TIMER0_IRQ_0, alarm_tick_irq);
    irq_set_enabled(TIMER0_IRQ_0, true);
}

static void alarm_tick_stop(void) {
    irq_set_enabled(TIMER0_IRQ_0, false);
    hw_clear_bits(&timer_hw->inte, 1u << 0);
    hw_clear_bits(&timer_hw->intr, 1u << 0);
}

// ── Time Math ─────────────────────────────────────────────────────
// Returns minutes until alarm fires. Wraps through midnight.
// Returns 0 if alarm is now or in the past (within current day).

static int alarm_minutes_until(const rtc_time_t *now, uint8_t alarm_h, uint8_t alarm_m) {
    int now_min   = (int)now->hours * 60 + (int)now->minutes;
    int alarm_min = (int)alarm_h    * 60 + (int)alarm_m;
    int delta = alarm_min - now_min;
    if (delta < 0) delta += 1440;  // wrap through midnight
    return delta;
}

// ── Alarm Sleep Loop ──────────────────────────────────────────────
// Wakes only on 60s tick; checks RTC each time.
// watchdog_disable() and release_power() must be called by the caller
// BEFORE entering this function.

static void alarm_sleep_loop(uint8_t alarm_h, uint8_t alarm_m) {
    alarm_tick_start_60s();

    while (true) {
        __wfi();
        tight_loop_contents();

        if (g_alarm_tick) {
            g_alarm_tick = false;
            rtc_time_t now;
            if (mcp7940n_get_time(RTC_I2C_PORT, &now)) {
                int mins = alarm_minutes_until(&now, alarm_h, alarm_m);
                printf("Alarm: %d min until wake\n", mins);
                if (mins <= ALARM_DAWN_WINDOW_MIN) break;
            }
        }
    }

    alarm_tick_stop();
}

// ── Color Interpolation ───────────────────────────────────────────
// 3-segment lerp through 4 waypoints over [0,1].
// Fixed-point: frac is 0-255.

typedef struct { uint8_t r, g, b; } rgb_t;

static rgb_t alarm_lerp_color(uint8_t a_r, uint8_t a_g, uint8_t a_b,
                               uint8_t b_r, uint8_t b_g, uint8_t b_b,
                               uint8_t frac)  // 0=all-a, 255=all-b
{
    rgb_t out;
    out.r = (uint8_t)(((uint16_t)a_r * (255u - frac) + (uint16_t)b_r * frac) >> 8);
    out.g = (uint8_t)(((uint16_t)a_g * (255u - frac) + (uint16_t)b_g * frac) >> 8);
    out.b = (uint8_t)(((uint16_t)a_b * (255u - frac) + (uint16_t)b_b * frac) >> 8);
    return out;
}

// Color waypoints:
//   deep-red   (0x3D,0x00,0x00)  at t=0.00
//   amber      (0xFF,0x8C,0x00)  at t=0.33
//   warm-white (0xFF,0xF0,0xE0)  at t=0.66
//   turquoise  (0x40,0xE0,0xD0)  at t=1.00
//
// t_frac: 0-255 maps to [0,1]
static rgb_t alarm_sunrise_color(uint8_t t_frac) {
    if (t_frac < 85) {
        // segment 0→1: deep-red → amber  (0..84 → 0..255)
        uint8_t f = (uint8_t)(((uint16_t)t_frac * 255u) / 84u);
        return alarm_lerp_color(0x3D, 0x00, 0x00,
                                0xFF, 0x8C, 0x00, f);
    } else if (t_frac < 170) {
        // segment 1→2: amber → warm-white  (85..169 → 0..255)
        uint8_t f = (uint8_t)(((uint16_t)(t_frac - 85u) * 255u) / 84u);
        return alarm_lerp_color(0xFF, 0x8C, 0x00,
                                0xFF, 0xF0, 0xE0, f);
    } else {
        // segment 2→3: warm-white → turquoise  (170..255 → 0..255)
        uint8_t f = (uint8_t)(((uint16_t)(t_frac - 170u) * 255u) / 85u);
        return alarm_lerp_color(0xFF, 0xF0, 0xE0,
                                0x40, 0xE0, 0xD0, f);
    }
}

// ── Audio: find first alarm audio file ───────────────────────────

static bool alarm_find_audio_file(char *out_path, size_t max_len) {
    FIL *f = (FIL *)calloc(1, sizeof(FIL));
    DIR *dir = (DIR *)calloc(1, sizeof(DIR));
    FILINFO *fno = (FILINFO *)calloc(1, sizeof(FILINFO));
    bool found = false;
    bool dir_open = false;

    if (!f || !dir || !fno) goto done;

    // Try exact name first
    if (f_open(f, ALARM_AUDIO_FILE, FA_READ) == FR_OK) {
        f_close(f);
        strncpy(out_path, ALARM_AUDIO_FILE, max_len - 1);
        out_path[max_len - 1] = '\0';
        found = true;
        goto done;
    }

    // Scan /alarm/ for first audio file
    if (f_opendir(dir, ALARM_AUDIO_DIR) != FR_OK) goto done;
    dir_open = true;

    while (f_readdir(dir, fno) == FR_OK && fno->fname[0]) {
        if (fno->fattrib & AM_DIR) continue;
        if (has_mp3_extension(fno->fname) ||
            has_wav_extension(fno->fname) ||
            has_flac_extension(fno->fname)) {
            snprintf(out_path, max_len, "%s/%s", ALARM_AUDIO_DIR, fno->fname);
            found = true;
            break;
        }
    }

done:
    if (dir_open) f_closedir(dir);
    if (fno) free(fno);
    if (dir) free(dir);
    if (f) free(f);
    return found;
}

// ── Sunrise Sequence ─────────────────────────────────────────────

static void alarm_sunrise_run(uint8_t alarm_h, uint8_t alarm_m) {
    // Determine how many seconds into the dawn window we already are
    // (in case sleep tick fired late).
    int elapsed_s = 0;
    {
        rtc_time_t now;
        if (mcp7940n_get_time(RTC_I2C_PORT, &now)) {
            int mins_left = alarm_minutes_until(&now, alarm_h, alarm_m);
            // elapsed = how far into the 15-min window we are
            int elapsed_min = ALARM_DAWN_WINDOW_MIN - mins_left;
            if (elapsed_min < 0)  elapsed_min = 0;
            if (elapsed_min > ALARM_DAWN_WINDOW_MIN) elapsed_min = ALARM_DAWN_WINDOW_MIN;
            elapsed_s = elapsed_min * 60;
        }
    }

    // Absolute start of the dawn window in seconds-of-day
    // We track elapsed using RTC polling every ~200ms during non-audio phases,
    // and naturally via DMA blocking during audio phase.
    absolute_time_t phase_start = get_absolute_time();

    // We need a local function to get current elapsed seconds:
    // elapsed_s_now = elapsed_s + (us_since(phase_start) / 1e6)
    // Use int arithmetic to avoid floating point.

    // ── Setup display ───────────────────────────────────────────
    // alarm_lvgl_init() has already been called by the caller.
    // Set backlight to 0 first (we ramp it up in phase 1).
    lcd_led_duty_cycle = 0;
    adjust_brightness(GPIO_LCD_LED, &lcd_led_duty_cycle, 0, true, false);
    button_led_duty_cycle = 0;
    adjust_brightness(GPIO_BUTTON_LED, &button_led_duty_cycle, 0, true, false);
    pwr_led_duty_cycle = 0;
    adjust_brightness(GPIO_PWR_LED, &pwr_led_duty_cycle, 0, true, false);

    bool audio_open = false;
    mp3_stream_t *stream = NULL;
    drmp3 *mp3_dec = NULL;
    bool mp3_dec_inited = false;
    drwav *wav_dec = NULL;
    drflac *flac_dec = NULL;
    bool is_wav = false;
    bool is_flac = false;
    int16_t *pcm_buf = NULL;
    enum { ALARM_AUDIO_PATH_MAX = 64 };
    char *audio_path = (char *)calloc(ALARM_AUDIO_PATH_MAX, sizeof(char));
    bool audio_available = false;

    // Mount SD once for potential audio use later
    FATFS *fs_audio = (FATFS *)calloc(1, sizeof(FATFS));
    sd_card_t *pSD = sd_get_by_num(0);

    // ── Main sunrise loop ────────────────────────────────────────
    uint8_t max_brightness = brightness_preset_to_value(g_alarm_brightness_preset);

    while (true) {
        watchdog_update();

        // Compute current elapsed seconds in dawn window
        int64_t us_elapsed = absolute_time_diff_us(phase_start, get_absolute_time());
        int cur_s = elapsed_s + (int)(us_elapsed / 1000000LL);
        if (cur_s < 0) cur_s = 0;

        // ── Phase 1: LCD color + backlight (0-420s) ────────────────
        {
            // t_frac: 0-255 over first 420s
            uint32_t t_frac;
            if (cur_s >= ALARM_PHASE1_END_S) {
                t_frac = 255;
            } else {
                t_frac = (uint32_t)(((uint64_t)cur_s * 255u) / ALARM_PHASE1_END_S);
            }
            rgb_t col = alarm_sunrise_color((uint8_t)t_frac);
            alarm_fill_screen_rgb(col.r, col.g, col.b);

            // Backlight: ramp to brightness over phase 1, then hold
            uint8_t target_bl = (cur_s >= ALARM_PHASE1_END_S)
                ? max_brightness
                : (uint8_t)(((uint32_t)cur_s * max_brightness) / ALARM_PHASE1_END_S);
            if (lcd_led_duty_cycle < target_bl) {
                adjust_brightness(GPIO_LCD_LED, &lcd_led_duty_cycle, 1, true, false);
            }
        }

        // ── Phase 2: Button LEDs (420-600s) ───────────────────────
        if (cur_s >= ALARM_PHASE1_END_S) {
            uint8_t target_bl;
            if (cur_s >= ALARM_PHASE2_END_S) {
                target_bl = max_brightness;
            } else {
                int phase2_s = cur_s - ALARM_PHASE1_END_S;
                int phase2_dur = ALARM_PHASE2_END_S - ALARM_PHASE1_END_S;
                target_bl = (uint8_t)(((uint32_t)phase2_s * max_brightness) / (uint32_t)phase2_dur);
            }
            if (button_led_duty_cycle < target_bl) {
                adjust_brightness(GPIO_BUTTON_LED, &button_led_duty_cycle, 1, true, false);
            }
            if (pwr_led_duty_cycle < target_bl) {
                adjust_brightness(GPIO_PWR_LED, &pwr_led_duty_cycle, 1, true, false);
            }
        }

        // ── Phase 3: Audio (600-900s) ─────────────────────────────
        if (cur_s >= ALARM_PHASE2_END_S) {
            if (!audio_open && !audio_available) {
                if (!fs_audio || !audio_path) {
                    audio_open = true;
                }
                // Try to open audio file once
                if (!audio_open && f_mount(fs_audio, pSD->pcName, 1) == FR_OK) {
                    if (alarm_find_audio_file(audio_path, ALARM_AUDIO_PATH_MAX)) {
                        is_wav = has_wav_extension(audio_path);
                        is_flac = has_flac_extension(audio_path);
                        stream = (mp3_stream_t *)malloc(sizeof(mp3_stream_t));
                        if (stream) {
                            memset(stream, 0, sizeof(mp3_stream_t));
                            FRESULT fr2 = f_open(&stream->file, audio_path, FA_READ);
                            if (fr2 == FR_OK) {
                                if (is_wav) {
                                    wav_dec = (drwav *)calloc(1, sizeof(drwav));
                                    if (wav_dec && drwav_init(wav_dec, wav_fatfs_read, wav_fatfs_seek, wav_fatfs_tell, &stream->file, NULL)) {
                                        pcm_buf = (int16_t *)malloc(PCM_FRAME_COUNT * 2 * sizeof(int16_t));
                                        if (pcm_buf) {
                                            audio_available = true;
                                            audio_open = true;
                                            set_volume(0, 0);
                                            i2s_set_sample_freq(&i2s_config, wav_dec->sampleRate, false);
                                            printf("Alarm audio open: %s\n", audio_path);
                                        }
                                    }
                                } else if (is_flac) {
                                    flac_dec = drflac_open(flac_fatfs_read, flac_fatfs_seek, flac_fatfs_tell, &stream->file, NULL);
                                    if (flac_dec) {
                                        pcm_buf = (int16_t *)malloc(PCM_FRAME_COUNT * 2 * sizeof(int16_t));
                                        if (pcm_buf) {
                                            audio_available = true;
                                            audio_open = true;
                                            set_volume(0, 0);
                                            i2s_set_sample_freq(&i2s_config, flac_dec->sampleRate, false);
                                            printf("Alarm audio open: %s\n", audio_path);
                                        }
                                    }
                                } else {
                                    stream->buf = (uint8_t *)malloc(MP3_STREAM_BUF_SIZE);
                                    if (stream->buf) {
                                        // Pre-fill ring buffer
                                        for (int i = 0; i < 4; i++) mp3_refill(stream);

                                        mp3_dec = (drmp3 *)calloc(1, sizeof(drmp3));
                                        if (mp3_dec && drmp3_init(mp3_dec, mp3_stream_read, NULL, NULL, NULL,
                                                       stream, &s_drmp3_alloc)) {
                                            mp3_dec_inited = true;
                                            pcm_buf = (int16_t *)malloc(PCM_FRAME_COUNT * 2 * sizeof(int16_t));
                                            if (pcm_buf) {
                                                audio_available = true;
                                                audio_open = true;
                                                set_volume(0, 0);
                                                i2s_set_sample_freq(&i2s_config, mp3_dec->sampleRate, false);
                                                printf("Alarm audio open: %s\n", audio_path);
                                            }
                                        }
                                    }
                                }
                            }
                            if (!audio_available) {
                                if (mp3_dec_inited) { drmp3_uninit(mp3_dec); mp3_dec_inited = false; }
                                if (mp3_dec) { free(mp3_dec); mp3_dec = NULL; }
                                if (wav_dec) { drwav_uninit(wav_dec); free(wav_dec); wav_dec = NULL; }
                                if (flac_dec) { drflac_close(flac_dec); flac_dec = NULL; }
                                if (stream->buf) free(stream->buf);
                                f_close(&stream->file);
                                free(stream);
                                stream = NULL;
                            }
                        }
                    }
                    if (!audio_available) {
                        f_mount(NULL, pSD->pcName, 0);
                        if (fs_audio) { free(fs_audio); fs_audio = NULL; }
                        if (audio_path) { free(audio_path); audio_path = NULL; }
                        audio_available = false;  // mark as not available to avoid retry
                        audio_open = true;         // suppress future attempts
                        printf("Alarm: no audio file in %s\n", ALARM_AUDIO_DIR);
                    }
                }
            }

            if (audio_available && stream && pcm_buf && (mp3_dec || wav_dec || flac_dec)) {
                // Ramp volume over phase 3
                int phase3_s = cur_s - ALARM_PHASE2_END_S;
                int phase3_dur = ALARM_PHASE3_END_S - ALARM_PHASE2_END_S;
                if (phase3_s > phase3_dur) phase3_s = phase3_dur;
                uint8_t vol = (uint8_t)(((uint32_t)phase3_s * DAC_MAX_VOL_SPK) / (uint32_t)phase3_dur);
                if (vol > current_volume_level + 1 || current_volume_level == 0) {
                    current_volume_level = vol;
                    set_volume(vol, vol);
                }

                // Decode and play one buffer (~139ms at 44.1kHz)
                uint64_t frames;
                if (is_wav) {
                    frames = (uint64_t)drwav_read_pcm_frames_s16(wav_dec, PCM_FRAME_COUNT, pcm_buf);
                } else if (is_flac) {
                    frames = (uint64_t)drflac_read_pcm_frames_s16(flac_dec, PCM_FRAME_COUNT, (drflac_int16 *)pcm_buf);
                } else {
                    frames = (uint64_t)drmp3_read_pcm_frames_s16(mp3_dec, PCM_FRAME_COUNT, pcm_buf);
                }
                if (frames == 0) {
                    if (is_wav) {
                        // EOF — seek back to start and loop
                        drwav_seek_to_pcm_frame(wav_dec, 0);
                        continue;
                    } else if (is_flac) {
                        // EOF — seek back to start and loop
                        drflac_seek_to_pcm_frame(flac_dec, 0);
                        continue;
                    }
                    // EOF — reset and loop MP3
                    f_lseek(&stream->file, 0);
                    stream->rd = stream->wr = stream->count = 0;
                    stream->eof = false;
                    if (mp3_dec_inited) {
                        drmp3_uninit(mp3_dec);
                        mp3_dec_inited = false;
                    }
                    memset(mp3_dec, 0, sizeof(*mp3_dec));
                    for (int i = 0; i < 4; i++) mp3_refill(stream);
                    if (!drmp3_init(mp3_dec, mp3_stream_read, NULL, NULL, NULL,
                                    stream, &s_drmp3_alloc)) {
                        f_close(&stream->file);
                        if (stream->buf) free(stream->buf);
                        free(stream);
                        stream = NULL;
                        free(pcm_buf);
                        pcm_buf = NULL;
                        if (mp3_dec) { free(mp3_dec); mp3_dec = NULL; }
                        f_mount(NULL, pSD->pcName, 0);
                        if (fs_audio) { free(fs_audio); fs_audio = NULL; }
                        if (audio_path) { free(audio_path); audio_path = NULL; }
                        audio_available = false;
                        audio_open = true;
                        continue;
                    }
                    mp3_dec_inited = true;
                    continue;  // skip i2s write this iteration
                }
                uint8_t ch = is_wav ? (uint8_t)wav_dec->channels : (is_flac ? (uint8_t)flac_dec->channels : (uint8_t)mp3_dec->channels);
                mp3_zero_pcm_tail(pcm_buf, frames, PCM_FRAME_COUNT, ch);
                i2s_dma_write(&i2s_config, (const uint16_t *)pcm_buf);
                // DMA write blocks ~139ms — that's our update rate during audio phase
                continue;  // skip the sleep_ms below when audio is running
            }
        }

        // Phase 4: sustain — ensure everything is at max
        if (cur_s >= ALARM_PHASE3_END_S) {
            if (lcd_led_duty_cycle < max_brightness)
                adjust_brightness(GPIO_LCD_LED, &lcd_led_duty_cycle, 1, true, false);
            if (button_led_duty_cycle < max_brightness)
                adjust_brightness(GPIO_BUTTON_LED, &button_led_duty_cycle, 1, true, false);
            if (pwr_led_duty_cycle < max_brightness)
                adjust_brightness(GPIO_PWR_LED, &pwr_led_duty_cycle, 1, true, false);
            if (audio_available && current_volume_level < DAC_MAX_VOL_SPK) {
                current_volume_level = DAC_MAX_VOL_SPK;
                set_volume(DAC_MAX_VOL_SPK, DAC_MAX_VOL_SPK);
            }
        }

        // Non-audio phases: sleep 200ms between updates
        sleep_ms(200);
    }

    // (Never reached — user powers off with switch)
}

// ── Entry Point ───────────────────────────────────────────────────

void run_alarm_clock(void) {
    alarm_settings_t alarm = {0};
    alarm_load(&alarm);  // hour=0xFF if never set

    // Init LVGL for UI
    alarm_lvgl_init();

    // Read current RTC time for display
    rtc_time_t rtc_now = {0};
    bool rtc_ok = mcp7940n_get_time(RTC_I2C_PORT, &rtc_now);

    alarm_field_t field = ALARM_FIELD_HOUR;

    // Working 12h values (initialise from loaded alarm or defaults)
    uint8_t disp_hour  = (alarm.hour != 0xFF) ? to_12h(alarm.hour) : 7;
    uint8_t disp_min   = (alarm.hour != 0xFF) ? alarm.minute        : 0;
    bool    disp_pm    = (alarm.hour != 0xFF) ? is_pm(alarm.hour)   : false;

    alarm_ui_t ui = alarm_create_ui(&alarm, field, rtc_ok ? &rtc_now : NULL);

    uint32_t last_rtc_refresh = 0;
    uint64_t last_lvgl_tick   = time_us_64();

    // Edge-detection state — initialise to released (active-low, so true = released)
    bool prev_left = true, prev_right = true;
    bool prev_up   = true, prev_down  = true, prev_start = true;
    // Current button state (active-low: false = pressed)
    bool btn_left = true, btn_right = true;
    bool btn_up   = true, btn_down  = true, btn_start = true;

    alarm_field_t prev_field = field;

    while (true) {
        watchdog_update();

        // Refresh RTC display every 2s
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - last_rtc_refresh >= 2000) {
            last_rtc_refresh = now_ms;
            rtc_ok = mcp7940n_get_time(RTC_I2C_PORT, &rtc_now);
            alarm_update_ui_fields(&ui, &alarm, field, rtc_ok ? &rtc_now : NULL);
        }

        // Read buttons only when IOX interrupt fires (same as sd.h)
        if (!gpio_read(GPIO_IOX_nINT)) {
            read_io_expander_states(0);
            btn_left  = gpio_read(IOX_B_LEFT);
            btn_right = gpio_read(IOX_B_RIGHT);
            btn_up    = gpio_read(IOX_B_UP);
            btn_down  = gpio_read(IOX_B_DOWN);
            btn_start = gpio_read(IOX_B_START);
        }

        bool changed = false;

        // Edge: only act on the falling edge (just pressed)
        if (!btn_left && prev_left) {
            field = (alarm_field_t)((field + ALARM_FIELD_COUNT - 1) % ALARM_FIELD_COUNT);
            changed = true;
        } else if (!btn_right && prev_right) {
            field = (alarm_field_t)((field + 1) % ALARM_FIELD_COUNT);
            changed = true;
        } else if ((!btn_up && prev_up) || (!btn_down && prev_down)) {
            int delta = (!btn_up && prev_up) ? 1 : -1;
            // Ensure we have a valid starting point
            if (alarm.hour == 0xFF) {
                disp_hour = 7; disp_min = 0; disp_pm = false;
                alarm.hour = to_24h(disp_hour, disp_pm);
                alarm.minute = disp_min;
            }
            switch (field) {
                case ALARM_FIELD_HOUR: {
                    int h = (int)disp_hour + delta;
                    if (h < 1)  h = 12;
                    if (h > 12) h = 1;
                    disp_hour = (uint8_t)h;
                    break;
                }
                case ALARM_FIELD_MINUTE: {
                    int m = (int)disp_min + delta;
                    if (m < 0)  m = 59;
                    if (m > 59) m = 0;
                    disp_min = (uint8_t)m;
                    break;
                }
                case ALARM_FIELD_AMPM:
                    disp_pm = !disp_pm;
                    break;
                case ALARM_FIELD_BRIGHTNESS: {
                    int p = (int)g_alarm_brightness_preset + delta;
                    if (p < 0) p = ALARM_BRIGHTNESS_PRESET_COUNT - 1;
                    if (p >= ALARM_BRIGHTNESS_PRESET_COUNT) p = 0;
                    g_alarm_brightness_preset = (alarm_brightness_preset_t)p;
                    break;
                }
                default: break;
            }
            alarm.magic  = ALARM_MAGIC;
            alarm.hour   = to_24h(disp_hour, disp_pm);
            alarm.minute = disp_min;
            changed = true;
        }

        if (changed) {
            alarm_update_ui_fields(&ui, &alarm, field, rtc_ok ? &rtc_now : NULL);
        }

        // Live brightness preview when brightness field is selected
        if (field == ALARM_FIELD_BRIGHTNESS && (prev_field != ALARM_FIELD_BRIGHTNESS || changed)) {
            uint8_t preview_brightness = brightness_preset_to_value(g_alarm_brightness_preset);
            lcd_led_duty_cycle = preview_brightness;
            button_led_duty_cycle = preview_brightness;
            adjust_brightness(GPIO_LCD_LED, &lcd_led_duty_cycle, 0, true, false);
            adjust_brightness(GPIO_BUTTON_LED, &button_led_duty_cycle, 0, true, false);
        }
        if (field != ALARM_FIELD_BRIGHTNESS && prev_field == ALARM_FIELD_BRIGHTNESS) {
            // Restore previous target brightness only when leaving the brightness field
            lcd_led_duty_cycle = lcd_target_brightness;
            button_led_duty_cycle = button_target_brightness;
            adjust_brightness(GPIO_LCD_LED, &lcd_led_duty_cycle, 0, true, false);
            adjust_brightness(GPIO_BUTTON_LED, &button_led_duty_cycle, 0, true, false);
        }

        if (!btn_start && prev_start && alarm.hour != 0xFF) {
            alarm.magic = ALARM_MAGIC;
            // ── Sleep screen ─────────────────────────────────────
            lv_obj_clean(lv_scr_act());
            lv_obj_t *sleep_lbl = lv_label_create(lv_scr_act());
            lv_label_set_text(sleep_lbl, "Good night,\nsleep well...\n\nzzz");
            lv_obj_set_style_text_color(sleep_lbl, lv_color_hex(ALARM_COLOR_AMBER), 0);
            lv_obj_align(sleep_lbl, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);
            lv_tick_inc(1);
            lv_timer_handler();
            __atomic_store_n(&show_gui, true, __ATOMIC_RELEASE);
            sleep_ms(2500);

            // Save alarm (holds power during SD write)
            alarm_save(&alarm);

#if !ALARM_DEBUG
            // ── Peripheral shutdown (mirrors sleep_and_shutdown_peripherals) ──
            shutdown_lcd(true, true);
            decrease_pwr_brightness(MAX_BRIGHTNESS);
            gpio_write(IOX_AUDIO_EN, 0);
            hyper_underclock_cpu(true);
            sleep_ms(100);
            release_power();
            watchdog_disable();

            // ── Sleep until dawn ─────────────────────────────────
            alarm_sleep_loop(alarm.hour, alarm.minute);

            // ── Wake ─────────────────────────────────────────────
            wakeup_and_start_peripherals();
            set_volume(0, 0);

            // LVGL state and buffers are intact from before sleep — just resume
            __atomic_store_n(&show_gui, true, __ATOMIC_RELEASE);
#endif
            // ── Sunrise ──────────────────────────────────────────
            alarm_sunrise_run(alarm.hour, alarm.minute);

            // Never returns — user powers off with switch
            while (1) tight_loop_contents();
        }

        // Save prev states for edge detection next iteration
        prev_left  = btn_left;
        prev_right = btn_right;
        prev_up    = btn_up;
        prev_down  = btn_down;
        prev_start = btn_start;
        prev_field = field;

        // Drive LVGL with actual elapsed time — every iteration, regardless of input
        uint64_t now_us = time_us_64();
        uint32_t dt_ms  = (uint32_t)((now_us - last_lvgl_tick) / 1000u);
        last_lvgl_tick  = now_us;
        lv_tick_inc(dt_ms);
        lv_timer_handler();

        sleep_ms(10);  // ~100 Hz UI refresh; fast enough for snappy response
    }
}
