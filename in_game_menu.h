
void in_game_increase_lcd_brightness() {
    if (!low_power) {
#if TIE_PWR_LED_TO_LCD
        pwr_led_duty_cycle = lcd_led_duty_cycle;
#endif
        step_pwr_brightness(true);
    }
    step_lcd_brightness(true);
}

void in_game_decrease_lcd_brightness() {
    if (!low_power) {
#if TIE_PWR_LED_TO_LCD
        pwr_led_duty_cycle = lcd_led_duty_cycle;
#endif
        step_pwr_brightness(false);
    }
    step_lcd_brightness(false);
}

void in_game_increase_button_brightness() {
    step_button_brightness(true);
}
void in_game_decrease_button_brightness() {
    step_button_brightness(false);
}

void in_game_cycle_color_palette() {
    /* cycle the next manual color palette: -1 → 0 → 1 … 12 → -1 */
    if (manual_palette_selected < 12) {
    	manual_palette_selected++;
    } else {
    	manual_palette_selected = -1; // wrap around
    }

    if (manual_palette_selected != -1) {
    	manual_assign_palette(*palette, manual_palette_selected);
    } else {
        char rom_title[16];
        auto_assign_palette(*palette, gb_colour_hash(&gb),gb_get_rom_name(&gb,rom_title));
    }
}


void in_game_save_game(bool hold_sd_busy) {
#if ENABLE_SDCARD				
    write_cart_ram_file(&gb, hold_sd_busy);
#endif				
}
static inline void in_game_save_game_no_hold() {
    in_game_save_game(false);
}
void in_game_increase_washout() {
    wash_out_level = increase_clamp(wash_out_level, 16);
}
void in_game_decrease_washout() {
    wash_out_level = decrease_clamp(wash_out_level, 16);
}

void in_game_save_state() {
    write_cart_save_state(&gb, false, -1);
}
void in_game_load_state() {
    read_cart_save_state(&gb, -1, false);
    if (g_in_game_menu) {
        g_request_exit_menu = true;
    }
}

// since this happens on power down we need to hold sd busy
void in_game_save_auto_state(bool hold_sd_busy) {
    write_cart_save_state(&gb, hold_sd_busy, 0);
}
void in_game_load_auto_state() {
    read_cart_save_state(&gb, 0, false);
    if (g_in_game_menu) {
        g_request_exit_menu = true;
    }
}


void in_game_screenshot() {
    write_screenshot_png_from_fb(front_fb, -1, false);
    if (g_in_game_menu) {
        g_request_exit_menu = true;
    }
}

void ig_toggle_auto_load_state() {
    auto_load_state = !auto_load_state;
}

// ================================================================
// In-game menu item model
// ================================================================
#define IG_VISIBLE_ITEMS 9

typedef enum {
    IG_ITEM_SLIDER,
    IG_ITEM_TOGGLE,
    IG_ITEM_VALUE,
    IG_ITEM_ACTION
} ig_item_type_t;

typedef enum {
    IG_ACT_NONE = 0,
    IG_ACT_EXIT_SAVE,
    IG_ACT_EXIT_NOSAVE,
    IG_ACT_SLEEP
} ig_menu_action_t;

typedef struct {
    const char        *label;
    ig_item_type_t     type;

    // for rendering current state/value (optional)
    void (*get_value_text)(char *out, size_t out_sz);

    // controls
    void (*inc)(void);
    void (*dec)(void);
    void (*press_a)(void);

    // for actions that should exit menu/game
    ig_menu_action_t   action;
} ig_menu_item_t;

// ================================================================
// Button edge helpers (your buttons are active-low)
// curr==false means pressed
// ================================================================
static inline bool pressed_edge(bool prev_level, bool curr_level) {
    return (prev_level == true) && (curr_level == false);
}

// ================================================================
// Small UI helpers
// ================================================================
static void make_slider_bar(char *out, size_t out_sz, int value, int max, int bars) {
    if (max <= 0) max = 1;
    if (value < 0) value = 0;
    if (value > max) value = max;

    int filled = (value * bars) / max;
    if (filled < 0) filled = 0;
    if (filled > bars) filled = bars;

    // example: [|||||....]
    size_t idx = 0;
    if (out_sz < 4) { if (out_sz) out[0] = 0; return; }
    out[idx++] = '[';
    for (int i = 0; i < bars && idx + 2 < out_sz; i++) out[idx++] = (i < filled) ? '|' : '.';
    out[idx++] = ']';
    out[idx] = 0;
}
// ================================================================
// Small UI helpers (accurate for *discrete* stepped values)
// ================================================================

static int find_level_index_le(uint8_t current, const uint8_t *levels, int n) {
    // Mirrors your stepping logic: first i where current <= levels[i]
    // If above all, returns last.
    if (n <= 1) return 0;
    for (int i = 0; i < n; i++) {
        if (current <= levels[i]) return i;
    }
    return n - 1;
}

// idx: 0..(steps-1). steps>=2.
// This maps discrete steps to a fixed bar count with rounding,
// so the visual progress matches the *step position*, not raw value.
static void make_slider_bar_steps(char *out, size_t out_sz, int idx, int steps, int bars) {
    if (out_sz < 4) { if (out_sz) out[0] = 0; return; }
    if (bars < 1) bars = 1;
    if (steps < 2) steps = 2;

    if (idx < 0) idx = 0;
    if (idx > (steps - 1)) idx = (steps - 1);

    int denom = steps - 1;

    // Rounded mapping: 0..denom -> 0..bars
    int filled = (idx * bars + (denom / 2)) / denom;
    if (filled < 0) filled = 0;
    if (filled > bars) filled = bars;

    size_t w = 0;
    out[w++] = '[';
    for (int i = 0; i < bars && (w + 2) < out_sz; i++) out[w++] = (i < filled) ? '|' : '.';
    out[w++] = ']';
    out[w] = 0;
}

// If you still want a linear bar for anything else, keep your old make_slider_bar()
// or rename it to make_slider_bar_linear().

// ================================================================
// Value text getters (render on the right side of the row)
// ================================================================

static void ig_get_lcd_brightness_text(char *out, size_t out_sz) {
    // 16 perceptual steps (brightness_levels[16])
    int idx = find_level_index_le(lcd_led_duty_cycle, brightness_levels, 16);

    static char bar[32];
    make_slider_bar_steps(bar, sizeof(bar), idx, 16, 10);

    // Show level (1..16) + raw duty for debugging/clarity
    snprintf(out, out_sz, "%s L%d/16", bar, idx + 1);
}

static void ig_get_btn_brightness_text(char *out, size_t out_sz) {
    // 6 discrete steps (button_brightness_levels[6])
    int idx = find_level_index_le(button_led_duty_cycle, button_brightness_levels, 6);

    static char bar[32];
    make_slider_bar_steps(bar, sizeof(bar), idx, 6, 10);

    snprintf(out, out_sz, "%s L%d/6", bar, idx + 1);
}

static void ig_get_washout_text(char *out, size_t out_sz) {
    // wash_out_level is 0..255 stepping by 16 => levels are 0..15
    int idx = (int)(wash_out_level >> 4);   // /16
    if (idx < 0) idx = 0;
    if (idx > 15) idx = 15;

    static char bar[32];
    make_slider_bar_steps(bar, sizeof(bar), idx, 16, 10);

    // Show both step and raw value
    snprintf(out, out_sz, "%s %d/15", bar, idx);
}

// ================================================================
// Washout wrap behavior for A (cycle upward, wrap to 0)
// (now matches 0..255 step 16)
// ================================================================
static void in_game_cycle_washout(void) {
    if (wash_out_level <= (uint8_t)(255 - 16)) wash_out_level = (uint8_t)(wash_out_level + 16);
    else wash_out_level = 0;
}

static void ig_get_fast_forward_text(char *out, size_t out_sz) {
    bool on = (run_mode == MODE_TURBO);
    snprintf(out, out_sz, "%s", on ? "ON" : "OFF");
}

static void ig_get_battery_save_text(char *out, size_t out_sz) {
    bool on = (run_mode == MODE_POWERSAVE);
    snprintf(out, out_sz, "%s", on ? "ON" : "OFF");
}

static void ig_get_auto_load_state_text(char *out, size_t out_sz) {
    snprintf(out, out_sz, "%s", auto_load_state ? "ON" : "OFF");
}

static void ig_get_crt_mode_text(char *out, size_t out_sz) {
    const char *names[] = { "Off", "Scanlines", "BFI", "Scanlines+BFI" };
    snprintf(out, out_sz, "%s", names[crt_mode % 4]);
}

static void ig_cycle_crt_mode_next(void) {
    crt_mode = (crt_mode + 1) % 4;
}
static void ig_cycle_crt_mode_prev(void) {
    crt_mode = (crt_mode == 0) ? 3 : crt_mode - 1;
}

#define MAX_PALETTE_SWATCH 6

static uint8_t collect_unique_rgb565(const palette_t pal, uint16_t *out, int out_max) {
    uint8_t n = 0;
    for (uint8_t row = 0; row < 3; row++) {
        for (uint8_t col = 0; col < 4; col++) {
            uint16_t v = pal[row][col];

            bool dup = false;
            for (int i = 0; i < n; i++) {
                if (out[i] == v) { dup = true; break; }
            }
            if (!dup) {
                if (n < out_max) out[n++] = v;
                else return n;
            }
        }
    }
    return n;
}

static void ig_get_palette_text(char *out, size_t out_sz) {
    if (!out_sz) return;

    // keep Auto thin
    if (manual_palette_selected < 0) {
        snprintf(out, out_sz, "Auto");
        return;
    }
    uint8_t palette_num = manual_palette_selected + 1;
    // Build colored swatches using LVGL recolor tags: #RRGGBB text#
    static uint16_t cols[MAX_PALETTE_SWATCH];
    uint8_t n = collect_unique_rgb565(*palette, cols, MAX_PALETTE_SWATCH);

    // Example output: "3 #ffcc00 |##00aaff |##000000 |#"
    size_t w = 0;
    int wrote = snprintf(out + w, (w < out_sz) ? (out_sz - w) : 0, "%d ", palette_num);
    if (wrote < 0) { out[0] = 0; return; }
    w += (size_t)wrote;

    for (int i = 0; i < n; i++) {
        uint8_t r, g, b;
        rgb565_to_rgb888(cols[i], &r, &g, &b);

        // each bar is colored "|"
        wrote = snprintf(out + w, (w < out_sz) ? (out_sz - w) : 0,
                         "#%02x%02x%02x |#", r, g, b);
        if (wrote < 0) break;
        w += (size_t)wrote;

        // optional spacing (keeps it readable)
        if (i != (n - 1)) {
            if (w + 2 < out_sz) { out[w++] = ' '; out[w] = 0; }
        }
    }

    // Make sure we don't end on a half tag if it clipped
    out[out_sz - 1] = 0;
}

// ================================================================
// Missing decrement for palette (since you wanted left/right)
// ================================================================
static void in_game_prev_color_palette(void) {
    // cycle previous: -1 <- 0 <- 1 ... 12 <- -1
    if (manual_palette_selected > -1) manual_palette_selected--;
    else manual_palette_selected = 12;

    if (manual_palette_selected != -1) {
        manual_assign_palette(*palette, manual_palette_selected);
    } else {
        char rom_title[16];
        auto_assign_palette(*palette, gb_colour_hash(&gb),gb_get_rom_name(&gb,rom_title));
    }
}
static void in_game_next_color_palette(void) {
    in_game_cycle_color_palette();
}

// ================================================================
// Run mode setters: guarantee mutual exclusivity and consistent side effects
// ================================================================
// Extreme battery save side-effects — called on every powersave enter/exit
// so no mode setter forgets them.
// ================================================================
#if ENABLE_EXTREME_BATTERY_SAVE
static void extreme_powersave_enter(void) {
    powersave_saved_button_brightness = button_led_duty_cycle;
    decrease_button_brightness(MAX_BRIGHTNESS);
    gpio_write(IOX_AUDIO_EN, 0);
}
static void extreme_powersave_exit(void) {
    gpio_write(IOX_AUDIO_EN, 1);
    sleep_ms(10); // let LDO stabilise before DAC init
    setup_dac();
    current_volume_level = 0;
    read_volume();
    button_led_duty_cycle = powersave_saved_button_brightness;
    pwm_set_chan_level(pwm_gpio_to_slice_num(GPIO_BUTTON_LED),
                      pwm_gpio_to_channel(GPIO_BUTTON_LED),
                      powersave_saved_button_brightness);
}
#endif

// ================================================================
// Run mode setters: guarantee mutual exclusivity and consistent side effects
// ================================================================
static void ig_set_mode_normal(void) {
    if (run_mode == MODE_POWERSAVE) {
        reconfigure_led_pwm_for_underclock(false);
        underclock_cpu(false);
#if ENABLE_EXTREME_BATTERY_SAVE
        extreme_powersave_exit();
#endif
    }
    run_mode = MODE_NORMAL;
    gb.direct.frame_skip = false;
}

static void ig_set_mode_turbo(void) {
    if (run_mode == MODE_POWERSAVE) {
        underclock_cpu(false);
#if ENABLE_EXTREME_BATTERY_SAVE
        extreme_powersave_exit();
#endif
    }
    run_mode = MODE_TURBO;
    gb.direct.frame_skip = true;
    underclock_cpu(false);
}

static void ig_set_mode_powersave(void) {
    // If we were turbo, we’re switching modes anyway. Just force powersave behavior.
    run_mode = MODE_POWERSAVE;
    underclock_cpu(true);
    reconfigure_led_pwm_for_underclock(true);
#if ENABLE_EXTREME_BATTERY_SAVE
    extreme_powersave_enter();
#endif
    gb.direct.frame_skip = true;
}

// Replace your toggles with these (or call these inside them)
static void ig_toggle_fast_forward(void) {
    if (run_mode == MODE_TURBO) ig_set_mode_normal();
    else ig_set_mode_turbo(); // this implicitly disables powersave
}

static void ig_toggle_battery_save(void) {
    if (run_mode == MODE_POWERSAVE) ig_set_mode_normal();
    else ig_set_mode_powersave(); // this implicitly disables turbo
}

// ================================================================
// In-game list draw (9 visible items, paging)
// ================================================================
static void draw_in_game_options_list(lv_obj_t *list,
                                      const ig_menu_item_t *items,
                                      int item_count,
                                      int ig_selected,
                                      int ig_page_start)
{
    lv_obj_clean(list);

    for (int i = 0; i < IG_VISIBLE_ITEMS && (i + ig_page_start) < item_count; i++) {
        int idx = i + ig_page_start;

        const char *label = items[idx].label ? items[idx].label : "";

        static char value[64];
        value[0] = '\0';
        if (items[idx].get_value_text) items[idx].get_value_text(value, sizeof(value));

        lv_obj_t *row_obj = lv_list_add_text(list, ""); // create label
        lv_obj_set_style_text_font(row_obj, LV_FONT_DEFAULT, 0);
        lv_obj_set_style_bg_color(row_obj, lv_color_hex(0xFFFFFF), 0);

        // Format directly into LVGL-managed memory
        if (value[0]) lv_label_set_text_fmt(row_obj, "%s  %s", label, value);
        else          lv_label_set_text_fmt(row_obj, "%s", label);

        // Colors
        bool is_sleep = (items[idx].action == IG_ACT_SLEEP);
        lv_obj_set_style_text_color(row_obj, is_sleep ? lv_color_hex(0xFF0000) : lv_color_black(), 0);

        if (idx == ig_selected) {
            lv_obj_set_style_bg_color(row_obj, lv_color_hex(0x33CC66), LV_PART_MAIN);
            lv_label_set_long_mode(row_obj, LV_LABEL_LONG_SCROLL_CIRCULAR);
            lv_obj_set_style_anim_speed(row_obj, 20, 0);
        } else {
            lv_obj_set_width(row_obj, DISP_HOR_RES - 30);
            lv_label_set_long_mode(row_obj, LV_LABEL_LONG_CLIP);
        }

        lv_label_set_recolor(row_obj, true);
    }
}

static void ig_update_hints(const ig_menu_item_t *it, lv_obj_t *hint_left, lv_obj_t *hint_right) {
    // Left hint = what A does
    // Right hint = what B does (you already want B = Exit Menu)
    lv_label_set_text(hint_right, "B: Exit Menu");

    switch (it->type) {
        case IG_ITEM_ACTION: lv_label_set_text(hint_left,  "A: Select"); break;
        case IG_ITEM_TOGGLE: lv_label_set_text(hint_left,  "A: Toggle"); break;
        case IG_ITEM_SLIDER: lv_label_set_text(hint_left,  "L/R: -/+"); break;
        case IG_ITEM_VALUE:  lv_label_set_text(hint_left,  "L/R: -/+");  break;
        default:             lv_label_set_text(hint_left,  ""); break;
    }
}
static uint8_t ig_selected = 0;
static uint8_t ig_page_start = 0;

static ig_menu_item_t *menu_items = NULL;
static uint8_t menu_count = 0;

#define MENU_COUNT 16  // keep this in sync with the items below

static bool ig_menu_items_init_heap(void) {
    menu_items = NULL;
    menu_count = 0;

    menu_items = (ig_menu_item_t *)malloc(MENU_COUNT * sizeof(*menu_items));
    if (!menu_items) {
        return false;
    }

    size_t i = 0;

    menu_items[i++] = (ig_menu_item_t){
        "Display      ", IG_ITEM_SLIDER, ig_get_lcd_brightness_text,
        in_game_increase_lcd_brightness, in_game_decrease_lcd_brightness,
        in_game_increase_lcd_brightness, IG_ACT_NONE
    };

    menu_items[i++] = (ig_menu_item_t){
        "Buttons     ", IG_ITEM_SLIDER, ig_get_btn_brightness_text,
        in_game_increase_button_brightness, in_game_decrease_button_brightness,
        in_game_increase_button_brightness, IG_ACT_NONE
    };

    menu_items[i++] = (ig_menu_item_t){ "Save Game",       IG_ITEM_ACTION, NULL, NULL, NULL, in_game_save_game_no_hold, IG_ACT_NONE };
    menu_items[i++] = (ig_menu_item_t){ "Save State",      IG_ITEM_ACTION, NULL, NULL, NULL, in_game_save_state, IG_ACT_NONE };
    menu_items[i++] = (ig_menu_item_t){ "Load State",      IG_ITEM_ACTION, NULL, NULL, NULL, in_game_load_state, IG_ACT_NONE };
    menu_items[i++] = (ig_menu_item_t){ "Load Auto State", IG_ITEM_ACTION, NULL, NULL, NULL, in_game_load_auto_state, IG_ACT_NONE };
    menu_items[i++] = (ig_menu_item_t){ "Screenshot",      IG_ITEM_ACTION, NULL, NULL, NULL, in_game_screenshot, IG_ACT_NONE };

    menu_items[i++] = (ig_menu_item_t){
        "Fast Forward     ", IG_ITEM_TOGGLE, ig_get_fast_forward_text,
        ig_toggle_fast_forward, ig_toggle_fast_forward, ig_toggle_fast_forward, IG_ACT_NONE
    };

    menu_items[i++] = (ig_menu_item_t){
        "Battery Saving  ", IG_ITEM_TOGGLE, ig_get_battery_save_text,
        ig_toggle_battery_save, ig_toggle_battery_save, ig_toggle_battery_save, IG_ACT_NONE
    };

    menu_items[i++] = (ig_menu_item_t){
        "Auto Load State ", IG_ITEM_TOGGLE, ig_get_auto_load_state_text,
        ig_toggle_auto_load_state, ig_toggle_auto_load_state, ig_toggle_auto_load_state, IG_ACT_NONE
    };

    menu_items[i++] = (ig_menu_item_t){
        "CRT         ", IG_ITEM_VALUE, ig_get_crt_mode_text,
        ig_cycle_crt_mode_next, ig_cycle_crt_mode_prev, ig_cycle_crt_mode_next, IG_ACT_NONE
    };

    menu_items[i++] = (ig_menu_item_t){
        "Color Palette", IG_ITEM_VALUE, ig_get_palette_text,
        in_game_next_color_palette, in_game_prev_color_palette,
        in_game_next_color_palette, IG_ACT_NONE
    };

    menu_items[i++] = (ig_menu_item_t){
        "Wash Out    ", IG_ITEM_SLIDER, ig_get_washout_text,
        in_game_increase_washout, in_game_decrease_washout,
        in_game_cycle_washout, IG_ACT_NONE
    };

    menu_items[i++] = (ig_menu_item_t){ "Exit Game & Save",   IG_ITEM_ACTION, NULL, NULL, NULL, NULL, IG_ACT_EXIT_SAVE };
    menu_items[i++] = (ig_menu_item_t){ "Exit Game w/o Save", IG_ITEM_ACTION, NULL, NULL, NULL, NULL, IG_ACT_EXIT_NOSAVE };
    menu_items[i++] = (ig_menu_item_t){ "Sleep",        IG_ITEM_ACTION, NULL, NULL, NULL, NULL, IG_ACT_SLEEP };

    // Finalize counts + sanity
    if (i != MENU_COUNT) {
        free(menu_items);
        menu_items = NULL;
        menu_count = 0;
        return false;
    }

    menu_count = (uint8_t)i;
    return true;
}

static void ig_menu_items_free_heap(void) {
    if (menu_items) {
        free(menu_items);
        menu_items = NULL;
    }
    menu_count = 0;
}


void in_game_menu() {
    g_in_game_menu = true;

    // Essential
    lv_deinit();

    if (lvgl_fb) {
        for (size_t i = 0; i < (lvgl_fb_bytes / 2); i++) {
            ((uint16_t*)lvgl_fb)[i] = 0xFFFF;
        }
    }

    // Create list
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
    lv_obj_set_style_bg_color(cont, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xFFFFFF), 0);

    // === Options List ===
    lv_obj_t *list = lv_list_create(cont);
    lv_obj_set_size(list, DISP_HOR_RES, DISP_VER_RES - 20);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 7);
    lv_obj_set_style_bg_color(list, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_color(list, lv_color_black(), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *status_label;
    lv_obj_t *top_bar = create_top_bar(cont, &status_label);
    (void)top_bar;

    update_status_label(status_label);

    // === Bottom hint bar ===
    lv_obj_t *hint_left;
    lv_obj_t *hint_right;
    lv_obj_t *hint_bar = create_bottom_bar(cont, &hint_left, &hint_right);
    (void)hint_bar;

    ig_menu_items_init_heap();

    draw_in_game_options_list(list, menu_items, menu_count, ig_selected, ig_page_start);
    ig_update_hints(&menu_items[ig_selected], hint_left, hint_right);

    ig_menu_action_t requested_action = IG_ACT_NONE;

    // Buttons (active-low levels)
    bool up = true, down = true, left = true, right = true, a = true, b = true, select_btn = true, start = true;
    bool prev_up = true, prev_down = true, prev_left = true, prev_right = true;
    bool prev_a = true, prev_b = true, prev_start = true, prev_select = true;

    // LVGL tick baseline (prevents huge first dt)
    static uint64_t last_lvgl_tick = 0;
    last_lvgl_tick = time_us_64();

    __atomic_store_n(&show_gui, true, __ATOMIC_RELEASE);

    while (1) {

#if ENABLE_BAT_MONITORING
        if (battery_task_flag) {
            battery_task_flag = false;
            process_bat_percent();
            update_status_label(status_label);
        }
#endif

        bool iox_nint = gpio_read(GPIO_IOX_nINT);
        select_btn = gpio_read(GPIO_B_SELECT);

        if (!iox_nint) {
            read_io_expander_states(0);

            a     = gpio_read(IOX_B_A);
            b     = gpio_read(IOX_B_B);
            up    = gpio_read(IOX_B_UP);
            down  = gpio_read(IOX_B_DOWN);
            left  = gpio_read(IOX_B_LEFT);
            right = gpio_read(IOX_B_RIGHT);
            start = gpio_read(IOX_B_START);
        }

        // Exit menu on B or power switch out or low power shutdown
        if (!b || !gpio_read(GPIO_SW_OUT) || low_power_shutdown || g_request_exit_menu) {
            // while(!b && gpio_read(GPIO_SW_OUT) && !low_power_shutdown) {
            //     if (!iox_nint) {
            //         read_io_expander_states(0);
            //         b = gpio_read(IOX_B_B);
            //     }
            //     watchdog_update();
            //     tight_loop_contents();
            // }
            sleep_ms(10);
            break;
        }

        // ============================================================
        // Input edges (active-low)
        // ============================================================
        bool up_edge    = pressed_edge(prev_up, up);
        bool down_edge  = pressed_edge(prev_down, down);
        bool left_edge  = pressed_edge(prev_left, left);
        bool right_edge = pressed_edge(prev_right, right);
        bool a_edge     = pressed_edge(prev_a, a);

        // Move selection (with wrap)
        if (up_edge && menu_count > 0) {
            if (ig_selected == 0) ig_selected = menu_count - 1;
            else ig_selected--;

            // ensure ig_selected is visible
            if (ig_selected < ig_page_start) {
                ig_page_start = ig_selected;
            } else if (ig_selected >= ig_page_start + IG_VISIBLE_ITEMS) {
                ig_page_start = ig_selected - (IG_VISIBLE_ITEMS - 1);
            }
        }

        if (down_edge && menu_count > 0) {
            if (ig_selected >= (menu_count - 1)) ig_selected = 0;
            else ig_selected++;

            // ensure ig_selected is visible
            if (ig_selected < ig_page_start) {
                ig_page_start = ig_selected;
            } else if (ig_selected >= ig_page_start + IG_VISIBLE_ITEMS) {
                ig_page_start = ig_selected - (IG_VISIBLE_ITEMS - 1);
            }
        }
        int max_page_start = (menu_count > IG_VISIBLE_ITEMS) ? (menu_count - IG_VISIBLE_ITEMS) : 0;
        if (ig_page_start > max_page_start) ig_page_start = max_page_start;
        if (ig_page_start < 0) ig_page_start = 0;


        ig_menu_item_t *it = &menu_items[ig_selected];

        // Adjust current item
        if (left_edge && it->dec)  it->dec();
        if (right_edge && it->inc) it->inc();

        // A action
        if (a_edge) {
            if (it->type == IG_ITEM_ACTION) {
                if (it->action != IG_ACT_NONE) {
                    requested_action = it->action;
                    break;
                } else if (it->press_a) {
                    it->press_a();
                }
            } else {
                if (it->press_a) it->press_a();
            }
        }

        // Redraw only if something changed
        if (up_edge || down_edge || left_edge || right_edge || a_edge) {
            draw_in_game_options_list(list, menu_items, menu_count, ig_selected, ig_page_start);
            ig_update_hints(it, hint_left, hint_right);
        }

        // Save states for edge detection
        prev_up = up; prev_down = down; prev_left = left; prev_right = right;
        prev_a = a; prev_b = b; prev_start = start; prev_select = select_btn;

        // LVGL tick
        uint64_t now = time_us_64();
        uint64_t dt_ms = (now - last_lvgl_tick) / 1000;
        last_lvgl_tick = now;

        lv_tick_inc((uint32_t)dt_ms);
        lv_timer_handler();

        watchdog_update();
        sleep_ms(5);
        tight_loop_contents();
    }

    lv_deinit();
    ig_menu_items_free_heap();
    sleep_ms(20);

#if UNDERCLOCK_IN_GAME_MENU
    underclock_cpu(false);
#endif

    // Handle requested_action however you prefer:
    // - IG_ACT_EXIT_SAVE: save battery + exit to ROM selector
    // - IG_ACT_EXIT_NOSAVE: exit without saving
    // - IG_ACT_SLEEP: enter sleep mode
    //
    // For now, just example stubs:
    if (requested_action == IG_ACT_EXIT_SAVE) {
        in_game_save_auto_state(true);
        in_game_save_game(false);
        g_request_exit_to_rom_selector = true;
    } else if (requested_action == IG_ACT_EXIT_NOSAVE) {
        g_request_exit_to_rom_selector = true;
    } else if (requested_action == IG_ACT_SLEEP) {

        uint8_t temp_lcd_led = lcd_led_duty_cycle;
        // sd busy handles lcd led turn off
#if LED_PHASE_OUT_PWR_DOWN
        uint8_t temp_button_led = button_led_duty_cycle;
        uint8_t temp_pwr_led = pwr_led_duty_cycle;
        fade_out_leds_powerdown();
#endif

        // we want to shut down the lcd here first and not let save game or state do it because they don't turn off the button leds and i dont trust it
        // this allows us to keep the is lcd on check to prevent back to back shut down lcds
        // wake and start peripherals will turn it back on
        // this is a volatile area to be kept with care and thought of everything going on because there's a lot.
        // First pass: keep UI dark + SD-safe, but defer hard core1 reset to sleep_and_shutdown_peripherals().
        // No longer needed due to modularization and better state management in shutdown lcd calls
        // shutdown_lcd(true, true);

#if ENABLE_SDCARD
        in_game_save_auto_state(true);
        in_game_save_game(true);
#endif

#if LED_PHASE_OUT_PWR_DOWN
        save_system_settings_if_changed(temp_lcd_led, temp_button_led, temp_pwr_led, manual_palette_selected, wash_out_level, last_filename_raw, auto_load_state, crt_mode, );
#else
# if TIE_PWR_LED_TO_LCD
        pwr_led_duty_cycle = temp_lcd_led;
# endif
        save_system_settings_if_changed(temp_lcd_led, saved_button_brightness, low_power ? prev_pwr_led_duty_cycle : pwr_led_duty_cycle, manual_palette_selected, wash_out_level, last_filename_raw, auto_load_state, crt_mode, true);
#endif


        sleep_and_shutdown_peripherals();

        // start back up everything
        wakeup_and_start_peripherals();

#if ENABLE_EXTREME_BATTERY_SAVE
        if (run_mode == MODE_POWERSAVE) {
            // wakeup restored button LEDs and audio amp — re-suppress them for extreme powersave
            gpio_write(IOX_AUDIO_EN, 0);
            decrease_button_brightness(MAX_BRIGHTNESS);
        }
#endif
    }

    g_request_exit_menu = false;
    g_in_game_menu = false;
    __atomic_store_n(&show_gui, false, __ATOMIC_RELEASE);
}
