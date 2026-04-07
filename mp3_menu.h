// mp3_menu.h — In-menu overlay for the MP3 player.
// Triggered by START+SELECT. Uses Blue Cola color scheme.
// Included after in_game_menu.h, mp3.h, sd.h, audio.h in main.c.
#pragma once

#define MP3_MENU_ACCENT  0x0094E1   // Blue Cola accent (matches ACCENT_COLOR in mp3.h)
#define MP3_MENU_COUNT   12
#define MP3_GAIN_MIN     80
#define MP3_GAIN_MAX     127

// ================================================================
// Value text getters
// ================================================================

static void mp3_get_shuffle_text(char *out, size_t sz) {
    snprintf(out, sz, "%s", g_shuffle_enabled ? "ON" : "OFF");
}

static void mp3_get_repeat_text(char *out, size_t sz) {
    static const char *names[] = { "Off", "Once", "Loop" };
    snprintf(out, sz, "%s", names[(int)g_repeat_mode % 3]);
}

static void mp3_get_lock_text(char *out, size_t sz) {
    snprintf(out, sz, "%s", g_buttons_locked ? "ON" : "OFF");
}

static void mp3_get_gain_text(char *out, size_t sz) {
    static char bar[32];
    make_slider_bar(bar, sizeof(bar),
                    g_analog_gain - MP3_GAIN_MIN,
                    MP3_GAIN_MAX  - MP3_GAIN_MIN, 10);
    snprintf(out, sz, "%s %d", bar, g_analog_gain);
}

static void mp3_get_3d_text(char *out, size_t sz) {
    snprintf(out, sz, "%s", g_3d_enabled ? "ON" : "OFF");
}

static void mp3_get_audio_mode_text(char *out, size_t sz) {
    static const char *names[] = { "Auto", "SPK+HP" };
    snprintf(out, sz, "%s", names[(int)audio_mode % 2]);
}

static void mp3_get_eq_text(char *out, size_t sz) {
    static const char *names[] = { "Flat", "Bass+", "Treble+", "V-Curve", "Vocal" };
    snprintf(out, sz, "%s", names[g_eq_preset % EQ_PRESET_COUNT]);
}

static void mp3_get_spk_gain_text(char *out, size_t sz) {
    static const char *names[] = { "6dB", "12dB", "18dB", "24dB" };
    snprintf(out, sz, "%s", names[g_spk_gain % 4]);
}

// ================================================================
// Controls
// ================================================================

static void mp3_toggle_shuffle(void) {
    g_shuffle_enabled       = !g_shuffle_enabled;
    g_shuffle_needs_rebuild = true;
}

static void mp3_repeat_inc(void) {
    g_repeat_mode = (repeat_mode_t)((g_repeat_mode + 1) % 3);
}
static void mp3_repeat_dec(void) {
    g_repeat_mode = (repeat_mode_t)((g_repeat_mode == 0) ? 2 : g_repeat_mode - 1);
}

static void mp3_toggle_lock(void) {
    g_buttons_locked = !g_buttons_locked;
    g_mp3_inactive   = g_buttons_locked;
}

static void mp3_gain_inc(void) {
    if (g_analog_gain < MP3_GAIN_MAX) { g_analog_gain++; set_gain(g_analog_gain); }
}
static void mp3_gain_dec(void) {
    if (g_analog_gain > MP3_GAIN_MIN) { g_analog_gain--; set_gain(g_analog_gain); }
}

static void mp3_toggle_3d(void) {
    g_3d_enabled = !g_3d_enabled;
    if (g_3d_enabled) set_3d(0x15, 0x00);
    else              set_3d(0x00, 0x00);
}

static void mp3_audio_mode_inc(void) {
    audio_mode = (audio_output_mode_t)((audio_mode + 1) % 2);
    apply_audio_mode();
}
static void mp3_audio_mode_dec(void) {
    audio_mode = (audio_output_mode_t)((audio_mode == 0) ? 1 : audio_mode - 1);
    apply_audio_mode();
}

static void mp3_eq_inc(void) {
    apply_eq_preset((eq_preset_t)((g_eq_preset + 1) % EQ_PRESET_COUNT));
}
static void mp3_eq_dec(void) {
    apply_eq_preset(g_eq_preset == EQ_FLAT ? (eq_preset_t)(EQ_PRESET_COUNT - 1) : (eq_preset_t)(g_eq_preset - 1));
}

static void mp3_spk_gain_inc(void) {
    if (g_spk_gain < SPK_GAIN_24DB) set_spk_driver_gain((spk_gain_t)(g_spk_gain + 1));
}
static void mp3_spk_gain_dec(void) {
    if (g_spk_gain > SPK_GAIN_6DB) set_spk_driver_gain((spk_gain_t)(g_spk_gain - 1));
}

static void mp3_reset_audio_defaults(void) {
    g_analog_gain = ANALOG_GAIN;
    g_3d_enabled  = true;
    audio_mode    = AUDIO_AUTO;
    g_eq_preset   = EQ_FLAT;
    g_spk_gain    = SPK_GAIN_6DB;

    set_gain(g_analog_gain);
    set_spk_driver_gain(g_spk_gain);
    apply_eq_preset(g_eq_preset);
    set_3d(0x15, 0x00);
    apply_audio_mode();
}

// ================================================================
// Menu items
// ================================================================

static ig_menu_item_t *mp3_menu_items = NULL;

static bool mp3_menu_items_init(void) {
    mp3_menu_items = (ig_menu_item_t *)malloc(MP3_MENU_COUNT * sizeof(*mp3_menu_items));
    if (!mp3_menu_items) return false;

    size_t i = 0;

    mp3_menu_items[i++] = (ig_menu_item_t){
        "Display       ", IG_ITEM_SLIDER, ig_get_lcd_brightness_text,
        in_game_increase_lcd_brightness, in_game_decrease_lcd_brightness,
        in_game_increase_lcd_brightness, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "Buttons      ", IG_ITEM_SLIDER, ig_get_btn_brightness_text,
        in_game_increase_button_brightness, in_game_decrease_button_brightness,
        in_game_increase_button_brightness, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "Shuffle            ", IG_ITEM_TOGGLE, mp3_get_shuffle_text,
        mp3_toggle_shuffle, mp3_toggle_shuffle, mp3_toggle_shuffle, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "Repeat            ", IG_ITEM_VALUE, mp3_get_repeat_text,
        mp3_repeat_inc, mp3_repeat_dec, mp3_repeat_inc, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "Button Lock   ", IG_ITEM_TOGGLE, mp3_get_lock_text,
        mp3_toggle_lock, mp3_toggle_lock, mp3_toggle_lock, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "EQ Preset       ", IG_ITEM_VALUE, mp3_get_eq_text,
        mp3_eq_inc, mp3_eq_dec, mp3_eq_inc, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "3D Effect        ", IG_ITEM_TOGGLE, mp3_get_3d_text,
        mp3_toggle_3d, mp3_toggle_3d, mp3_toggle_3d, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "Analog Gain   ", IG_ITEM_SLIDER, mp3_get_gain_text,
        mp3_gain_inc, mp3_gain_dec, mp3_gain_inc, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "Speaker Gain ", IG_ITEM_VALUE, mp3_get_spk_gain_text,
        mp3_spk_gain_inc, mp3_spk_gain_dec, mp3_spk_gain_inc, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "Audio Output", IG_ITEM_VALUE, mp3_get_audio_mode_text,
        mp3_audio_mode_inc, mp3_audio_mode_dec, mp3_audio_mode_inc, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "Reset Audio  ", IG_ITEM_ACTION, NULL,
        mp3_reset_audio_defaults, NULL, mp3_reset_audio_defaults, IG_ACT_NONE
    };
    mp3_menu_items[i++] = (ig_menu_item_t){
        "Settings ->  ", IG_ITEM_ACTION, NULL,
        NULL, NULL, NULL, IG_ACT_NONE   // handled by index check in loop
    };

    if (i != MP3_MENU_COUNT) { free(mp3_menu_items); mp3_menu_items = NULL; return false; }
    return true;
}

static void mp3_menu_items_free(void) {
    if (mp3_menu_items) { free(mp3_menu_items); mp3_menu_items = NULL; }
}

// ================================================================
// Draw — same as draw_in_game_options_list but Blue Cola accent
// ================================================================

static void draw_mp3_options_list(lv_obj_t *list,
                                   const ig_menu_item_t *items,
                                   int item_count,
                                   int selected,
                                   int page_start)
{
    lv_obj_clean(list);

    for (int i = 0; i < IG_VISIBLE_ITEMS && (i + page_start) < item_count; i++) {
        int idx = i + page_start;

        const char *label = items[idx].label ? items[idx].label : "";

        static char value[64];
        value[0] = '\0';
        if (items[idx].get_value_text) items[idx].get_value_text(value, sizeof(value));

        lv_obj_t *row = lv_list_add_text(list, "");
        lv_obj_set_style_text_font(row, LV_FONT_DEFAULT, 0);
        lv_obj_set_style_bg_color(row, lv_color_hex(0xFFFFFF), 0);

        if (value[0]) lv_label_set_text_fmt(row, "%s  %s", label, value);
        else          lv_label_set_text_fmt(row, "%s", label);

        lv_obj_set_style_text_color(row, lv_color_black(), 0);

        if (idx == selected) {
            lv_obj_set_style_bg_color(row, lv_color_hex(MP3_MENU_ACCENT), LV_PART_MAIN);
            lv_label_set_long_mode(row, LV_LABEL_LONG_SCROLL_CIRCULAR);
            lv_obj_set_style_anim_speed(row, 20, 0);
        } else {
            lv_obj_set_width(row, DISP_HOR_RES - 30);
            lv_label_set_long_mode(row, LV_LABEL_LONG_CLIP);
        }

        lv_label_set_recolor(row, true);
    }
}

// ================================================================
// Hints bar
// ================================================================

static void mp3_update_hints(const ig_menu_item_t *it,
                              lv_obj_t *hint_left, lv_obj_t *hint_right)
{
    lv_label_set_text(hint_right, "B: Close");
    switch (it->type) {
        case IG_ITEM_ACTION: lv_label_set_text(hint_left, "A: Open");   break;
        case IG_ITEM_TOGGLE: lv_label_set_text(hint_left, "A: Toggle"); break;
        case IG_ITEM_SLIDER: /* fall-through */
        case IG_ITEM_VALUE:  lv_label_set_text(hint_left, "L/R: -/+"); break;
        default:             lv_label_set_text(hint_left, "");          break;
    }
}

// ================================================================
// Main menu function — called from mp3.h on START+SELECT
// LVGL is already initialized; uses overlay container, no deinit/reinit.
// ================================================================

void open_mp3_menu(void) {
    __atomic_store_n(&show_gui, true, __ATOMIC_RELEASE);

    // Full-screen opaque overlay — sits on top of existing MP3 LVGL objects.
    lv_obj_t *overlay = lv_obj_create(lv_scr_act());
    lv_obj_set_size(overlay, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_center(overlay);
    lv_obj_set_style_bg_color(overlay, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_scrollbar_mode(overlay, LV_SCROLLBAR_MODE_OFF);

    // Screen background
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xFFFFFF), 0);

    // List (occupies most of screen, leaving room for top/bottom bars)
    lv_obj_t *list = lv_list_create(overlay);
    lv_obj_set_size(list, DISP_HOR_RES, DISP_VER_RES - 20);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 7);
    lv_obj_set_style_bg_color(list, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_color(list, lv_color_black(), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);

    // Top bar
    lv_obj_t *status_label;
    lv_obj_t *top_bar = create_top_bar(overlay, &status_label);
    (void)top_bar;
    update_status_label(status_label);

    // Bottom bar
    lv_obj_t *hint_left, *hint_right;
    lv_obj_t *hint_bar = create_bottom_bar(overlay, &hint_left, &hint_right);
    (void)hint_bar;

    // Populate menu
    mp3_menu_items_init();

    // Preserve cursor position across opens
    static uint8_t mp3_sel  = 0;
    static uint8_t mp3_page = 0;

    draw_mp3_options_list(list, mp3_menu_items, MP3_MENU_COUNT, mp3_sel, mp3_page);
    mp3_update_hints(&mp3_menu_items[mp3_sel], hint_left, hint_right);

    // Button state (active-low: 0=pressed, 1=released)
    bool up=1, down=1, left=1, right=1, a=1, b=1;
    bool prev_up=1, prev_down=1, prev_left=1, prev_right=1, prev_a=1, prev_b=1;

    static uint64_t last_tick = 0;
    last_tick = time_us_64();

    // ================================================================
    // Input loop
    // ================================================================
    while (true) {

#if ENABLE_BAT_MONITORING
        if (battery_task_flag) {
            battery_task_flag = false;
            process_bat_percent();
            update_status_label(status_label);
        }
#endif

        bool iox_nint = gpio_read(GPIO_IOX_nINT);
        if (!iox_nint) {
            read_io_expander_states(0);
            a     = gpio_read(IOX_B_A);
            b     = gpio_read(IOX_B_B);
            up    = gpio_read(IOX_B_UP);
            down  = gpio_read(IOX_B_DOWN);
            left  = gpio_read(IOX_B_LEFT);
            right = gpio_read(IOX_B_RIGHT);
        }

        // B, power switch, or low battery → exit
        if (!b || !gpio_read(GPIO_SW_OUT) || low_power_shutdown) {
            sleep_ms(10);
            break;
        }

        bool up_edge    = pressed_edge(prev_up,    up);
        bool down_edge  = pressed_edge(prev_down,  down);
        bool left_edge  = pressed_edge(prev_left,  left);
        bool right_edge = pressed_edge(prev_right, right);
        bool a_edge     = pressed_edge(prev_a,     a);

        // Navigation (wrap-around with paging)
        if (up_edge) {
            if (mp3_sel == 0) mp3_sel = MP3_MENU_COUNT - 1;
            else              mp3_sel--;
            if (mp3_sel < mp3_page)
                mp3_page = mp3_sel;
            else if (mp3_sel >= mp3_page + IG_VISIBLE_ITEMS)
                mp3_page = mp3_sel - (IG_VISIBLE_ITEMS - 1);
        }
        if (down_edge) {
            if (mp3_sel >= MP3_MENU_COUNT - 1) mp3_sel = 0;
            else                               mp3_sel++;
            if (mp3_sel < mp3_page)
                mp3_page = mp3_sel;
            else if (mp3_sel >= mp3_page + IG_VISIBLE_ITEMS)
                mp3_page = mp3_sel - (IG_VISIBLE_ITEMS - 1);
        }
        int max_page = (MP3_MENU_COUNT > IG_VISIBLE_ITEMS) ? (MP3_MENU_COUNT - IG_VISIBLE_ITEMS) : 0;
        if (mp3_page > (uint8_t)max_page) mp3_page = (uint8_t)max_page;

        ig_menu_item_t *it = &mp3_menu_items[mp3_sel];

        // Adjust slider / value items
        if (left_edge  && it->dec) it->dec();
        if (right_edge && it->inc) it->inc();

        // A press
        if (a_edge) {
            // Settings → sub-screen: hand off the same list widget
            if (mp3_sel == MP3_MENU_COUNT - 1) {
                run_settings_screen(list, hint_left, hint_right);
                // Restore the menu into the same list and reset hints
                draw_mp3_options_list(list, mp3_menu_items, MP3_MENU_COUNT, mp3_sel, mp3_page);
                mp3_update_hints(it, hint_left, hint_right);
            } else if (it->press_a) {
                it->press_a();
            }
        }

        // Redraw on any input change
        if (up_edge || down_edge || left_edge || right_edge || a_edge) {
            draw_mp3_options_list(list, mp3_menu_items, MP3_MENU_COUNT, mp3_sel, mp3_page);
            mp3_update_hints(&mp3_menu_items[mp3_sel], hint_left, hint_right);
        }

        prev_up=up; prev_down=down; prev_left=left; prev_right=right;
        prev_a=a; prev_b=b;

        uint64_t now = time_us_64();
        uint64_t dt_ms = (now - last_tick) / 1000;
        last_tick = now;

        lv_tick_inc((uint32_t)dt_ms);
        lv_timer_handler();

        watchdog_update();
        sleep_ms(5);
        tight_loop_contents();
    }

    // Remove overlay — existing MP3 LVGL objects become visible again
    lv_obj_del(overlay);
    mp3_menu_items_free();
    sleep_ms(20);
}
