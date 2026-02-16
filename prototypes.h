void shutdown_screen(uint32_t duration_ms);
void sleep_and_shutdown_peripherals();
void gpio_write(uint8_t gpio_num, bool value);
void start_lcd(bool button_leds_restore, bool start_core1);
void shutdown_lcd(bool button_leds_restore, bool shutdown_core1);
void draw_now_playing(lv_obj_t *parent);
void draw_track_list(lv_obj_t *list,
                     char filenames[][256],
                     uint16_t num_file,
                     uint16_t selected,
                     uint16_t page_start);
// MP3_MAX_PATH_LEN = 256
static inline void mp3_select_relative(int delta,
                                       lv_obj_t *mp3_list_obj,
                                       char (*g_playlist)[256],
                                       int g_track_count);

void update_mp3_bottom_bar_shuffle_repeat(lv_obj_t *right,
                                            uint8_t repeat_state,
                                            bool shuffle, 
                                            bool paused);
void mp3_apply_now_playing_theme();
             
size_t sfe_mem_size(void);
size_t sfe_mem_max_free_size(void);
size_t sfe_mem_used(void);