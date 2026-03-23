#if ENABLE_LCD
// MARK: DPI TIMINGS
#define H_ACTIVE 320
#define H_PULSE 12
#define H_F_PORCH 45
#define H_B_PORCH 100

#define V_ACTIVE 320
#define V_PULSE 6
#define V_F_PORCH 10
#define V_B_PORCH 15

// Always 320 scanlines / yscale=1 so CRT mode can be toggled at runtime.
// Actual content/black pattern is controlled by the crt_mode variable below.
#define ENABLE_SCANLINES 1
#define ENABLE_V_SCANLINES 0

#define DISP_HOR_RES 160
#define DISP_VER_RES 144
#define LV_BUF_LINES DISP_VER_RES  // Number of lines per buffer chunk

const scanvideo_timing_t tft_timing_320x320_60 = {
    .clock_freq      =  20 * 1000 * 1000,
    .h_active        =  H_ACTIVE,
    .v_active        =  V_ACTIVE,
    .h_front_porch   =  H_F_PORCH,
    .h_pulse         =  H_PULSE,
    .h_total         =  H_ACTIVE + H_F_PORCH + H_PULSE + H_B_PORCH,
    .h_sync_polarity =  1,
    .v_front_porch   =  V_F_PORCH,
    .v_pulse         =  V_PULSE,
    .v_total         =  V_ACTIVE + V_F_PORCH + V_PULSE + V_B_PORCH,
    .v_sync_polarity =  1,
    .enable_clock    =  1,
    .clock_polarity  =  1,
    .enable_den      =  1
};

extern const struct scanvideo_pio_program video_24mhz_composable;
const scanvideo_mode_t tft_mode_320x320_60 = {
    .default_timing     = &tft_timing_320x320_60,
    .pio_program        = &video_24mhz_composable, 
    .width              = ENABLE_V_SCANLINES ? 320 : 160,
    .height             = ENABLE_SCANLINES ? 320 : 160,
    .xscale             = ENABLE_V_SCANLINES ? 1 : DISPLAY_SCALE,
    .yscale             = ENABLE_SCANLINES ? 1 : DISPLAY_SCALE,
    .yscale_denominator = 1
};
#define VGA_MODE tft_mode_320x320_60

#define ADD_BLACK_BAR 0

static void frame_update_logic();
static void scanline_update_logic();
static void render_scanline(struct scanvideo_scanline_buffer *dest, const uint16_t *fb);
uint16_t shift_components(uint16_t pixel);
#if PEANUT_FULL_GBC_SUPPORT
static inline void refresh_cgb_palette_cache(struct gb_s *gb);
#endif
extern uint8_t wash_out_level; // defined later in this file
extern uint8_t crt_mode;       // defined in global.h
extern bool gb_active;         // defined in global.h
#define CRT_OFF       0
#define CRT_SCANLINES 1
#define CRT_BFI       2
#define CRT_BOTH      3
// Phosphor: 4-step exponential backlight decay per 60Hz game cycle.
// Same 240Hz dark intervals as BFI but bright phases fade: 100%→70%→35%→10%.
// Mimics CRT phosphor persistence — softer than hard BFI, same motion clarity.
#define CRT_PHOSPHOR  4
// BFI strobe floor as % of current brightness.
// At >=144Hz there is no perceptible flicker, so 0 = full black gives true BFI
// (maximum motion clarity).
#define CRT_STROBE_PCT 0
// Scanlines per BFI half-period (toggle interval). BFI_HZ must be 120 or 240.
// 120Hz → 160 scanlines/toggle, 240Hz → 80 scanlines/toggle.
#define BFI_SCANLINES_PER_TOGGLE (38400u / ((uint32_t)BFI_HZ * 2u))
#if (BFI_HZ != 120) && (BFI_HZ != 240)
#error "BFI_HZ must be 120 or 240 — other values cause per-frame brightness variation at 120fps"
#endif

// Automatic brightness compensation for CRT modes.
// BFI at 50% duty cycle halves average brightness → 2× compensation.
// Scanlines black every other row → 1.25× compensation (perceptually less severe).
// Both combined → 2.5×. lcd_led_duty_cycle is never modified; compensation only
// affects the PWM register so settings save/load always reflect the user's real preference.
// Phosphor decay levels per bright phase (% of compensated peak), repeated every 60Hz.
// 4 bright phases per 60Hz cycle at 240Hz BFI: 100%→70%→35%→10%.
static const uint8_t phosphor_levels[4] = {100, 85, 65, 45};

static inline uint8_t crt_compensated_brightness(uint8_t base) {
    float comp = (float)base;
    if (crt_mode == CRT_SCANLINES || crt_mode == CRT_BOTH) comp *= 1.25f;
    // BFI and Phosphor both use 2× to compensate for 50% dark duty cycle.
    if (crt_mode == CRT_BFI || crt_mode == CRT_BOTH || crt_mode == CRT_PHOSPHOR) comp *= 2.0f;
    return (uint8_t)(comp > 255.0f ? 255.0f : comp);
}

// Cached conversion of CGB 15-bit palette entries to output RGB565.
#if PEANUT_FULL_GBC_SUPPORT
static uint16_t cgb_palette_rgb565_cache[0x40];
static uint8_t cgb_palette_wash_cache = 0xFF;
static uint32_t cgb_palette_epoch_cache = UINT32_MAX;
static bool cgb_palette_cache_valid = false;
#endif

static uint16_t *black_fb = NULL;
bool black_fb_init(void) {
    if (black_fb) return true;  // already allocated

    black_fb = (uint16_t *)malloc(sizeof(uint16_t) * LCD_WIDTH);
    if (!black_fb) return false;

    memset(black_fb, 0, sizeof(uint16_t) * LCD_WIDTH);
    return true;
}
void black_fb_deinit(void) {
    if (!black_fb) return;
    free(black_fb);
    black_fb = NULL;
}

// MARK: - RENDER LOOP
// "Worker thread" for each core

typedef enum { FB_FREE = 0, FB_READY = 1, FB_DISPLAYED = 2 } fb_state_t;

typedef struct {
    uint16_t data[LCD_HEIGHT][LCD_WIDTH];
    uint32_t state;   // store fb_state_t values here
} framebuffer_t;

// three buffers
static framebuffer_t *fb0 = NULL;
static framebuffer_t *fb1 = NULL;
static framebuffer_t *fb2 = NULL;

// Core1 display pointer (core1 reads it constantly)
static framebuffer_t *front_fb = NULL;

// Core0-only write/free pointers
static framebuffer_t *write_fb = NULL;
static framebuffer_t *free_fb  = NULL;

// Cross-core mailbox: published by core0, consumed by core1
static framebuffer_t *ready_fb = NULL;  // atomic pointer

static framebuffer_t *freed_fb = NULL; // core1->core0 mailbox

// Alias with 2D array syntax
static uint16_t (*lvgl_fb)[LCD_WIDTH];
#define lvgl_fb_bytes   ((size_t)LCD_WIDTH * (size_t)LCD_HEIGHT * sizeof(uint16_t))
#define LVGL_BUF1_BYTES ((size_t)LCD_WIDTH * (size_t)LV_BUF_LINES * sizeof(lv_color_t))
// Alias backbuffer as 1D LVGL buffer
static lv_color_t *lv_buf1;

static bool show_gui = false;


#define ENABLE_INTERLACING ENABLE_SCANLINES && 0
bool interlacing_field = 1; // 0 = even, 1 = odd
uint16_t scanline_count = 0;

#define ADD_TOP_PADDING 1
#define TOP_PADDING 6
uint16_t top_padding_counter = 0;

#if ENABLE_FRAME_DEBUGGING
static uint32_t fps_last_time = 0;
static uint32_t fps_counter = 0;
#endif
#if SKIP_FRAMES
uint8_t fps_divider = 4;
uint8_t fps_skip_counter = 0;
#endif
static bool core1_parked = false;

bool wait_for_core1_parked(uint32_t timeout_us) {
    uint64_t start = time_us_64();

    while (!__atomic_load_n(&core1_parked, __ATOMIC_ACQUIRE)) {
        if ((time_us_64() - start) > timeout_us) {
            return false;   // timed out
        }
        tight_loop_contents();
    }
    return true;
}

void render_loop() {
    static uint32_t last_frame_num = 0;
    static uint32_t y = 0;
    // Phosphor decay: step (0-3) tracks which decay level to use next bright phase.
    // disp_frame_count resets step every 2 display frames (= 1 game frame @ 60Hz).
    static uint32_t disp_frame_count = 0;
    static uint8_t  phosphor_step    = 0;

    while (true) {
        if (__atomic_load_n(&sd_busy, __ATOMIC_ACQUIRE)) {
            // Park core1
            __atomic_store_n(&core1_parked, true, __ATOMIC_RELEASE);

            // Park until sd_busy clears
            sleep_ms(1);
            // this is so much more stable than using wfe here.. 
            // wfe just doesn't always wake up for some reason, even with the doorbell.
            __wfi();

            __atomic_store_n(&core1_parked, false, __ATOMIC_RELEASE);
            continue;
        }
        // Wait for scanvideo to be ready for next scanline
        struct scanvideo_scanline_buffer *scanline_buffer =
            scanvideo_begin_scanline_generation(false);

        if (!scanline_buffer) {
            // no scanline ready right now; allow sd_busy to be observed quickly
            // __wfe();
            continue;
        }

        uint32_t frame_num = scanvideo_frame_number(scanline_buffer->scanline_id);

        // Only update frame pointer when a new frame is ready
        if (frame_num != last_frame_num) {
            last_frame_num = frame_num;
            y = 0;
#if SKIP_FRAMES
            fps_skip_counter++;
            if (fps_skip_counter == fps_divider) {
#endif
                // Grab newest ready buffer (and clear mailbox)
                framebuffer_t *next = __atomic_exchange_n(&ready_fb, NULL, __ATOMIC_ACQUIRE);

                if (next) {
                    // If somehow it isn't READY, ignore (safety)
                    if (__atomic_load_n(&next->state, __ATOMIC_ACQUIRE) == FB_READY) {
                        framebuffer_t *old = __atomic_load_n(&front_fb, __ATOMIC_RELAXED);
                        __atomic_store_n(&front_fb, next, __ATOMIC_RELEASE);


                        // States: new is displayed, old becomes free
                        __atomic_store_n(&next->state, FB_DISPLAYED, __ATOMIC_RELEASE);
                        __atomic_store_n(&old->state,  FB_FREE,      __ATOMIC_RELEASE);

                        __atomic_store_n(&freed_fb, old, __ATOMIC_RELEASE);
                        __sev();
                    }
                }
#if SKIP_FRAMES
            }
#endif
#if ENABLE_FRAME_DEBUGGING
            // Only count a frame if we swapped or at least reached a new video frame
            fps_counter++;

            uint32_t now = time_us_32();
            if (now - fps_last_time >= 1000000) { // 1 second
                printf("FPS: %lu\n", fps_counter);
                fps_counter = 0;
                fps_last_time = now;
            }
#endif
            scanline_count = 0;
#if ENABLE_INTERLACING
            interlacing_field = !interlacing_field;
#endif
#if ADD_TOP_PADDING
            top_padding_counter = 0;
#endif
            // At the start of each frame, set the correct backlight for non-BFI/phosphor modes.
            // BFI and Phosphor manage the backlight themselves per-scanline below.
            // This also handles restoring normal brightness when those modes are disabled.
            {
                bool crt_guard = __atomic_load_n(&gb_active, __ATOMIC_ACQUIRE)
                              && run_mode != MODE_POWERSAVE
                              && !__atomic_load_n(&show_gui, __ATOMIC_ACQUIRE);
                bool strobing = (crt_mode == CRT_BFI || crt_mode == CRT_BOTH
                              || crt_mode == CRT_PHOSPHOR) && crt_guard;
                if (!strobing) {
                    bool sl = (crt_mode == CRT_SCANLINES) && crt_guard;
                    lcd_set_pwm_direct(sl ? crt_compensated_brightness(lcd_led_duty_cycle)
                                         : lcd_led_duty_cycle);
                }
                // Phosphor: reset decay step every 2 display frames (60Hz cycle).
                disp_frame_count++;
                if (crt_mode == CRT_PHOSPHOR && crt_guard && (disp_frame_count % 2 == 0)) {
                    phosphor_step = 0;
                }
            }
        }
        // Render scanline
        framebuffer_t *fb = __atomic_load_n(&front_fb, __ATOMIC_ACQUIRE);

        // Common CRT guard: BFI and scanlines both require GB gameplay, no power save, no GUI.
        bool crt_active = __atomic_load_n(&gb_active, __ATOMIC_ACQUIRE)
                       && (run_mode != MODE_POWERSAVE)
                       && !__atomic_load_n(&show_gui, __ATOMIC_ACQUIRE);

        // BFI/Phosphor: fire at every toggle boundary (every BFI_SCANLINES_PER_TOGGLE scanlines).
        if ((scanline_count % BFI_SCANLINES_PER_TOGGLE == 0) && crt_active) {
            bool bright = ((scanline_count / BFI_SCANLINES_PER_TOGGLE) % 2 == 0);
            uint8_t comp = crt_compensated_brightness(lcd_led_duty_cycle);

            if (crt_mode == CRT_BFI || crt_mode == CRT_BOTH) {
                // Hard BFI: full bright or full dark (CRT_STROBE_PCT = 0).
                lcd_set_pwm_direct(bright ? comp : comp * CRT_STROBE_PCT / 100);
            } else if (crt_mode == CRT_PHOSPHOR) {
                // Phosphor: bright phases follow exponential decay curve; dark phases = 0.
                if (bright) {
                    lcd_set_pwm_direct(comp * phosphor_levels[phosphor_step % 4] / 100);
                    phosphor_step++;
                } else {
                    lcd_set_pwm_direct(0);
                }
            }
        }

        // Scanlines active for CRT_SCANLINES and CRT_BOTH modes only.
        // Even scanlines = content, odd = black. Y advances on content lines.
        // Without scanlines: both scanlines show content (software line doubling),
        // y advances on odd scanlines (after second copy of each row).
        bool scanlines_active = (crt_mode == CRT_SCANLINES || crt_mode == CRT_BOTH)
                             && crt_active;
        bool is_even_scanline = (scanline_count % 2 == 0);
        bool is_content_scanline = scanlines_active ? is_even_scanline : true;
        bool advance_y = scanlines_active ? is_even_scanline : !is_even_scanline;

        render_scanline(scanline_buffer, (y < LCD_HEIGHT)
        && is_content_scanline
        && (!ADD_TOP_PADDING || (top_padding_counter > TOP_PADDING))
        ? (__atomic_load_n(&show_gui, __ATOMIC_ACQUIRE) ? lvgl_fb[y] : fb->data[y]) : black_fb);

        scanvideo_end_scanline_generation(scanline_buffer);

        if (advance_y && (!ADD_TOP_PADDING || (top_padding_counter > TOP_PADDING))) {
            y++;
        }
#if ADD_TOP_PADDING
        top_padding_counter++;
#endif
        scanline_count++;
    }
}

// MARK: - CORE1 DOORBELL SETUP
#define ENABLE_DOORBELL 1
static uint8_t g_core1_db = 0xFF;
bool doorbell_setup = false;

// ISR runs on CORE1 when CORE0 rings the doorbell
static void __isr core1_doorbell_isr(void) {
    multicore_doorbell_clear_current_core(g_core1_db);
}

static inline void core1_doorbell_setup() {
    g_core1_db = multicore_doorbell_claim_unused((1<<NUM_CORES) -1, true);

    uint irq = multicore_doorbell_irq_num(g_core1_db); // resolves to SIO_IRQ_BELL(_NS) as needed by SDK
    irq_set_exclusive_handler(irq, core1_doorbell_isr);
    irq_set_enabled(irq, true);
    doorbell_setup = true;
}

static inline void core1_doorbell_free(void) {
    if (!doorbell_setup) return;
    if (g_core1_db == 0xFF) { // never successfully claimed
        doorbell_setup = false;
        return;
    }

    uint irq = multicore_doorbell_irq_num(g_core1_db);

    // stop ISR first
    irq_set_enabled(irq, false);

    // clear any latched state for this core
    multicore_doorbell_clear_current_core(g_core1_db);

    // optional cleanup (fine to omit if you prefer)
    irq_remove_handler(irq, core1_doorbell_isr);
    // if your SDK doesn't have irq_remove_handler():
    // irq_set_exclusive_handler(irq, NULL);

    // "free" the doorbell claim (the SDK name is unclaim)
    multicore_doorbell_unclaim(g_core1_db, (1u << NUM_CORES) - 1u);

    g_core1_db = 0xFF;
    doorbell_setup = false;
}

// MARK: - MAIN CORE1 LOOP
_Noreturn
void main_core1(void) {
#if ENABLE_DOORBELL
    if (!doorbell_setup) core1_doorbell_setup();
#endif
    #if !PICO_SCANVIDEO_ENABLE_DEN_PIN
    #warning "We'll take this out at some point"
    gpio_write(GPIO_DPI_DEN, 1);
    #endif
    render_loop();
    
    HEDLEY_UNREACHABLE();
}

void fb_deinit(void)
{
    if (fb0) { free(fb0); fb0 = NULL; }
    if (fb1) { free(fb1); fb1 = NULL; }
    if (fb2) { free(fb2); fb2 = NULL; }

    front_fb = NULL;
    write_fb = NULL;
    free_fb  = NULL;
    __atomic_store_n(&ready_fb, (framebuffer_t *)NULL, __ATOMIC_RELAXED);
}

static framebuffer_t *fb_alloc_zeroed(void)
{
    framebuffer_t *p = (framebuffer_t *)malloc(sizeof(framebuffer_t));
    if (!p) return NULL;
    memset(p, 0, sizeof(*p));
    return p;
}

static bool lvgl_alloc_buffers() {
    // Full-screen framebuffer for what gets displayed (RGB565)
    if (!lvgl_fb) {
        // Allocate as a flat block, then alias as [ver][hor]
        void *p = malloc(lvgl_fb_bytes);
        if (!p) return false;

        memset(p, 0, lvgl_fb_bytes);
        lvgl_fb = (uint16_t (*)[LCD_WIDTH])p;   // LCD_WIDTH must == hor_res
    }

    // LVGL draw buffer (partial buffer, LVGL flushes chunks)
    if (!lv_buf1) {
        lv_buf1 = (lv_color_t *)malloc(LVGL_BUF1_BYTES);
        if (!lv_buf1) return false;

        memset(lv_buf1, 0, LVGL_BUF1_BYTES);
    }

    return true;
}

// Optional cleanup (call on shutdown if you want; not required)
static void lvgl_free_buffers(void) {
    if (lvgl_fb) {
        free((void*)lvgl_fb);
        lvgl_fb = NULL;
    }
    if (lv_buf1) {
        free(lv_buf1);
        lv_buf1 = NULL;
    }
}

void fb_init(void)
{
    fb_deinit();
    black_fb_deinit();

    fb0 = fb_alloc_zeroed();
    fb1 = fb_alloc_zeroed();
    fb2 = fb_alloc_zeroed();

    if (!fb0 || !fb1 || !fb2) {
        // Cleanup partial allocs
        fb_deinit();
        printf("E fb_init: malloc failed (need %u bytes each)\n",
               (unsigned)sizeof(framebuffer_t));
        return;
    }

    // Initial states
    fb0->state = FB_DISPLAYED;
    fb1->state = FB_FREE;
    fb2->state = FB_FREE;


    front_fb = fb0;   // core1 starts by displaying fb0
    write_fb = fb1;   // core0 starts writing fb1
    free_fb  = fb2;   // spare free buffer

    lvgl_alloc_buffers();
    
    __atomic_store_n(&ready_fb, (framebuffer_t *)NULL, __ATOMIC_RELAXED);

    black_fb_init();
}

// MARK: - DPI SETUP
// Must be done before sd and i2s init
void setup_dpi() {
    fb_init();

    scanvideo_setup(&VGA_MODE);
    scanvideo_timing_enable(true);
}

static inline uint16_t *raw_scanline_prepare(struct scanvideo_scanline_buffer *dest, uint width) {
    assert(width >= 3);
    assert(width % 2 == 0);
    // +1 for the black pixel at the end, -3 because the program outputs n+3 pixels.
    dest->data[0] = COMPOSABLE_RAW_RUN | (width - 3 << 16);
    // After user pixels, 1 black pixel then discard remaining FIFO data
    dest->data[width / 2 + 2] = 0x0000u | (COMPOSABLE_EOL_ALIGN << 16);
    dest->data_used = width / 2 + 2;
    assert(dest->data_used <= dest->data_max);
    return (uint16_t *) &dest->data[1];
}

static inline void raw_scanline_finish(struct scanvideo_scanline_buffer *dest) {
    // Need to pivot the first pixel with the count so that PIO can keep up
    // with its 1 pixel per 2 clocks
    uint32_t first = dest->data[0];
    uint32_t second = dest->data[1];
    dest->data[0] = (first & 0x0000ffffu) | ((second & 0x0000ffffu) << 16);
    dest->data[1] = (second & 0xffff0000u) | ((first & 0xffff0000u) >> 16);
    dest->status = SCANLINE_OK;
}


// MARK: RENDER SCANLINE
void render_scanline(struct scanvideo_scanline_buffer *dest, const uint16_t *fb) {
    uint16_t *colour_buf = raw_scanline_prepare(dest, VGA_MODE.width); // VGA_MODE.width = 320

#if ENABLE_V_SCANLINES
    for (int x = 0; x < LCD_WIDTH; x++) {
        colour_buf[x * 2]     = fb[x];     // original pixel
        colour_buf[x * 2 + 1] = 0x0000;    // black pixel
    }
#else
    memcpy(colour_buf, fb, VGA_MODE.width * sizeof(uint16_t));
#endif

    raw_scanline_finish(dest);
}

void scanvideo_display_enable(bool enable) {
    gpio_set_dir(GPIO_DPI_DEN, GPIO_OUT);
    gpio_write(GPIO_DPI_DEN, enable);

    sleep_ms(1);
    if (enable) {
        gpio_set_function(GPIO_DPI_DEN, GPIO_FUNC_PIO0);
    }
}

#if ENABLE_LCD
// MARK: CORE0 LCD RETRIEVE LINE
void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[LCD_WIDTH],
                   const uint_fast8_t line)
{
    // Write THIS line into the current write buffer (core0-only pointer)
    uint16_t *dst = write_fb->data[line];

#if PEANUT_FULL_GBC_SUPPORT
    if (gb->cgb.cgbMode) {
        if (manual_palette_selected == -1) {
            refresh_cgb_palette_cache(gb);
            const uint8_t *src = pixels;
            uint16_t *out = dst;
            for (unsigned int x = 0; x < LCD_WIDTH; x += 8) {
                out[0] = cgb_palette_rgb565_cache[src[0] & 0x3F];
                out[1] = cgb_palette_rgb565_cache[src[1] & 0x3F];
                out[2] = cgb_palette_rgb565_cache[src[2] & 0x3F];
                out[3] = cgb_palette_rgb565_cache[src[3] & 0x3F];
                out[4] = cgb_palette_rgb565_cache[src[4] & 0x3F];
                out[5] = cgb_palette_rgb565_cache[src[5] & 0x3F];
                out[6] = cgb_palette_rgb565_cache[src[6] & 0x3F];
                out[7] = cgb_palette_rgb565_cache[src[7] & 0x3F];
                src += 8;
                out += 8;
            }
        } else {
            // Force 4-color manual palette regardless of CGB palette number / OAM mark
            const uint8_t *src = pixels;
            uint16_t *out = dst;
            for (unsigned x = 0; x < LCD_WIDTH; x++) {
                uint8_t p = *src++;
                uint8_t row = (p & 0x20) ? 1 : 2;  // sprite->1, bg->2 (pick whatever you want)
                *out++ = (*palette)[row][p & LCD_COLOUR];
            }
        }
    } else
#endif
    {
        const uint16_t (*pal)[4] = *palette;
        const uint8_t *src = pixels;
        uint16_t *out = dst;
        for (unsigned int x = 0; x < LCD_WIDTH; x += 8) {
            uint8_t p0 = src[0], p1 = src[1], p2 = src[2], p3 = src[3];
            uint8_t p4 = src[4], p5 = src[5], p6 = src[6], p7 = src[7];
            out[0] = pal[(p0 & LCD_PALETTE_ALL) >> 4][p0 & LCD_COLOUR];
            out[1] = pal[(p1 & LCD_PALETTE_ALL) >> 4][p1 & LCD_COLOUR];
            out[2] = pal[(p2 & LCD_PALETTE_ALL) >> 4][p2 & LCD_COLOUR];
            out[3] = pal[(p3 & LCD_PALETTE_ALL) >> 4][p3 & LCD_COLOUR];
            out[4] = pal[(p4 & LCD_PALETTE_ALL) >> 4][p4 & LCD_COLOUR];
            out[5] = pal[(p5 & LCD_PALETTE_ALL) >> 4][p5 & LCD_COLOUR];
            out[6] = pal[(p6 & LCD_PALETTE_ALL) >> 4][p6 & LCD_COLOUR];
            out[7] = pal[(p7 & LCD_PALETTE_ALL) >> 4][p7 & LCD_COLOUR];
            src += 8;
            out += 8;
        }
    }

    // End of frame: publish this buffer as READY
    if (line == (LCD_HEIGHT - 1)) {

        // 1) Pull any buffer core1 just freed (so we know what's actually free)
        framebuffer_t *ret = __atomic_exchange_n(&freed_fb, NULL, __ATOMIC_ACQUIRE);
        if (ret) {
            // Safety: ensure it's marked free (core1 should have done this already)
            __atomic_store_n(&ret->state, FB_FREE, __ATOMIC_RELEASE);
            free_fb = ret;
        }

        // 2) Publish finished frame (drop older if still pending)
        __atomic_store_n(&write_fb->state, FB_READY, __ATOMIC_RELEASE);

        framebuffer_t *prev_pending =
            __atomic_exchange_n(&ready_fb, write_fb, __ATOMIC_ACQ_REL);

        if (prev_pending) {
            // We are dropping it (it never got displayed) — reclaim it
            __atomic_store_n(&prev_pending->state, FB_FREE, __ATOMIC_RELEASE);

            // Optional: keep it as the free buffer candidate
            free_fb = prev_pending;
        }

        __sev();

        // 3) Rotate to a free buffer for the next frame
        if (free_fb && (__atomic_load_n(&free_fb->state, __ATOMIC_ACQUIRE) == FB_FREE)) {
            framebuffer_t *tmp = write_fb;
            write_fb = free_fb;
            free_fb  = tmp;     // old write_fb becomes our spare candidate
            // Optional: mark new write_fb as "in progress" if you add such a state
        } else {
            // No free buffer available -> drop next frame by reusing write_fb
            // Optional: drop counter here
        }

        // framebuffer_t *front_fb_local = __atomic_load_n(&front_fb, __ATOMIC_ACQUIRE);
        // framebuffer_t *write_fb_local = __atomic_load_n(&write_fb, __ATOMIC_ACQUIRE);
        // framebuffer_t *freed_fb_local  = __atomic_load_n(&free_fb,  __ATOMIC_ACQUIRE);
        // if (front_fb_local == write_fb_local || front_fb_local == freed_fb_local || write_fb_local == freed_fb_local) {
        //     printf("FB ptrs: front=%p write=%p free=%p\n", front_fb_local, write_fb_local, freed_fb_local);
        //     printf("E: framebuffer alias detected!\n");
        // }

    }
}
#endif


// MARK: - SPI
void init_spi9_gpio() {
    gpio_write(IOX_LCD_nCS, 1);  // Idle high

    gpio_init(GPIO_SPI0_SCK);
    gpio_set_dir(GPIO_SPI0_SCK, GPIO_OUT);
    gpio_write(GPIO_SPI0_SCK, 0); // Idle low

    gpio_init(GPIO_SPI0_MOSI);
    gpio_set_dir(GPIO_SPI0_MOSI, GPIO_OUT);
    gpio_write(GPIO_SPI0_MOSI, 0);  // Idle low
}

// Function to initialize the SPI bus
void init_spi_lcd() {
    init_spi9_gpio();
}
#define RST_ON 0
#define RST_OFF 1
void lcd_power_on_reset() {
    #if !PICO_SCANVIDEO_ENABLE_DEN_PIN
    gpio_set_function(GPIO_DPI_DEN, GPIO_FUNC_SIO);
	gpio_set_dir(GPIO_DPI_DEN, true);
    gpio_pull_up(GPIO_DPI_DEN);
    gpio_write(GPIO_DPI_DEN, 0);
    #endif

    gpio_write(IOX_LCD_nRST, RST_ON);
    gpio_write(IOX_LCD_nCS, 1);

    sleep_ms(1);
    gpio_write(IOX_LCD_nRST, RST_OFF);
}

void spi9_write_bitbang(uint8_t dc, uint8_t data) {
    gpio_write(IOX_LCD_nCS, 0);  // Start transmission

    uint16_t out = (dc ? 0x100 : 0x000) | data; // 9 bits: MSB is DC bit

    for (int i = 8; i >= 0; i--) {
        gpio_write(GPIO_SPI0_SCK, 0);                       // Clock low
        gpio_write(GPIO_SPI0_MOSI, (out >> i) & 0x01);        // Set data bit
        sleep_us(1);                                // Setup time
        gpio_write(GPIO_SPI0_SCK, 1);                       // Clock high
        sleep_us(1);                                // Hold time
    }

    gpio_write(IOX_LCD_nCS, 1);  // End transmission
}

void writecommand(uint8_t cmd) {
    spi9_write_bitbang(0, cmd);
}

void writedata(uint8_t data) {
    spi9_write_bitbang(1, data);
}

// disea config:
void lcd_config() {
    writecommand(0x11); // Exit Sleep just in case watchdog reset while sleeping

    writecommand(0xC0); writedata(0x14); writedata(0x14);
    writecommand(0xC1); writedata(0x66); // VGH = 4*VCI   VGL = -4*VCI

    writecommand(0xC5); writedata(0x00); writedata(0x43); writedata(0x80);

    writecommand(0xB0); writedata(0x00); // RGB
    writecommand(0xB1); writedata(0xA0);
    writecommand(0xB4); writedata(0x02);
    writecommand(0xB6); writedata(0x32); writedata(0x02); // RGB

    writecommand(0x36); writedata(0x48);
    writecommand(0x3A); writedata(0x55); // 55  66

    writecommand(0x21); writedata(0x00); // IPS

    writecommand(0xE9); writedata(0x00);
    writecommand(0xF7); writedata(0xA9); writedata(0x51); writedata(0x2C); writedata(0x82);

    writecommand(0xE0);
    writedata(0x00); writedata(0x07); writedata(0x0C); writedata(0x03); writedata(0x10);
    writedata(0x06); writedata(0x35); writedata(0x37); writedata(0x4C); writedata(0x01);
    writedata(0x0B); writedata(0x08); writedata(0x2E); writedata(0x34); writedata(0x0F);

    writecommand(0xE1);
    writedata(0x00); writedata(0x0E); writedata(0x14); writedata(0x04); writedata(0x12);
    writedata(0x06); writedata(0x37); writedata(0x33); writedata(0x4A); writedata(0x06);
    writedata(0x0F); writedata(0x0C); writedata(0x2E); writedata(0x31); writedata(0x0F);

    writecommand(0x11);
    sleep_ms(1);
    writecommand(0x29);
    sleep_ms(1);
    writecommand(0x2C);
}

void sleep_lcd() {
    init_spi_lcd();
    gpio_write(IOX_LCD_nCS, 0);  // Start transmission
    writecommand(0x10); // Enter Sleep
    gpio_write(IOX_LCD_nCS, 1);  // End transmission

    sleep_ms(1);
    gpio_set_function(GPIO_SPI0_SCK, GPIO_FUNC_SPI);
    gpio_set_function(GPIO_SPI0_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(GPIO_SPI0_MISO, GPIO_FUNC_SPI);
}
void wake_lcd() {
    init_spi_lcd();
    gpio_write(IOX_LCD_nCS, 0);  // Start transmission
    writecommand(0x11); // Exit Sleep
    gpio_write(IOX_LCD_nCS, 1);  // End transmission

    sleep_ms(1);
    gpio_set_function(GPIO_SPI0_SCK, GPIO_FUNC_SPI);
    gpio_set_function(GPIO_SPI0_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(GPIO_SPI0_MISO, GPIO_FUNC_SPI);
}

// MARK: - WASH OUT
uint8_t wash_out_level = 0;
static inline void washed_desaturate_level(uint8_t *r, uint8_t *g, uint8_t *b, uint8_t level) {
    // level = 0   -> no change (full vibrant)
    // level = 255 -> full grayscale
    if (level == 0) return;

    uint8_t gray = (*r + *g + *b) / 3;

    *r = (*r * (255 - level) + gray * level) / 255;
    *g = (*g * (255 - level) + gray * level) / 255;
    *b = (*b * (255 - level) + gray * level) / 255;
}

// MARK: Shift GBC Color
uint16_t shift_components(uint16_t pixel) {
    // Extract components
    uint8_t blue  =  pixel        & 0x1F;   // 5 bits
    uint8_t green = (pixel >> 5)  & 0x1F;   // 5 bits (GBC actually stores 5 here, not 6)
    uint8_t red   = (pixel >> 10) & 0x1F;   // 5 bits

    // Scale to 8-bit
    uint8_t r8 = (red   << 3) | (red   >> 2);
    uint8_t g8 = (green << 3) | (green >> 2);
    uint8_t b8 = (blue  << 3) | (blue  >> 2);

    washed_desaturate_level(&r8, &g8, &b8, wash_out_level);

    // Pack into RGB565
    return ((r8 & 0xF8) << 8) | ((g8 & 0xFC) << 3) | (b8 >> 3);
}

#if PEANUT_FULL_GBC_SUPPORT
static inline void refresh_cgb_palette_cache(struct gb_s *gb) {
    if (!cgb_palette_cache_valid || cgb_palette_wash_cache != wash_out_level) {
        for (uint8_t i = 0; i < 0x40; i++) {
            cgb_palette_rgb565_cache[i] = shift_components(gb->cgb.fixPalette[i]);
        }
        gb->cgb.paletteDirtyMask = 0;
        cgb_palette_wash_cache = wash_out_level;
        cgb_palette_epoch_cache = gb->cgb.paletteEpoch;
        cgb_palette_cache_valid = true;
        return;
    }

    if (cgb_palette_epoch_cache == gb->cgb.paletteEpoch) {
        return;
    }

    uint64_t dirty = gb->cgb.paletteDirtyMask;
    if (dirty == 0) {
        cgb_palette_epoch_cache = gb->cgb.paletteEpoch;
        return;
    }

    while (dirty) {
        uint8_t i = (uint8_t)__builtin_ctzll(dirty);
        cgb_palette_rgb565_cache[i] = shift_components(gb->cgb.fixPalette[i]);
        dirty &= (dirty - 1);
    }

    gb->cgb.paletteDirtyMask = 0;
    cgb_palette_epoch_cache = gb->cgb.paletteEpoch;
}
#endif

// MARK: RGB TO BGR
uint16_t make_pixel_bgr(uint8_t r, uint8_t g, uint8_t b) {
    return ((b & 0x1F) << 11) | ((g & 0x3F) << 5) | (r & 0x1F);
}
static inline uint16_t rgb565_to_bgr565(uint16_t rgb) {
    // Extract R, G, B from RGB565
    uint8_t r = rgb & 0x1F;           // bits 0–4
    uint8_t g = (rgb >> 5) & 0x3F;    // bits 5–10
    uint8_t b = (rgb >> 11) & 0x1F;   // bits 11–15

    // Re-pack into BGR565 format
    return (r << 11) | (g << 5) | b;
}





// #define LVGL_WIDTH  320
// #define LVGL_HEIGHT 320

// static lv_color_t lvgl_buf1[LVGL_WIDTH * 20]; // partial buffer: 20 rows
// static lv_color_t lvgl_buf2[LVGL_WIDTH * 20]; // second buf for double buffering
// static lv_disp_draw_buf_t draw_buf;

// static void my_lvgl_flush(lv_disp_drv_t *drv,
//                           const lv_area_t *area,
//                           lv_color_t *color_p)
// {
//     int32_t x1 = area->x1;
//     int32_t y1 = area->y1;
//     int32_t x2 = area->x2;
//     int32_t y2 = area->y2;

//     for (int y = y1; y <= y2; y++) {
//         // Wait for previous scanline to finish if needed
//         while(__atomic_load_n(&lcd_line_busy, __ATOMIC_SEQ_CST))
//             tight_loop_contents();

//         // copy row from LVGL buffer into pixels_buffer
//         for (int x = x1; x <= x2; x++) {
//             uint16_t px = color_p->full; // already RGB565
//             pixels_buffer[x] = px;
//             color_p++;
//         }

//         // send to your pipeline (like lcd_draw_line does)
//         union core_cmd cmd;
//         cmd.cmd = CORE_CMD_LCD_LINE;
//         cmd.data = y;
//         __atomic_store_n(&lcd_line_busy, 1, __ATOMIC_SEQ_CST);
//         multicore_fifo_push_blocking(cmd.full);
//     }

//     lv_disp_flush_ready(drv);
// }


// void lvgl_init() {
//     // Initialize LVGL
//     lv_init();

//     // Initialize display buffer
//     lv_disp_draw_buf_init(&draw_buf, lvgl_buf1, lvgl_buf2, LVGL_WIDTH * 20);

//     // Initialize and register the display driver
//     static lv_disp_drv_t disp_drv;
//     lv_disp_drv_init(&disp_drv);
//     disp_drv.hor_res = LVGL_WIDTH;
//     disp_drv.ver_res = LVGL_HEIGHT;
//     disp_drv.flush_cb = my_lvgl_flush;
//     disp_drv.draw_buf = &draw_buf;
//     lv_disp_drv_register(&disp_drv);
// }

// void lvgl_test() {
//     lv_obj_t *list = lv_list_create(lv_scr_act());
//     lv_obj_set_size(list, 200, 200);
//     lv_obj_center(list);

//     lv_list_add_text(list, "ROMs");
//     lv_list_add_btn(list, NULL, "Pokemon Red");
//     lv_list_add_btn(list, NULL, "Zelda: Oracle of Ages");
//     lv_list_add_btn(list, NULL, "Tetris");
// }

// Replace with your panel's width/height

// static uint16_t lvgl_fb[DISP_HOR_RES][DISP_VER_RES];

// Allocate draw buffers (double buffer, partial height to save RAM)

// static lv_color_t lv_buf1[DISP_HOR_RES * LV_BUF_LINES];
// static lv_color_t lv_buf2[DISP_HOR_RES * LV_BUF_LINES];


static void lvgl_flush_cb(struct _lv_disp_drv_t * disp_drv, const lv_area_t * area, lv_color_t * color_p) {
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;
    int32_t x2 = area->x2;
    int32_t y2 = area->y2;

    scanvideo_wait_for_vblank();
    for (int y = y1; y <= y2; y++) {
        // Fill the region (x1..x2) with LVGL's color data
        // color_p holds the line data row by row
        for (int x = x1; x <= x2; x++) {
            lvgl_fb[y][x] = color_p->full;  // Adjust depending on your RGB565/other format
            color_p++;
        }
    }

    lv_disp_flush_ready(disp_drv);
}

void lvgl_render_loop() {
    static uint32_t last_frame_num = 0;
    static uint32_t y = 0;

    while (true) {
        // Wait for scanvideo to be ready for next scanline
        struct scanvideo_scanline_buffer *scanline_buffer = scanvideo_begin_scanline_generation(true);
        uint32_t frame_num = scanvideo_frame_number(scanline_buffer->scanline_id);

        // Only update frame pointer when a new frame is ready
        if (frame_num != last_frame_num) {
            last_frame_num = frame_num;
            y = 0;   
        }
        // Render scanline
        render_scanline(scanline_buffer, y < DISP_VER_RES ? lvgl_fb[y] : black_fb);

        scanvideo_end_scanline_generation(scanline_buffer);

        y++;
    }
}

_Noreturn
void lvgl_core1(void) {
    lvgl_render_loop();
    HEDLEY_UNREACHABLE();
}

static lv_disp_drv_t disp_drv;

void lvgl_setup(void)
{
    lv_init();

    __atomic_store_n(&show_gui, true, __ATOMIC_RELEASE);

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, lv_buf1, NULL, DISP_HOR_RES * LV_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_VER_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;

    lv_disp_drv_register(&disp_drv);
}

void lvgl_task_handler(void)
{
    lv_timer_handler();
    // call periodically, e.g., every 5–10ms
}

void lvgl_test(void) {
    // Create a container to hold UI
	lv_obj_t *cont = lv_obj_create(lv_scr_act());
	lv_obj_set_size(cont, DISP_HOR_RES, DISP_VER_RES);
	lv_obj_center(cont);

	// Title label
	lv_obj_t *title = lv_label_create(cont);
	lv_label_set_text(title, "Starlight Test UI");
	lv_obj_set_style_text_font(title, LV_FONT_DEFAULT, 0);
	lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

	// Slider
	// lv_obj_t *slider = lv_slider_create(cont);
	// lv_obj_set_width(slider, 120);
	// lv_obj_align(slider, LV_ALIGN_CENTER, 0, 0);
}

#endif // ENABLE_LCD
