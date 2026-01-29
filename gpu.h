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

#define ENABLE_SCANLINES 0
#define ENABLE_V_SCANLINES 0

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
static uint16_t black_fb[LCD_WIDTH] = {0}; // all black
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
// Alias backbuffer as 1D LVGL buffer
static lv_color_t *lv_buf1;

bool double_frame_needs_bfi = 0;
#define ENABLE_BFI 0
#define ENABLE_BACKLIGHT_STROBING 0
#define USE_DUMB_BACKLIGHT_STROBING 1
// Keep track of previous brightness level
static uint8_t strobe_brightness = 0;

#define ENABLE_INTERLACING ENABLE_SCANLINES && 0
bool interlacing_field = 1; // 0 = even, 1 = odd
#if !USE_DUMB_BACKLIGHT_STROBING
#define STROBE_DELAY_US   1500  // wait after vblank (settle)
#define STROBE_WIDTH_US   2000  // ON pulse length


// Forward declarations
int64_t backlight_on_callback(alarm_id_t id, void *user_data);
int64_t backlight_off_callback(alarm_id_t id, void *user_data);

int64_t backlight_on_callback(alarm_id_t id, void *user_data) {
    #warning "This might need fixing"
    increase_lcd_brightness(strobe_brightness); 
    // schedule OFF pulse after STROBE_WIDTH_US
    add_alarm_in_us(STROBE_WIDTH_US, backlight_off_callback, NULL, true);
    return 0;
}

int64_t backlight_off_callback(alarm_id_t id, void *user_data) {
    decrease_lcd_brightness(255); // off
    return 0; // no repeat
}

// Call this once per frame at vblank
void backlight_strobe_start(uint8_t duty_cycle) {
    strobe_brightness = duty_cycle;
    // Immediately OFF at frame start
    decrease_lcd_brightness(255);
    // Schedule ON pulse after delay
    add_alarm_in_us(STROBE_DELAY_US, backlight_on_callback, NULL, true);
}
#endif
uint16_t scanline_count = 0;

#define ADD_TOP_PADDING 1
#define TOP_PADDING 3
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

    while (true) {
        if (__atomic_load_n(&sd_busy, __ATOMIC_ACQUIRE)) {
            // Park core1
            __atomic_store_n(&core1_parked, true, __ATOMIC_RELEASE);

            // Park until sd_busy clears
            while (__atomic_load_n(&sd_busy, __ATOMIC_ACQUIRE)) {
                sleep_us(10);
                tight_loop_contents();
                __wfi();
            }

            __atomic_store_n(&core1_parked, false, __ATOMIC_RELEASE);
            continue;
        }
        // Wait for scanvideo to be ready for next scanline
        struct scanvideo_scanline_buffer *scanline_buffer =
            scanvideo_begin_scanline_generation(false);

        if (!scanline_buffer) {
            // no scanline ready right now; allow sd_busy to be observed quickly
            __wfe();
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
                        framebuffer_t *old = front_fb;
                        front_fb = next;

                        // States: new is displayed, old becomes free
                        __atomic_store_n(&next->state, FB_DISPLAYED, __ATOMIC_RELEASE);
                        __atomic_store_n(&old->state,  FB_FREE,      __ATOMIC_RELEASE);

                        // Hand the freed buffer back to core0 as its free_fb if you want
                        // (This part is optional; simplest is to let core0 manage free_fb itself.)
                        free_fb = old; // <-- only do this if free_fb is NOT used by core0 concurrently

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
#if ENABLE_SCANLINES
            scanline_count = 0;
#endif
#if ENABLE_INTERLACING
            interlacing_field = !interlacing_field;
#endif
#if ENABLE_BACKLIGHT_STROBING
#if USE_DUMB_BACKLIGHT_STROBING
            if (double_frame_needs_bfi) {
                strobe_brightness = lcd_led_duty_cycle;
                decrease_lcd_brightness(MAX_BRIGHTNESS);
            } else {
                increase_lcd_brightness(strobe_brightness);
            }
#else
            backlight_strobe_start(lcd_led_duty_cycle);
#endif
#endif
#if ENABLE_BACKLIGHT_STROBING || ENABLE_BFI
            double_frame_needs_bfi = !double_frame_needs_bfi;
#endif

#if ADD_TOP_PADDING
            top_padding_counter = 0;
#endif
        }
        #warning "GB Games don't have this issue... so the opposite happens on them"
        // Render scanline
        render_scanline(scanline_buffer, (y < LCD_HEIGHT) 
        && (!ENABLE_BFI || !double_frame_needs_bfi || gb.direct.frame_skip == 1) 
        && (!ENABLE_SCANLINES || (scanline_count % 2 == interlacing_field)) 
        && (!ADD_TOP_PADDING || (top_padding_counter > TOP_PADDING))
        ? front_fb->data[y] : black_fb);

        scanvideo_end_scanline_generation(scanline_buffer);

        if ((!ENABLE_SCANLINES || (scanline_count % 2 == interlacing_field))  
        && (!ADD_TOP_PADDING || (top_padding_counter > TOP_PADDING))) {
            y++;
        }
#if ADD_TOP_PADDING
        top_padding_counter++;
#endif
#if ENABLE_SCANLINES
        scanline_count++;
#endif
    }
}

// MARK: - CORE1 DOORBELL SETUP
static uint8_t g_core1_db = 0xFF;

// ISR runs on CORE1 when CORE0 rings the doorbell
static void __isr core1_doorbell_isr(void) {
    multicore_doorbell_clear_current_core(g_core1_db);
}

static inline void core1_doorbell_setup() {
    g_core1_db = multicore_doorbell_claim_unused((1<<NUM_CORES) -1, true);

    uint irq = multicore_doorbell_irq_num(g_core1_db); // resolves to SIO_IRQ_BELL(_NS) as needed by SDK
    irq_set_exclusive_handler(irq, core1_doorbell_isr);
    irq_set_enabled(irq, true);
}

// MARK: - MAIN CORE1 LOOP
_Noreturn
void main_core1(void) {
    core1_doorbell_setup();
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

void fb_init(void)
{
    fb_deinit();

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

    lvgl_fb = front_fb->data;
    lv_buf1 = (lv_color_t *)write_fb->data;
    
    __atomic_store_n(&ready_fb, (framebuffer_t *)NULL, __ATOMIC_RELAXED);
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
            for (unsigned int x = 0; x < LCD_WIDTH; x++)
                dst[x] = shift_components(gb->cgb.fixPalette[pixels[x]]);
        } else {
            // Force 4-color manual palette regardless of CGB palette number / OAM mark
            for (unsigned x = 0; x < LCD_WIDTH; x++) {
                uint8_t row = (pixels[x] & 0x20) ? 1 : 2;  // sprite->1, bg->2 (pick whatever you want)
                dst[x] = (*palette)[row][pixels[x] & LCD_COLOUR];
            }
        }
    } else
#endif
    {
        for (unsigned int x = 0; x < LCD_WIDTH; x++)
            dst[x] = (*palette)[(pixels[x] & LCD_PALETTE_ALL) >> 4][pixels[x] & 3];
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

        // 2) Publish the finished frame
        __atomic_store_n(&write_fb->state, FB_READY, __ATOMIC_RELEASE);
        __atomic_store_n(&ready_fb, write_fb, __ATOMIC_RELEASE);
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
#define DISP_HOR_RES 160
#define DISP_VER_RES 144

// static uint16_t lvgl_fb[DISP_HOR_RES][DISP_VER_RES];

// Allocate draw buffers (double buffer, partial height to save RAM)
#define LV_BUF_LINES DISP_VER_RES  // Number of lines per buffer chunk

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
	lv_obj_t *slider = lv_slider_create(cont);
	lv_obj_set_width(slider, 120);
	lv_obj_align(slider, LV_ALIGN_CENTER, 0, 0);
}

#endif // ENABLE_LCD