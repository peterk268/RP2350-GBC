#if ENABLE_LCD
// MARK: DPI TIMINGS
#define H_ACTIVE 320
#define H_PULSE 12
#define H_F_PORCH 50
#define H_B_PORCH 95

#define V_ACTIVE 320
#define V_PULSE 6
#define V_F_PORCH 10
#define V_B_PORCH 15

const scanvideo_timing_t tft_timing_320x320_60 = {
    .clock_freq      =  10 * 1000 * 1000,  // 15 MHz
    .h_active        =  H_ACTIVE,
    .v_active        =  V_ACTIVE,
    .h_front_porch   =  H_F_PORCH,
    .h_pulse         =  H_PULSE,
    .h_total         =  H_ACTIVE + H_F_PORCH + H_PULSE + H_B_PORCH,  // back porch = 20
    .h_sync_polarity =  1,
    .v_front_porch   =  V_F_PORCH,
    .v_pulse         =  V_PULSE,
    .v_total         =  V_ACTIVE + V_F_PORCH + V_PULSE + V_B_PORCH,    // back porch = 8
    .v_sync_polarity =  1,
    .enable_clock    =  1,
    .clock_polarity  =  1,
    .enable_den      =  1
};

extern const struct scanvideo_pio_program video_24mhz_composable;  // ← swap in 12 MHz
const scanvideo_mode_t tft_mode_320x320_60 = {
    .default_timing     = &tft_timing_320x320_60,
    .pio_program        = &video_24mhz_composable, 
    .width              = 160,
    .height             = 160,
    .xscale             = 2,
    .yscale             = 2,
    .yscale_denominator = 1
};
#define VGA_MODE tft_mode_320x320_60

#define ADD_BLACK_BAR 0

static struct mutex frame_logic_mutex;
static void frame_update_logic();
static void scanline_update_logic();
static void render_scanline(struct scanvideo_scanline_buffer *dest, const uint16_t *fb);
uint16_t shift_components(uint16_t pixel);
static uint16_t black_fb[LCD_WIDTH] = {0}; // all black
// MARK: - RENDER LOOP
// "Worker thread" for each core
static uint16_t fb[LCD_HEIGHT][LCD_WIDTH];
static uint16_t fb2[LCD_HEIGHT][LCD_WIDTH];

// Front buffer (being displayed)
static uint16_t (*front_fb)[LCD_WIDTH] = fb;

// Back buffer (being written by CPU/emulator)
static uint16_t (*back_fb)[LCD_WIDTH] = fb2;

static uint32_t y = 0;

void render_loop() {
    static uint32_t last_frame_num = 0;
    while (true) {
        // Wait for scanvideo to be ready for next scanline
        struct scanvideo_scanline_buffer *scanline_buffer = scanvideo_begin_scanline_generation(true);
        uint32_t frame_num = scanvideo_frame_number(scanline_buffer->scanline_id);

        // Only update frame pointer when a new frame is ready
        if (frame_num != last_frame_num) {
            last_frame_num = frame_num;
            // while(!__atomic_load_n(&lcd_fb_ready, __ATOMIC_SEQ_CST)) tight_loop_contents();
            y = 0;
        }
        int src_y = (y + LCD_HEIGHT - 1) % LCD_HEIGHT;
        // Render scanline
        render_scanline(scanline_buffer, (y < LCD_HEIGHT) ? front_fb[src_y] : black_fb);

        scanvideo_end_scanline_generation(scanline_buffer);

        y++;
    }
}

struct semaphore video_setup_complete;

// MARK: - MAIN CORE1 LOOP
_Noreturn
void main_core1(void) {
    sem_acquire_blocking(&video_setup_complete);

    #if !PICO_SCANVIDEO_ENABLE_DEN_PIN
    #warning "We'll take this out at some point"
    gpio_write(GPIO_DPI_DEN, 1);
    #endif
    render_loop();
    
    HEDLEY_UNREACHABLE();
}

// Must be done before sd and i2s init
void setup_dpi() {
    mutex_init(&frame_logic_mutex);
    sem_init(&video_setup_complete, 0, 1);

    scanvideo_setup(&VGA_MODE);
    scanvideo_timing_enable(true);

    sem_release(&video_setup_complete);
}

#define MIN_COLOR_RUN 3

int32_t single_color_scanline(uint32_t *buf, size_t buf_length, int width, uint32_t color16) {
    assert(buf_length >= 2);

    assert(width >= MIN_COLOR_RUN);
    // | jmp color_run | color | count-3 |  buf[0] =
    buf[0] = COMPOSABLE_COLOR_RUN | (color16 << 16);
    buf[1] = (width - MIN_COLOR_RUN) | (COMPOSABLE_RAW_1P << 16);
    // note we must end with a black pixel
    buf[2] = 0 | (COMPOSABLE_EOL_ALIGN << 16);

    return 3;
}

#define VISIBLE_HEIGHT 288
#define VERTICAL_OFFSET 2

#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 288

static inline uint16_t *raw_scanline_prepare(struct scanvideo_scanline_buffer *dest, uint width) {
    assert(width >= 3);
    assert(width % 2 == 0);
    // +1 for the black pixel at the end, -3 because the program outputs n+3 pixels.
    dest->data[0] = COMPOSABLE_RAW_RUN | (width + 1 - 3 << 16);
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

    memcpy(colour_buf, fb, LCD_WIDTH * sizeof(uint16_t));

    raw_scanline_finish(dest);
    dest->status = SCANLINE_OK;
}

#if ENABLE_LCD
uint fb_line = 0;
// MARK: CORE0 LCD RETRIEVE LINE
void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[LCD_WIDTH],
		   const uint_fast8_t line)
{
	gbc = gb;
#if PEANUT_FULL_GBC_SUPPORT
    if (gbc->cgb.cgbMode) {
        // user has not assigned palette.
        if (manual_palette_selected == -1) {
            for(unsigned int x = 0; x < LCD_WIDTH; x++){
                back_fb[fb_line][x] = shift_components(gbc->cgb.fixPalette[pixels[x]]);
            }
        } else {
            for(unsigned int x = 0; x < LCD_WIDTH; x++){
                back_fb[fb_line][x] = palette[(pixels[x] & LCD_PALETTE_ALL) >> 4]
                        [pixels[x] & 3];
            }
        }
    }
    else {
#endif
        for(unsigned int x = 0; x < LCD_WIDTH; x++){
            back_fb[fb_line][x] = palette[(pixels[x] & LCD_PALETTE_ALL) >> 4]
                    [pixels[x] & 3];
        }
#if PEANUT_FULL_GBC_SUPPORT
    }
#endif	

    fb_line++;
    if (fb_line >= LCD_HEIGHT) {
        fb_line = 0;
        // Swap front and back buffers atomically
        uint16_t (*tmp)[LCD_WIDTH] = front_fb;
        front_fb = back_fb;
        back_fb = tmp;
        __atomic_store_n(&lcd_fb_ready, 1, __ATOMIC_SEQ_CST);
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

// MARK: Shift GBC Color
uint16_t shift_components(uint16_t pixel) {
    // Green component: Extract (bits 5–10) -> 6 bits
    uint16_t green_mask = 0b0000011111100000; // Green mask (6 bits)
    uint16_t green = (pixel & green_mask) >> 5;

    // Shift the green component forward by 1, preserving the 6-bit range
    green = (green << 1) & 0b111111;  // Ensuring we stay within 6 bits (0-63)

    // Clear the old green bits in the original pixel
    pixel &= ~green_mask;

    // Insert the shifted green back into the pixel
    pixel |= (green << 5);

    // Red component: Extract (bits 11–15) -> 5 bits
    uint16_t red_mask = 0b1111100000000000; // Red mask (5 bits)
    uint16_t red = (pixel & red_mask) >> 11;

    // Shift the red component forward by 1, preserving the 5-bit range
    red = (red << 1) & 0b11111;  // Ensuring we stay within 5 bits (0-31)

    // Clear the old red bits in the original pixel
    pixel &= ~red_mask;

    // Insert the shifted red back into the pixel
    pixel |= (red << 11);

    // Blue component: Extract (bits 0–4) -> 5 bits
    uint16_t blue_mask = 0b0000000000011111; // Blue mask (5 bits)
    uint16_t blue = pixel & blue_mask;

    // Shift the blue component forward by 1, preserving the 5-bit range
    blue = (blue << 1) & 0b11111;  // Ensuring we stay within 5 bits (0-31)

    // Clear the old blue bits in the original pixel
    pixel &= ~blue_mask;

    // Insert the shifted blue back into the pixel
    pixel |= blue;

    return pixel;
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
#define DISP_HOR_RES 320
#define DISP_VER_RES 320

static void lvgl_flush_cb(lv_disp_t * disp, const lv_area_t * area, lv_color_t * color_p) {
    int32_t x1 = area->x1;
    int32_t y1 = area->y1;
    int32_t x2 = area->x2;
    int32_t y2 = area->y2;

    for (int y = y1; y <= y2; y++) {
        // Wait for LVGL to give scanline for this row
        struct scanvideo_scanline_buffer *scanline_buffer = scanvideo_begin_scanline_generation(true);

        uint16_t *dst = (uint16_t *)scanline_buffer->data;
        // Fill the region (x1..x2) with LVGL's color data
        // color_p holds the line data row by row
        for (int x = x1; x <= x2; x++) {
            dst[x] = color_p->full;  // Adjust depending on your RGB565/other format
            color_p++;
        }

        // If the rest of the scanline outside x1..x2 should stay black or unchanged,
        // fill dst[0..x1-1] and dst[x2+1..end] accordingly.

        scanvideo_end_scanline_generation(scanline_buffer);
    }

    lv_disp_flush_ready(disp);
}

// Allocate draw buffers (double buffer, partial height to save RAM)
#define LV_BUF_LINES 20  // Number of lines per buffer chunk

static lv_color_t lv_buf1[DISP_HOR_RES * LV_BUF_LINES];
static lv_color_t lv_buf2[DISP_HOR_RES * LV_BUF_LINES];

void lvgl_setup(void)
{
    lv_init();

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, lv_buf1, lv_buf2, DISP_HOR_RES * LV_BUF_LINES);

    static lv_disp_drv_t disp_drv;
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

#endif // ENABLE_LCD