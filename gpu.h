const scanvideo_timing_t tft_timing_320x320_60 = {
    //pclk multiple of 2 in reference to system clock 150mhz
    .clock_freq      = 18.75 * 1000 * 1000,   // ↓ now 12 MHz (within your panel’s 20 MHz max)
    .h_active        = 320,
    .v_active        = 320,
    .h_front_porch   =   4,
    .h_pulse         =    500,
    .h_total         = 320 + 4 + 500 + 4,  // back porch = 20
    .h_sync_polarity =    0,
    .v_front_porch   =    2,
    .v_pulse         =    1,
    .v_total         = 320 + 2 + 1 + 2,    // back porch = 8
    .v_sync_polarity =    0,
    .enable_clock    =    1,
    .clock_polarity  =    0,
    .enable_den      =    1
};

extern const struct scanvideo_pio_program video_24mhz_composable;  // ← swap in 12 MHz
const scanvideo_mode_t tft_mode_320x320_60 = {
    .default_timing     = &tft_timing_320x320_60,
    .pio_program        = &video_24mhz_composable, 
    .width              = 320,
    .height             = 320,
    .xscale             = 1,
    .yscale             = 1,
    .yscale_denominator = 1
};
#define VGA_MODE tft_mode_320x320_60

static struct mutex frame_logic_mutex;

static void frame_update_logic();
static void scanline_update_logic();
static void render_scanline(struct scanvideo_scanline_buffer *dest, int core, const uint16_t *fb);
uint16_t shift_components(uint16_t pixel);

// MARK: - RENDER LOOP
// "Worker thread" for each core
void render_loop() {
    static uint32_t last_frame_num = 0;
    int core_num = get_core_num();
    printf("Rendering on core %d\n", core_num);
    while (true) {
        // Command processing from core 0
        union core_cmd cmd;
		cmd.full = multicore_fifo_pop_blocking();

        static uint16_t fb[LCD_WIDTH];

		switch(cmd.cmd) {
		case CORE_CMD_LCD_LINE:
#if PEANUT_FULL_GBC_SUPPORT
            if (gbc->cgb.cgbMode) {
                // user has not assigned palette.
                if (manual_palette_selected == -1) {
                    for(unsigned int x = 0; x < LCD_WIDTH; x++){
                        fb[x] = shift_components(gbc->cgb.fixPalette[pixels_buffer[x]]);
                    }
                } else {
                    for(unsigned int x = 0; x < LCD_WIDTH; x++){
                        fb[x] = palette[(pixels_buffer[x] & LCD_PALETTE_ALL) >> 4]
                                [pixels_buffer[x] & 3];
                    }
                }
            }
            else {
#endif
                for(unsigned int x = 0; x < LCD_WIDTH; x++){
                    fb[x] = palette[(pixels_buffer[x] & LCD_PALETTE_ALL) >> 4]
                            [pixels_buffer[x] & 3];
                }
#if PEANUT_FULL_GBC_SUPPORT
            }
#endif	

			break;
		case CORE_CMD_IDLE_SET:
            printf("Idle mode set to %d on core %d\n", cmd.data, core_num);
			continue;
		case CORE_CMD_NOP:
            printf("No operation on core %d\n", core_num);
            continue;
		default:
			continue;
		}

        // Scanline generation
        printf("Rendering scanline %d on core %d\n", cmd.data, core_num);
        struct scanvideo_scanline_buffer *scanline_buffer = scanvideo_begin_scanline_generation(true);
        printf("STARTING\n");
        mutex_enter_blocking(&frame_logic_mutex);
        uint32_t frame_num = scanvideo_frame_number(scanline_buffer->scanline_id);
        // Frame and scanline update logic within mutex
        if (frame_num != last_frame_num) {
            last_frame_num = frame_num;
            frame_update_logic();
        }
        scanline_update_logic();
        mutex_exit(&frame_logic_mutex);

        render_scanline(scanline_buffer, core_num, fb);

        // Release the rendered buffer into the wild
        scanvideo_end_scanline_generation(scanline_buffer);

        // lcd line no longer busy
        __atomic_store_n(&lcd_line_busy, 0, __ATOMIC_SEQ_CST);
    }
}

struct semaphore video_setup_complete;

// MARK: - MAIN CORE1 LOOP
_Noreturn
void main_core1(void) {
    mutex_init(&frame_logic_mutex);
    sem_init(&video_setup_complete, 0, 1);

    scanvideo_setup(&VGA_MODE);
    scanvideo_timing_enable(true);

    sem_release(&video_setup_complete);

    #warning "We'll take this out at some point"
    gpio_write(GPIO_DPI_DEN, 1);
    
    render_loop();
    
    HEDLEY_UNREACHABLE();
}

// MARK: FRAME LOGIC
void frame_update_logic() {
    // here i will need to let the frame set its position. by setting the black lines or idk...
}

// MARK: SCANLINE LOGIC
void scanline_update_logic() {
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
void render_scanline(struct scanvideo_scanline_buffer *dest, int core, const uint16_t *fb) {
    uint16_t *colour_buf = raw_scanline_prepare(dest, VGA_MODE.width); // VGA_MODE.width = 320

    for (int x = 0; x < LCD_WIDTH; x++) {  // LCD_WIDTH = 160
        uint16_t px = fb[x];
        colour_buf[2*x]     = px;  // left pixel
        colour_buf[2*x + 1] = px;  // right pixel (duplicate)
    }

    raw_scanline_finish(dest);
    dest->status = SCANLINE_OK;
}

#if ENABLE_LCD
// MARK: CORE0 LCD RETRIEVE LINE
void lcd_draw_line(struct gb_s *gb, const uint8_t pixels[LCD_WIDTH],
		   const uint_fast8_t line)
{
    union core_cmd cmd;

	/* Wait until previous line is sent. */
	while(__atomic_load_n(&lcd_line_busy, __ATOMIC_SEQ_CST))
		tight_loop_contents();

    // Reverse the order of pixel data
    uint8_t reversed_pixels[LCD_WIDTH];
    for (unsigned int i = 0; i < LCD_WIDTH; ++i)
    {
        reversed_pixels[i] = pixels[LCD_WIDTH - 1 - i];
    }

	memcpy(pixels_buffer, reversed_pixels, LCD_WIDTH);

	gbc = gb;
	// memcpy(pixels_buffer, pixels, LCD_WIDTH);

    /* Populate command. */
    cmd.cmd = CORE_CMD_LCD_LINE;
    cmd.data = line;

    __atomic_store_n(&lcd_line_busy, 1, __ATOMIC_SEQ_CST);
    multicore_fifo_push_blocking(cmd.full);
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
    gpio_set_function(GPIO_DPI_DEN, GPIO_FUNC_SIO);
	gpio_set_dir(GPIO_DPI_DEN, true);
    gpio_pull_up(GPIO_DPI_DEN);
    gpio_write(GPIO_DPI_DEN, 0);

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
