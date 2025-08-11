#if USE_IPS_LCD
#warning "add ips set up here"
// #define DUAL_CORE_RENDER

#define USE_CUSTOM 1
#define USE_GPT 0
#define ENABLE_SPI 1

// #if !USE_CUSTOM
// #define VGA_MODE vga_mode_320x240_60
// extern const struct scanvideo_pio_program video_24mhz_composable;
// #else

// // MARK: - DONT TOUCH
// #if !USE_GPT
// const scanvideo_timing_t tft_timing_320x320_60 = {
//     //pclk multiple of 2 in reference to system clock 150mhz
//     .clock_freq      = 18.75 * 1000 * 1000,   // ↓ now 12 MHz (within your panel’s 20 MHz max)
//     .h_active        = 320,
//     .v_active        = 320,
//     .h_front_porch   =   4,
//     .h_pulse         =    500,
//     .h_total         = 320 + 4 + 500 + 4,  // back porch = 20
//     .h_sync_polarity =    0,
//     .v_front_porch   =    2,
//     .v_pulse         =    1,
//     .v_total         = 320 + 2 + 1 + 2,    // back porch = 8
//     .v_sync_polarity =    0,
//     .enable_clock    =    1,
//     .clock_polarity  =    0,
//     .enable_den      =    1
// };
// #else
// //chatgpt
// const scanvideo_timing_t tft_timing_320x320_60 = {
//     .clock_freq      = 10 * 1000 * 1000, // 12 MHz pixel clock (based on tCYC ≥ 50 ns)
//     .h_active        = 320,
//     .v_active        = 320,
//     .h_front_porch   = 10,
//     .h_pulse         = 2,
//     .h_total         = 342,
//     .h_sync_polarity = 0,
//     .v_front_porch   = 4,
//     .v_pulse         = 1,
//     .v_total         = 330,
//     .v_sync_polarity = 0,
//     .enable_clock    = 1,
//     .clock_polarity  = 0,
//     .enable_den      = 1
// };
// #endif
// extern const struct scanvideo_pio_program video_24mhz_composable;  // ← swap in 12 MHz
// const scanvideo_mode_t tft_mode_320x320_60 = {
//     .default_timing     = &tft_timing_320x320_60,
//     .pio_program        = &video_24mhz_composable, 
//     .width              = 320,
//     .height             = 320,
//     .xscale             = 1,
//     .yscale             = 1,
//     .yscale_denominator = 1
// };
// #define VGA_MODE tft_mode_320x320_60
// #endif

// uint16_t make_pixel_bgr(uint8_t r, uint8_t g, uint8_t b) {
//     return ((b & 0x1F) << 11) | ((g & 0x3F) << 5) | (r & 0x1F);
// }
// static inline uint16_t rgb565_to_bgr565(uint16_t rgb) {
//     // Extract R, G, B from RGB565
//     uint8_t r = rgb & 0x1F;           // bits 0–4
//     uint8_t g = (rgb >> 5) & 0x3F;    // bits 5–10
//     uint8_t b = (rgb >> 11) & 0x1F;   // bits 11–15

//     // Re-pack into BGR565 format
//     return (r << 11) | (g << 5) | b;
// }

// // IN SCAN LINE DO THIS: 
// //     uint16_t bgcolour = rgb565_to_bgr565(rgb_color);


// // MARK: - EXAMPLE CODE
// // https://github.com/raspberrypi/pico-playground/tree/master

// // to make sure only one core updates the state when the frame number changes
// // todo note we should actually make sure here that the other core isn't still rendering (i.e. all must arrive before either can proceed - a la barrier)
// static struct mutex frame_logic_mutex;

// static void frame_update_logic();
// static void render_scanline(struct scanvideo_scanline_buffer *dest, int core);

// // "Worker thread" for each core
// void render_loop() {
//     static uint32_t last_frame_num = 0;
//     int core_num = get_core_num();
//     printf("Rendering on core %d\n", core_num);
//     while (true) {
//         struct scanvideo_scanline_buffer *scanline_buffer = scanvideo_begin_scanline_generation(true);
//         mutex_enter_blocking(&frame_logic_mutex);
//         uint32_t frame_num = scanvideo_frame_number(scanline_buffer->scanline_id);
//         // Note that with multiple cores we may have got here not for the first
//         // scanline, however one of the cores will do this logic first before either
//         // does the actual generation
//         if (frame_num != last_frame_num) {
//             last_frame_num = frame_num;
//             frame_update_logic();
//         }
//         mutex_exit(&frame_logic_mutex);

//         render_scanline(scanline_buffer, core_num);

//         // Release the rendered buffer into the wild
//         scanvideo_end_scanline_generation(scanline_buffer);
//     }
// }

// struct semaphore video_setup_complete;

// void core1_func() {
//     sem_acquire_blocking(&video_setup_complete);
//     render_loop();
// }

// int vga_main(void) {
//     mutex_init(&frame_logic_mutex);
//     sem_init(&video_setup_complete, 0, 1);

//     // Core 1 will wait for us to finish video setup, and then start rendering
// #ifdef DUAL_CORE_RENDER
//     multicore_launch_core1(core1_func);
// #endif

//     scanvideo_setup(&VGA_MODE);
//     scanvideo_timing_enable(true);

//     sem_release(&video_setup_complete);
//     render_loop();
//     return 0;
// }
// void frame_update_logic() {
// }

// #define MIN_COLOR_RUN 3

// int32_t single_color_scanline(uint32_t *buf, size_t buf_length, int width, uint32_t color16) {
//     assert(buf_length >= 2);

//     assert(width >= MIN_COLOR_RUN);
//     // | jmp color_run | color | count-3 |  buf[0] =
//     buf[0] = COMPOSABLE_COLOR_RUN | (color16 << 16);
//     buf[1] = (width - MIN_COLOR_RUN) | (COMPOSABLE_RAW_1P << 16);
//     // note we must end with a black pixel
//     buf[2] = 0 | (COMPOSABLE_EOL_ALIGN << 16);

//     return 3;
// }

// #define VISIBLE_HEIGHT 288
// #define VERTICAL_OFFSET 2

// #define IMAGE_WIDTH 320
// #define IMAGE_HEIGHT 288

// static inline uint16_t *raw_scanline_prepare(struct scanvideo_scanline_buffer *dest, uint width) {
//     assert(width >= 3);
//     assert(width % 2 == 0);
//     // +1 for the black pixel at the end, -3 because the program outputs n+3 pixels.
//     dest->data[0] = COMPOSABLE_RAW_RUN | (width + 1 - 3 << 16);
//     // After user pixels, 1 black pixel then discard remaining FIFO data
//     dest->data[width / 2 + 2] = 0x0000u | (COMPOSABLE_EOL_ALIGN << 16);
//     dest->data_used = width / 2 + 2;
//     assert(dest->data_used <= dest->data_max);
//     return (uint16_t *) &dest->data[1];
// }

// static inline void raw_scanline_finish(struct scanvideo_scanline_buffer *dest) {
//     // Need to pivot the first pixel with the count so that PIO can keep up
//     // with its 1 pixel per 2 clocks
//     uint32_t first = dest->data[0];
//     uint32_t second = dest->data[1];
//     dest->data[0] = (first & 0x0000ffffu) | ((second & 0x0000ffffu) << 16);
//     dest->data[1] = (second & 0xffff0000u) | ((first & 0xffff0000u) >> 16);
//     dest->status = SCANLINE_OK;
// }


// void render_scanline(struct scanvideo_scanline_buffer *dest, int core) {
//     int l = scanvideo_scanline_number(dest->scanline_id);

//     if (l < VERTICAL_OFFSET || l >= (VERTICAL_OFFSET + IMAGE_HEIGHT)) {
//         // Outside the image area – black line
//         dest->data_used = single_color_scanline(dest->data, dest->data_max, VGA_MODE.width, 0x0000);
//     } else {
//         // Line within image area
//         // int line_in_image = l - VERTICAL_OFFSET;

//         // // Prepare the scanline for raw pixels
//         // uint16_t *colour_buf = raw_scanline_prepare(dest, VGA_MODE.width);

//         // // Get the pointer to the correct line of image data
//         // const uint16_t *src_line = &my_image_raw[line_in_image * IMAGE_WIDTH];

//         // // Copy the line to the scanline buffer
//         // memcpy(colour_buf, src_line, IMAGE_WIDTH * sizeof(uint16_t));

//         // // Finish the scanline
//         // raw_scanline_finish(dest);
//     }

//     dest->status = SCANLINE_OK;
// }


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
#else
/* Functions required for communication with the ILI9225. */
void mk_ili9225_set_rst(bool state)
{
	// lcd needs por
	#warning "This was done because of inverter, but please change back once hardware is changed"
	gpio_write(IOX_LCD_nRST, !state);
}

void mk_ili9225_set_rs(bool state)
{
	gpio_write(GPIO_SPI0_MISO, state);
}
bool iscs = false;
void mk_ili9225_set_cs(bool state)
{
	// this was done due to slowness of iox write
	if (!iscs) {
		gpio_write(IOX_LCD_nCS, 0);
		iscs = true;
	}
}

void mk_ili9225_set_led(bool state)
{
	gpio_write(GPIO_LCD_LED, state);
}

void mk_ili9225_spi_write16(const uint16_t *halfwords, size_t len)
{
	spi_write16_blocking(LCD_SPI, halfwords, len);
}

void mk_ili9225_delay_ms(unsigned ms)
{
	sleep_ms(ms);
}
#endif

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

#if ENABLE_LCD 
void core1_lcd_draw_line(const uint_fast8_t line)
{
	static uint16_t fb[LCD_WIDTH];

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

	// // Calculate the start line for the rotated display
    uint_fast8_t rotated_line = LCD_HEIGHT - line - 1;

#if USE_IPS_LCD
	#warning "ips: send out pixels"
#else
	// mk_ili9225_set_x(rotated_line + 16);
	// // mk_ili9225_set_x(line + 16);

	// mk_ili9225_write_pixels(fb, LCD_WIDTH);
#endif
	__atomic_store_n(&lcd_line_busy, 0, __ATOMIC_SEQ_CST);
}

// MARK: - SCALED DISPLAY LOGIC
/*
#if ENABLE_LCD 
void core1_lcd_draw_line(const uint_fast8_t line)
{
	// scaling pixels 2x horizontally
	static uint16_t scaled_pixels[LCD_WIDTH*DISPLAY_SCALE];

	// function to scale each pixel horizontally
	void scale_pixels(const uint8_t x, const uint16_t pixel) {
		const uint16_t starting_point = x * DISPLAY_SCALE;
		for (int j = 0; j < DISPLAY_SCALE; j++) {
			scaled_pixels[starting_point + j] = pixel;
		}
	}

	// Selecting color pallete based on GBC (full) or GB (limited) and calling `scale_pixels`
#if PEANUT_FULL_GBC_SUPPORT
	if (gbc->cgb.cgbMode) {
		for(uint8_t x = 0; x < LCD_WIDTH; x++) {
			// if we allow full gbc support and the game being played used cgbMode we fix the pallete with 64 colors.
			uint16_t pixel = gbc->cgb.fixPalette[pixels_buffer[x]];
			
			scale_pixels(x, pixel);
		}
	} 
	else {
#endif
		for(uint8_t x = 0; x < LCD_WIDTH; x++) {
			// if we are not using the full gbc support, ignore all the previous logic and just use limited pallete.
			// other case could be that we are supporting full gbc but the current game does not.
			uint16_t pixel = palette[(pixels_buffer[x] & LCD_PALETTE_ALL) >> 4]
				[pixels_buffer[x] & 3];
			
			scale_pixels(x, pixel);
		}
#if PEANUT_FULL_GBC_SUPPORT
	}
#endif	

	// here we are writing the same line 2x vertically.
	for (uint8_t x = 0; x < DISPLAY_SCALE; x++) {
		// Calculate the start line for the rotated and scaled display
		uint16_t newLine = ((LCD_HEIGHT - line - 1) * DISPLAY_SCALE) + x;

		// setting x starting point for write line
		mk_ili9225_set_x(newLine + 16);

		// writing scaled_pixels to display
		mk_ili9225_write_pixels(scaled_pixels, LCD_WIDTH*DISPLAY_SCALE);
	}

	// letting cpu core know that lcd line is not busy and can process the next
	__atomic_store_n(&lcd_line_busy, 0, __ATOMIC_SEQ_CST);
}
*/
_Noreturn
void main_core1(void)
{
	union core_cmd cmd;

#if USE_IPS_LCD
	#warning "ips: add ips init"
#else
	/* Initialise and control LCD on core 1. */
	// mk_ili9225_init();

	// /* Clear LCD screen. */
	// mk_ili9225_fill(0x0000);

	// /* Set LCD window to DMG size. */
	// mk_ili9225_fill_rect(31,16,LCD_WIDTH,LCD_HEIGHT,0x0000);
#endif
	// Sleep used for debugging LCD window.
	//sleep_ms(1000);

	/* Handle commands coming from core0. */
	while(1)
	{
		cmd.full = multicore_fifo_pop_blocking();
		switch(cmd.cmd)
		{
		case CORE_CMD_LCD_LINE:
			core1_lcd_draw_line(cmd.data);
			break;

		case CORE_CMD_IDLE_SET:
#if USE_IPS_LCD
			#warning "ips: idle the lcd, idk. tbh"
#else
			// mk_ili9225_display_control(true, cmd.data);
#endif
			break;

		case CORE_CMD_NOP:
		default:
			break;
		}
	}

	HEDLEY_UNREACHABLE();
}
#endif

#if ENABLE_LCD
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



#if ENABLE_LCD
#define IPS_LCD_WIDTH  320
#define IPS_LCD_HEIGHT 320
#define IMAGE_HEIGHT 288
#define IMAGE_WIDTH 320
#define DOTCLOCK_FREQ 15000000 // 15 MHz typical

static void lcd_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p);
static void lv_port_disp_init(void);

void lv_port_disp_init(void) {
    static lv_disp_draw_buf_t draw_buf;
    static lv_color_t buf1[IPS_LCD_WIDTH * 10]; // 10 lines buffer
    static lv_color_t buf2[IPS_LCD_WIDTH * 10];

    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, IPS_LCD_WIDTH * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    
    disp_drv.hor_res = IPS_LCD_WIDTH;
    disp_drv.ver_res = IPS_LCD_HEIGHT;
    disp_drv.flush_cb = lcd_flush_cb;
    disp_drv.draw_buf = &draw_buf;

    lv_disp_drv_register(&disp_drv);
}

static void lcd_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    // for (int y = area->y1; y <= area->y2; y++) {
    //     gpio_write(GPIO_DPI_HSYNC, 0); // Start of line
    //     for (int x = area->x1; x <= area->x2; x++) {
    //         // Extract RGB values from the LVGL color buffer
    //         drive_rgb_pins(color_p->full);

    //         // Toggle pixel clock
    //         gpio_write(GPIO_DPI_PCLK, 1);
    //         gpio_write(GPIO_DPI_PCLK, 0);

    //         color_p++;
    //     }
    //     gpio_write(GPIO_DPI_HSYNC, 1); // End of line
    // }

    // gpio_write(GPIO_DPI_VSYNC, 1); // Frame ready
    // gpio_write(GPIO_DPI_DEN, 1);   // Data enable

    // lv_disp_flush_ready(drv); // Notify LVGL that flush is complete
}

#warning "Implement this function in when we are reading for the ips lcd"
/*
static void dpi_draw_line(const uint_fast8_t line) {
	static uint16_t fb[LCD_WIDTH];

#if PEANUT_FULL_GBC_SUPPORT
 	if (gbc->cgb.cgbMode) {
 		for(unsigned int x = 0; x < LCD_WIDTH; x++){
			fb[x] = gbc->cgb.fixPalette[pixels_buffer[x]];
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

    // Copy pixel data into the framebuffer, centered vertically
    size_t vertical_offset = (IPS_LCD_HEIGHT - IMAGE_HEIGHT) / 2;
    // Calculate the actual line position for centering
    uint_fast8_t centered_line = vertical_offset + line;

	// Start the display at the offset if this is the first line == 0
	if (line == 0) {
		dpi_set_y(centered_line);
	} 
	if (line == LCD_HEIGHT) {
		new_frame();
	}

	// Scaling 2x vertically
	// Line outputted once with the 2x horizontal, then again on a new line for 2x vertical.
	for (int h = 0; h < DISPLAY_SCALE; h++) {
		// Output the line to the display
		for (int i = 0; i < LCD_WIDTH; i++) {
			// Scaling 2x horizontally
			for (int j = 0; j < DISPLAY_SCALE; j++) {
				drive_rgb_pins(fb[i]);

				// Simulate pixel clock pulse
				gpio_put(GPIO_DPI_PCLK, 1);
				gpio_put(GPIO_DPI_PCLK, 0);
			}
		}
		new_line();
	}


	__atomic_store_n(&lcd_line_busy, 0, __ATOMIC_SEQ_CST);
}
*/
// int main() {
//     stdio_init_all();
//     init_dpi_pins();
//     dpi_set_pixel_clock(DOTCLOCK_FREQ);

//     lv_init();
//     lv_port_disp_init();

//     while (1) {
//         lv_task_handler();
//         sleep_ms(5);
//     }
// }

#endif



//TMDS
// Define the bit mapping for the TMDS lanes
// static const int lane_to_output_bit[3] = {4, 6, 0}; // Mapping D0, D1, D2

// // Assign clock pair (CLK) to GPIO 14 & 15
// hstx_ctrl_hw->bit[2] = HSTX_CTRL_BIT0_CLK_BITS;           // CLK+
// hstx_ctrl_hw->bit[3] = HSTX_CTRL_BIT0_CLK_BITS | HSTX_CTRL_BIT0_INV_BITS; // CLK-

// // Assign TMDS lanes to GPIO pins
// for (uint lane = 0; lane < 3; ++lane) {
//     int bit = lane_to_output_bit[lane];
//     uint32_t lane_data_sel_bits =
//         (lane * 10    ) << HSTX_CTRL_BIT0_SEL_P_LSB |
//         (lane * 10 + 1) << HSTX_CTRL_BIT0_SEL_N_LSB;
//     hstx_ctrl_hw->bit[bit    ] = lane_data_sel_bits; // Assign data to lane
//     hstx_ctrl_hw->bit[bit + 1] = lane_data_sel_bits | HSTX_CTRL_BIT0_INV_BITS; // Inverted data
// }

// // Set the function of GPIO pins to HSTX for DVI output
// for (int i = 12; i <= 19; ++i) {
//     gpio_set_function(i, GPIO_FUNC_HSTX); // Set GPIO function to HSTX
// }
