/* Functions required for communication with the ILI9225. */
void mk_ili9225_set_rst(bool state)
{
	// lcd needs por
	#warning "This was done because of inverter, but please change back once hardware is changed"
	gpio_write(IOX_LCD_RST, !state);
}

void mk_ili9225_set_rs(bool state)
{
	gpio_write(GPIO_LCD_MISO, state);
}
bool iscs = false;
void mk_ili9225_set_cs(bool state)
{
	// this was done due to slowness of iox write
	if (!iscs) {
		gpio_write(IOX_TFT_nCS, 0);
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

// Structure to hold enhancement parameters
typedef struct {
    bool apply_gamma;        // Enable/disable gamma correction
    bool apply_contrast;     // Enable/disable contrast adjustment
    bool apply_vibrancy;     // Enable/disable vibrancy boost
    float gamma_value;       // Gamma correction value (e.g., 2.2)
    float contrast_factor;   // Contrast adjustment factor (e.g., 1.2)
    uint8_t vibrancy_boost;  // Amount to boost each color (e.g., 2)
} PixelEnhancementParams;

uint16_t enhance_pixel(uint16_t pixel, PixelEnhancementParams params) {
    // Extract RGB components from RGB565
    uint8_t red = (pixel >> 11) & 0x1F;
    uint8_t green = (pixel >> 5) & 0x3F;
    uint8_t blue = pixel & 0x1F;

    // Threshold for near-black detection
    const uint8_t black_threshold = 2;

    // Detect near-black pixels
    if (red <= black_threshold && green <= black_threshold && blue <= black_threshold) {
        red = 0;
        green = 0;
        blue = 0;
    } else {
        // Apply gamma correction
        if (params.apply_gamma) {
            float gamma = params.gamma_value;
            red = (uint8_t)(31 * pow(red / 31.0, gamma));
            green = (uint8_t)(63 * pow(green / 63.0, gamma));
            blue = (uint8_t)(31 * pow(blue / 31.0, gamma));
        }

        // Apply contrast adjustment
        if (params.apply_contrast) {
            float contrast = params.contrast_factor;
            red = (uint8_t)(red * contrast);
            green = (uint8_t)(green * contrast);
            blue = (uint8_t)(blue * contrast);
        }

        // Apply vibrancy boost
        if (params.apply_vibrancy) {
            int8_t boost = params.vibrancy_boost;  // Allow negative values

            // Adjust each channel with the vibrancy boost and clamp
            red = (boost > 0) ? (red + boost <= 31 ? red + boost : 31)
                              : (red + boost >= 0 ? red + boost : 0);
            green = (boost > 0) ? (green + boost <= 63 ? green + boost : 63)
                                : (green + boost >= 0 ? green + boost : 0);
            blue = (boost > 0) ? (blue + boost <= 31 ? blue + boost : 31)
                               : (blue + boost >= 0 ? blue + boost : 0);
        }

        // Clamp values to their respective ranges (safety check)
        red = (red > 31) ? 31 : red;
        green = (green > 63) ? 63 : green;
        blue = (blue > 31) ? 31 : blue;
    }

    // Recombine into RGB565
    return (red << 11) | (green << 5) | blue;
}

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

	// PixelEnhancementParams params = {
	// 	.apply_gamma = false,
	// 	.apply_contrast = true,
	// 	.apply_vibrancy = false,
	// 	.gamma_value = 2.2,         // Gamma correction factor
	// 	.contrast_factor = 0.5,     // Increase contrast by 20%
	// 	.vibrancy_boost = -5        // Boost each color by 2
	// };

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
	#warning "ips: add ips init"
#else
	mk_ili9225_set_x(rotated_line + 16);
	// mk_ili9225_set_x(line + 16);

	mk_ili9225_write_pixels(fb, LCD_WIDTH);
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
	mk_ili9225_init();

	/* Clear LCD screen. */
	mk_ili9225_fill(0x0000);

	/* Set LCD window to DMG size. */
	mk_ili9225_fill_rect(31,16,LCD_WIDTH,LCD_HEIGHT,0x0000);
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
			#warning "ips: add ips init"
#else
			mk_ili9225_display_control(true, cmd.data);
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

// MARK: - NEW LCD
static const uint16_t dpi_pins[] = {
    GPIO_DPI_B0, GPIO_DPI_B1, GPIO_DPI_B2, GPIO_DPI_B3, GPIO_DPI_B4,
    GPIO_DPI_G0, GPIO_DPI_G1, GPIO_DPI_G2, GPIO_DPI_G3, GPIO_DPI_G4, GPIO_DPI_G5,
    GPIO_DPI_R0, GPIO_DPI_R1, GPIO_DPI_R2, GPIO_DPI_R3, GPIO_DPI_R4,
    GPIO_DPI_VSYNC, GPIO_DPI_HSYNC, GPIO_DPI_PCLK, GPIO_DPI_DEN
};

static void init_dpi_pins(void) {
    for (size_t i = 0; i < sizeof(dpi_pins) / sizeof(dpi_pins[0]); i++) {
        gpio_init(dpi_pins[i]);
        gpio_set_dir(dpi_pins[i], GPIO_OUT);
    }
}


static void lv_port_disp_init(void);

static void dpi_set_pixel_clock(uint32_t frequency) {
    uint32_t div = clock_get_hz(clk_sys) / frequency;
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, div - 1);
    pwm_config_set_clkdiv_mode(&config, PWM_DIV_FREE_RUNNING);

    pwm_init(pwm_gpio_to_slice_num(GPIO_DPI_PCLK), &config, true);
    gpio_set_function(GPIO_DPI_PCLK, GPIO_FUNC_PWM);
}

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

void drive_rgb_pins(uint16_t pixel) {
	uint8_t r = (pixel >> 11) & 0x1F; // Red 5 bits
	uint8_t g = (pixel >> 5) & 0x3F;  // Green 6 bits
	uint8_t b = pixel & 0x1F;         // Blue 5 bits

	gpio_put_masked64((uint64_t)0b11111 << GPIO_DPI_R0, (uint64_t)r << GPIO_DPI_R0);
	gpio_put_masked64((uint64_t)0b111111 << GPIO_DPI_G0, (uint64_t)g << GPIO_DPI_G0);
	gpio_put_masked64((uint64_t)0b11111 << GPIO_DPI_B0, (uint64_t)b << GPIO_DPI_B0);
}

static void lcd_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    for (int y = area->y1; y <= area->y2; y++) {
        gpio_write(GPIO_DPI_HSYNC, 0); // Start of line
        for (int x = area->x1; x <= area->x2; x++) {
            // Extract RGB values from the LVGL color buffer
            drive_rgb_pins(color_p->full);

            // Toggle pixel clock
            gpio_write(GPIO_DPI_PCLK, 1);
            gpio_write(GPIO_DPI_PCLK, 0);

            color_p++;
        }
        gpio_write(GPIO_DPI_HSYNC, 1); // End of line
    }

    gpio_write(GPIO_DPI_VSYNC, 1); // Frame ready
    gpio_write(GPIO_DPI_DEN, 1);   // Data enable

    lv_disp_flush_ready(drv); // Notify LVGL that flush is complete
}

// static void dpi_display_draw()

static void new_frame() {
	gpio_put(GPIO_DPI_VSYNC, 0); // Start of Vsync
    sleep_us(1);                 // Short pulse width for Vsync
    gpio_put(GPIO_DPI_VSYNC, 1); // End of Vsync
}
static void new_line() {
	gpio_put(GPIO_DPI_HSYNC, 0); // Hsync pulse for each line
    sleep_us(1);                 // Short pulse width for Hsync
    gpio_put(GPIO_DPI_HSYNC, 1); // End of Hsync
}
static void dpi_set_y(uint_fast8_t y) {
    // Trigger Vsync for a specific Y position
	new_frame();

    // Wait until the display reaches the target line
    for (int i = 0; i < y; i++) {
        new_line();
    }
}

#warning "Implement this function in when we are reading for the ips lcd"
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
