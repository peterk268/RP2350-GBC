/* Functions required for communication with the ILI9225. */
void mk_ili9225_set_rst(bool state)
{
	gpio_put(GPIO_RST, state);
}

void mk_ili9225_set_rs(bool state)
{
	gpio_put(GPIO_RS, state);
}

void mk_ili9225_set_cs(bool state)
{
	gpio_put(GPIO_CS, state);
}

void mk_ili9225_set_led(bool state)
{
	gpio_put(GPIO_LED, state);
}

void mk_ili9225_spi_write16(const uint16_t *halfwords, size_t len)
{
	spi_write16_blocking(spi0, halfwords, len);
}

void mk_ili9225_delay_ms(unsigned ms)
{
	sleep_ms(ms);
}



#if ENABLE_LCD 
void core1_lcd_draw_line(const uint_fast8_t line)
{
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

	// // Calculate the start line for the rotated display
    uint_fast8_t rotated_line = LCD_HEIGHT - line - 1;

	mk_ili9225_set_x(rotated_line + 16);
	// mk_ili9225_set_x(line + 16);

	mk_ili9225_write_pixels(fb, LCD_WIDTH);
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

	/* Initialise and control LCD on core 1. */
	mk_ili9225_init();

	/* Clear LCD screen. */
	mk_ili9225_fill(0x0000);

	/* Set LCD window to DMG size. */
	mk_ili9225_fill_rect(31,16,LCD_WIDTH,LCD_HEIGHT,0x0000);

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
			mk_ili9225_display_control(true, cmd.data);
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
