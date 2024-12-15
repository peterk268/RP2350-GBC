/* C Headers */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* RP2350 Headers */
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include <hardware/sync.h>
#include <hardware/flash.h>
#include <hardware/timer.h>
#include <hardware/vreg.h>
#include <pico/bootrom.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/multicore.h>
#include <sys/unistd.h>
#include <hardware/irq.h>
#include "hardware/pwm.h"
#include "hardware/powman.h"

// Essential headers
#include "settings.h"
#include "gpio.h"

/* Project headers */
#include "hedley.h"
#include "minigb_apu.h"
#include "peanut_gb.h"
#include "mk_ili9225.h"
#include "sdcard.h"
#include "i2s.h"
#include "gbcolors.h"

/* Main Modularization */
#include "global.h"
#include "gpu.h"
#include "gb.h"
#include "sd.h"
#include "audio.h"
#include "battery.h"


int main(void)
{
	static struct gb_s gb;
	enum gb_init_error_e ret;
	
	// MARK: - Overclock
    const unsigned vco = 1596*1000*1000;	/* 266MHz */
    const unsigned div1 = 6, div2 = 1;

    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(2);
    set_sys_clock_pll(vco, div1, div2);
    sleep_ms(2);


	/* Initialise USB serial connection for debugging. */
	stdio_init_all();
	// time_init();
	// sleep_ms(5000);
	putstdio("INIT: ");

    // MARK: - Initialise GPIO
	config_iox_ports();
	// Set up sleep interrupt asap.
	setup_switch_sleep();

	gpio_set_function(GPIO_B_SELECT, GPIO_FUNC_SIO);

	gpio_set_function(GPIO_LCD_SCK, GPIO_FUNC_SPI);
	gpio_set_function(GPIO_LCD_MOSI, GPIO_FUNC_SPI);
	gpio_set_function(GPIO_LCD_MISO, GPIO_FUNC_SIO);

	gpio_set_function(GPIO_LCD_LED, GPIO_FUNC_PWM);

	gpio_set_dir(GPIO_B_SELECT, false);
	gpio_set_dir(GPIO_LCD_MISO, true);

    // MARK: - PWM Set up
	uint slice_num = pwm_gpio_to_slice_num(GPIO_LCD_LED);
	uint8_t led_pwm_duty_cycle = 64; // set to 6/8 brightness level, 0 highest, 255 lowest
	pwm_set_wrap(slice_num, 255); // Set PWM period
    pwm_set_chan_level(slice_num, PWM_CHAN_A, led_pwm_duty_cycle); // Set PWM duty cycle
	pwm_set_enabled(slice_num, true); // Enable PWM

	gpio_set_slew_rate(GPIO_LCD_SCK, GPIO_SLEW_RATE_FAST);
	gpio_set_slew_rate(GPIO_LCD_MOSI, GPIO_SLEW_RATE_FAST);
	
	gpio_pull_up(GPIO_B_SELECT);

    // MARK: - LCD SPI Config
	/* Set SPI clock to use high frequency. */
	clock_configure(clk_peri, 0,
			CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
			125 * 1000 * 1000, 125 * 1000 * 1000);
	spi_init(spi1, 30*1000*1000);
	spi_set_format(spi1, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // MARK: - I2S Config
#if ENABLE_SOUND
	// Allocate memory for the stream buffer
	stream=malloc(AUDIO_BUFFER_SIZE_BYTES);
    assert(stream!=NULL);
    memset(stream,0,AUDIO_BUFFER_SIZE_BYTES);  // Zero out the stream buffer
	
	// Initialize I2S sound driver
	i2s_config = i2s_get_default_config();
	i2s_config.sample_freq=AUDIO_SAMPLE_RATE;
	i2s_config.dma_trans_count =AUDIO_SAMPLES;
	i2s_volume(&i2s_config,0);
	i2s_init(&i2s_config);
#endif

// MARK: - Infinite Loop
while(true)
{
#if ENABLE_LCD
#if ENABLE_SDCARD
	// MARK: - ROM File selector
	mk_ili9225_init();
	mk_ili9225_fill(0x0000);

	rom_file_selector();
#endif
#endif

	// MARK: - Initialise GB context
	memcpy(rom_bank0, rom, sizeof(rom_bank0));
	ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
		      &gb_cart_ram_write, &gb_error, NULL);
	putstdio("GB ");

	if(ret != GB_INIT_NO_ERROR)
	{
		printf("Error: %d\n", ret);
		goto out;
	}

	// MARK: - Auto Assign Palette
	char rom_title[16];
	auto_assign_palette(palette, gb_colour_hash(&gb),gb_get_rom_name(&gb,rom_title));
	
    // MARK: - GPU Init
#if ENABLE_LCD
	gb_init_lcd(&gb, &lcd_draw_line);

	/* Start Core1, which processes requests to the LCD. */
	putstdio("CORE1 ");
	multicore_launch_core1(main_core1);
	
	putstdio("LCD ");
#endif

    // MARK: - Audio Init
#if ENABLE_SOUND
	// Initialize audio emulation
	audio_init();
	
	putstdio("AUDIO ");
#endif

#if ENABLE_SDCARD
    // MARK: - Load Save File
	read_cart_ram_file(&gb);
#endif

	putstdio("\n> ");
	uint_fast32_t frames = 0;
	uint64_t start_time = time_us_64();
    // MARK: - Game Play
	while(1)
	{
		int input;

		gb.gb_frame = 0;
        // Stepping cpu instruction
		do {
			__gb_step_cpu(&gb);
			tight_loop_contents();
		} while(HEDLEY_LIKELY(gb.gb_frame == 0));

		frames++;
#if ENABLE_SOUND
		if(!gb.direct.frame_skip) {
			audio_callback(NULL, stream, AUDIO_BUFFER_SIZE_BYTES);
			i2s_dma_write(&i2s_config, stream);
		}
#endif

		// MARK: - Update buttons state
		// Read IOX port 0
		read_io_expander_states(0);
		// Store previous joypad states
		prev_joypad_bits.up      = gb.direct.joypad_bits.up;
		prev_joypad_bits.down    = gb.direct.joypad_bits.down;
		prev_joypad_bits.left    = gb.direct.joypad_bits.left;
		prev_joypad_bits.right   = gb.direct.joypad_bits.right;
		prev_joypad_bits.a       = gb.direct.joypad_bits.a;
		prev_joypad_bits.b       = gb.direct.joypad_bits.b;
		prev_joypad_bits.select  = gb.direct.joypad_bits.select;
		prev_joypad_bits.start   = gb.direct.joypad_bits.start;

		// Update joypad states with values from IOX
		gb.direct.joypad_bits.up      = gpio_read(IOX_B_UP);
		gb.direct.joypad_bits.down    = gpio_read(IOX_B_DOWN);
		gb.direct.joypad_bits.left    = gpio_read(IOX_B_LEFT);
		gb.direct.joypad_bits.right   = gpio_read(IOX_B_RIGHT);
		gb.direct.joypad_bits.a       = gpio_read(IOX_B_A);
		gb.direct.joypad_bits.b       = gpio_read(IOX_B_B);
		gb.direct.joypad_bits.select  = gpio_read(GPIO_B_SELECT);
		gb.direct.joypad_bits.start   = gpio_read(IOX_B_START);

		// MARK: - Hotkeys
        // (select + * combo)
		if(!gb.direct.joypad_bits.select) {
#if ENABLE_SOUND
			if(!gb.direct.joypad_bits.up && prev_joypad_bits.up) {
				/* select + up: increase sound volume */
				// i2s_increase_volume(&i2s_config);
				if (led_pwm_duty_cycle >= 32) { // Ensure it won't go below 0
					led_pwm_duty_cycle -= 32;
				} else {
					led_pwm_duty_cycle = 0; // Clamp to 0 if it would go below 0
				}
				pwm_set_chan_level(slice_num, PWM_CHAN_A, led_pwm_duty_cycle);
			}
			if(!gb.direct.joypad_bits.down && prev_joypad_bits.down) {
				/* select + down: decrease sound volume */
				// i2s_decrease_volume(&i2s_config);
				if (led_pwm_duty_cycle <= (255 - 32)) { // Ensure it won't exceed 255
					led_pwm_duty_cycle += 32;
				} else {
					led_pwm_duty_cycle = 255; // Clamp to 255 if it would exceed
				}
				pwm_set_chan_level(slice_num, PWM_CHAN_A, led_pwm_duty_cycle);
			}
#endif
			if(!gb.direct.joypad_bits.right && prev_joypad_bits.right) {
				/* select + right: select the next manual color palette */
				if(manual_palette_selected<12) {
					manual_palette_selected++;
					manual_assign_palette(palette,manual_palette_selected);
				}	
			}
			if(!gb.direct.joypad_bits.left && prev_joypad_bits.left) {
				/* select + left: select the previous manual color palette */
				if(manual_palette_selected>0) {
					manual_palette_selected--;
					manual_assign_palette(palette,manual_palette_selected);
				}
			}
			if(!gb.direct.joypad_bits.start && prev_joypad_bits.start) {
				/* select + start: save ram and resets to the game selection menu */
#if ENABLE_SDCARD				
				write_cart_ram_file(&gb);
#endif				
				goto out;
			}
			if(!gb.direct.joypad_bits.a && prev_joypad_bits.a) {
				/* select + A: enable/disable frame-skip => fast-forward */
				gb.direct.frame_skip=!gb.direct.frame_skip;
				printf("I gb.direct.frame_skip = %d\n",gb.direct.frame_skip);
			}
			if (!gb.direct.joypad_bits.b && prev_joypad_bits.b) {
				/* select + B: Save game ram*/
#if ENABLE_SDCARD				
				write_cart_ram_file(&gb);
#endif				
			}
		}
    }
    // MARK: - Ending Emulation
    out:
        puts("\nEmulation Ended");
        /* stop lcd task running on core 1 */
        multicore_reset_core1(); 
    }

}