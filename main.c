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
#include "pico/sleep.h"
#include "hardware/adc.h"
#include "hardware/watchdog.h"

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
#include "lvgl.h"

/* Main Modularization */
#include "global.h"
#include "leds.h"
#include "gpu.h"
#include "gb.h"
#include "sd.h"
#include "audio.h"
#include "rtc.h"
#include "battery.h"


int main(void)
{
	static struct gb_s gb;
	enum gb_init_error_e ret;
	
	// MARK: - Overclock
	// vreg_disable_voltage_limit();
    // const unsigned vco = 1596*1000*1000;	/* 266MHz */
    // const unsigned div1 = 6, div2 = 1;

    // vreg_set_voltage(VREG_VOLTAGE_1_15);
    // sleep_ms(2);
    // set_sys_clock_pll(vco, div1, div2);
    // sleep_ms(2);

    const unsigned vco = 1500 * 1000 * 1000;  // VCO frequency: 1500 MHz
    const unsigned div1 = 5;  // Post-divider 1
    const unsigned div2 = 1;  // Post-divider 2

    // Increase voltage to support higher clock speeds
    vreg_set_voltage(VREG_VOLTAGE_1_15);
    sleep_ms(2);  // Wait for voltage to stabilize
    // Configure system clock to 300 MHz using the PLL
    set_sys_clock_pll(vco, div1, div2);
    sleep_ms(2);  // Wait for clock to stabilize


	/* Initialise USB serial connection for debugging. */
	stdio_init_all();
	// time_init();
	// sleep_ms(5000);
	putstdio("INIT: ");

	set_up_select();

	// MARK: - I2C INIT
	i2c_init(IOX_I2C_PORT, 400 * 1000); // 400 kHz
    gpio_set_function(GPIO_I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(GPIO_I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(GPIO_I2C_SDA);
    gpio_pull_up(GPIO_I2C_SCL);

	#if ENABLE_RTC
	initialize_rtc(RTC_DEFAULT_VALUE);
	#endif
	
	sleep_ms(10);

	// MARK: - Battery Monitor Config
	config_battery_monitor();

    // MARK: - Initialise GPIO
	config_iox_ports();
	// Set up sleep interrupt asap.
	setup_switch_sleep();
	sleep_ms(10);

	// Trigger the battery check immediately
    process_bat_percent();
	// Set up a repeating timer for 10 seconds
    if (!add_repeating_timer_ms(10000, battery_timer_callback, NULL, &timer)) {
        printf("Failed to add repeating timer\n");
    }

	// turn on 3v3
	gpio_write(IOX_n3V3_MCU_EN, false);

	// Initialize the ADC for GPIO_AUD_POT_ADC and nHP_DETECT
    init_adc(GPIO_AUD_POT_ADC);
	init_adc(GPIO_nHP_DETECT);

	gpio_set_function(GPIO_LCD_SCK, GPIO_FUNC_SPI);
	gpio_set_function(GPIO_LCD_MOSI, GPIO_FUNC_SPI);
	gpio_set_function(GPIO_LCD_MISO, GPIO_FUNC_SIO);

	gpio_set_dir(GPIO_LCD_MISO, true);

    // MARK: - PWM Set up
	config_leds();

	gpio_set_slew_rate(GPIO_LCD_SCK, GPIO_SLEW_RATE_FAST);
	gpio_set_slew_rate(GPIO_LCD_MOSI, GPIO_SLEW_RATE_FAST);

    // MARK: - LCD SPI Config
	/* Set SPI clock to use high frequency. */
	clock_configure(clk_peri, 0,
			CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
			125 * 1000 * 1000, 125 * 1000 * 1000);
	spi_init(LCD_SPI, 30*1000*1000);
	spi_set_format(LCD_SPI, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

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
	// MARK: - ROM File selector
#if ENABLE_LCD
#if ENABLE_SDCARD
#if USE_IPS_LCD
	#warning "ips: add ips init"
#else
	mk_ili9225_init();
	mk_ili9225_fill(0x0000);
#endif
#endif
#endif

#if ENABLE_SDCARD
#if USE_IPS_LCD
	#warning "ips: add ips init"
#else
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

	#if ENABLE_RTC
	synchronize_gb_rtc(&gb);
	#endif

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
	// RTC tick counter
	uint_fast8_t frames = 0;
	uint64_t start_time = time_us_64();
	#warning "fix the led flashing thing at some point"
	// resetting led timer because of bug
	low_power = false;
	cancel_repeating_timer(&timer);
	// Set up a repeating timer for 10 seconds
    if (!add_repeating_timer_ms(10000, battery_timer_callback, NULL, &timer)) {
        printf("Failed to add repeating timer\n");
    }

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

		// Tick RTC approximately once per second (assuming ~60 FPS)
		uint8_t fps = gb.direct.frame_skip ? 120 : 60;
		if (frames >= fps) {
			gb_tick_rtc(&gb);
			frames = 0; // Reset counter
		}

#if ENABLE_SOUND
		// if(!gb.direct.frame_skip) {
			read_volume(&i2s_config);
			audio_callback(NULL, stream, AUDIO_BUFFER_SIZE_BYTES);
			i2s_dma_write(&i2s_config, stream);
		// }
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
				increase_lcd_brightness(16);
				increase_pwr_brightness(4);
			}
			if(!gb.direct.joypad_bits.down && prev_joypad_bits.down) {
				/* select + down: decrease sound volume */
				// i2s_decrease_volume(&i2s_config);
				decrease_lcd_brightness(16);
				decrease_pwr_brightness(4);
			}
#endif
			if(!gb.direct.joypad_bits.right && prev_joypad_bits.right) {
				/* select + right: select the next manual color palette */
				// config_led(GPIO_BUTTON_LED, button_led_duty_cycle, true);
				// sleep_ms(10);
				// increase_button_brightness(32);
				if(manual_palette_selected<12) {
					manual_palette_selected++;
					manual_assign_palette(palette,manual_palette_selected);
				}	
			}
			if(!gb.direct.joypad_bits.left && prev_joypad_bits.left) {
				/* select + left: select the previous manual color palette */
				// config_led(GPIO_BUTTON_LED, button_led_duty_cycle, true);
				// sleep_ms(10);
				// decrease_button_brightness(32);
				if(manual_palette_selected>-1) {
					manual_palette_selected--;
					if (manual_palette_selected != -1) {
						manual_assign_palette(palette,manual_palette_selected);
					}
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
				i2s_set_sample_freq(&i2s_config, 44100, gb.direct.frame_skip);
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