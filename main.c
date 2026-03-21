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
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#if ENABLE_PSRAM
#include "sparkfun_pico/sfe_pico.h"
#endif

// Essential headers
#include "settings.h"
#include "gpio.h"

/* Project headers */
#include "hedley.h"
#include "minigb_apu.h"
#include "peanut_gb.h"
#include "sdcard.h"
#include "i2s.h"
#include "gbcolors.h"
#include "lvgl.h"
#define DR_MP3_IMPLEMENTATION
#include "dr_mp3.h"
#define DR_WAV_IMPLEMENTATION
#include "dr_wav.h"
#define DR_FLAC_IMPLEMENTATION
#include "dr_flac.h"

/* Main Modularization */
#include "prototypes.h"
#include "global.h"
#include "leds.h"
#include "gpu.h"
#include "gb.h"
#if ENABLE_PSRAM
#include "psram.h"
#endif
#include "sd.h"
#include "audio.h"
#include "rtc.h"
#include "battery.h"
#include "mp3.h"
#include "imu.h"
#include "imu_dash.h"
#include "in_game_menu.h"

int main(void)
{	
	set_up_select();
	overclock_cpu(UNDERCLOCK_CPU_IN_NORMAL_EMULATION);

	/* Initialise USB serial connection for debugging. */
	setup_default_uart();
	stdio_init_all();

	watchdog_enable(WATCHDOG_STARTUP_TIMEOUT_MS, true);
	watchdog_update();
#if ENABLE_PSRAM
	enable_psram_cs1();
#endif

    setup_hold_power();
	watchdog_update();

#if I2C_HAS_TROUBLES
	sleep_ms(100); // this helps i2c to get things in order and prevent hangs
#endif
	// MARK: - I2C INIT
	init_i2c();

#if ENABLE_RTC
	initialize_rtc(RTC_DEFAULT_VALUE);
#endif

#if I2C_HAS_TROUBLES
	// this is needed a lot more than you'd think.. not really bro
	i2c_wait_ready(BAT_MONITOR_I2C_PORT, BAT_MONITOR_I2C_ADDR, 100);
#endif

	watchdog_update();

	// MARK: - Battery Monitor Config
	config_battery_monitor();

	// Makes LED blink on power up with the phase in, I like it.
	config_led(GPIO_PWR_LED, 128, false);

    // MARK: - Initialise GPIO
	config_iox_ports();
	// Set up sleep interrupt asap.
	// #warning "We'll come back to this at some point... well not really cause now we got power hold"
	// setup_switch_sleep();
	sleep_ms(10);

	watchdog_update();

	#if ENABLE_BAT_MONITORING
#if BAT_IMMEDIATE_CHECK
	// Trigger the battery check immediately
    process_bat_percent();
#endif
	// Set up a repeating timer for 10 seconds
    if (!add_repeating_timer_ms(BATTERY_TIMER_INTERVAL_MS, battery_timer_callback, NULL, &battery_timer)) {
        printf("Failed to add repeating timer\n");
    }
#endif
	// Initialize the ADC for GPIO_AUD_POT_ADC and nHP_DETECT
    init_adc(GPIO_AUD_POT_ADC);

	watchdog_update();
#if ENABLE_LCD && USE_IPS_LCD
	lcd_power_on_reset();
	init_spi_lcd();
	gpio_write(IOX_LCD_nCS, 0);
	lcd_config();
	gpio_write(IOX_LCD_nCS, 1);

	sleep_ms(1);
	gpio_deinit(GPIO_SPI0_SCK);
	gpio_deinit(GPIO_SPI0_MOSI);
	gpio_deinit(GPIO_SPI0_MISO);
	setup_dpi();

	/* Start Core1, which processes requests to the LCD. */
	putstdio("CORE1 ");
	multicore_launch_core1(main_core1);
	putstdio("LCD ");

#endif

	watchdog_update();
	// Enable Audio and SD Card
#if ENABLE_SDCARD || ENABLE_SOUND
	printf("Enabling Audio and SD Card\n");
	write_iox_port1(NO_UPDATE, NO_UPDATE, NO_UPDATE, NO_UPDATE, NO_UPDATE, 1, 0, NO_UPDATE);
	sleep_ms(10);
	gpio_write(IOX_AUDIO_EN, 0);
# if ENABLE_SOUND
	sleep_ms(10);
	gpio_write(IOX_AUDIO_EN, 1);
	sleep_ms(10); // this is needed or audio won't work sometimes
# endif
#endif

	last_filename_init();
	watchdog_update();
	read_system_settings(&lcd_target_brightness, &button_target_brightness, &pwr_target_brightness, &manual_palette_selected, &wash_out_level, last_filename_raw, &auto_load_state);
#if TIE_PWR_LED_TO_LCD
	pwr_target_brightness = lcd_target_brightness;
#endif
	// mcp7940n_init(RTC_I2C_PORT);
	// Doesn't work :P will crash.. sike changing i2c pull up to 1k from 5.1k fixed it :)
	printf("Setting RTC\n");
	mcp7940n_set_time_if_unset(RTC_I2C_PORT, &default_rtc);

	watchdog_update();
	// SD WILL HANDLE SPI INIT
	// gpio_set_function(GPIO_SPI0_SCK, GPIO_FUNC_SPI);
	// gpio_set_function(GPIO_SPI0_MOSI, GPIO_FUNC_SPI);
	// gpio_set_function(GPIO_SPI0_MISO, GPIO_FUNC_SPI);

	// gpio_set_slew_rate(GPIO_SPI0_SCK, GPIO_SLEW_RATE_FAST);
	// gpio_set_slew_rate(GPIO_SPI0_MOSI, GPIO_SLEW_RATE_FAST);
	// gpio_set_slew_rate(GPIO_SPI0_MISO, GPIO_SLEW_RATE_FAST);

    // MARK: - LCD SPI Config
	/* Set SPI clock to use high frequency. */
	// clock_configure(clk_peri, 0,
	// 		CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
	// 		125 * 1000 * 1000, 125 * 1000 * 1000);
	// spi_init(LCD_SPI, 30*1000*1000);
	// spi_set_format(LCD_SPI, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // MARK: - I2S Config
#if ENABLE_SOUND
	// Allocate memory for the stream buffer
	stream=malloc(AUDIO_BUFFER_SIZE_BYTES);
    assert(stream!=NULL);
    memset(stream,0,AUDIO_BUFFER_SIZE_BYTES);  // Zero out the stream buffer
	
	setup_dac();

	// If the mp3 application is selected we need to switch dma_trans_count to 2048.
	read_io_expander_states(0);

	// Initialize I2S sound driver
	i2s_config = i2s_get_default_config();
	i2s_config.sample_freq=AUDIO_SAMPLE_RATE;
	i2s_config.dma_trans_count = gpio_read(IOX_B_B) ^ GBC_MAIN_APP ? PCM_FRAME_COUNT : AUDIO_SAMPLES;
	i2s_config.mclk_enabled = (USE_MCLK != 0);
	// Keep MCLK aligned with codec clock tree in setup_dac().
	i2s_config.mclk_mult = DAC_MCLK_FS_RATIO;

	i2s_volume(&i2s_config,0);
	i2s_init(&i2s_config);

	volume_lut_init();	

	#warning "I should really have diagnostics for all my peripherals on start up"
	// sleep_ms(10);
	// check_dac();
#endif
	watchdog_disable();
	// CPU Starts overclocked then goes back to normal after initializing DPI
	// This is to prevent a bug where overclocking messes DPI timings.
#if UNDERCLOCK_CPU_IN_NORMAL_EMULATION
	overclock_cpu(false);
#endif

    // MARK: - PWM Set up
	fade_in_leds_startup();

	// read_io_expander_states(0);
	if (!gpio_read(IOX_B_A)) {
		underclock_cpu(true); // 10mA saved
		run_gmeter_dashboard();
		while (1) { tight_loop_contents(); }
	}
	if (gpio_read(IOX_B_B) ^ GBC_MAIN_APP) {
		// hyper_underclock_cpu(true); // ~52mA consumed, 62mA with regular underclock, 75mA with overclock
		play_mp3_stream(NULL);
		while (1) { tight_loop_contents(); }
	}

// MARK: - Infinite Loop
while(true)
{
	read_io_expander_states(0);
	bool force_auto_load = !gpio_read(IOX_B_START);
	// MARK: - ROM File selector
#if ENABLE_SDCARD && ENABLE_ROM_SELECTOR
	rom_file_selector();
#endif

    __atomic_store_n(&show_gui, false, __ATOMIC_RELEASE);


	// MARK: - Initialise GB context
	gb_alloc_ram();
	ret = gb_init(&gb, &gb_rom_read, &gb_cart_ram_read,
		      &gb_cart_ram_write, &gb_error, NULL);
#if ENABLE_BOOTROM
	printf("Setting Boorom");
	gb_set_bootrom(&gb, dmg_bootrom_read); // attach boot ROM
	gb_reset(&gb); // apply boot ROM
	printf("Resetting GB");
#endif
	putstdio("GB ");

	if(ret != GB_INIT_NO_ERROR)
	{
		printf("Error: %d\n", ret);
		goto out;
	}

	/* Use direct memory pointers for hot ROM/RAM accesses in the emulator core. */
	gb_set_direct_memory(&gb, (const uint8_t *)rom, ram);

	// MARK: - Auto Assign Palette
	if (!palette) {
		if (!palette_heap_init()) {
			printf("E: palette malloc failed\n");
		}
	}

	char rom_title[16];
	auto_assign_palette(*palette, gb_colour_hash(&gb),gb_get_rom_name(&gb,rom_title));
	
    // MARK: - GPU Init
#if ENABLE_LCD
	gb_init_lcd(&gb, &lcd_draw_line);
	if (manual_palette_selected != -1) {
		manual_assign_palette(*palette, manual_palette_selected);
	}
#endif

    // MARK: - Audio Init
#if ENABLE_SOUND
	// Initialize audio emulation
	audio_init();

	putstdio("AUDIO ");
#endif

#if ENABLE_SDCARD		
	// MARK: - Load Auto State
	if (auto_load_state || force_auto_load) {
		bool load_success = read_cart_save_state(&gb, 0, false);

		// If auto-load fails, fall back to battery save
		if (!load_success) {
			printf("Auto load state failed.\n");
			read_cart_ram_file(&gb, false);
		}
	} else {
		// MARK: - Load Save File
		// Auto-load disabled -> battery save only
		read_cart_ram_file(&gb, false);
	}
#endif

	putstdio("\n> ");
	// resetting led timer because of bug
#if ENABLE_BAT_MONITORING
	low_power = false;
	low_power_shutdown = false;
	cancel_repeating_timer(&battery_timer);
	// Set up a repeating timer for 10 seconds
    if (!add_repeating_timer_ms(BATTERY_TIMER_INTERVAL_MS, battery_timer_callback, NULL, &battery_timer)) {
        printf("Failed to add repeating timer\n");
    }
#endif

	watchdog_enable(WATCHDOG_TIMEOUT_MS, true); // 2 second timeout, pause-on-debug = true
#if ENABLE_SAVE_ON_POWER_OFF
	hold_power(); // keep power on for gameplay
#endif
#if ENABLE_AUTO_SAVE
	uint8_t save_wait_counter = 0;
#endif

	sync_gb_rtc(&gb);
    // MARK: - Game Play
	while(1)
	{
	
#if ENABLE_SAVE_ON_POWER_OFF
		// power switch off
		if (!gpio_read(GPIO_SW_OUT)) {
			// LED Phase out on power down didn't make any sense with how fast the shutdown is..
			// Suffering from success I guess but it's fine..
			// I don't think anyone will miss their Pico Pal taking longer to turn off just for the LEDs to fade out lol.
			// Power on made sense because that takes like a second to init everything but power off is instant.

			uint8_t temp_lcd_led = lcd_led_duty_cycle;
			uint8_t temp_button_led = button_led_duty_cycle;
			decrease_button_brightness(MAX_BRIGHTNESS);
			// sd busy handles lcd led turn off
#if LED_PHASE_OUT_PWR_DOWN
			uint8_t temp_button_led = button_led_duty_cycle;
			uint8_t temp_pwr_led = pwr_led_duty_cycle;
			fade_out_leds_powerdown();
#endif

#if ENABLE_SDCARD
			// save to sd card
			write_cart_ram_file(&gb, true);
			in_game_save_auto_state(true);
#endif			
#if LED_PHASE_OUT_PWR_DOWN
			save_system_settings_if_changed(temp_lcd_led, temp_button_led, temp_pwr_led, manual_palette_selected, wash_out_level, last_filename_raw, auto_load_state, );
#else
# if TIE_PWR_LED_TO_LCD
			pwr_led_duty_cycle = temp_lcd_led;
# endif
			save_system_settings_if_changed(temp_lcd_led, temp_button_led, low_power ? prev_pwr_led_duty_cycle : pwr_led_duty_cycle, manual_palette_selected, wash_out_level, last_filename_raw, auto_load_state, true);
#endif
			printf("Done");

			release_power(); // turn power off

			// if we reach here, something went wrong
			sleep_ms(100);
			watchdog_reboot(0, 0, 0); // Force reboot
		}
#endif

#if ENABLE_BAT_MONITORING
		// Check battery status periodically
		if (battery_task_flag) {
			battery_task_flag = false;

			// best to not have too much i2c bus contention in the same frame cycle
			// so we alternate between rtc and battery checks
			// each gets called every 20s.
			if (do_rtc_update) {
				sync_gb_rtc(&gb);
			} else {
				process_bat_percent();
			}

			do_rtc_update = !do_rtc_update; // toggle for next round
        }
		if (low_power_shutdown) {
			// need this before everything else because sd busy will shut off lcd
			shutdown_screen(1500);

			uint8_t temp_lcd_led = lcd_led_duty_cycle;
			uint8_t temp_button_led = button_led_duty_cycle;
			decrease_button_brightness(MAX_BRIGHTNESS);
			// sd busy handles lcd led turn off
#if ENABLE_SDCARD
			write_cart_ram_file(&gb, true);
			in_game_save_auto_state(true);
# if TIE_PWR_LED_TO_LCD
			pwr_led_duty_cycle = temp_lcd_led;
# endif
			save_system_settings_if_changed(temp_lcd_led, temp_button_led, low_power ? prev_pwr_led_duty_cycle : pwr_led_duty_cycle, manual_palette_selected, wash_out_level, last_filename_raw, auto_load_state, true);
#endif
			release_power(); // Cut power hold
			sleep_ms(1);
			watchdog_disable();
			shutdown_peripherals(true);
			sleep_ms(10);
			// powman_example_init(0);
			// powman_example_off();
		}
		while(low_power_shutdown) {
			process_bat_percent();
			sleep_ms(BATTERY_TIMER_INTERVAL_MS);
		}
#endif

		int input;

        // Stepping cpu instructions for frame
		gb_run_frame(&gb);

#if ENABLE_SOUND
#if ENABLE_EXTREME_BATTERY_SAVE
		if (run_mode != MODE_POWERSAVE)
#endif
		{
			// if its not frame skip and not skipping audio frame and not enabling skipping then play audio
			if (!should_skip_audio_frame || !gb.direct.frame_skip || !SKIP_AUDIO_FRAMES_IN_FRAME_SKIP) {
				read_volume(&i2s_config);
				audio_callback(NULL, stream, AUDIO_BUFFER_SIZE_BYTES);
				i2s_dma_write(&i2s_config, stream);
			}
# if SKIP_AUDIO_FRAMES_IN_FRAME_SKIP
			should_skip_audio_frame = !should_skip_audio_frame;
# endif
		}
#endif

		// MARK: - Update buttons state
		// Store previous joypad states
		prev_joypad_bits.up      = gb.direct.joypad_bits.up;
		prev_joypad_bits.down    = gb.direct.joypad_bits.down;
		prev_joypad_bits.left    = gb.direct.joypad_bits.left;
		prev_joypad_bits.right   = gb.direct.joypad_bits.right;
		prev_joypad_bits.a       = gb.direct.joypad_bits.a;
		prev_joypad_bits.b       = gb.direct.joypad_bits.b;
		prev_joypad_bits.start   = gb.direct.joypad_bits.start;
		prev_joypad_bits.select  = gb.direct.joypad_bits.select;
		// Read current joypad states
		gb.direct.joypad_bits.select  = gpio_read(GPIO_B_SELECT);

		bool iox_nint = gpio_read(GPIO_IOX_nINT);
		if (!iox_nint) {
			// Read IOX port 0
			read_io_expander_states(0);

			// Update joypad states with values from IOX
			gb.direct.joypad_bits.up      = gpio_read(IOX_B_UP);
			gb.direct.joypad_bits.down    = gpio_read(IOX_B_DOWN);
			gb.direct.joypad_bits.left    = gpio_read(IOX_B_LEFT);
			gb.direct.joypad_bits.right   = gpio_read(IOX_B_RIGHT);
			gb.direct.joypad_bits.a       = gpio_read(IOX_B_A);
			gb.direct.joypad_bits.b       = gpio_read(IOX_B_B);
			gb.direct.joypad_bits.start   = gpio_read(IOX_B_START);
		}
		// MARK: - Hotkeys
		// (select + * combo)
		if(!gb.direct.joypad_bits.select) {
#if ENABLE_SOUND
			if(!gb.direct.joypad_bits.up && prev_joypad_bits.up) {
				/* select + up: increase sound volume */
				// i2s_increase_volume(&i2s_config);
				in_game_increase_lcd_brightness();
			}
			if(!gb.direct.joypad_bits.down && prev_joypad_bits.down) {
				/* select + down: decrease sound volume */
				// i2s_decrease_volume(&i2s_config);
				in_game_decrease_lcd_brightness();
			}
#endif
			if(!gb.direct.joypad_bits.right && prev_joypad_bits.right) {
				/* select + right: increase button led brightness */
				in_game_increase_button_brightness();
			}
			if(!gb.direct.joypad_bits.left && prev_joypad_bits.left) {
				/* select + left: decrease button led brightness */
				in_game_decrease_button_brightness();
			}

			// in game menu
			if(!gb.direct.joypad_bits.start && prev_joypad_bits.start) {
				in_game_menu();
				// we need to invalidate the previous button press of start + select and now b and now a with load state
				gb.direct.joypad_bits.start = true;
				gb.direct.joypad_bits.select = true;
				prev_joypad_bits.start = true;
				prev_joypad_bits.select = true;

				gb.direct.joypad_bits.b = true;
				prev_joypad_bits.b = true;
				gb.direct.joypad_bits.a = true;
				prev_joypad_bits.a = true;

				// Check if we need to exit to ROM selector
				if (g_request_exit_to_rom_selector) {
					g_request_exit_to_rom_selector = false;
					goto out;
				}
			}

			if(!gb.direct.joypad_bits.a && prev_joypad_bits.a) {
				/* select + A: enable/disable frame-skip => fast-forward */
				ig_toggle_fast_forward();
			}
			if (!gb.direct.joypad_bits.b && prev_joypad_bits.b) {
				/* select + B: Save game ram*/
				in_game_save_game(false);
			}
		}
		// Start + combo
		if(!gb.direct.joypad_bits.start) {
			// Step up wash out
			if (!gb.direct.joypad_bits.up && prev_joypad_bits.up) {
				in_game_increase_washout();
			}
			// Step down wash out 
			if (!gb.direct.joypad_bits.down && prev_joypad_bits.down) {
				in_game_decrease_washout();
			}
			
			// Save State
			if (!gb.direct.joypad_bits.left && prev_joypad_bits.left) {
				in_game_save_state();
			}
			// Load State 
			if (!gb.direct.joypad_bits.right && prev_joypad_bits.right) {
				in_game_load_state();
			}

			// But currently getting like 125mW down from 195mW so it's a good start.
			if (!gb.direct.joypad_bits.a && prev_joypad_bits.a) {
				/* start + A: Battery Saving Mode*/
				ig_toggle_battery_save();
			}

			// Screenshot
			if (!gb.direct.joypad_bits.b && prev_joypad_bits.b) {
				in_game_screenshot();
			}
		}

#if ENABLE_FRAME_DEBUGGING
		static uint32_t last_ts = 0;
		uint32_t now = time_us_32();
		if (last_ts != 0) {
			printf("frame time us: %u\n", now - last_ts);
		}
		last_ts = now;
#endif

		static uint32_t fps_counter = 0;
		static uint32_t fps_last_time = 0;

		fps_counter++;
		uint32_t now = time_us_32();
		if (now - fps_last_time >= 1000000) { // 1 second
			watchdog_update();
#if ENABLE_AUTO_SAVE
			save_wait_counter++;
			// Auto-save every 30 seconds if there is a change in RAM
			if (save_wait_counter >= 30) {
				save_wait_counter = 0;
				if (ram_changed) {
					printf("RAM changed, saving...\n");
					ram_changed = false;
					// save to sd card
#if ENABLE_SDCARD				
					write_cart_ram_file(&gb, false);
					in_game_save_auto_state(false);
#endif
				}
			}
#endif


#if ENABLE_FPS_MONITORING
			printf("FPS: %lu\n", fps_counter);
#endif
			fps_counter = 0;
			fps_last_time = now;
		}

#if FPS_LIMITER_ENABLED
		// --- Dynamic FPS control ---
		// Only needed for power save mode
		if (run_mode == MODE_POWERSAVE) {
			static uint32_t frame_start = 0;
			uint32_t now = time_us_32();

			if (frame_start == 0) frame_start = now;

			uint32_t elapsed = now - frame_start;

			uint32_t target_us;

			switch (run_mode) {
				case MODE_NORMAL:
					target_us = 16666;   // ~60 FPS
					break;

				case MODE_TURBO:
					target_us = 16666/2; // 120 FPS
					break;

				case MODE_POWERSAVE:
					target_us = 16666;   // 60 emulation FPS, this matches 2x frameskip * half clock speed.. 30fps effective
					break;
			}

			if (elapsed < target_us)
				sleep_us(target_us - elapsed);
			frame_start = time_us_32();
		}
#endif

		sleep_us(10); // small delay for timers
        tight_loop_contents();
    }
    // MARK: - Ending Emulation
    out:
        printf("\nEmulation Ended");

		// reset emulation mode back to normal
		if (run_mode == MODE_POWERSAVE) ig_toggle_battery_save();
		if (run_mode == MODE_TURBO) ig_toggle_fast_forward();
		
		// To prevent the flicker of a game frame when exiting to rom selector
		memset(front_fb->data, 0, sizeof(front_fb->data));

		// Clean up for the ROM selector
		if (rom) {
			free(rom);
			rom = NULL;
		}
		if (ram) {
			free(ram);
			ram = NULL;
		}
		if (filename) {
			free(filename);
			filename = NULL;
		}

		memory_stats();
		release_power(); // release power hold
		watchdog_disable();
		sleep_ms(100);
    }

}
