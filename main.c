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

#include "settings.h"

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
#include "gpio.h"
#include "gpu.h"
#include "gb.h"
#include "sd.h"


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
	gpio_set_function(GPIO_UP, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_DOWN, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_LEFT, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_RIGHT, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_A, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_B, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_SELECT, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_START, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_CS, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_CLK, GPIO_FUNC_SPI);
	gpio_set_function(GPIO_SDA, GPIO_FUNC_SPI);
	gpio_set_function(GPIO_RS, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_RST, GPIO_FUNC_SIO);
	gpio_set_function(GPIO_LED, GPIO_FUNC_PWM);

	gpio_set_dir(GPIO_UP, false);
	gpio_set_dir(GPIO_DOWN, false);
	gpio_set_dir(GPIO_LEFT, false);
	gpio_set_dir(GPIO_RIGHT, false);
	gpio_set_dir(GPIO_A, false);
	gpio_set_dir(GPIO_B, false);
	gpio_set_dir(GPIO_SELECT, false);
	gpio_set_dir(GPIO_START, false);
	gpio_set_dir(GPIO_CS, true);
	gpio_set_dir(GPIO_RS, true);
	gpio_set_dir(GPIO_RST, true);

    // MARK: - PWM Set up
	uint slice_num = pwm_gpio_to_slice_num(GPIO_LED);
	uint8_t led_pwm_duty_cycle = 64; // set to 6/8 brightness level, 0 highest, 255 lowest
	pwm_set_wrap(slice_num, 255); // Set PWM period
    pwm_set_chan_level(slice_num, PWM_CHAN_A, led_pwm_duty_cycle); // Set PWM duty cycle
	pwm_set_enabled(slice_num, true); // Enable PWM

	gpio_set_slew_rate(GPIO_CLK, GPIO_SLEW_RATE_FAST);
	gpio_set_slew_rate(GPIO_SDA, GPIO_SLEW_RATE_FAST);
	
	gpio_pull_up(GPIO_UP);
	gpio_pull_up(GPIO_DOWN);
	gpio_pull_up(GPIO_LEFT);
	gpio_pull_up(GPIO_RIGHT);
	gpio_pull_up(GPIO_A);
	gpio_pull_up(GPIO_B);
	gpio_pull_up(GPIO_SELECT);
	gpio_pull_up(GPIO_START);

    // MARK: - LCD SPI Config
	/* Set SPI clock to use high frequency. */
	clock_configure(clk_peri, 0,
			CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
			125 * 1000 * 1000, 125 * 1000 * 1000);
	spi_init(spi0, 30*1000*1000);
	spi_set_format(spi0, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

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
	// multicore_launch_core1(play_wav_default);
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
		prev_joypad_bits.up=gb.direct.joypad_bits.up;
		prev_joypad_bits.down=gb.direct.joypad_bits.down;
		prev_joypad_bits.left=gb.direct.joypad_bits.left;
		prev_joypad_bits.right=gb.direct.joypad_bits.right;
		prev_joypad_bits.a=gb.direct.joypad_bits.a;
		prev_joypad_bits.b=gb.direct.joypad_bits.b;
		prev_joypad_bits.select=gb.direct.joypad_bits.select;
		prev_joypad_bits.start=gb.direct.joypad_bits.start;
		gb.direct.joypad_bits.up=gpio_get(GPIO_UP);
		gb.direct.joypad_bits.down=gpio_get(GPIO_DOWN);
		gb.direct.joypad_bits.left=gpio_get(GPIO_LEFT);
		gb.direct.joypad_bits.right=gpio_get(GPIO_RIGHT);
		gb.direct.joypad_bits.a=gpio_get(GPIO_A);
		gb.direct.joypad_bits.b=gpio_get(GPIO_B);
		gb.direct.joypad_bits.select=gpio_get(GPIO_SELECT);
		gb.direct.joypad_bits.start=gpio_get(GPIO_START);

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

// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/spi.h"
// #include "hardware/i2c.h"
// #include "hardware/dma.h"
// #include "hardware/pio.h"
// #include "hardware/timer.h"
// #include "hardware/clocks.h"

// // SPI Defines
// // We are going to use SPI 0, and allocate it to the following GPIO pins
// // Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
// #define SPI_PORT spi0
// #define PIN_MISO 16
// #define PIN_CS   17
// #define PIN_SCK  18
// #define PIN_MOSI 19

// // I2C defines
// // This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// // Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
// #define I2C_PORT i2c0
// #define I2C_SDA 8
// #define I2C_SCL 9

// // Data will be copied from src to dst
// const char src[] = "Hello, world! (from DMA)";
// char dst[count_of(src)];

// #include "blink.pio.h"

// void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
//     blink_program_init(pio, sm, offset, pin);
//     pio_sm_set_enabled(pio, sm, true);

//     printf("Blinking pin %d at %d Hz\n", pin, freq);

//     // PIO counter program takes 3 more cycles in total than we pass as
//     // input (wait for n + 1; mov; jmp)
//     pio->txf[sm] = (125000000 / (2 * freq)) - 3;
// }

// int64_t alarm_callback(alarm_id_t id, void *user_data) {
//     // Put your timeout handler code in here
//     return 0;
// }




// int main()
// {
//     stdio_init_all();

//     // SPI initialisation. This example will use SPI at 1MHz.
//     spi_init(SPI_PORT, 1000*1000);
//     gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
//     gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
//     gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
//     gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
//     // Chip select is active-low, so we'll initialise it to a driven-high state
//     gpio_set_dir(PIN_CS, GPIO_OUT);
//     gpio_put(PIN_CS, 1);
//     // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi

//     // I2C Initialisation. Using it at 400Khz.
//     i2c_init(I2C_PORT, 400*1000);
    
//     gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
//     gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
//     gpio_pull_up(I2C_SDA);
//     gpio_pull_up(I2C_SCL);
//     // For more examples of I2C use see https://github.com/raspberrypi/pico-examples/tree/master/i2c

//     // Get a free channel, panic() if there are none
//     int chan = dma_claim_unused_channel(true);
    
//     // 8 bit transfers. Both read and write address increment after each
//     // transfer (each pointing to a location in src or dst respectively).
//     // No DREQ is selected, so the DMA transfers as fast as it can.
    
//     dma_channel_config c = dma_channel_get_default_config(chan);
//     channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
//     channel_config_set_read_increment(&c, true);
//     channel_config_set_write_increment(&c, true);
    
//     dma_channel_configure(
//         chan,          // Channel to be configured
//         &c,            // The configuration we just created
//         dst,           // The initial write address
//         src,           // The initial read address
//         count_of(src), // Number of transfers; in this case each is 1 byte.
//         true           // Start immediately.
//     );
    
//     // We could choose to go and do something else whilst the DMA is doing its
//     // thing. In this case the processor has nothing else to do, so we just
//     // wait for the DMA to finish.
//     dma_channel_wait_for_finish_blocking(chan);
    
//     // The DMA has now copied our text from the transmit buffer (src) to the
//     // receive buffer (dst), so we can print it out from there.
//     puts(dst);

//     // PIO Blinking example
//     PIO pio = pio0;
//     uint offset = pio_add_program(pio, &blink_program);
//     printf("Loaded program at %d\n", offset);
    
//     #ifdef PICO_DEFAULT_LED_PIN
//     blink_pin_forever(pio, 0, offset, PICO_DEFAULT_LED_PIN, 3);
//     #else
//     blink_pin_forever(pio, 0, offset, 6, 3);
//     #endif
//     // For more pio examples see https://github.com/raspberrypi/pico-examples/tree/master/pio

//     // Timer example code - This example fires off the callback after 2000ms
//     add_alarm_in_ms(2000, alarm_callback, NULL, false);
//     // For more examples of timer use see https://github.com/raspberrypi/pico-examples/tree/master/timer

//     printf("System Clock Frequency is %d Hz\n", clock_get_hz(clk_sys));
//     printf("USB Clock Frequency is %d Hz\n", clock_get_hz(clk_usb));
//     // For more examples of clocks use see https://github.com/raspberrypi/pico-examples/tree/master/clocks

//     while (true) {
//         printf("Hello, world!\n");
//         sleep_ms(1000);
//     }
// }