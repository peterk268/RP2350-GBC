#if ENABLE_SOUND
/**
 * Global variables for audio task
 * stream contains N=AUDIO_SAMPLES samples
 * each sample is 32 bits
 * 16 bits for the left channel + 16 bits for the right channel in stereo interleaved format)
 * This is intended to be played at AUDIO_SAMPLE_RATE Hz
 */
uint16_t *stream;

i2s_config_t i2s_config;
#endif

/** Definition of ROM data
 * We're going to erase and reprogram a region 2Mb from the end of the flash
 * Once done, we can access this at XIP_BASE + 2Mb.
 * Game Boy DMG ROM size ranges from 32768 bytes (e.g. Tetris) to 2,097,152 bytes (e.g. Pokemon Silver)
 */
#if ENABLE_PSRAM && !ROM_FLASH
uint8_t *rom;
#else
#define FLASH_TARGET_OFFSET (4 * 1024 * 1024)
const uint8_t *rom = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
#endif
enum gb_init_error_e ret;

uint8_t *ram = NULL;
bool gb_alloc_ram(void)
{
    ram = malloc(32768);   // goes to PSRAM on your system
    if (!ram) {
        printf("ERROR: Failed to alloc GB 32KB RAM block\n");
        return false;
    }

    memset(ram, 0, 32768);
    return true;
}

static palette_t *palette = NULL;

bool palette_heap_init(void) {
    palette = (palette_t *)malloc(sizeof(*palette));
    if (!palette) return false;
    memset(palette, 0, sizeof(*palette));
    return true;
}

void palette_heap_deinit(void) {
    if (palette) {
        free(palette);
        palette = NULL;
    }
}
static int8_t manual_palette_selected=-1;
static char *last_filename_raw = NULL;
bool last_filename_init(void) {
    last_filename_raw = (char *)malloc(FILENAME_MAX_LEN);
    if (!last_filename_raw) {
        return false;
    }
    last_filename_raw[0] = '\0'; // start as empty string
    return true;
}
void last_filename_deinit(void) {
    free(last_filename_raw);
    last_filename_raw = NULL;
}

bool auto_load_state = false;

static struct
{
	bool a	: 1;
	bool b	: 1;
	bool select	: 1;
	bool start	: 1;
	bool right	: 1;
	bool left	: 1;
	bool up	: 1;
	bool down	: 1;
} prev_joypad_bits;

/* Multicore command structure. */
union core_cmd {
    struct {
	/* Does nothing. */
#define CORE_CMD_NOP		0
	/* Set line "data" on the LCD. Pixel data is in pixels_buffer. */
#define CORE_CMD_LCD_LINE	1
	/* Control idle mode on the LCD. Limits colours to 2 bits. */
#define CORE_CMD_IDLE_SET	2
	/* Set a specific pixel. For debugging. */
#define CORE_CMD_SET_PIXEL	3
	uint8_t cmd;
	uint8_t unused1;
	uint8_t unused2;
	uint8_t data;
    };
    uint32_t full;
};

/* Pixel data is stored in here. */
static struct gb_s gb;

#define putstdio(x) write(1, x, strlen(x))

repeating_timer_t battery_timer;
repeating_timer_t pwr_led_timer;
repeating_timer_t watchdog_timer;

volatile bool battery_task_flag = false;
bool low_power = false;
bool low_power_shutdown = false;

static bool do_rtc_update = false;

static bool sd_busy = false;

bool ram_changed = false;

#if ENABLE_FPS_MONITORING
static uint32_t fps_counter = 0;
static uint32_t fps_last_time = 0;
#endif

static volatile bool g_in_game_menu = false;
static volatile bool g_request_exit_to_rom_selector = false;
static volatile bool g_request_exit_menu = false;

bool should_skip_audio_frame = false;

static enum {
    MODE_NORMAL,
    MODE_TURBO,
    MODE_POWERSAVE
} run_mode = MODE_NORMAL;

static uint8_t powersave_saved_button_brightness = 0;

// MARK: - Overclock
// Not stable. Unpredictable and heat is a problem. 320MHz is most I'll go.
#define SAFE_OVERCLOCK 1
// Keep normal clock as an integer multiple of DPI_PCLK (20MHz) so the DPI PIO pixel clock divider is integer.
// 340MHz = 17×20MHz: best audio balance (+0.39% at 44.1kHz, -1.18% at 48kHz) while still giving
// ~6% more throughput vs 320MHz for demanding GBC games (e.g. Shantae).
#define SYS_CLOCK_NORMAL_KHZ 340000u
// 378MHz = 15×25.2MHz: use this instead of 340MHz when HDMI output is enabled (HSTX hstx_csr_clkdiv=15).
#define SYS_CLOCK_HDMI_KHZ 378000u
// 300MHz = 15×20MHz: use this when playing 48kHz audio for a perfect integer I2S clock divider.
// At 340MHz, 48kHz has -1.18% pitch error (div=14); at 300MHz, div=12 gives exact 48kHz.
#define SYS_CLOCK_48KHZ_KHZ 300000u
// Underclock speed for battery-saving mode.
// With extreme battery save (audio disabled), we can go lower since audio quality is no longer a constraint.
// No we cannot go lower.. Some GBC games will run slow so lets keep 180MHz.
#if ENABLE_EXTREME_BATTERY_SAVE
#define POWERSAVE_CLOCK_KHZ 180000u
#else
#define POWERSAVE_CLOCK_KHZ 180000u
#endif

// Switch between normal DPI clock (340MHz, 1.25V) and HDMI clock (378MHz, 1.30V).
// 1.30V = VREG_VOLTAGE_MAX (no voltage limit disable needed) — safe per Pimoroni thermal tests (~33C).
// Device is docked/charging when HDMI is active so the extra draw is fine.
// After switching to HDMI, call video_output_reconfigure_clock() from Core 1 to update clk_hstx.
void switch_to_hdmi_clock(bool hdmi) {
    if (hdmi) {
        // Going up: raise voltage first, then frequency
        vreg_set_voltage(VREG_VOLTAGE_1_30);
        sleep_ms(10);
        set_sys_clock_khz(SYS_CLOCK_HDMI_KHZ, true);
    } else {
        // Going down: lower frequency first, then voltage
        set_sys_clock_khz(SYS_CLOCK_NORMAL_KHZ, true);
        vreg_set_voltage(VREG_VOLTAGE_1_25);
    }
    sleep_ms(10);
#if ENABLE_PSRAM
    sfe_psram_update_timing();
#endif
}

// audible click with changing the frequency with i2s running so we disabled it.
#define ALLOW_48KHz_PERFECT_PITCH 0
#if ALLOW_48KHz_PERFECT_PITCH
bool enabled_48khz = false;
// Switch to 300MHz when playing 48kHz audio (perfect integer I2S divider), back to 340MHz otherwise.
// Voltage stays at 1.25V — no change needed. Both clocks are integer multiples of DPI_PCLK (20MHz).
// PWM clkdiv is adjusted proportionally so backlight brightness stays constant — at 300MHz the minimum
// divider (1.0) gives ~1.172MHz; at 340MHz we set 340/300≈1.133 to match that same frequency.
void switch_to_48khz_clock(bool enable_48khz) {
    if (enable_48khz == enabled_48khz) return; // no change
    enabled_48khz = enable_48khz;
    if (enable_48khz) {
        // Going down: lower frequency first, voltage unchanged (1.25V is fine at 300MHz)
        set_sys_clock_khz(SYS_CLOCK_48KHZ_KHZ, true);
    } else {
        // Going up: raise frequency back to normal 340MHz
        set_sys_clock_khz(SYS_CLOCK_NORMAL_KHZ, true);
    }
    // Compensate LED PWM clkdiv so backlight frequency stays constant (~1.172 MHz) at both sys clocks.
    // At 300MHz: div=1.0 (hardware minimum). At 340MHz: div=340/300 to match.
    float led_div = enable_48khz ? 1.0f : ((float)SYS_CLOCK_NORMAL_KHZ / (float)SYS_CLOCK_48KHZ_KHZ);
    pwm_set_clkdiv(pwm_gpio_to_slice_num(GPIO_LCD_LED), led_div);  // GPIO_LCD_LED
    pwm_set_clkdiv(pwm_gpio_to_slice_num(GPIO_PWR_LED), led_div);  // GPIO_PWR_LED
    pwm_set_clkdiv(pwm_gpio_to_slice_num(GPIO_BUTTON_LED), led_div);  // GPIO_BUTTON_LED
    sleep_ms(10);
#if ENABLE_PSRAM
    sfe_psram_update_timing();
#endif
}
#endif
// Compensate LED PWM clkdiv when switching between 340MHz (normal) and POWERSAVE_CLOCK_KHZ.
// At POWERSAVE_CLOCK_KHZ: div=1.0 (hardware min). At 340MHz: div=ratio to match same PWM freq.
void reconfigure_led_pwm_for_underclock(bool underclocked) {
    float led_div = underclocked ? 1.0f : ((float)SYS_CLOCK_NORMAL_KHZ / (float)POWERSAVE_CLOCK_KHZ);
    pwm_set_clkdiv(pwm_gpio_to_slice_num(GPIO_LCD_LED),    led_div);
    pwm_set_clkdiv(pwm_gpio_to_slice_num(GPIO_PWR_LED),    led_div);
    pwm_set_clkdiv(pwm_gpio_to_slice_num(GPIO_BUTTON_LED), led_div);
}

void overclock_cpu(bool enable) {
    if (enable) {
#if !SAFE_OVERCLOCK
		vreg_disable_voltage_limit();
#endif
        // Going up: raise voltage first, then frequency
        vreg_set_voltage(SAFE_OVERCLOCK ? VREG_VOLTAGE_1_25 : VREG_VOLTAGE_1_70);
        sleep_ms(10);

        set_sys_clock_khz((SAFE_OVERCLOCK ? 360 : 520) * 1000, true);
        sleep_ms(10);
#if ENABLE_PSRAM
        sfe_psram_update_timing();
#endif
    } else {
        // "Disable overclock" path — returns to normal 360MHz operating clock.
        // Raise voltage first (1.25V), then frequency, for stability.
        vreg_set_voltage(VREG_VOLTAGE_1_25);
        sleep_ms(10);
        set_sys_clock_khz(SYS_CLOCK_NORMAL_KHZ, true);
        sleep_ms(10);
#if ENABLE_PSRAM
        sfe_psram_update_timing();
#endif
    }
}

// Clocking CPU down for battery-saving mode (POWERSAVE_CLOCK_KHZ).
// This does save good power, up to 100mW which equates to ~1.75h more battery life at mid level consumption,
//  so it's useful in a low power mode for non-demanding games like Pokemon.
// VREG_VOLTAGE_DEFAULT is 1.1V and gave the best power consumption. Going lower doesn't help.
// I'd also prefer to keep the default voltage for stability. Default clock speed is 150MHz.
void underclock_cpu(bool enable) {
	if (enable) {
		// Going down: lower frequency first, then voltage
		set_sys_clock_khz(POWERSAVE_CLOCK_KHZ, true);
		sleep_ms(10);

		vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
		sleep_ms(10);
#if ENABLE_PSRAM
        sfe_psram_update_timing();
#endif
	} else {
        // Going up: up voltage then frequency
        vreg_set_voltage(VREG_VOLTAGE_1_25);
        sleep_ms(10);

        set_sys_clock_khz(SYS_CLOCK_NORMAL_KHZ, true);
        sleep_ms(10);
#if ENABLE_PSRAM
        sfe_psram_update_timing();
#endif
	}
}

#define USE_XOSC_FOR_UNDERCLOCK 0

#if USE_XOSC_FOR_UNDERCLOCK
#include "hardware/regs/clocks.h"   // for CLOCKS_* constants
#include "hardware/pll.h"

static void run_sys_from_xosc_and_kill_pll_sys(void) {
    // Make clk_sys come from clk_ref (which is normally XOSC / 1)
    // 12*MHZ assumes a 12MHz crystal; if yours differs, use clock_get_hz(clk_ref) instead.
    clock_configure(clk_sys,
                    CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF,
                    0,
                    12 * MHZ,
                    12 * MHZ);

    // Keep peripheral clock sourced from clk_sys (important for UART/SPI timing expectations)
    clock_configure(clk_peri,
                    0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS,
                    12 * MHZ,
                    12 * MHZ);

    // Now that clk_sys is NOT using PLL_SYS anymore, shut PLL_SYS down to save power
    pll_deinit(pll_sys);
}
#endif

void hyper_underclock_cpu(bool enable) {
	if (enable) {
#if USE_XOSC_FOR_UNDERCLOCK
		run_sys_from_xosc_and_kill_pll_sys();
#else
		// Going down: lower frequency first, then voltage
		set_sys_clock_khz(20 * 1000, true);
		sleep_ms(10);

		vreg_set_voltage(VREG_VOLTAGE_1_00);
		sleep_ms(10);
#if ENABLE_PSRAM
        sfe_psram_update_timing();
#endif
#endif
	} else {
        // Going up: up voltage then frequency
        vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
        sleep_ms(10);

        set_sys_clock_khz(180 * 1000, true);
        sleep_ms(10);
#if ENABLE_PSRAM
        sfe_psram_update_timing();
#endif
	}
}

void underclock_cpu_for_psram(bool enable) {
	if (enable) {
		// Going down: lower frequency first, then voltage
		set_sys_clock_khz(140 * 1000, true);
		sleep_ms(10);

		vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
		sleep_ms(10);
#if ENABLE_PSRAM
        sfe_psram_update_timing();
#endif
	} else {
        // Going up: up voltage then frequency
        vreg_set_voltage(VREG_VOLTAGE_1_25);
        sleep_ms(10);

        set_sys_clock_khz(SYS_CLOCK_NORMAL_KHZ, true);
        sleep_ms(10);
#if ENABLE_PSRAM
        sfe_psram_update_timing();
#endif
	}
}

bool watchdog_callback(repeating_timer_t *rt) {
    watchdog_update();
    return true; // Return true to keep the timer running
}

void process_bat_percent();
void shutdown_peripherals(bool keep_i2c);
bool minimal_battery_monitoring_cb() {
#if ENABLE_BAT_MONITORING
	bool was_task_flagged = false;
	// Check battery status periodically
	if (battery_task_flag) {
		battery_task_flag = false;
		process_bat_percent();
		was_task_flagged = true;
	}
	if (low_power_shutdown) {
		release_power(); // Cut power hold
		sleep_ms(1);
		watchdog_disable();
		shutdown_screen(1500);
		shutdown_peripherals(true);
		sleep_ms(10);
	}
	while(low_power_shutdown) {
		process_bat_percent();
		sleep_ms(BATTERY_TIMER_INTERVAL_MS);
	}
	return was_task_flagged;
#endif
}
