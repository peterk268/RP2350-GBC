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
const uint8_t *rom;
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

static int lcd_fb_ready = 0;
static palette_t palette;	// Colour palette
static int8_t manual_palette_selected=-1;
char last_filename_raw[FILENAME_MAX_LEN];

static struct
{
	unsigned a	: 1;
	unsigned b	: 1;
	unsigned select	: 1;
	unsigned start	: 1;
	unsigned right	: 1;
	unsigned left	: 1;
	unsigned up	: 1;
	unsigned down	: 1;
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

volatile bool sd_busy = false;

bool ram_changed = false;

#if ENABLE_FPS_MONITORING
static uint32_t fps_counter = 0;
static uint32_t fps_last_time = 0;
#endif

bool should_skip_audio_frame = false;

static enum {
    MODE_NORMAL,
    MODE_TURBO,
    MODE_POWERSAVE
} run_mode = MODE_NORMAL;

// MARK: - Overclock
// Not stable. Unpredictable and heat is a problem. 300MHz is most I'll go.
#define SAFE_OVERCLOCK 1
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
    } else {
        // Going down: lower frequency first, then voltage
        set_sys_clock_khz(300 * 1000, true);
        sleep_ms(10);

        vreg_set_voltage(VREG_VOLTAGE_1_15);
        sleep_ms(10);
    }
}

// Clocking CPU to 260MHz similar to an earlier version of the Pico Pal with the RP2040
// More advanced GBC games will not reach 60fps with this and need 300MHz.
// This does save good power, up to 100mW which equates to ~1.75h more battery life at mid level consumption,
//  so it's useful in a low power mode for non-demanding games like Pokemon.
// VREG_VOLTAGE_DEFAULT is 1.1V and gave the best power consumption. Going lower doesn't help.
// I'd also prefer to keep the default voltage for stability. Default clock speed is 150MHz.
void underclock_cpu(bool enable) {
	if (enable) {
		// Going down: lower frequency first, then voltage
		set_sys_clock_khz(180 * 1000, true);
		sleep_ms(10);

		vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
		sleep_ms(10);
	} else {
        // Going up: up voltage then frequency
        vreg_set_voltage(VREG_VOLTAGE_1_15);
        sleep_ms(10);

        set_sys_clock_khz(300 * 1000, true);
        sleep_ms(10);
	}
}

void underclock_cpu_for_psram(bool enable) {
	if (enable) {
		// Going down: lower frequency first, then voltage
		set_sys_clock_khz(140 * 1000, true);
		sleep_ms(10);

		vreg_set_voltage(VREG_VOLTAGE_DEFAULT);
		sleep_ms(10);
	} else {
        // Going up: up voltage then frequency
        vreg_set_voltage(VREG_VOLTAGE_1_15);
        sleep_ms(10);

        set_sys_clock_khz(300 * 1000, true);
        sleep_ms(10);
	}
}

bool watchdog_callback(repeating_timer_t *rt) {
    watchdog_update();
    return true; // Return true to keep the timer running
}

void process_bat_percent();
void shutdown_peripherals(bool keep_i2c);
void minimal_battery_monitoring_cb() {
#if ENABLE_BAT_MONITORING
	// Check battery status periodically
	if (battery_task_flag) {
		battery_task_flag = false;
		process_bat_percent();
	}
	if (low_power_shutdown) {
		release_power(); // Cut power hold
		sleep_ms(1);
		watchdog_disable();
		shutdown_peripherals(true);
		sleep_ms(10);
	}
	while(low_power_shutdown) {
		process_bat_percent();
		sleep_ms(BATTERY_TIMER_INTERVAL_MS);
	}
#endif
}