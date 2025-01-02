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
#define FLASH_TARGET_OFFSET (4 * 1024 * 1024)
const uint8_t *rom = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);
static unsigned char rom_bank0[65536];

static uint8_t ram[32768];
static int lcd_line_busy = 0;
static palette_t palette;	// Colour palette
static int8_t manual_palette_selected=-1;

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
static uint8_t pixels_buffer[LCD_WIDTH];
static struct gb_s *gbc;

#define putstdio(x) write(1, x, strlen(x))

repeating_timer_t timer;
