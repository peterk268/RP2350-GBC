// Peanut-GB emulator settings
#define ENABLE_LCD	1
#define ENABLE_RTC 0
#define ENABLE_SOUND	1
#define ENABLE_SDCARD	1
#define ENABLE_ROM_SELECTOR	1
#define ENABLE_SAVE_ON_POWER_OFF 1
#define ENABLE_AUTO_SAVE 0
#define ENABLE_BOOTROM 0
#define PEANUT_GB_HIGH_LCD_ACCURACY 0
#define PEANUT_GB_USE_BIOS 0
#define PEANUT_FULL_GBC_SUPPORT 1
// Place hot emulation functions in SRAM to avoid XIP flash cache misses
#define PGB_HOT __attribute__((hot, section(".time_critical.peanut_gb")))
/* Use DMA for all drawing to LCD. Benefits aren't fully realised at the moment
 * due to busy loops waiting for DMA completion. */
#define USE_DMA		0

/**
 * Reducing VSYNC calculation to lower multiple.
 * When setting a clock IRQ to DMG_CLOCK_FREQ_REDUCED, count to
 * SCREEN_REFRESH_CYCLES_REDUCED to obtain the time required each VSYNC.
 * DMG_CLOCK_FREQ_REDUCED = 2^18, and SCREEN_REFRESH_CYCLES_REDUCED = 4389.
 * Currently unused.
 */
#define VSYNC_REDUCTION_FACTOR 16u
#define SCREEN_REFRESH_CYCLES_REDUCED (SCREEN_REFRESH_CYCLES/VSYNC_REDUCTION_FACTOR)
#define DMG_CLOCK_FREQ_REDUCED (DMG_CLOCK_FREQ/VSYNC_REDUCTION_FACTOR)

#define GBC_MAIN_APP 0

#define DISPLAY_SCALE 2
#define USE_IPS_LCD 1
#define ENABLE_FRAME_DEBUGGING 0
#define ENABLE_FPS_MONITORING 1
/* Log LCD ON/OFF transitions and draw_line call counts to serial.
 * Useful for diagnosing black screen issues (e.g. with GB Studio games). */
#define DEBUG_LCD_STATE 0
// Log VRAM writes, DMA register setup, KEY1/STOP (double-speed), and MBC5 PC on bank writes.
#define DEBUG_VRAM_DMA 0

// BFI strobe frequency. Must be 120 or 240 (must divide evenly into 120fps).
// 120Hz = 1 cycle/frame (160 scanlines/phase), 240Hz = 2 cycles/frame (80 scanlines/phase).
// 144Hz causes per-frame brightness variation (1.2 cycles/frame) and is not supported.
#define BFI_HZ 240
// Set to 1 to expose the Phosphor CRT mode in the menu.
// Phosphor requires interpolated sub-frames to look authentic; on 60fps content it
// just dims the same frame repeatedly and looks worse than hard BFI.
#define ENABLE_CRT_PHOSPHOR 0

// Set to 1 for vanilla audio: PRB 11, 256×Fs MCLK, no DSP features.
// Set to 0 for full DSP: PRB 25, 384×Fs MCLK, DRC, EQ, 3D.
#define VANILLA_AUDIO 0

#if VANILLA_AUDIO
#define ENABLE_EQ  0
#define ENABLE_DRC 0
#else
#define ENABLE_EQ  1
#define ENABLE_DRC 1
#endif

#define SKIP_FRAMES 0
// Its not stable unfortunately.. processor ain't fast enough.. yet. I might overclock more
#define ENABLE_120FPS_FASTFORWARD 0
#define UNDERCLOCK_CPU_IN_NORMAL_EMULATION 0
#define SKIP_AUDIO_FRAMES_IN_FRAME_SKIP 1
#define FPS_LIMITER_ENABLED 1

#define ENABLE_BAT_MONITORING 1
// Extreme battery save: shuts off audio amp, stops audio processing, kills button LEDs,
// and fixes LED PWM clkdiv for the 180MHz underclock instead of stepping LCD brightness.
#define ENABLE_EXTREME_BATTERY_SAVE 1
#define BAT_IMMEDIATE_CHECK 1
#define BATTERY_TIMER_INTERVAL_MS 10000
#define BAT_MONITOR_DEBUG 0
#define BAT_HAS_PERCENT_ISSUES 0
// 1: show battery rate as percent per hour in settings screen, 0: show signed current in mA.
#define SHOW_BAT_RATE_AS_PERCENT_PER_HOUR 1

#define ENABLE_PSRAM 1
#define ROM_FLASH 0
#define DEBUG_PSRAM 0

#define WATCHDOG_TIMEOUT_MS 2000
#define WATCHDOG_STARTUP_TIMEOUT_MS 750

#define LED_PHASE_OUT_PWR_DOWN 0
#define TIE_PWR_LED_TO_LCD 1

// I2C Troubles with 5.1K pull up fixed by using 1k
#define I2C_HAS_TROUBLES 0

#define SHOW_PICS_IN_MENU 0

#include <malloc.h>


uint32_t _getTotalHeap()
{
    extern char __StackLimit, __bss_end__;
    return &__StackLimit - &__bss_end__;
}
uint32_t getFreeHeap()
{
    struct mallinfo m = mallinfo();
    return _getTotalHeap() - m.uordblks;
}

void print_memory_usage() {
    struct mallinfo info = mallinfo();

    printf("Total allocated: %d bytes\n", info.uordblks);
    printf("Total free: %d bytes\n", info.fordblks);
    printf("Total heap size: %d bytes\n", info.arena);
    printf("Largest free block: %d bytes\n", info.ordblks);
    printf("Total Heap: %d bytes\n", _getTotalHeap());
    printf("Free Heap: %d bytes\n", getFreeHeap());

}

void HardFault_Handler(void) {
    __asm volatile
    (
        "TST lr, #4\n"
        "ITE EQ\n"
        "MRSEQ r0, MSP\n"
        "MRSNE r0, PSP\n"
        "B hardfault_handler_c\n"
    );
}

// MARK: - Debugging to see file and line that caused crash with this command
// arm-none-eabi-addr2line -e RP2350_GBC.elf 0xPCXXXX
void hardfault_handler_c(uint32_t *sp) {
    uint32_t stacked_r0 = sp[0];
    uint32_t stacked_r1 = sp[1];
    uint32_t stacked_r2 = sp[2];
    uint32_t stacked_r3 = sp[3];
    uint32_t stacked_r12 = sp[4];
    uint32_t stacked_lr = sp[5];
    uint32_t stacked_pc = sp[6];
    uint32_t stacked_psr = sp[7];

    printf("\n[HARDFAULT]\n");
    printf(" R0  = 0x%08lX\n", stacked_r0);
    printf(" R1  = 0x%08lX\n", stacked_r1);
    printf(" R2  = 0x%08lX\n", stacked_r2);
    printf(" R3  = 0x%08lX\n", stacked_r3);
    printf(" R12 = 0x%08lX\n", stacked_r12);
    printf(" LR  = 0x%08lX\n", stacked_lr);
    printf(" PC  = 0x%08lX\n", stacked_pc); // <- This is where it crashed
    printf(" PSR = 0x%08lX\n", stacked_psr);

    while (1);
}

// --- System settings ---
#define SETTINGS_DIR       "/settings"
#define SYSTEM_FILE_PATH   SETTINGS_DIR "/system.bin"
#define ROM_FILE_PATH      SETTINGS_DIR "/rom.bin"
#define ALARM_FILE_PATH    SETTINGS_DIR "/alarm.bin"
#define MP3_SETTINGS_PATH  SETTINGS_DIR "/mp3.bin"
#define FILENAME_MAX_LEN   256
#define PATH_MAX_LEN 512

bool read_system_settings(uint8_t *lcd_brightness,
                          uint8_t *button_brightness,
                          uint8_t *power_brightness,
                          int8_t  *selected_palette,
                          uint8_t *wash_out_level,
                          char last_filename_raw[FILENAME_MAX_LEN],
                          bool *auto_load_state,
                          uint8_t *out_crt_mode);

void save_system_settings(uint8_t lcd_brightness,
                          uint8_t button_brightness,
                          uint8_t power_brightness,
                          int8_t  selected_palette,
                          uint8_t wash_out_level,
                          char last_filename_raw[FILENAME_MAX_LEN],
                          bool auto_load_state,
                          uint8_t in_crt_mode,
                          bool hold_sd_busy);

// --- ROM settings ---
bool read_rom_settings(char *out_filename,
                       size_t max_len,
                       uint8_t *battery_slot,
                       uint8_t *state_slot,
                       bool unmount);

void save_rom_settings(const char *filename,
                       uint8_t battery_slot,
                       uint8_t state_slot);
