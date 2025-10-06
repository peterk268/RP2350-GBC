// Peanut-GB emulator settings
#define ENABLE_LCD	1
#define ENABLE_RTC 0
#define ENABLE_SOUND	1
#define ENABLE_SDCARD	1
#define ENABLE_ROM_SELECTOR	1
#define ENABLE_SAVE_ON_POWER_OFF 1
#define ENABLE_AUTO_SAVE 0
#define ENABLE_BOOTROM 0
#warning "Keep monitoring this, I prefer not to use it for efficiency and I don't quite see the difference."
#define PEANUT_GB_HIGH_LCD_ACCURACY 0
#define PEANUT_GB_USE_BIOS 0
#define PEANUT_FULL_GBC_SUPPORT 1
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

#define DISPLAY_SCALE 2
#define USE_IPS_LCD 1
#define ENABLE_FRAME_DEBUGGING 0
#define ENABLE_FPS_MONITORING 1

#define SKIP_FRAMES 0
// Its not stable unfortunately.. processor ain't fast enough.. yet. I might overclock more
#define ENABLE_120FPS_FASTFORWARD 0
#define UNDERCLOCK_CPU_IN_NORMAL_EMULATION 0

#define ENABLE_BAT_MONITORING 1
#define BAT_IMMEDIATE_CHECK 1
#define BATTERY_TIMER_INTERVAL_MS 10000

#define WATCHDOG_TIMEOUT_MS 2000

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

#define SETTINGS_OFFSET (8 * 1024 * 1024)  // 8 MB offset, 4 KB page
#define SYSTEM_SETTINGS_OFFSET  (SETTINGS_OFFSET)                   // base
#define ROM_SETTINGS_OFFSET     (SETTINGS_OFFSET + FLASH_PAGE_SIZE) // next page

#define FILENAME_MAX_LEN 256

// --- Structs ---
typedef struct {
    uint32_t magic;
    uint8_t lcd_brightness;
    uint8_t button_brightness;
    uint8_t power_brightness;
    int8_t selected_palette;
    uint8_t wash_out_level;
} system_settings_t;

typedef struct {
    uint32_t magic;
    char last_filename[FILENAME_MAX_LEN];
    uint8_t battery_slot;
    uint8_t state_slot;
} rom_settings_t;


// --- System settings ---
void save_system_settings(uint8_t lcd_brightness, uint8_t button_brightness,
                          uint8_t power_brightness, int8_t selected_palette,
                          uint8_t wash_out_level) {
    system_settings_t s = {
        .magic = 0xCAFEBABE,
        .lcd_brightness = lcd_brightness,
        .button_brightness = button_brightness,
        .power_brightness = power_brightness,
        .selected_palette = selected_palette,
        .wash_out_level = wash_out_level
    };

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(SYSTEM_SETTINGS_OFFSET, FLASH_PAGE_SIZE);
    flash_range_program(SYSTEM_SETTINGS_OFFSET, (uint8_t*)&s, sizeof(s));
    restore_interrupts(ints);
}

bool read_system_settings(uint8_t *lcd_brightness, uint8_t *button_brightness,
                          uint8_t *power_brightness, int8_t *selected_palette,
                          uint8_t *wash_out_level) {
    const system_settings_t *s = (const system_settings_t*)(XIP_BASE + SYSTEM_SETTINGS_OFFSET);
    if (s->magic != 0xCAFEBABE) return false;

    *lcd_brightness = s->lcd_brightness;
    *button_brightness = s->button_brightness;
    *power_brightness = s->power_brightness;
    *selected_palette = s->selected_palette;
    *wash_out_level = s->wash_out_level;
    return true;
}

void save_system_settings_if_changed(uint8_t lcd_brightness,
                                     uint8_t button_brightness,
                                     uint8_t power_brightness,
                                     int8_t selected_palette,
                                     uint8_t wash_out_level) {
    uint8_t saved_lcd, saved_button, saved_power;
    int8_t saved_palette;
    uint8_t saved_washout;

    // Read current settings from flash
    bool valid = read_system_settings(&saved_lcd, &saved_button,
                                      &saved_power, &saved_palette,
                                      &saved_washout);

    // If invalid or any value changed, save to flash
    if (!valid ||
        saved_lcd     != lcd_brightness ||
        saved_button  != button_brightness ||
        saved_power   != power_brightness ||
        saved_palette != selected_palette ||
        saved_washout != wash_out_level) {

        save_system_settings(lcd_brightness,
                             button_brightness,
                             power_brightness,
                             selected_palette,
                             wash_out_level);
    }
}

// --- ROM settings ---
void save_rom_settings(const char *filename, uint8_t battery_slot, uint8_t state_slot) {
    rom_settings_t r = {0};
    r.magic = 0xA5A5A5A5;
    strncpy(r.last_filename, filename, FILENAME_MAX_LEN - 1);
    r.battery_slot = battery_slot;
    r.state_slot = state_slot;

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(ROM_SETTINGS_OFFSET, FLASH_PAGE_SIZE);
    flash_range_program(ROM_SETTINGS_OFFSET, (uint8_t*)&r, sizeof(r));
    restore_interrupts(ints);
}

bool read_rom_settings(char *out_filename, size_t max_len, uint8_t *battery_slot, uint8_t *state_slot) {
    const rom_settings_t *r = (const rom_settings_t*)(XIP_BASE + ROM_SETTINGS_OFFSET);
    if (r->magic != 0xA5A5A5A5) return false;

    strncpy(out_filename, r->last_filename, max_len - 1);
    out_filename[max_len - 1] = 0;
    *battery_slot = r->battery_slot;
    *state_slot = r->state_slot;
    return true;
}
