// Peanut-GB emulator settings
#define ENABLE_LCD	1
#define ENABLE_RTC 0
#define ENABLE_SOUND	1
#define ENABLE_SDCARD	1
#define ENABLE_ROM_SELECTOR	1
#define ENABLE_SAVE_ON_POWER_OFF 1
#define ENABLE_AUTO_SAVE 0
#define ENABLE_BOOTROM 0
#define PEANUT_GB_HIGH_LCD_ACCURACY 1
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
#define ENABLE_BAT_MONITORING 0

#define WATCHDOG_TIMEOUT_MS 2000

#define SETTINGS_OFFSET (8 * 1024 * 1024)  // 8 MB offset, 4 KB page

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