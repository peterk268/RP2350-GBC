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
#define ENABLE_FRAME_DEBUGGING 0
#define ENABLE_BAT_MONITORING 1
#define BAT_IMMEDIATE_CHECK 1
#define BATTERY_TIMER_INTERVAL_MS 10000

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
