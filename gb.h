#include "cgb_boot.h"
#include "dmg_boot.h"

/**
 * Returns a byte from the ROM file at the given address.
 */
uint8_t __not_in_flash_func(gb_rom_read)(struct gb_s *gb, const uint_fast32_t addr)
{
	(void) gb;
	return rom[addr];
}

/**
 * Returns a byte from the cartridge RAM at the given address.
 */
uint8_t gb_cart_ram_read(struct gb_s *gb, const uint_fast32_t addr)
{
	(void) gb;
	return ram[addr];
}

/**
 * Writes a given byte to the cartridge RAM at the given address.
 */
void gb_cart_ram_write(struct gb_s *gb, const uint_fast32_t addr,
		       const uint8_t val)
{
	ram[addr] = val;
	ram_changed = true;
}

/* MBC7 accelerometer callback — reads X/Y from LSM6DSO for Kirby Tilt etc. */
static void mbc7_get_accel(struct gb_s *gb_ctx, int16_t *x, int16_t *y)
{
	(void)gb_ctx;
	int16_t az;
	imu_read_accel_raw(x, y, &az);
}

/**
 * Ignore all errors.
 */
void gb_error(struct gb_s *gb, const enum gb_error_e gb_err, const uint16_t addr)
{
#if 1
	const char* gb_err_str[4] = {
			"UNKNOWN",
			"INVALID OPCODE",
			"INVALID READ",
			"INVALID WRITE"
		};
	printf("Error %d occurred: %s at %04X\n.\n", gb_err, gb_err_str[gb_err], addr);
//	abort();
#endif
}

#if ENABLE_BOOTROM
// Function that reads a byte from the boot ROM array
static uint8_t cgb_bootrom_read(struct gb_s *gb, const uint_fast16_t addr) {
    return cgb_boot_bin[addr];
}
static uint8_t dmg_bootrom_read(struct gb_s *gb, const uint_fast16_t addr) {
    return dmg_boot_bin[addr];
}
#endif

