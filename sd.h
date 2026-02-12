#if ENABLE_SDCARD
/**
 * Load a save file from the SD card
 */

#define MAX_FILES 9

static lv_obj_t *rom_list;
static uint16_t num_page = 0;
static uint16_t num_file = 0;
static char (*filename)[256] = NULL;
static uint16_t selected = 0;

static bool sd_filename_init(void) {
    if (filename) return true;
    filename = (char (*)[256])malloc(MAX_FILES * 256);
    if (!filename) {
        printf("filename malloc fail\n");
        return false;
    }
    memset(filename, 0, MAX_FILES * 256);
    return true;
}

uint8_t sd_prev_lcd_led_duty_cycle;

void set_sd_busy(bool is_sd_busy) {
    // Atomic read current state
    bool cur = __atomic_load_n(&sd_busy, __ATOMIC_ACQUIRE);
    if (cur == is_sd_busy) {
        return; // No change
    }

    // Publish new state
    __atomic_store_n(&sd_busy, is_sd_busy, __ATOMIC_RELEASE);
   
    // Wake core1 from parked state regardless so it can observe the new state immediately
#if ENABLE_DOORBELL
    multicore_doorbell_set_other_core(g_core1_db);
#endif
    __sev();

    if (sd_busy) {
        sd_prev_lcd_led_duty_cycle = lcd_led_duty_cycle;
        // LCD LED off
        decrease_lcd_brightness(MAX_BRIGHTNESS);
        // sleep_ms(20);
    } else {
        sleep_ms(20); // waiting the 16.7ms to let a frame be output to the lcd.
        increase_lcd_brightness(sd_prev_lcd_led_duty_cycle);
    }
}


typedef enum {
    SAVE_BATTERY,   // .sav
    SAVE_STATE,     // .sta
    SAVE_SCREENSHOT // .png
} save_type_t;

/**
 * Build a full save path for a ROM, supporting battery saves, states, and screenshots.
 *
 * @param rom_filename  The ROM filename
 * @param type          The type of save
 * @param slot          For battery: battery slot (0=default save.sav)
 *                      For state/screenshot: state/screenshot number
 * @param out_path      Buffer to store the resulting full path
 * @param out_path_size Size of out_path buffer
 * @return 0 on success, non-zero on error
 */
int build_save_path(const char *rom_filename, save_type_t type, int slot, char *out_path, size_t out_path_size) {
    char folder[256];
    char folder_name[256];

    // Copy ROM filename and strip extension
    strncpy(folder_name, rom_filename, sizeof(folder_name));
    folder_name[sizeof(folder_name)-1] = 0;
    char *dot = strrchr(folder_name, '.');
    if(dot) *dot = 0;

	char safe_name[256];
	const char *src = folder_name;  // folder_name has extension stripped
	char *dst = safe_name;
	while(*src && (dst - safe_name) < sizeof(safe_name)-1) {
		char c = *src++;
		if( (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
			(c >= '0' && c <= '9') || c == '_' || c == '-' ) { // remove space
			*dst++ = c;
		} else {
			*dst++ = '_';
		}
	}
	*dst = '\0';
	printf("Creating folder: GBC/Saves/%s\n", safe_name);


    // Build folder path
    snprintf(folder, sizeof(folder), "GBC/Saves/%s", safe_name);

	f_mkdir("GBC");
	f_mkdir("GBC/Saves");
    // Make sure folder exists
    FRESULT fr = f_mkdir(folder);
    if(fr != FR_OK && fr != FR_EXIST) {
        printf("E f_mkdir error: %d\n", fr);
        return fr;
    }

    // Build file path based on type
    switch(type) {
        case SAVE_BATTERY:
            if(slot <= 0)
                snprintf(out_path, out_path_size, "%s/save.sav", folder);
            else
                snprintf(out_path, out_path_size, "%s/save%d.sav", folder, slot);
            break;
        case SAVE_STATE:
            snprintf(out_path, out_path_size, "%s/state%d.sta", folder, slot);
            break;
        case SAVE_SCREENSHOT:
            snprintf(out_path, out_path_size, "%s/screenshot%d.png", folder, slot);
            break;
        default:
            return -1;
    }

    return 0;
}

/**
 * Build save path using ROM filename and battery slot stored in flash
 */
int build_save_path_from_flash(save_type_t type, char *out_path, size_t out_path_size, int override_slot) {
    char rom_filename[FILENAME_MAX_LEN];
    uint8_t battery_slot, save_state_slot;

    if(!read_rom_settings(rom_filename, sizeof(rom_filename), &battery_slot, &save_state_slot, false)) {
        printf("E: No valid flash settings found\n");
        return -1;
    }

    // For SAVE_BATTERY, use the flash battery_slot as the "slot" parameter
    // we can override it if needed by passing override_slot >= 0
    int effective_slot = override_slot < 0 ? ((type == SAVE_BATTERY) ? battery_slot : save_state_slot) : override_slot;

    return build_save_path(rom_filename, type, effective_slot, out_path, out_path_size);
}


void read_cart_ram_file(struct gb_s *gb) {

	uint_fast32_t save_size;
	UINT br;

	save_size=gb_get_save_size(gb);
	if(save_size>0) {
        // sd busy not needed here since its held until emulation starts
		// set_sd_busy(true);

		sd_card_t *pSD=sd_get_by_num(0);
		FRESULT fr=f_mount(&pSD->fatfs,pSD->pcName,1);
		if (FR_OK!=fr) {
			printf("E f_mount error: %s (%d)\n",FRESULT_str(fr),fr);
			return;
		}

		char *save_path = (char *)malloc(PATH_MAX_LEN);
        if (!save_path) {
            printf("E malloc save_path failed\n");
            return; // or handle error
        }

        int rc = build_save_path_from_flash(SAVE_BATTERY, save_path, PATH_MAX_LEN, -1);
        if (rc != 0) {
            printf("E build_save_path_from_flash(SAVE_BATTERY) failed\n");
            free(save_path);
            return;
        }

		FIL fil;
		fr=f_open(&fil,save_path,FA_READ);
		if (fr==FR_OK) {
			f_read(&fil,ram,f_size(&fil),&br);
		} else {
			printf("E f_open(%s) error: %s (%d)\n",save_path,FRESULT_str(fr),fr);
		}
		
		fr=f_close(&fil);
		if(fr!=FR_OK) {
			printf("E f_close error: %s (%d)\n", FRESULT_str(fr), fr);
		}
		f_unmount(pSD->pcName);	

		// set_sd_busy(false);
		printf("I read_cart_ram_file(%s) COMPLETE (%lu bytes)\n",save_path,save_size);
        free(save_path);
	}
}

/**
 * Write a save file to the SD card
 */
void write_cart_ram_file(struct gb_s *gb, bool hold_sd_busy) {
	uint_fast32_t save_size;
	UINT bw;
	
	save_size=gb_get_save_size(gb);
	if(save_size>0) {
		set_sd_busy(true);

		sd_card_t *pSD=sd_get_by_num(0);
		FRESULT fr=f_mount(&pSD->fatfs,pSD->pcName,1);
		if (FR_OK!=fr) {
			printf("E f_mount error: %s (%d)\n",FRESULT_str(fr),fr);
			return;
		}

		char *save_path = (char *)malloc(PATH_MAX_LEN);
        if (!save_path) {
            printf("E malloc save_path failed\n");
            return; // or handle error
        }

        int rc = build_save_path_from_flash(SAVE_BATTERY, save_path, PATH_MAX_LEN, -1);
        if (rc != 0) {
            printf("E build_save_path_from_flash(SAVE_BATTERY) failed\n");
            free(save_path);
            return;
        }

		FIL fil;
		fr=f_open(&fil,save_path,FA_CREATE_ALWAYS | FA_WRITE);
		if (fr==FR_OK) {
			f_write(&fil,ram,save_size,&bw);
		} else {
			printf("E f_open(%s) error: %s (%d)\n",save_path,FRESULT_str(fr),fr);
		}
		
		fr=f_close(&fil);
		if(fr!=FR_OK) {
			printf("E f_close error: %s (%d)\n", FRESULT_str(fr), fr);
		}
		f_unmount(pSD->pcName);
        if (!hold_sd_busy)
            set_sd_busy(false);
		printf("I write_cart_ram_file(%s) COMPLETE (%lu bytes)\n",save_path,save_size);
        free(save_path);
	} else {
        if (!hold_sd_busy)
            set_sd_busy(false);
    }
}

// ============================================================
// Peanut-GB Save State Serialization (SAFE: no function ptrs)
// ============================================================

#ifndef SAVE_STATE_MAGIC
#define SAVE_STATE_MAGIC 0x50504753u  // 'PPGS' (PicoPal GB Save) - pick anything
#endif

#ifndef SAVE_STATE_VERSION
#define SAVE_STATE_VERSION 1u
#endif

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t payload_size;   // sizeof(gb_save_state_t)
    uint32_t crc32;          // optional; 0 if unused
} save_state_header_t;

// NOTE: This struct ONLY contains deterministic emulation state.
//       No callbacks, no front-end pointers, no bitfields.
typedef struct {
    struct cpu_registers_s cpu_reg;
    struct count_s         counter;

    uint16_t selected_rom_bank;
    uint8_t  cart_ram_bank;
    uint8_t  enable_cart_ram;
    uint8_t  cart_mode_select;

    union cart_rtc rtc_latched;
    union cart_rtc rtc_real;

    uint8_t wram[WRAM_SIZE];
    uint8_t vram[VRAM_SIZE];
    uint8_t oam[OAM_SIZE];
    uint8_t hram_io[HRAM_IO_SIZE];

    uint8_t gb_ime;
    uint8_t gb_halt;

    uint8_t ram[32768];

#if PEANUT_FULL_GBC_SUPPORT
    struct {
        uint8_t  cgbMode;
        uint8_t  doubleSpeed;
        uint8_t  doubleSpeedPrep;
        uint8_t  wramBank;
        uint16_t wramBankOffset;
        uint8_t  vramBank;
        uint16_t vramBankOffset;

        uint16_t fixPalette[0x40];
        uint8_t  OAMPalette[0x40];
        uint8_t  BGPalette[0x40];
        uint8_t  OAMPaletteID;
        uint8_t  BGPaletteID;
        uint8_t  OAMPaletteInc;
        uint8_t  BGPaletteInc;

        uint8_t  dmaActive;
        uint8_t  dmaMode;
        uint8_t  dmaSize;
        uint16_t dmaSource;
        uint16_t dmaDest;
    } cgb;
#endif
} gb_save_state_t;


// ------------------------------------------------------------
// Optional CRC32 (FatFS-safe). If you don't want CRC, keep it
// but return 0 and skip validation.
// ------------------------------------------------------------
static uint32_t crc32_ieee(const void *data, size_t len) {
    // Small, straightforward CRC32 (IEEE 802.3). If you want to save flash,
    // you can remove CRC usage by always returning 0.
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    while (len--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}


// ============================================================
// WRITE SAVE STATE
// ============================================================
void write_cart_save_state(struct gb_s *gb, bool hold_sd_busy, int override_slot) {
    UINT bw = 0;
    FRESULT fr = FR_OK;

    sd_card_t *pSD = NULL;
    FIL fil;
    bool file_opened = false;

    char *save_path = NULL;

    gb_save_state_t *s = NULL;
    save_state_header_t hdr;

    if (!gb) return;

    set_sd_busy(true);

    save_path = (char *)malloc(PATH_MAX_LEN);
    if (!save_path) {
        printf("E malloc save_path failed\n");
        goto cleanup;
    }

    pSD = sd_get_by_num(0);
    fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) {
        printf("E f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        goto cleanup;
    }

    if (build_save_path_from_flash(SAVE_STATE, save_path, PATH_MAX_LEN, override_slot) != 0) {
        printf("E build_save_path_from_flash(SAVE_STATE) failed\n");
        goto cleanup;
    }

    // Allocate payload (avoid stack blow-up)
    s = (gb_save_state_t *)malloc(sizeof(*s));
    if (!s) {
        printf("E malloc failed for save state payload (%u bytes)\n", (unsigned)sizeof(*s));
        goto cleanup;
    }
    memset(s, 0, sizeof(*s));

    // Build payload
    s->cpu_reg = gb->cpu_reg;
    s->counter = gb->counter;

    s->selected_rom_bank = gb->selected_rom_bank;
    s->cart_ram_bank     = gb->cart_ram_bank;
    s->enable_cart_ram   = gb->enable_cart_ram;
    s->cart_mode_select  = gb->cart_mode_select;

    s->rtc_latched = gb->rtc_latched;
    s->rtc_real    = gb->rtc_real;

    s->gb_ime  = gb->gb_ime;
    s->gb_halt = gb->gb_halt;

    memcpy(s->ram, ram, sizeof(s->ram));

    memcpy(s->wram,    gb->wram,    WRAM_SIZE);
    memcpy(s->vram,    gb->vram,    VRAM_SIZE);
    memcpy(s->oam,     gb->oam,     OAM_SIZE);
    memcpy(s->hram_io, gb->hram_io, HRAM_IO_SIZE);

#if PEANUT_FULL_GBC_SUPPORT
    s->cgb.cgbMode         = gb->cgb.cgbMode;
    s->cgb.doubleSpeed     = gb->cgb.doubleSpeed;
    s->cgb.doubleSpeedPrep = gb->cgb.doubleSpeedPrep;
    s->cgb.wramBank        = gb->cgb.wramBank;
    s->cgb.wramBankOffset  = gb->cgb.wramBankOffset;
    s->cgb.vramBank        = gb->cgb.vramBank;
    s->cgb.vramBankOffset  = gb->cgb.vramBankOffset;

    memcpy(s->cgb.fixPalette, gb->cgb.fixPalette, sizeof(s->cgb.fixPalette));
    memcpy(s->cgb.OAMPalette, gb->cgb.OAMPalette, sizeof(s->cgb.OAMPalette));
    memcpy(s->cgb.BGPalette,  gb->cgb.BGPalette,  sizeof(s->cgb.BGPalette));

    s->cgb.OAMPaletteID  = gb->cgb.OAMPaletteID;
    s->cgb.BGPaletteID   = gb->cgb.BGPaletteID;
    s->cgb.OAMPaletteInc = gb->cgb.OAMPaletteInc;
    s->cgb.BGPaletteInc  = gb->cgb.BGPaletteInc;

    s->cgb.dmaActive = gb->cgb.dmaActive;
    s->cgb.dmaMode   = gb->cgb.dmaMode;
    s->cgb.dmaSize   = gb->cgb.dmaSize;
    s->cgb.dmaSource = gb->cgb.dmaSource;
    s->cgb.dmaDest   = gb->cgb.dmaDest;
#endif
    // End build payload

    hdr.magic        = SAVE_STATE_MAGIC;
    hdr.version      = SAVE_STATE_VERSION;
    hdr.payload_size = (uint32_t)sizeof(*s);
    hdr.crc32        = 0; // or: crc32_ieee(s, sizeof(*s));

    fr = f_open(&fil, save_path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("E f_open(%s) error: %s (%d)\n", save_path, FRESULT_str(fr), fr);
        goto cleanup;
    }
    file_opened = true;

    // Write header
    fr = f_write(&fil, &hdr, sizeof(hdr), &bw);
    if (fr != FR_OK || bw != sizeof(hdr)) {
        printf("E f_write(hdr) error: %s (%d), bw=%u\n", FRESULT_str(fr), fr, (unsigned)bw);
        goto cleanup;
    }

    // Write payload
    fr = f_write(&fil, s, sizeof(*s), &bw);
    if (fr != FR_OK || bw != sizeof(*s)) {
        printf("E f_write(state) error: %s (%d), bw=%u\n", FRESULT_str(fr), fr, (unsigned)bw);
        goto cleanup;
    }

    fr = f_close(&fil);
    file_opened = false;
    if (fr != FR_OK) {
        printf("E f_close error: %s (%d)\n", FRESULT_str(fr), fr);
        // continue cleanup anyway
    }

    printf("I write_cart_save_state(%s) COMPLETE (%u bytes payload)\n",
           save_path, (unsigned)sizeof(*s));

cleanup:
    if (file_opened) {
        FRESULT fr2 = f_close(&fil);
        if (fr2 != FR_OK) {
            printf("E f_close (cleanup) error: %s (%d)\n", FRESULT_str(fr2), fr2);
        }
    }

    if (pSD) {
        f_unmount(pSD->pcName);
    }

    if (s) free(s);

    if (save_path) {
        free(save_path);
        save_path = NULL;
    }


    if (!hold_sd_busy) set_sd_busy(false);
}

// ============================================================
// READ SAVE STATE
// Returns true if state loaded, false if missing/invalid.
// ============================================================
bool read_cart_save_state(struct gb_s *gb, int override_slot) {
    UINT br = 0;
    FRESULT fr;
    FIL fil;
    save_state_header_t hdr;
    char *save_path = NULL;
    gb_save_state_t *s = NULL;
    sd_card_t *pSD;
    bool success = false;

    if (!gb) return false;

    set_sd_busy(true);

    save_path = (char *)malloc(PATH_MAX_LEN);
    if (!save_path) {
        printf("E malloc save_path failed\n");
        goto cleanup;
    }

    pSD = sd_get_by_num(0);
    fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) {
        printf("E f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        goto cleanup;
    }

    if (build_save_path_from_flash(SAVE_STATE, save_path, PATH_MAX_LEN, override_slot) != 0) {
        printf("E build_save_path_from_flash(SAVE_STATE) failed\n");
        goto cleanup;
    }

    fr = f_open(&fil, save_path, FA_READ);
    if (fr != FR_OK) {
        printf("I no save state (%s): %s (%d)\n", save_path, FRESULT_str(fr), fr);
        goto cleanup;
    }

    fr = f_read(&fil, &hdr, sizeof(hdr), &br);
    if (fr != FR_OK || br != sizeof(hdr)) {
        printf("E f_read(hdr) error: %s (%d), br=%u\n", FRESULT_str(fr), fr, (unsigned)br);
        goto cleanup;
    }

    if (hdr.magic != SAVE_STATE_MAGIC) {
        printf("E save state magic mismatch: 0x%08lx\n", (unsigned long)hdr.magic);
        goto cleanup;
    }

    if (hdr.version != SAVE_STATE_VERSION) {
        printf("E save state version mismatch: file=%lu expected=%u\n",
               (unsigned long)hdr.version, (unsigned)SAVE_STATE_VERSION);
        goto cleanup;
    }

    if (hdr.payload_size != sizeof(gb_save_state_t)) {
        printf("E save state size mismatch: file=%lu expected=%u\n",
               (unsigned long)hdr.payload_size, (unsigned)sizeof(gb_save_state_t));
        goto cleanup;
    }

    s = (gb_save_state_t *)malloc(sizeof(*s));
    if (!s) {
        printf("E malloc failed for save state (%u bytes)\n", (unsigned)sizeof(*s));
        goto cleanup;
    }

    fr = f_read(&fil, s, sizeof(*s), &br);
    if (fr != FR_OK || br != sizeof(*s)) {
        printf("E f_read(state) error: %s (%d), br=%u\n", FRESULT_str(fr), fr, (unsigned)br);
        goto cleanup;
    }

    if (hdr.crc32 != 0) {
        uint32_t crc = crc32_ieee(s, sizeof(*s));
        if (crc != hdr.crc32) {
            printf("E save state CRC mismatch: file=0x%08lx calc=0x%08lx\n",
                   (unsigned long)hdr.crc32, (unsigned long)crc);
            goto cleanup;
        }
    }

    // ---- APPLY RESTORE ----
    memcpy(gb->wram,    s->wram,    WRAM_SIZE);
    memcpy(gb->vram,    s->vram,    VRAM_SIZE);
    memcpy(gb->oam,     s->oam,     OAM_SIZE);
    memcpy(gb->hram_io, s->hram_io, HRAM_IO_SIZE);

    memcpy(ram, s->ram, sizeof(s->ram));

    gb->cpu_reg = s->cpu_reg;
    gb->counter = s->counter;

    gb->selected_rom_bank = s->selected_rom_bank;
    gb->cart_ram_bank     = s->cart_ram_bank;
    gb->enable_cart_ram   = s->enable_cart_ram;
    gb->cart_mode_select  = s->cart_mode_select;

    gb->rtc_latched = s->rtc_latched;
    gb->rtc_real    = s->rtc_real;

    gb->gb_ime  = s->gb_ime;
    gb->gb_halt = s->gb_halt;

#if PEANUT_FULL_GBC_SUPPORT
    gb->cgb.cgbMode          = s->cgb.cgbMode;
    gb->cgb.doubleSpeed      = s->cgb.doubleSpeed;
    gb->cgb.doubleSpeedPrep  = s->cgb.doubleSpeedPrep;
    gb->cgb.wramBank         = s->cgb.wramBank;
    gb->cgb.wramBankOffset   = s->cgb.wramBankOffset;
    gb->cgb.vramBank         = s->cgb.vramBank;
    gb->cgb.vramBankOffset   = s->cgb.vramBankOffset;

    memcpy(gb->cgb.fixPalette,  s->cgb.fixPalette,  sizeof(s->cgb.fixPalette));
    memcpy(gb->cgb.OAMPalette,  s->cgb.OAMPalette,  sizeof(s->cgb.OAMPalette));
    memcpy(gb->cgb.BGPalette,   s->cgb.BGPalette,   sizeof(s->cgb.BGPalette));

    gb->cgb.OAMPaletteID    = s->cgb.OAMPaletteID;
    gb->cgb.BGPaletteID     = s->cgb.BGPaletteID;
    gb->cgb.OAMPaletteInc   = s->cgb.OAMPaletteInc;
    gb->cgb.BGPaletteInc    = s->cgb.BGPaletteInc;

    gb->cgb.dmaActive       = s->cgb.dmaActive;
    gb->cgb.dmaMode         = s->cgb.dmaMode;
    gb->cgb.dmaSize         = s->cgb.dmaSize;
    gb->cgb.dmaSource       = s->cgb.dmaSource;
    gb->cgb.dmaDest         = s->cgb.dmaDest;
#endif
    // ---- END APPLY RESTORE ----

    success = true;
    printf("I read_cart_save_state(%s) COMPLETE (%u bytes payload)\n",
           save_path, (unsigned)sizeof(*s));

cleanup:
    // close/unmount only if they were opened/mounted; keep it simple:
    // If f_open failed, f_close is not valid. Track a bool if you want to be strict.
    f_close(&fil);                // optional: guard with a "file_opened" bool
    if (pSD) f_unmount(pSD->pcName);

    if (s) free(s);
    if (save_path) { free(save_path); save_path = NULL; }

    set_sd_busy(false);

    return success && (fr == FR_OK);
}

// ------------------------------------------------------------
// Small helpers: big-endian write, CRC32, Adler32
// ------------------------------------------------------------
static void be32(uint8_t out[4], uint32_t v) {
    out[0] = (uint8_t)((v >> 24) & 0xFF);
    out[1] = (uint8_t)((v >> 16) & 0xFF);
    out[2] = (uint8_t)((v >>  8) & 0xFF);
    out[3] = (uint8_t)((v >>  0) & 0xFF);
}

static uint32_t crc32_update(uint32_t crc, const void *data, size_t len) {
    const uint8_t *p = (const uint8_t*)data;
    crc = ~crc;
    while (len--) {
        crc ^= *p++;
        for (int k = 0; k < 8; k++) {
            uint32_t mask = -(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

// Adler32 for zlib stream trailer
static void adler32_update(uint32_t *a, uint32_t *b, const uint8_t *data, size_t len) {
    // Adler32 mod prime 65521
    const uint32_t MOD = 65521u;
    uint32_t aa = *a;
    uint32_t bb = *b;
    while (len--) {
        aa += *data++;
        if (aa >= MOD) aa -= MOD;
        bb += aa;
        bb %= MOD;
    }
    *a = aa;
    *b = bb;
}

// ------------------------------------------------------------
// Convert RGB565 -> RGB888 (nice expansion)
// ------------------------------------------------------------
static inline void rgb565_to_rgb888(uint16_t p, uint8_t *r, uint8_t *g, uint8_t *b) {
    uint8_t r5 = (p >> 11) & 0x1F;
    uint8_t g6 = (p >>  5) & 0x3F;
    uint8_t b5 = (p >>  0) & 0x1F;

    // Expand to 8-bit: replicate MSBs
    *r = (r5 << 3) | (r5 >> 2);
    *g = (g6 << 2) | (g6 >> 4);
    *b = (b5 << 3) | (b5 >> 2);
}

// ------------------------------------------------------------
// Build screenshot path using ROM filename stored in flash
// (uses your existing read_rom_settings + build_save_path)
// ------------------------------------------------------------
static int build_screenshot_path_from_flash(int screenshot_num, char *out_path, size_t out_path_size) {
    char rom_filename[FILENAME_MAX_LEN];
    uint8_t battery_slot, save_state_slot;

    if (!read_rom_settings(rom_filename, sizeof(rom_filename), &battery_slot, &save_state_slot, false)) {
        printf("E: No valid flash settings found\n");
        return -1;
    }

    // screenshot_num is the slot for screenshotN.png
    return build_save_path(rom_filename, SAVE_SCREENSHOT, screenshot_num, out_path, out_path_size);
}

// ------------------------------------------------------------
// Chunk writer that streams and maintains CRC
// ------------------------------------------------------------
static FRESULT write_bytes(FIL *fil, const void *buf, UINT len) {
    UINT bw = 0;
    FRESULT fr = f_write(fil, buf, len, &bw);
    if (fr != FR_OK) return fr;
    if (bw != len)   return FR_DISK_ERR;
    return FR_OK;
}

static FRESULT write_png_chunk_begin(FIL *fil, const char type[4], uint32_t data_len, uint32_t *crc_io) {
    uint8_t len_be[4];
    be32(len_be, data_len);

    FRESULT fr = write_bytes(fil, len_be, 4);
    if (fr != FR_OK) return fr;

    fr = write_bytes(fil, type, 4);
    if (fr != FR_OK) return fr;

    // CRC starts over type+data
    uint32_t crc = 0;
    crc = crc32_update(crc, type, 4);
    *crc_io = crc;
    return FR_OK;
}

static FRESULT write_png_chunk_data(FIL *fil, const void *data, uint32_t len, uint32_t *crc_io) {
    FRESULT fr = write_bytes(fil, data, (UINT)len);
    if (fr != FR_OK) return fr;

    *crc_io = crc32_update(*crc_io, data, len);
    return FR_OK;
}

static FRESULT write_png_chunk_end(FIL *fil, uint32_t crc) {
    uint8_t crc_be[4];
    be32(crc_be, crc);
    return write_bytes(fil, crc_be, 4);
}

// Returns true and sets *out_num to the first available screenshot index.
// Returns false if it couldn't find one (or an unexpected FS error happened).
static bool screenshot_find_free_num(sd_card_t *pSD, int *out_num) {
    if (!out_num) return false;

    FILINFO fno;
    char path[PATH_MAX_LEN];

    // Start at 0; you can start at 1 if you prefer.
    for (int n = 0; n < 100000; n++) { // cap so we don't loop forever
        if (build_screenshot_path_from_flash(n, path, sizeof(path)) != 0) {
            return false;
        }

        FRESULT fr = f_stat(path, &fno);
        if (fr == FR_NO_FILE) {
            *out_num = n;
            return true; // free slot found
        }
        if (fr != FR_OK) {
            // Some other filesystem error
            printf("E f_stat(%s) error: %s (%d)\n", path, FRESULT_str(fr), fr);
            return false;
        }

        // FR_OK => file exists, keep going
    }

    printf("E screenshot_find_free_num: ran out of indices\n");
    return false;
}

// ------------------------------------------------------------
// Main: Write screenshot PNG from framebuffer (RGB565)
// ------------------------------------------------------------
bool write_screenshot_png_from_fb(const framebuffer_t *front_fb,
                                  int screenshot_num,
                                  bool hold_sd_busy)
{
    if (!front_fb) return false;

    // Your existing busy indicator
    set_sd_busy(true);

    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) {
        printf("E f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        if (!hold_sd_busy) set_sd_busy(false);
        return false;
    }

    int use_num = screenshot_num;

    if (use_num < 0) {
        if (!screenshot_find_free_num(pSD, &use_num)) {
            printf("E could not find free screenshot number\n");
            f_unmount(pSD->pcName);
            if (!hold_sd_busy) set_sd_busy(false);
            return false;
        }
    }


    char path[PATH_MAX_LEN];
    if (build_screenshot_path_from_flash(use_num, path, sizeof(path)) != 0) {
        printf("E build_screenshot_path_from_flash failed\n");
        f_unmount(pSD->pcName);
        if (!hold_sd_busy) set_sd_busy(false);
        return false;
    }

    FIL fil;
    fr = f_open(&fil, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        printf("E f_open(%s) error: %s (%d)\n", path, FRESULT_str(fr), fr);
        f_unmount(pSD->pcName);
        if (!hold_sd_busy) set_sd_busy(false);
        return false;
    }

    // ----------------------------
    // PNG signature
    // ----------------------------
    static const uint8_t png_sig[8] = { 137,80,78,71,13,10,26,10 };
    fr = write_bytes(&fil, png_sig, 8);
    if (fr != FR_OK) goto fail;

    // ----------------------------
    // IHDR chunk (13 bytes)
    // ----------------------------
    {
        uint8_t ihdr[13];
        // width, height
        be32(&ihdr[0],  LCD_WIDTH);
        be32(&ihdr[4],  LCD_HEIGHT);
        ihdr[8]  = 8;  // bit depth
        ihdr[9]  = 2;  // color type: truecolor RGB
        ihdr[10] = 0;  // compression
        ihdr[11] = 0;  // filter
        ihdr[12] = 0;  // interlace

        uint32_t crc = 0;
        fr = write_png_chunk_begin(&fil, "IHDR", 13, &crc);
        if (fr != FR_OK) goto fail;
        fr = write_png_chunk_data(&fil, ihdr, 13, &crc);
        if (fr != FR_OK) goto fail;
        fr = write_png_chunk_end(&fil, crc);
        if (fr != FR_OK) goto fail;
    }

    // ----------------------------
    // IDAT chunk:
    // We write a zlib stream containing uncompressed scanlines:
    // Each scanline = [filter=0][RGBRGB...]
    // zlib format: 2-byte header + stored blocks + Adler32
    // ----------------------------
    const uint32_t row_bytes = 1 + (3 * LCD_WIDTH);       // filter + RGB
    const uint32_t raw_bytes = row_bytes * LCD_HEIGHT;    // total uncompressed data
    const uint32_t max_block = 65535u;

    // number of stored blocks needed
    uint32_t blocks = (raw_bytes + max_block - 1) / max_block;

    // zlib stream size: header(2) + blocks*(5) + raw_bytes + adler(4)
    uint32_t idat_data_len = 2 + blocks * 5 + raw_bytes + 4;

    uint32_t idat_crc = 0;
    fr = write_png_chunk_begin(&fil, "IDAT", idat_data_len, &idat_crc);
    if (fr != FR_OK) goto fail;

    // zlib header: CMF/FLG
    // 0x78 0x01 = deflate, 32K window, fast/none (valid for stored blocks)
    {
        uint8_t zhdr[2] = { 0x78, 0x01 };
        fr = write_png_chunk_data(&fil, zhdr, 2, &idat_crc);
        if (fr != FR_OK) goto fail;
    }

    // Adler32 over the *raw* (uncompressed) scanline bytes
    uint32_t adler_a = 1;
    uint32_t adler_b = 0;

    // We generate scanlines on the fly into a small row buffer
    uint8_t rowbuf[1 + 3 * LCD_WIDTH];

    // Stream raw bytes with stored blocks
    uint32_t remaining = raw_bytes;
    uint32_t raw_pos = 0; // counts how many raw bytes we've emitted (for block boundaries)

    for (uint32_t bi = 0; bi < blocks; bi++) {
        uint32_t block_len = (remaining > max_block) ? max_block : remaining;
        remaining -= block_len;

        // Stored block header:
        // 1 byte: BFINAL(1) + BTYPE(2) -> for stored: BTYPE=00
        // then LEN (2 LE), NLEN (2 LE)
        uint8_t bhdr[5];
        uint8_t bfinal = (bi == (blocks - 1)) ? 1 : 0;
        bhdr[0] = (uint8_t)(bfinal); // BTYPE=00, so just BFINAL in bit0

        uint16_t LEN  = (uint16_t)block_len;
        uint16_t NLEN = (uint16_t)~LEN;

        bhdr[1] = (uint8_t)(LEN & 0xFF);
        bhdr[2] = (uint8_t)((LEN >> 8) & 0xFF);
        bhdr[3] = (uint8_t)(NLEN & 0xFF);
        bhdr[4] = (uint8_t)((NLEN >> 8) & 0xFF);

        fr = write_png_chunk_data(&fil, bhdr, 5, &idat_crc);
        if (fr != FR_OK) goto fail;

        // Now emit block_len raw bytes.
        // Our raw data is scanlines, so we generate by rows and slice as needed.
        uint32_t block_left = block_len;

        while (block_left > 0) {
            // Determine which row we're in and offset within that row
            uint32_t row_index = raw_pos / row_bytes;
            uint32_t row_off   = raw_pos % row_bytes;

            // Build row buffer if we are at row start
            if (row_off == 0) {
                rowbuf[0] = 0; // filter type 0
                for (uint32_t x = 0; x < LCD_WIDTH; x++) {
                    uint8_t r, g, b;
                    rgb565_to_rgb888(front_fb->data[row_index][x], &r, &g, &b);
                    rowbuf[1 + 3*x + 0] = r;
                    rowbuf[1 + 3*x + 1] = g;
                    rowbuf[1 + 3*x + 2] = b;
                }
            }

            // Emit as much as we can from current row
            uint32_t can = row_bytes - row_off;
            if (can > block_left) can = block_left;

            // Update Adler32 with the raw bytes we are emitting
            adler32_update(&adler_a, &adler_b, &rowbuf[row_off], can);

            fr = write_png_chunk_data(&fil, &rowbuf[row_off], can, &idat_crc);
            if (fr != FR_OK) goto fail;

            raw_pos   += can;
            block_left -= can;
        }
    }

    // Write Adler32 (big-endian) at end of zlib stream
    {
        uint32_t adler = (adler_b << 16) | adler_a;
        uint8_t adler_be[4];
        be32(adler_be, adler);
        fr = write_png_chunk_data(&fil, adler_be, 4, &idat_crc);
        if (fr != FR_OK) goto fail;
    }

    fr = write_png_chunk_end(&fil, idat_crc);
    if (fr != FR_OK) goto fail;

    // ----------------------------
    // IEND chunk
    // ----------------------------
    {
        uint32_t crc = 0;
        fr = write_png_chunk_begin(&fil, "IEND", 0, &crc);
        if (fr != FR_OK) goto fail;
        fr = write_png_chunk_end(&fil, crc);
        if (fr != FR_OK) goto fail;
    }

    fr = f_close(&fil);
    if (fr != FR_OK) {
        printf("E f_close error: %s (%d)\n", FRESULT_str(fr), fr);
    }

    f_unmount(pSD->pcName);
    if (!hold_sd_busy) set_sd_busy(false);

    printf("I screenshot saved: %s (%dx%d)\n", path, LCD_WIDTH, LCD_HEIGHT);
    return true;

fail:
    printf("E screenshot write failed: %s (%d)\n", FRESULT_str(fr), fr);
    f_close(&fil);
    f_unmount(pSD->pcName);
    if (!hold_sd_busy) set_sd_busy(false);
    return false;
}

/**
 * Load a .gb rom file in flash from the SD card 
 */ 
#define ERASE_SIZE   (64 * 1024)    // 4 KB minimum erase size (SDK requirement)
#define PAGE_SIZE    (4 * 1024)    // 4 KB program page
#define BLOCK_SIZE   (64 * 1024)   // SD read buffer size (big enough but not too big)


void __not_in_flash_func(load_cart_rom_file)(const char *filename) {
    while(led_ramp_done == false) sleep_ms(1); // Wait for LED fade-in to complete
    
    // Don't touch again please..
    memset(front_fb->data, 0, sizeof(front_fb->data));
    memset(write_fb->data, 0, sizeof(write_fb->data));
    // memset(spare_fb->data, 0, sizeof(spare_fb->data));
    // sleep_ms(10); // wait for core1 to render black screen, 8.33ms refresh time
    
    FRESULT fr;
    FIL fil;
    UINT br;
    sd_card_t *pSD = sd_get_by_num(0);

	set_sd_busy(true);

    // Mount SD card
    fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) {
        printf("E f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }

    fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) {
        printf("E f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
        f_unmount(pSD->pcName);
        return;
    }

	watchdog_disable();

    // Get file size
    uint32_t rom_size = f_size(&fil);
    printf("I ROM size = %lu bytes\n", rom_size);

    uint32_t ints;

#if ENABLE_PSRAM && !ROM_FLASH
    rom = (const uint8_t*) malloc(rom_size);
    if (!rom) {
        printf("Big block built in allocation failed\n"); 
        f_close(&fil);
        f_unmount(pSD->pcName);
        return;
    }
    printf("\nAllocated %u bytes using built-in PSRAM allocator\n", (unsigned)rom_size);
    memory_stats();
#else
    // // Round erase size up to nearest 4 KB
    uint32_t erase_size = (rom_size + ERASE_SIZE - 1) & ~(ERASE_SIZE - 1);
    // Erase flash region up front
    ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_TARGET_OFFSET, erase_size);
    restore_interrupts(ints);
    printf("I Erased %lu KB flash\n", erase_size / 1024);
    // Allocate buffer (not on stack)
    static uint8_t *buffer = NULL;
    if (!buffer) {
        buffer = malloc(BLOCK_SIZE);
        if (!buffer) {
            printf("E malloc failed\n");
            f_close(&fil);
            f_unmount(pSD->pcName);
            return;
        }
    }
#endif

#if ENABLE_PSRAM && !ROM_FLASH
    // --- Load ROM into PSRAM ---
    uint32_t psram_offset = 0;
    uint32_t total_bytes = 0;

    for (;;) {
        fr = f_read(&fil, rom, rom_size, &br);
        if (fr != FR_OK) {
            printf("E f_read error: %s (%d)\n", FRESULT_str(fr), fr);
            break;
        }
        if (br == 0) break; // EOF
        total_bytes += br;
        sleep_us(10);
        tight_loop_contents();
    }

    printf("ROM loaded successfully. Size = %lu bytes (%.2f KB)\n",
           total_bytes, total_bytes / 1024.0);

#if DEBUG_PSRAM
    // --- Verification phase ---
    printf("Verifying PSRAM contents...\n");

    // Rewind file
    f_lseek(&fil, 0);

    uint8_t verify_buf[BLOCK_SIZE];
    uint32_t verify_offset = 0;
    bool mismatch_found = false;

    for (;;) {
        fr = f_read(&fil, verify_buf, BLOCK_SIZE, &br);
        if (fr != FR_OK) {
            printf("E verify read error: %s (%d)\n", FRESULT_str(fr), fr);
            mismatch_found = true;
            break;
        }
        if (br == 0) break; // EOF

        for (size_t i = 0; i < br; i++) {
            if (rom[verify_offset + i] != verify_buf[i]) {
                printf("First real mismatch at offset 0x%08lX: PSRAM=%02X SD=%02X\n",
                    verify_offset + i, rom[verify_offset + i], verify_buf[i]);
                mismatch_found = true;
                break;
            }
        }

        verify_offset += br;
    }

    if (!mismatch_found) {
        printf("✅ Verification successful: PSRAM contents match SD file!\n");
    } else {
        printf("❌ Verification failed.\n");
    }
#endif
#else
    // Stream ROM file into flash
    uint32_t flash_target_offset = FLASH_TARGET_OFFSET;
    for (;;) {
		#warning "DMA/FLASH/RESOURCE CONTENTION WITH SD AND SCANVIDEO, SLEEPING FIXES"
		// sleep_ms(1);
        fr = f_read(&fil, buffer, BLOCK_SIZE, &br);
        if (fr != FR_OK) {
            printf("E f_read error: %s (%d)\n", FRESULT_str(fr), fr);
            break;
        }
        if (br == 0) break; // EOF

        // Program flash in 256-byte pages
        for (uint32_t i = 0; i < br; i += PAGE_SIZE) {
            uint32_t write_len = (br - i >= PAGE_SIZE) ? PAGE_SIZE : (br - i);

            ints = save_and_disable_interrupts();
            flash_range_program(flash_target_offset + i, buffer + i, write_len);
            restore_interrupts(ints);
        }

        flash_target_offset += br;
    }
#endif

    f_close(&fil);
    f_unmount(pSD->pcName);

    // we start save state with 1 so that we can use 0 as the auto save
    // we use 0 for the battery save slot because it will just save as save.sav to be known as the primary save file
	save_rom_settings(filename, 0, 1);
    strncpy(last_filename_raw, filename, FILENAME_MAX_LEN - 1);

    // Crashes occur without setting sd busy false here but I want sd busy to be on until the first gb frame is rendered
    // So that it can instantly go from the black screen to gb instead of black screen to gui to gb
	set_sd_busy(false);
    set_sd_busy(true);
	watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    printf("I load_cart_rom_file(%s) COMPLETE (%lu bytes written)\n", filename, rom_size);
}

/**
 * Function used by the rom file selector to display one page of .gb rom files
 */
uint16_t rom_file_selector_display_page(char filename[MAX_FILES][256],uint16_t num_page) {
    if (!sd_filename_init()) return 0;
    char (*names)[256] = filename;
	sd_card_t *pSD=sd_get_by_num(0);
    DIR dj;
    FILINFO fno;
    FRESULT fr;

    fr=f_mount(&pSD->fatfs,pSD->pcName,1);
    if (FR_OK!=fr) {
        printf("E f_mount error: %s (%d)\n",FRESULT_str(fr),fr);
        return 0;
    }

	/* clear the filenames array */
	for(uint8_t ifile=0;ifile<MAX_FILES;ifile++) {
		strcpy(names[ifile],"");
	}

    /* search *.gb files */
	#warning "This is a problem for .gba"
	uint16_t num_file=0;
	fr=f_findfirst(&dj, &fno, "", "*.gb*");

	/* skip the first N pages */
	if(num_page>0) {
		while(num_file<num_page*MAX_FILES && fr == FR_OK && fno.fname[0]) {
			if (fno.fname[0] != '.') {
				num_file++;
			}
			fr=f_findnext(&dj, &fno);
		}
	}

	/* store the filenames of this page */
	num_file=0;
    while(num_file<MAX_FILES && fr == FR_OK && fno.fname[0]) {
		if (fno.fname[0] != '.') {
			printf(fno.fname);
			strcpy(names[num_file],fno.fname);
			num_file++;
		}
		fr=f_findnext(&dj, &fno);
    }
	f_closedir(&dj);
	f_unmount(pSD->pcName);

	/* display *.gb rom files on screen */
	#if USE_IPS_LCD
	#warning "Display roms on ips screen"
	for(uint8_t ifile=0;ifile<num_file;ifile++) {
		printf("%s\n", filename[ifile]);
	}
	#else
	// mk_ili9225_fill(0x0000);
	for(uint8_t ifile=0;ifile<num_file;ifile++) {
		// mk_ili9225_text(filename[ifile],0,ifile*8,0xFFFF,0x0000);
    }
	#endif
	return num_file;
}

// Forward declaration
uint16_t rom_file_selector_display_page(char filename[MAX_FILES][256], uint16_t num_page);

uint16_t get_bat_charge_percent();
uint16_t get_bat_charge_percent_with_learning(uint16_t voltage_mV, uint16_t reported_percent);
uint16_t get_remaining_bat_capacity_mAh();
uint16_t get_full_bat_capacity_mAh();
uint16_t read_voltage_mV();
int16_t get_average_current_mA();

static int powman_example_off(void);

#define VISIBLE_ITEMS 9  // depends on your screen height / font
static uint16_t page_start = 0; // first item currently visible
bool show_settings = false;

bool mcp7940n_get_tm(i2c_inst_t *i2c, struct tm *out_tm);
bool mcp7940n_set_tm(i2c_inst_t *i2c, struct tm *in_tm);
uint8_t selected_date_value = 1;  // 0=weekday, 1=month, 2=day, 3=year, 4=hour, 5=minute, 6=second
struct tm draft_tm; // current editable time

// static lv_timer_t *scroll_timer = NULL;

// // callback to start scrolling
// void start_scroll_cb(lv_timer_t * t) {
//     lv_obj_t *label = (lv_obj_t *)t->user_data;
//     lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); // enable scroll
//     lv_obj_set_style_anim_speed(label, 20, 0); // adjust speed
//     lv_timer_del(t); // delete timer after running once
// }
void modify_draft_tm(struct tm *draft_tm, uint8_t selected_date_value, int8_t increment) {
    switch(selected_date_value) {
        case 0: // weekday manually (1=Sunday, 7=Saturday)
            draft_tm->tm_wday += increment;
            if(draft_tm->tm_wday < 0) draft_tm->tm_wday = 6;
            else if(draft_tm->tm_wday > 6) draft_tm->tm_wday = 0;
            break;

        case 1: // year
            draft_tm->tm_year += increment;
            if(draft_tm->tm_year < 0) draft_tm->tm_year = 0;
            break;

        case 2: // month 0-11
            draft_tm->tm_mon += increment;
            if(draft_tm->tm_mon < 0) draft_tm->tm_mon = 11;
            else if(draft_tm->tm_mon > 11) draft_tm->tm_mon = 0;
            break;

        case 3: // day of month
            draft_tm->tm_mday += increment;
            break;

        case 4: // hour
            draft_tm->tm_hour += increment;
            break;

        case 5: // minute
            draft_tm->tm_min += increment;
            break;

        case 6: // second
            draft_tm->tm_sec += increment;
            break;
    }

    // Normalize tm struct (handles overflows and recalculates tm_wday/tm_yday)
    mktime(draft_tm);
}

void draw_settings(lv_obj_t *list) {
    lv_obj_clean(list);

    uint16_t bat_percent = get_bat_charge_percent();
    int16_t current_mA = get_average_current_mA();
    uint16_t remaining_cap = get_remaining_bat_capacity_mAh();
    uint16_t full_cap = get_full_bat_capacity_mAh();
    uint16_t voltage_mV = read_voltage_mV();

    struct tm now = draft_tm;

    lv_color_t normal_color = lv_color_hex(0x202020);
    lv_color_t highlight_color = lv_color_hex(0x33CC66);

    int y_offset = 6;
    int spacing = 6;

    // ------------------------------
    // SECTION: BATTERY
    // ------------------------------
    lv_obj_t *bat_title = lv_label_create(list);
    lv_obj_set_style_text_color(bat_title, normal_color, 0);
    lv_obj_set_style_text_font(bat_title, LV_FONT_DEFAULT, 0);

    char bat_title_text[48];
    snprintf(bat_title_text, sizeof(bat_title_text),
            "Battery  %d%%  %.2fV",
            bat_percent, voltage_mV / 1000.0f);

    lv_label_set_text(bat_title, bat_title_text);
    lv_obj_align(bat_title, LV_ALIGN_TOP_MID, 0, y_offset);

    // Second line: signed current + capacity
    lv_obj_t *bat_line2 = lv_label_create(list);
    lv_obj_set_style_text_color(bat_line2, normal_color, 0);

    char line2_text[48];
    snprintf(line2_text, sizeof(line2_text),
            "%+dmA  %d/%dmAh",
            current_mA, remaining_cap, full_cap);

    lv_label_set_text(bat_line2, line2_text);
    lv_obj_align_to(bat_line2, bat_title, LV_ALIGN_OUT_BOTTOM_MID, 0, spacing);


    // ------------------------------
    // SECTION: DATE
    // ------------------------------
    lv_obj_t *date_title = lv_label_create(list);
    lv_label_set_text(date_title, "\nDate");
    lv_obj_set_style_text_color(date_title, normal_color, 0);
    lv_obj_align_to(date_title, line2_text, LV_ALIGN_OUT_BOTTOM_MID, 0, spacing);

    // create label for the formatted date
    lv_obj_t *date_label = lv_label_create(list);
    lv_label_set_recolor(date_label, true);
    lv_obj_set_style_text_color(date_label, normal_color, 0);
    lv_obj_set_style_text_font(date_label, LV_FONT_DEFAULT, 0);

    char date_str[48];

    if (selected_date_value == 1) {
        snprintf(date_str, sizeof(date_str),
                 "#33CC66 %04d#/%02d/%02d",
                 now.tm_year + 1900, now.tm_mon + 1, now.tm_mday);
    } else if (selected_date_value == 2) {
        snprintf(date_str, sizeof(date_str),
                 "%04d/#33CC66 %02d#/%02d",
                 now.tm_year + 1900, now.tm_mon + 1, now.tm_mday);
    } else if (selected_date_value == 3) {
        snprintf(date_str, sizeof(date_str),
                 "%04d/%02d/#33CC66 %02d#",
                 now.tm_year + 1900, now.tm_mon + 1, now.tm_mday);
    } else {
        snprintf(date_str, sizeof(date_str),
                 "%04d/%02d/%02d",
                 now.tm_year + 1900, now.tm_mon + 1, now.tm_mday);
    }

    lv_label_set_text(date_label, date_str);
    lv_obj_align_to(date_label, date_title, LV_ALIGN_OUT_BOTTOM_MID, 0, spacing);


    // Weekday (not editable)
    const char *weekdays[] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
    lv_obj_t *weekday_label = lv_label_create(list);
    char weekday_str[16];
    snprintf(weekday_str, sizeof(weekday_str), "Weekday: %s", weekdays[now.tm_wday]);
    lv_label_set_text(weekday_label, weekday_str);
    lv_obj_set_style_text_color(weekday_label, normal_color, 0);
    lv_obj_align_to(weekday_label, date_label, LV_ALIGN_OUT_BOTTOM_MID, 0, spacing);


    // ------------------------------
    // SECTION: TIME
    // ------------------------------
    lv_obj_t *time_title = lv_label_create(list);
    lv_label_set_text(time_title, "\nTime");
    lv_obj_set_style_text_color(time_title, normal_color, 0);
    lv_obj_align_to(time_title, weekday_label, LV_ALIGN_OUT_BOTTOM_MID, 0, spacing);


    lv_obj_t *time_label = lv_label_create(list);
    lv_label_set_recolor(time_label, true);
    lv_obj_set_style_text_color(time_label, normal_color, 0);
    lv_obj_set_style_text_font(time_label, LV_FONT_DEFAULT, 0);

    char time_str[48];

    if (selected_date_value == 4) {
        snprintf(time_str, sizeof(time_str),
                 "#33CC66 %02d#:%02d:%02d",
                 now.tm_hour, now.tm_min, now.tm_sec);
    } else if (selected_date_value == 5) {
        snprintf(time_str, sizeof(time_str),
                 "%02d:#33CC66 %02d#:%02d",
                 now.tm_hour, now.tm_min, now.tm_sec);
    } else if (selected_date_value == 6) {
        snprintf(time_str, sizeof(time_str),
                 "%02d:%02d:#33CC66 %02d#",
                 now.tm_hour, now.tm_min, now.tm_sec);
    } else {
        snprintf(time_str, sizeof(time_str),
                 "%02d:%02d:%02d",
                 now.tm_hour, now.tm_min, now.tm_sec);
    }

    lv_label_set_text(time_label, time_str);
    lv_obj_align_to(time_label, time_title, LV_ALIGN_OUT_BOTTOM_MID, 0, spacing);
}

static const char* basename_from_path(const char *path);
void draw_rom_list(lv_obj_t *list, char filenames[][256], uint16_t num_file, uint16_t selected, uint16_t page_start) {
    lv_obj_clean(list);
    if (show_settings) {
        draw_settings(list);
        return;
    }
    for (uint16_t i = 0; i < VISIBLE_ITEMS && (i + page_start) < num_file; i++) {
        lv_obj_t *list_title = lv_list_add_text(list, basename_from_path(filenames[i + page_start]));
        lv_obj_set_style_text_font(list_title, LV_FONT_DEFAULT, 0); // &lv_font_dejavu_16_persian_hebrew
        lv_obj_set_style_text_color(list_title, lv_color_black(), 0);
        lv_obj_set_style_bg_color(list_title, lv_color_hex(0xFFFFFF), 0);

#if SHOW_PICS_IN_MENU
        lv_obj_set_style_bg_opa(list_title, LV_OPA_80, LV_PART_MAIN);
#endif
        // lv_obj_set_width(list_title, DISP_HOR_RES - 30); // important for clipping/ellipsis
        // lv_label_set_long_mode(list_title, LV_LABEL_LONG_CLIP); // clip/ellipsis for others

        if ((i + page_start) == selected) {
            lv_obj_set_style_bg_color(list_title, lv_color_hex(0x33CC66), LV_PART_MAIN); // faint green.. pure = 0x00C000
            lv_obj_set_style_text_color(list_title, lv_color_black(), 0);
            lv_label_set_long_mode(list_title, LV_LABEL_LONG_SCROLL_CIRCULAR); // enable scroll
            lv_obj_set_style_anim_speed(list_title, 20, 0); // adjust speed

            // create a one-shot timer to wait 500ms before starting scroll
            // if (scroll_timer) {
            //     lv_timer_del(scroll_timer);
            //     scroll_timer = NULL;
            // }
            // scroll_timer = lv_timer_create(start_scroll_cb, 500, list_title);
        } else {
            lv_obj_set_width(list_title, DISP_HOR_RES - 30); // important for clipping/ellipsis
            lv_label_set_long_mode(list_title, LV_LABEL_LONG_CLIP); // clip/ellipsis for others
        }
    }
}

lv_obj_t *create_top_bar(lv_obj_t *parent, lv_obj_t **status_label_out) {
    lv_obj_t *top_bar = lv_obj_create(parent);
    lv_obj_set_size(top_bar, DISP_HOR_RES, 20);
    lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, -15);
    lv_obj_set_style_bg_color(top_bar, lv_color_hex(0xE0E0E0), 0);
#if SHOW_PICS_IN_MENU
    lv_obj_set_style_bg_opa(top_bar, LV_OPA_80, 0);
#endif
    lv_obj_set_style_border_width(top_bar, 0, 0);
    lv_obj_set_scrollbar_mode(top_bar, LV_SCROLLBAR_MODE_OFF);

    // Bottom border line
    lv_obj_set_style_border_width(top_bar, 1, 0);
    lv_obj_set_style_border_side(top_bar, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(top_bar, lv_color_hex(0x000000), 0);

    // Left label (title)
    lv_obj_t *title_label = lv_label_create(top_bar);
    lv_label_set_text(title_label, "Pico Pal");
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(title_label, lv_color_hex(0x202020), 0);
    lv_obj_align(title_label, LV_ALIGN_LEFT_MID, 5, 0);

    // Right label (time + battery)
    lv_obj_t *status_label = lv_label_create(top_bar);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(status_label, lv_color_hex(0x202020), 0);
    lv_obj_align(status_label, LV_ALIGN_RIGHT_MID, -5, 0);

    if (status_label_out) *status_label_out = status_label;
    return top_bar;
}

void update_status_label(lv_obj_t *status_label) {
    uint16_t bat_percent = get_bat_charge_percent_with_learning(read_voltage_mV(), get_bat_charge_percent());

    struct tm now;
    mcp7940n_get_tm(RTC_I2C_PORT, &now);

    int hour12 = now.tm_hour % 12;
    if (hour12 == 0) hour12 = 12;
    const char *ampm = (now.tm_hour >= 12) ? "PM" : "AM";

    char status_text[32];
    snprintf(status_text, sizeof(status_text), "%d:%02d%s  %d%%",
             hour12, now.tm_min, ampm, bat_percent);

    lv_label_set_text(status_label, status_text);
}

lv_obj_t *create_bottom_bar(lv_obj_t *parent, lv_obj_t **left_out, lv_obj_t **right_out) {
    lv_obj_t *bottom_bar = lv_obj_create(parent);
    lv_obj_set_size(bottom_bar, DISP_HOR_RES, 20);
    lv_obj_align(bottom_bar, LV_ALIGN_BOTTOM_MID, 0, 15);
    lv_obj_set_style_bg_color(bottom_bar, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_border_width(bottom_bar, 0, 0);
    lv_obj_set_scrollbar_mode(bottom_bar, LV_SCROLLBAR_MODE_OFF);

#if SHOW_PICS_IN_MENU
    lv_obj_set_style_bg_opa(bottom_bar, LV_OPA_80, 0);
#endif

    // Top border line
    lv_obj_set_style_border_width(bottom_bar, 1, 0);
    lv_obj_set_style_border_side(bottom_bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_color(bottom_bar, lv_color_hex(0x000000), 0);

    // Left label
    lv_obj_t *left = lv_label_create(bottom_bar);
    lv_label_set_text(left, "A: Load");
    lv_obj_set_style_text_font(left, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(left, lv_color_hex(0x202020), 0);
    lv_obj_align(left, LV_ALIGN_LEFT_MID, 5, 0);

    // Right label
    lv_obj_t *right = lv_label_create(bottom_bar);
    lv_label_set_text(right, "Start: Recent");
    lv_obj_set_style_text_font(right, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(right, lv_color_hex(0x202020), 0);
    lv_obj_align(right, LV_ALIGN_RIGHT_MID, -5, 0);

    if (left_out)  *left_out  = left;
    if (right_out) *right_out = right;
    return bottom_bar;
}

void update_bottom_bar(lv_obj_t *left, lv_obj_t *right, bool show_settings) {
    if (show_settings) {
        lv_label_set_text(left, "Select: Exit");
        lv_label_set_text(right,  "Start: Save");
    } else {
        lv_label_set_text(left,  "A: Load");
        lv_label_set_text(right, "Start: Recent");
    }
}

// -------------------------
// Small helpers
// -------------------------
static lv_img_dsc_t *g_bg_dsc   = NULL;
static void         *g_bg_pix   = NULL;  // malloc'd pixel buffer (RGB565)
static lv_obj_t     *g_bg_img   = NULL;
static lv_obj_t     *g_bg_fade  = NULL;

static void lvgl_bg_free_current(void) {
    if (g_bg_img) {
        // Detach source so LVGL isn't referencing freed memory
        lv_img_set_src(g_bg_img, NULL);
    }
    if (g_bg_dsc) { free(g_bg_dsc); g_bg_dsc = NULL; }
    if (g_bg_pix) { free(g_bg_pix); g_bg_pix = NULL; }
}

static bool read_exact(FIL *f, void *dst, UINT len) {
    UINT br = 0;
    return (f_read(f, dst, len, &br) == FR_OK) && (br == len);
}

static uint32_t be32_u(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static uint16_t pack_rgb565(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t v = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
#if defined(LV_COLOR_16_SWAP) && LV_COLOR_16_SWAP
    v = (uint16_t)((v << 8) | (v >> 8));
#endif
    return v;
}

// Decodes ONLY the PNGs you generate:
// - IHDR: bit_depth=8, color_type=2 (RGB), interlace=0, filter per-row = 0
// - IDAT: zlib header + DEFLATE stored blocks (no compression), then Adler32
static bool png_load_rgb565_from_fatfs(const char *path,
                                      uint16_t **out_pixels,
                                      int *out_w, int *out_h)
{
    if (!out_pixels || !out_w || !out_h) return false;
    *out_pixels = NULL;
    *out_w = *out_h = 0;

    FIL f;
    FRESULT fr = f_open(&f, path, FA_READ);
    if (fr != FR_OK) {
        printf("E png open %s: %s (%d)\n", path, FRESULT_str(fr), fr);
        return false;
    }

    // PNG signature
    uint8_t sig[8];
    if (!read_exact(&f, sig, 8)) { f_close(&f); return false; }
    static const uint8_t png_sig[8] = {137,80,78,71,13,10,26,10};
    if (memcmp(sig, png_sig, 8) != 0) { f_close(&f); return false; }

    // PASS 1: read IHDR and sum IDAT sizes
    uint32_t w=0, h=0;
    uint8_t  bit_depth=0, color_type=0, comp=0, filt=0, interlace=0;
    uint32_t idat_total = 0;
    bool got_ihdr = false;

    for (;;) {
        uint8_t len_be[4], type[4];
        if (!read_exact(&f, len_be, 4)) break;
        if (!read_exact(&f, type, 4)) break;

        uint32_t len = be32_u(len_be);

        if (memcmp(type, "IHDR", 4) == 0) {
            uint8_t ihdr[13];
            if (len != 13 || !read_exact(&f, ihdr, 13)) { f_close(&f); return false; }
            w = be32_u(&ihdr[0]);
            h = be32_u(&ihdr[4]);
            bit_depth = ihdr[8];
            color_type = ihdr[9];
            comp = ihdr[10];
            filt = ihdr[11];
            interlace = ihdr[12];
            got_ihdr = true;

            // skip CRC
            uint8_t crc[4];
            if (!read_exact(&f, crc, 4)) { f_close(&f); return false; }
        } else if (memcmp(type, "IDAT", 4) == 0) {
            idat_total += len;
            // skip data + CRC
            if (f_lseek(&f, f_tell(&f) + len + 4) != FR_OK) { f_close(&f); return false; }
        } else if (memcmp(type, "IEND", 4) == 0) {
            // skip CRC (len should be 0)
            if (f_lseek(&f, f_tell(&f) + len + 4) != FR_OK) { f_close(&f); return false; }
            break;
        } else {
            // skip chunk + CRC
            if (f_lseek(&f, f_tell(&f) + len + 4) != FR_OK) { f_close(&f); return false; }
        }
    }

    if (!got_ihdr) { f_close(&f); return false; }

    // Validate it matches YOUR writer
    if (bit_depth != 8 || color_type != 2 || comp != 0 || filt != 0 || interlace != 0) {
        printf("E png format unsupported: bd=%u ct=%u c=%u f=%u i=%u\n",
               bit_depth, color_type, comp, filt, interlace);
        f_close(&f);
        return false;
    }

    // PASS 2: re-scan and collect IDAT bytes into one buffer
    fr = f_lseek(&f, 8);
    if (fr != FR_OK) { f_close(&f); return false; }

    uint8_t *idat = (uint8_t*)malloc(idat_total);
    if (!idat) { f_close(&f); return false; }
    uint32_t idat_off = 0;

    for (;;) {
        uint8_t len_be[4], type[4];
        if (!read_exact(&f, len_be, 4)) break;
        if (!read_exact(&f, type, 4)) break;

        uint32_t len = be32_u(len_be);

        if (memcmp(type, "IDAT", 4) == 0) {
            if (idat_off + len > idat_total) { free(idat); f_close(&f); return false; }
            if (!read_exact(&f, &idat[idat_off], len)) { free(idat); f_close(&f); return false; }
            idat_off += len;

            // skip CRC
            uint8_t crc[4];
            if (!read_exact(&f, crc, 4)) { free(idat); f_close(&f); return false; }
        } else {
            // skip chunk data + CRC
            if (f_lseek(&f, f_tell(&f) + len + 4) != FR_OK) { free(idat); f_close(&f); return false; }
            if (memcmp(type, "IEND", 4) == 0) break;
        }
    }

    f_close(&f);

    if (idat_off != idat_total) {
        free(idat);
        return false;
    }

    // Decode zlib(stored) -> raw scanlines
    // raw per row: [filter=0][RGB...]
    const uint32_t row_bytes = 1 + 3 * w;
    const uint32_t raw_need  = row_bytes * h;

    if (idat_total < 2 + 4) { free(idat); return false; }
    uint32_t p = 0;

    // zlib header
    uint8_t cmf = idat[p++], flg = idat[p++];
    (void)cmf; (void)flg;

    uint8_t *raw = (uint8_t*)malloc(raw_need);
    if (!raw) { free(idat); return false; }
    uint32_t raw_off = 0;

    // stored blocks
    for (;;) {
        if (p + 5 > idat_total) { free(raw); free(idat); return false; }

        uint8_t bh = idat[p++];              // your writer uses full byte, bit0 = BFINAL
        uint8_t bfinal = (uint8_t)(bh & 1);
        uint8_t btype_bits = (uint8_t)(bh & 0x06);
        if (btype_bits != 0) { free(raw); free(idat); return false; } // must be stored (00)

        uint16_t LEN  = (uint16_t)(idat[p] | (idat[p+1] << 8)); p += 2;
        uint16_t NLEN = (uint16_t)(idat[p] | (idat[p+1] << 8)); p += 2;
        if ((uint16_t)~LEN != NLEN) { free(raw); free(idat); return false; }

        if (p + LEN > idat_total) { free(raw); free(idat); return false; }
        if (raw_off + LEN > raw_need) { free(raw); free(idat); return false; }

        memcpy(&raw[raw_off], &idat[p], LEN);
        raw_off += LEN;
        p += LEN;

        if (bfinal) break;
    }

    // There is an Adler32 at end of zlib stream (4 bytes). We can ignore/skip validation.
    // Ensure we got exactly the raw bytes we need.
    if (raw_off != raw_need) { free(raw); free(idat); return false; }

    // Convert raw scanlines -> RGB565 pixels
    uint32_t pixel_count = w * h;
    uint16_t *pix = (uint16_t*)malloc(pixel_count * sizeof(uint16_t));
    if (!pix) { free(raw); free(idat); return false; }

    for (uint32_t y = 0; y < h; y++) {
        const uint8_t *row = &raw[y * row_bytes];
        if (row[0] != 0) { // only filter 0 supported
            free(pix); free(raw); free(idat);
            return false;
        }
        const uint8_t *rgb = &row[1];
        for (uint32_t x = 0; x < w; x++) {
            uint8_t r = rgb[3*x + 0];
            uint8_t g = rgb[3*x + 1];
            uint8_t b = rgb[3*x + 2];
            pix[y*w + x] = pack_rgb565(r, g, b);
        }
    }

    free(raw);
    free(idat);

    *out_pixels = pix;
    *out_w = (int)w;
    *out_h = (int)h;
    return true;
}

static int str_endswith_ci(const char *s, const char *suffix) {
    size_t ls = strlen(s), lf = strlen(suffix);
    if (lf > ls) return 0;
    s += (ls - lf);
    for (size_t i = 0; i < lf; i++) {
        if (tolower((unsigned char)s[i]) != tolower((unsigned char)suffix[i])) return 0;
    }
    return 1;
}

static int str_startswith_ci(const char *s, const char *prefix) {
    while (*prefix && *s) {
        if (tolower((unsigned char)*s) != tolower((unsigned char)*prefix)) return 0;
        s++; prefix++;
    }
    return *prefix == '\0';
}

// Match: screenshot*.png  (case-insensitive)
static bool is_screenshot_png_name(const char *name) {
    if (!name || !name[0]) return false;
    if (!str_endswith_ci(name, ".png")) return false;
    if (!str_startswith_ci(name, "screenshot")) return false;
    return true;
}

// Reservoir sampling: when we see the Nth candidate, replace selection with prob 1/N
static void reservoir_maybe_take(char *selected, size_t selected_sz,
                                 const char *candidate,
                                 uint32_t *count) {
    (*count)++;
    // rand() must be seeded somewhere in your app for "different each boot"
    // (e.g. seed from RTC/time_us_64/ADC noise, etc.)
    uint32_t r = (uint32_t)(rand() % (int)(*count));
    if (r == 0) {
        strncpy(selected, candidate, selected_sz);
        selected[selected_sz - 1] = '\0';
    }
}

static FRESULT find_random_screenshot_recursive(const char *fatfs_dir,
                                               char *out_path, size_t out_path_sz,
                                               uint32_t *seen_count)
{
    FRESULT fr;
    DIR dir;
    FILINFO fno;

    fr = f_opendir(&dir, fatfs_dir);
    if (fr != FR_OK) return fr;

    for (;;) {
        fr = f_readdir(&dir, &fno);
        if (fr != FR_OK) break;
        if (fno.fname[0] == 0) break; // end

        const char *name = (const char *)fno.fname; // LFN-capable in your build

        if (strcmp(name, ".") == 0 || strcmp(name, "..") == 0) continue;

        char child[PATH_MAX_LEN];
        int n = snprintf(child, sizeof(child), "%s/%s", fatfs_dir, name);
        if (n <= 0 || n >= (int)sizeof(child)) continue;

        if (fno.fattrib & AM_DIR) {
            FRESULT fr2 = find_random_screenshot_recursive(child, out_path, out_path_sz, seen_count);
            if (fr2 != FR_OK && fr2 != FR_NO_PATH && fr2 != FR_NO_FILE) { fr = fr2; break; }
        } else {
            if (is_screenshot_png_name(name)) {
                reservoir_maybe_take(out_path, out_path_sz, child, seen_count);
            }
        }
    }

    f_closedir(&dir);
    return fr;
}

// -------------------------
// PUBLIC API
// -------------------------
// Sets a random screenshot PNG from root_dir as LVGL background.
// root_dir example: "GBC/Saves" (FatFs path) OR "0:/GBC/Saves" depending on your setup.
// Returns true if it found one and applied it.
bool lvgl_set_random_gbc_screenshot_background_from_sd(lv_obj_t *parent,
                                                       int sd_num,
                                                       const char *subdir,   // "GBC/Saves"
                                                       lv_opa_t fade_opa,
                                                       bool hold_sd_busy)
{
    set_sd_busy(true);

    sd_card_t *pSD = sd_get_by_num(sd_num);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) {
        printf("E f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        if (!hold_sd_busy) set_sd_busy(false);
        return false;
    }

    // Build volume-prefixed root for FatFs recursion
    char root[PATH_MAX_LEN];
    snprintf(root, sizeof(root), "%s/%s", pSD->pcName, subdir);   // e.g. "0:/GBC/Saves"

    // Find random screenshot
    char chosen[PATH_MAX_LEN] = {0};
    uint32_t seen = 0;
    fr = find_random_screenshot_recursive(root, chosen, sizeof(chosen), &seen);

    if (fr != FR_OK && fr != FR_NO_FILE && fr != FR_NO_PATH) {
        printf("E random screenshot scan failed: %s (%d)\n", FRESULT_str(fr), fr);
        f_unmount(pSD->pcName);
        if (!hold_sd_busy) set_sd_busy(false);
        return false;
    }
    if (seen == 0) {
        printf("W no screenshots found under %s\n", root);
        f_unmount(pSD->pcName);
        if (!hold_sd_busy) set_sd_busy(false);
        return false;
    }

    // Decode PNG now (while mounted)
    uint16_t *pix = NULL;
    int w = 0, h = 0;
    if (!png_load_rgb565_from_fatfs(chosen, &pix, &w, &h)) {
        printf("E png decode failed: %s\n", chosen);
        f_unmount(pSD->pcName);
        if (!hold_sd_busy) set_sd_busy(false);
        return false;
    }

    // Done with SD now
    f_unmount(pSD->pcName);
    if (!hold_sd_busy) set_sd_busy(false);

    // Replace any previous bg
    lvgl_bg_free_current();

    // Create LVGL objects if needed
    if (!g_bg_img) {
        g_bg_img = lv_img_create(parent);
        lv_obj_clear_flag(g_bg_img, LV_OBJ_FLAG_SCROLLABLE);
    }
    if (!g_bg_fade) {
        g_bg_fade = lv_obj_create(parent);
        lv_obj_remove_style_all(g_bg_fade);
        lv_obj_set_size(g_bg_fade, LV_PCT(100), LV_PCT(100));
        lv_obj_align(g_bg_fade, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(g_bg_fade, lv_color_black(), 0);
        lv_obj_clear_flag(g_bg_fade, LV_OBJ_FLAG_SCROLLABLE);
    }
    lv_obj_set_style_bg_opa(g_bg_fade, fade_opa, 0);

    // Build an in-RAM image descriptor (malloc)
    lv_img_dsc_t *dsc = (lv_img_dsc_t*)malloc(sizeof(lv_img_dsc_t));
    if (!dsc) {
        free(pix);
        return false;
    }
    memset(dsc, 0, sizeof(*dsc));
    dsc->header.cf = LV_IMG_CF_TRUE_COLOR;
    dsc->header.w  = (uint32_t)w;
    dsc->header.h  = (uint32_t)h;
    dsc->data_size = (uint32_t)(w * h * sizeof(lv_color_t));
    dsc->data      = (const uint8_t*)pix;

    // Keep pointers so we can free later
    g_bg_dsc = dsc;
    g_bg_pix = pix;

    // Apply
    lv_img_set_src(g_bg_img, dsc);
    lv_obj_center(g_bg_img);

    // Make sure ordering is: bg image -> fade -> your UI items (created after)
    lv_obj_move_background(g_bg_img);
    lv_obj_move_to_index(g_bg_fade, 1);

    printf("I random screenshot bg loaded to RAM: %dx%d from %s\n", w, h, chosen);
    return true;
}

void rom_file_selector() {	
    if (!sd_filename_init()) return;
    char (*names)[256] = filename;
    // Create list
    lv_init();

    memset(lvgl_fb, 0, lvgl_fb_bytes);

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, lv_buf1, NULL, DISP_HOR_RES * LV_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_VER_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;

	lv_disp_t * disp = lv_disp_drv_register(&disp_drv);
	///

	num_file = rom_file_selector_display_page(names, num_page);

    // Create a container to hold UI
	lv_obj_t *cont = lv_obj_create(lv_scr_act());
	lv_obj_set_size(cont, DISP_HOR_RES, DISP_VER_RES);
	lv_obj_center(cont);
#if !SHOW_PICS_IN_MENU
    lv_obj_set_style_bg_color(cont, lv_color_hex(0xFFFFFF), 0);  // dark gray background
#else
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);   // IMPORTANT: make container transparent so bg shows through
#endif
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);

#if SHOW_PICS_IN_MENU
    lvgl_set_random_gbc_screenshot_background_from_sd(cont, 0, "GBC/Saves", LV_OPA_TRANSP, false);
#endif
#if !SHOW_PICS_IN_MENU
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xFFFFFF), 0);  // match your container
#endif

	// // Title label
	// lv_obj_t *title = lv_label_create(cont);
	// lv_label_set_text(title, filename[selected]);
	// lv_obj_set_style_text_font(title, LV_FONT_DEFAULT, 0);
	// lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
    // lv_obj_set_scrollbar_mode(title, LV_SCROLLBAR_MODE_OFF);


    // === ROM List ===
    lv_obj_t *list = lv_list_create(cont);
    lv_obj_set_size(list, DISP_HOR_RES, DISP_VER_RES - 20);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 7);
    lv_obj_set_style_bg_color(list, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_color(list, lv_color_black(), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);
#if SHOW_PICS_IN_MENU
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, LV_PART_MAIN);
#endif

    draw_rom_list(list, names, num_file, selected, page_start);
        
    lv_obj_t *status_label;
    lv_obj_t *top_bar = create_top_bar(cont, &status_label);

    // Initial update
    update_status_label(status_label);

    // === Bottom hint bar ===
    lv_obj_t *hint_left;
    lv_obj_t *hint_right;
    lv_obj_t *hint_bar = create_bottom_bar(cont, &hint_left, &hint_right);
    update_bottom_bar(hint_left, hint_right, show_settings);


	// input loop
    bool up = true, down = true, left = true, right = true, a = true, b = true, select = true, start = true;
	// Keep previous states (initialize all to "released" = true, since buttons are active-low)
	static bool prev_up = true, prev_down = true, prev_left = true, prev_right = true;
	static bool prev_a = true, prev_b = true, prev_start = true, prev_select = true;

    // need this for the start recent on power up straight in.
    // the state gets refreshed in main so no need to read iox
    start = gpio_read(IOX_B_START);

	// sleep_ms(3000);
	print_memory_usage();

    __atomic_store_n(&show_gui, true, __ATOMIC_RELEASE);

	while (true) {
#if ENABLE_BAT_MONITORING
		if (battery_task_flag) {
            battery_task_flag = false;
            process_bat_percent();
            update_status_label(status_label);
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
#endif
        select = gpio_read(GPIO_B_SELECT);
		bool iox_nint = gpio_read(GPIO_IOX_nINT);
		if (!iox_nint) {
			read_io_expander_states(0);
			up      = gpio_read(IOX_B_UP);
			down    = gpio_read(IOX_B_DOWN);
			left    = gpio_read(IOX_B_LEFT);
			right   = gpio_read(IOX_B_RIGHT);
			a       = gpio_read(IOX_B_A);
			b       = gpio_read(IOX_B_B);
			start   = gpio_read(IOX_B_START);
		}

		if (!start && prev_start) {   // just pressed
            if (show_settings) {
                mcp7940n_set_tm(RTC_I2C_PORT, &draft_tm);
                show_settings = false;
                draw_rom_list(list, filename, num_file, selected, page_start);
                update_status_label(status_label);
                update_bottom_bar(hint_left, hint_right, show_settings);
            } else {
                bool launched = false;
#if ENABLE_PSRAM && !ROM_FLASH
                if (last_filename_raw[0] != '\0') {
                    load_cart_rom_file(last_filename_raw);
                    launched = true;
                }
#endif
                if (launched || ROM_FLASH) break;
            }
		}
		if (!a && prev_a) {
            if (!show_settings) {
                /* copy the rom from the SD card to flash and start the game */
                printf("LOADING\n");
                load_cart_rom_file(filename[selected]);
                break;
            }
		}
        if (!b && prev_b) {
        }
        if (!down && prev_down) {
            if (show_settings) {
                modify_draft_tm(&draft_tm, selected_date_value, -1);
            } else {
                if (selected + 1 < num_file) selected++;
                else selected = 0; // wrap around

                // scroll if selected goes beyond visible items
                if (selected >= page_start + VISIBLE_ITEMS) page_start++;
                else if (selected == 0) page_start = 0; // wrap
            }
            draw_rom_list(list, filename, num_file, selected, page_start);
        }

        if (!up && prev_up) {
            if (show_settings) {
                modify_draft_tm(&draft_tm, selected_date_value, +1);
            } else {
                if (selected == 0) selected = num_file - 1;
                else selected--;

                if (selected < page_start) page_start--;
                else if (selected == num_file - 1) page_start = (num_file > VISIBLE_ITEMS) ? num_file - VISIBLE_ITEMS : 0;
            }
            draw_rom_list(list, filename, num_file, selected, page_start);
        }

		if (!right && prev_right) {
            if (show_settings) {
                selected_date_value++;
                if (selected_date_value > 6) {
                    selected_date_value = 1;
                }
            } else {
                /* select the next page */
                num_page++;
                num_file=rom_file_selector_display_page(filename,num_page);
                if(num_file==0) {
                    /* no files in this page, go to the previous page */
                    num_page--;
                    num_file=rom_file_selector_display_page(filename,num_page);
                }
                /* select the first file */
                selected=0;
                // lv_label_set_text(title, filename[selected]);
                page_start = 0;
            }
            draw_rom_list(list, filename, num_file, selected, page_start);

		}
		if (!left && prev_left) {
            if (show_settings) {
                selected_date_value--;
                if (selected_date_value < 1) {
                    selected_date_value = 6;
                }
            } else if (num_page > 0) {
                /* select the previous page */
                num_page--;
                num_file=rom_file_selector_display_page(filename,num_page);
                /* select the first file */
                selected=0;
                // lv_label_set_text(title, filename[selected]);
                page_start = 0;
            }
            draw_rom_list(list, filename, num_file, selected, page_start);

		}

        if (!select && prev_select) {
            show_settings = !show_settings;
            if (show_settings) {
                mcp7940n_get_tm(RTC_I2C_PORT, &draft_tm);
            }
            draw_rom_list(list, filename, num_file, selected, page_start);
            update_bottom_bar(hint_left, hint_right, show_settings);
        }

		// Save states for edge detection
		prev_up = up;
		prev_down = down;
		prev_left = left;
		prev_right = right;
		prev_a = a;
		prev_b = b;
		prev_start = start;
        prev_select = select;

		// in a 1ms timer interrupt or loop
		lv_tick_inc(5);
		lv_timer_handler();
		sleep_ms(3);
	}
}

#endif

// MARK: SETTINGS
// --- System settings ---

typedef struct {
    uint32_t magic;
    uint8_t lcd_brightness;
    uint8_t button_brightness;
    uint8_t power_brightness;
    int8_t selected_palette;
    uint8_t wash_out_level;
    char last_filename_raw[FILENAME_MAX_LEN];
    bool auto_load_state;
} system_settings_t;

system_settings_t g_saved_settings = {0};

typedef struct {
    uint32_t magic;
    char last_filename[FILENAME_MAX_LEN];
    uint8_t battery_slot;
    uint8_t state_slot;
} rom_settings_t;

#define SYSTEM_MAGIC 0xCAFEBABE
#define ROM_MAGIC    0xA5A5A5A5

// --- Helpers ---
static bool ensure_settings_dir(void) {
    FRESULT fr;
    DIR dir;
    fr = f_opendir(&dir, SETTINGS_DIR);
    if (fr == FR_NO_PATH) {
        fr = f_mkdir(SETTINGS_DIR);
        if (fr != FR_OK) {
            printf("E Failed to create settings dir: %s (%d)\n", FRESULT_str(fr), fr);
            return false;
        }
    } else if (fr != FR_OK) {
        printf("E Failed to open settings dir: %s (%d)\n", FRESULT_str(fr), fr);
        return false;
    } else {
        f_closedir(&dir);
    }
    return true;
}

// --- System settings ---
bool read_system_settings(uint8_t *lcd_brightness, uint8_t *button_brightness,
                          uint8_t *power_brightness, int8_t *selected_palette,
                          uint8_t *wash_out_level, char last_filename_raw[FILENAME_MAX_LEN],
                          bool *auto_load_state) {
    FIL fil;
    UINT br;
    system_settings_t s = {0};

    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) return false;

    fr = f_open(&fil, SYSTEM_FILE_PATH, FA_READ);
    if (fr != FR_OK) {
        f_unmount(pSD->pcName);
        return false;
    }

    fr = f_read(&fil, &s, sizeof(s), &br);
    f_close(&fil);
    f_unmount(pSD->pcName);

    if (fr != FR_OK || br != sizeof(s) || s.magic != SYSTEM_MAGIC) return false;

    // Copy to output params
    *lcd_brightness    = s.lcd_brightness;
    *button_brightness = s.button_brightness;
    *power_brightness  = s.power_brightness;
    *selected_palette  = s.selected_palette;
    *wash_out_level    = s.wash_out_level;
    strncpy(last_filename_raw, s.last_filename_raw, FILENAME_MAX_LEN);
    last_filename_raw[FILENAME_MAX_LEN - 1] = '\0'; // ensure null termination
    *auto_load_state   = s.auto_load_state;
    
    // Save to global
    g_saved_settings = s;

    return true;
}

void save_system_settings(uint8_t lcd_brightness, uint8_t button_brightness,
                          uint8_t power_brightness, int8_t selected_palette,
                          uint8_t wash_out_level, char last_filename_raw[FILENAME_MAX_LEN],
                          bool auto_load_state,
                          bool hold_sd_busy) {
    system_settings_t s = {
        .magic = SYSTEM_MAGIC,
        .lcd_brightness = lcd_brightness,
        .button_brightness = button_brightness,
        .power_brightness = power_brightness,
        .selected_palette = selected_palette,
        .wash_out_level = wash_out_level,
        .auto_load_state = auto_load_state
    };
    // Properly copy the filename into the struct
    strncpy(s.last_filename_raw, last_filename_raw, FILENAME_MAX_LEN);
    s.last_filename_raw[FILENAME_MAX_LEN - 1] = '\0';  // ensure null termination

    set_sd_busy(true);
    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) return;

    if (!ensure_settings_dir()) {
        f_unmount(pSD->pcName);
        return;
    }

    FIL fil;
    fr = f_open(&fil, SYSTEM_FILE_PATH, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr == FR_OK) {
        UINT bw;
        f_write(&fil, &s, sizeof(s), &bw);
        f_close(&fil);
        printf("I Saved system settings (%u bytes)\n", bw);
    } else {
        printf("E f_open system settings: %s (%d)\n", FRESULT_str(fr), fr);
    }

    f_unmount(pSD->pcName);
    if (!hold_sd_busy)
        set_sd_busy(false);
}

static inline bool system_settings_equal(const system_settings_t *a, const system_settings_t *b) {
    return a->lcd_brightness     == b->lcd_brightness &&
           a->button_brightness  == b->button_brightness &&
           a->power_brightness   == b->power_brightness &&
           a->selected_palette   == b->selected_palette &&
           a->wash_out_level     == b->wash_out_level &&
           a->auto_load_state    == b->auto_load_state &&
           strncmp(a->last_filename_raw, b->last_filename_raw, FILENAME_MAX_LEN) == 0;
}

void save_system_settings_if_changed(uint8_t lcd_brightness, uint8_t button_brightness,
                                     uint8_t power_brightness, int8_t selected_palette,
                                     uint8_t wash_out_level, char last_filename_raw[FILENAME_MAX_LEN],
                                     bool auto_load_state,
                                     bool hold_sd_busy) {

	system_settings_t current = {
        .magic = SYSTEM_MAGIC,
        .lcd_brightness = lcd_brightness,
        .button_brightness = button_brightness,
        .power_brightness = power_brightness,
        .selected_palette = selected_palette,
        .wash_out_level = wash_out_level,
        .auto_load_state = auto_load_state
    };
    // Properly copy the filename into the struct
    strncpy(current.last_filename_raw, last_filename_raw, FILENAME_MAX_LEN);
    current.last_filename_raw[FILENAME_MAX_LEN - 1] = '\0';  // ensure null termination


    if (!system_settings_equal(&g_saved_settings, &current)) {
        printf("Settings changed, saving...\n");
        save_system_settings(lcd_brightness,
                             button_brightness,
                             power_brightness,
                             selected_palette,
                             wash_out_level,
                             last_filename_raw,
                             auto_load_state,
                             hold_sd_busy);
        g_saved_settings = current;
    }
}

// --- ROM settings ---
bool read_rom_settings(char *out_filename, size_t max_len, uint8_t *battery_slot, uint8_t *state_slot, bool unmount) {
    FIL fil;
    UINT br;
    rom_settings_t r = {0};

    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) return false;

    fr = f_open(&fil, ROM_FILE_PATH, FA_READ);
    if (fr != FR_OK) {
        f_unmount(pSD->pcName);
        return false;
    }

    fr = f_read(&fil, &r, sizeof(r), &br);
    f_close(&fil);
	if (unmount) {
		f_unmount(pSD->pcName);
	}

    if (fr != FR_OK || br != sizeof(r) || r.magic != ROM_MAGIC) return false;

    strncpy(out_filename, r.last_filename, max_len - 1);
    out_filename[max_len - 1] = '\0';
    *battery_slot = r.battery_slot;
    *state_slot = r.state_slot;

	printf("I Loaded ROM settings: %s (battery slot %d, state slot %d)\n",
		   out_filename, *battery_slot, *state_slot);

    return true;
}

void save_rom_settings(const char *filename, uint8_t battery_slot, uint8_t state_slot) {
    rom_settings_t r = {0};
    r.magic = ROM_MAGIC;
    strncpy(r.last_filename, filename, FILENAME_MAX_LEN - 1);
    r.battery_slot = battery_slot;
    r.state_slot = state_slot;

    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) return;

    if (!ensure_settings_dir()) {
        f_unmount(pSD->pcName);
        return;
    }

    FIL fil;
    fr = f_open(&fil, ROM_FILE_PATH, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr == FR_OK) {
        UINT bw;
        f_write(&fil, &r, sizeof(r), &bw);
        f_close(&fil);
        printf("I Saved ROM settings (%u bytes)\n", bw);
    } else {
        printf("E f_open ROM settings: %s (%d)\n", FRESULT_str(fr), fr);
    }

    f_unmount(pSD->pcName);
}


void print_sd_free_space(void) {
    sd_card_t *pSD = sd_get_by_num(0);
    FATFS *fs = &pSD->fatfs;
    DWORD fre_clust, fre_sect, tot_sect;
    FRESULT fr;

    // Mount SD card
    fr = f_mount(fs, pSD->pcName, 1);
    if (fr != FR_OK) {
        printf("E: SD mount failed: %d\n", fr);
        return;
    }

    // Get free clusters
    fr = f_getfree("", &fre_clust, &fs);
    if (fr != FR_OK) {
        printf("E: f_getfree failed: %d\n", fr);
        f_unmount(pSD->pcName);
        return;
    }

    // Calculate total and free space (in bytes)
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;

    printf("SD total space: %lu KB\n", tot_sect / 2);  // 512B per sector, so /2 for KB
    printf("SD free space:  %lu KB\n", fre_sect / 2);

    f_unmount(pSD->pcName);
}
