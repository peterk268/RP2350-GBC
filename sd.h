#if ENABLE_SDCARD
/**
 * Load a save file from the SD card
 */

#define MAX_FILES 9

static lv_obj_t *rom_list;
static uint16_t num_page = 0;
static uint16_t num_file = 0;
static char filename[MAX_FILES][256];
static uint16_t selected = 0;

void set_sd_busy(bool is_sd_busy) {
	sd_busy = is_sd_busy;
	__sev();
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
int build_save_path_from_flash(save_type_t type, char *out_path, size_t out_path_size) {
    char rom_filename[FILENAME_MAX_LEN];
    uint8_t battery_slot, save_state_slot;

    if(!read_rom_settings(rom_filename, sizeof(rom_filename), &battery_slot, &save_state_slot, false)) {
        printf("E: No valid flash settings found\n");
        return -1;
    }

    // For SAVE_BATTERY, use the flash battery_slot as the "slot" parameter
    int effective_slot = (type == SAVE_BATTERY) ? battery_slot : save_state_slot;

    return build_save_path(rom_filename, type, effective_slot, out_path, out_path_size);
}


void read_cart_ram_file(struct gb_s *gb) {

	uint_fast32_t save_size;
	UINT br;

	save_size=gb_get_save_size(gb);
	if(save_size>0) {
		// set_sd_busy(true);

		sd_card_t *pSD=sd_get_by_num(0);
		FRESULT fr=f_mount(&pSD->fatfs,pSD->pcName,1);
		if (FR_OK!=fr) {
			printf("E f_mount error: %s (%d)\n",FRESULT_str(fr),fr);
			return;
		}

		char save_path[512];
		build_save_path_from_flash(SAVE_BATTERY, save_path, sizeof(save_path));

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
	}
}

/**
 * Write a save file to the SD card
 */
void write_cart_ram_file(struct gb_s *gb) {
	uint_fast32_t save_size;
	UINT bw;
	
	save_size=gb_get_save_size(gb);
	if(save_size>0) {
		// set_sd_busy(true);

		sd_card_t *pSD=sd_get_by_num(0);
		FRESULT fr=f_mount(&pSD->fatfs,pSD->pcName,1);
		if (FR_OK!=fr) {
			printf("E f_mount error: %s (%d)\n",FRESULT_str(fr),fr);
			return;
		}

		char save_path[512];
		build_save_path_from_flash(SAVE_BATTERY, save_path, sizeof(save_path));

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

		// set_sd_busy(false);
		printf("I write_cart_ram_file(%s) COMPLETE (%lu bytes)\n",save_path,save_size);
	}
}

/**
 * Load a .gb rom file in flash from the SD card 
 */ 
#define ERASE_SIZE   (64 * 1024)    // 4 KB minimum erase size (SDK requirement)
#define PAGE_SIZE    (4 * 1024)    // 4 KB program page
#define BLOCK_SIZE   (64 * 1024)   // SD read buffer size (big enough but not too big)


void load_cart_rom_file(const char *filename) {
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

    // // Round erase size up to nearest 4 KB
    uint32_t erase_size = (rom_size + ERASE_SIZE - 1) & ~(ERASE_SIZE - 1);
    // Erase flash region up front
    uint32_t ints = save_and_disable_interrupts();
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
    f_close(&fil);
    f_unmount(pSD->pcName);

	save_rom_settings(filename, 0, 0);

	set_sd_busy(false);
	watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    printf("I load_cart_rom_file(%s) COMPLETE (%lu bytes written)\n", filename, rom_size);
}

/**
 * Function used by the rom file selector to display one page of .gb rom files
 */
uint16_t rom_file_selector_display_page(char filename[MAX_FILES][256],uint16_t num_page) {
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
		strcpy(filename[ifile],"");
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
			strcpy(filename[num_file],fno.fname);
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

static void event_handler(lv_event_t * e){
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        LV_LOG_USER("Clicked: %s", lv_list_get_btn_text(rom_list, obj));
    }
}

static void rom_list_update() {
    lv_obj_clean(rom_list);

	lv_list_add_text(rom_list, "Roms");

    for (uint16_t i = 0; i < num_file; i++) {
		lv_obj_t * btn = lv_list_add_btn(rom_list, LV_SYMBOL_FILE, filename[i]);
		lv_obj_add_event_cb(btn, event_handler, LV_EVENT_CLICKED, NULL);
    }

    // auto focus first item
    if (lv_obj_get_child_cnt(rom_list) > 0) {
        lv_obj_t *first = lv_obj_get_child(rom_list, 0);
        lv_group_focus_obj(first);
    }
}


// bool my_btn_read() {

// 	return a || b;
// }
// void button_read(lv_indev_drv_t * drv, lv_indev_data_t*data){
//     static uint32_t last_btn = 0;   /*Store the last pressed button*/

//     int btn_pr = my_btn_read();     /*Get the ID (0,1,2...) of the pressed button*/
//     if(btn_pr >= 0) {               /*Is there a button press? (E.g. -1 indicated no button was pressed)*/
//        last_btn = btn_pr;           /*Save the ID of the pressed button*/
//        data->state = LV_INDEV_STATE_PRESSED;  /*Set the pressed state*/
//     } else {
//        data->state = LV_INDEV_STATE_RELEASED; /*Set the released state*/
//     }

//     data->point = last_btn;            /*Save the last button*/
// }

void encoder_with_keys_read(lv_indev_drv_t * drv, lv_indev_data_t*data) {
	bool up, down, left, right, a, b, select, start;
	static bool prev_up = true, prev_down = true, prev_left = true, prev_right = true;
	static bool prev_a = true, prev_b = true, prev_start = true;

	bool iox_nint = gpio_read(GPIO_IOX_nINT);
	select = gpio_read(GPIO_B_SELECT);
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

	if ((!a && prev_a) || (!b && prev_b)) {
		data->key = LV_KEY_ENTER;
		data->state = LV_INDEV_STATE_PRESSED;
	}
	if (!up && prev_up) {
		data->key = LV_KEY_UP;
		data->state = LV_INDEV_STATE_PRESSED;
	}
	if (!down && prev_down) {
		data->key = LV_KEY_DOWN;
		data->state = LV_INDEV_STATE_PRESSED;
	}

	
	// if(!iox_nint) data->state = LV_INDEV_STATE_PRESSED;
	// else {
	// 	data->state = LV_INDEV_STATE_RELEASED;
	// 	/* Optionally you can also use enc_diff, if you have encoder*/
	// 	data->enc_diff = enc_get_new_moves();
	// }
}

void my_input_read(lv_indev_drv_t * indev, lv_indev_data_t * data)
{
    static uint32_t last_key = 0;
    bool up, down, left, right, a, b, start;
		printf("YO");

    // Read IO expander (active low)
    bool iox_nint = gpio_read(GPIO_IOX_nINT);
    if (!iox_nint) {
        read_io_expander_states(0);
        up      = !gpio_read(IOX_B_UP);     // invert since active low
        down    = !gpio_read(IOX_B_DOWN);
        left    = !gpio_read(IOX_B_LEFT);
        right   = !gpio_read(IOX_B_RIGHT);
        a       = !gpio_read(IOX_B_A);
        b       = !gpio_read(IOX_B_B);
        start   = !gpio_read(IOX_B_START);
    }

    // Default: released
    data->state = LV_INDEV_STATE_RELEASED;
    data->key = last_key;

    // Map buttons to LVGL keys
    if(up) {
        last_key = LV_KEY_LEFT;
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = LV_KEY_LEFT;
    }
    else if(down) {
        last_key = LV_KEY_RIGHT;
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = LV_KEY_RIGHT;
		printf("DOWN");
    }
    else if(a) {
        last_key = LV_KEY_ENTER;
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = LV_KEY_ENTER;
    }
    else if(b) {
        last_key = LV_KEY_ESC; // You can use ESC or assign another key
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = LV_KEY_ESC;
    }
	data->continue_reading = true;
}

/*
// Main ROM selector
void rom_file_selector() {	
    // Create list
    lv_init();

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, lv_buf1, NULL, DISP_HOR_RES * LV_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_VER_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;

	lv_disp_t * disp = lv_disp_drv_register(&disp_drv);

    
	lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_KEYPAD;   // <--- important
	indev_drv.read_cb = my_input_read;
	lv_indev_t * indev_keypad = lv_indev_drv_register(&indev_drv);

	///


    rom_list = lv_list_create(lv_scr_act());
    lv_obj_set_size(rom_list, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_center(rom_list);


	num_file = rom_file_selector_display_page(filename, num_page);

	lv_obj_clean(rom_list);

	lv_list_add_text(rom_list, "Roms");

	lv_group_t * g = lv_group_create();

    for (uint16_t i = 0; i < num_file; i++) {
		lv_obj_t * btn = lv_list_add_btn(rom_list, LV_SYMBOL_FILE, filename[i]);
		lv_obj_add_event_cb(btn, event_handler, LV_EVENT_CLICKED, NULL);

		// Add the button to the group so keypad Up/Down will change focus between buttons
        lv_group_add_obj(g, btn);

    }

	// lv_group_add_obj(g, rom_list);     // attach your list
	lv_indev_set_group(indev_keypad, g);

	// input loop
    bool up, down, left, right, a, b, select, start;
	// Keep previous states (initialize all to "released" = true, since buttons are active-low)
	static bool prev_up = true, prev_down = true, prev_left = true, prev_right = true;
	static bool prev_a = true, prev_b = true, prev_start = true;

	// sleep_ms(3000);
	print_memory_usage();

	while (true) {
		// bool iox_nint = gpio_read(GPIO_IOX_nINT);
		// if (!iox_nint) {
		// 	read_io_expander_states(0);
		// 	up      = gpio_read(IOX_B_UP);
		// 	down    = gpio_read(IOX_B_DOWN);
		// 	left    = gpio_read(IOX_B_LEFT);
		// 	right   = gpio_read(IOX_B_RIGHT);
		// 	a       = gpio_read(IOX_B_A);
		// 	b       = gpio_read(IOX_B_B);
		// 	start   = gpio_read(IOX_B_START);
		// }

		// // START button (edge detect)
		// if (!start && prev_start) {   // just pressed
		// 	break;  // exit menu
		// }

		// // A/B button (edge detect)
		// if ((!a && prev_a) || (!b && prev_b)) {
		// 	printf("LOADING %s\n", filename[selected]);
		// 	load_cart_rom_file(filename[selected]);
		// 	break;
		// }

		// // DOWN button
		// if (!down && prev_down) {
		// 	int new_index = (selected + 1) % num_file;
		// 	// rom_highlight_update(new_index);

		// 	selected++;
		// 	if(selected>=num_file) selected=0;
		// 	printf("%s\n", filename[selected]);
		// }

		// // UP button
		// if (!up && prev_up) {
		// 	int new_index = (selected == 0) ? num_file - 1 : selected - 1;
		// 	// rom_highlight_update(new_index);
		// }

		// // RIGHT button (next page)
		// if (!right && prev_right) {
		// 	num_page++;
		// 	num_file = rom_file_selector_display_page(filename, num_page);
		// 	if (num_file == 0) {
		// 		num_page--;
		// 		num_file = rom_file_selector_display_page(filename, num_page);
		// 	}
		// }

		// // LEFT button (prev page)
		// if (!left && prev_left && num_page > 0) {
		// 	num_page--;
		// 	num_file = rom_file_selector_display_page(filename, num_page);
		// }

		// // Save states for edge detection
		// prev_up = up;
		// prev_down = down;
		// prev_left = left;
		// prev_right = right;
		// prev_a = a;
		// prev_b = b;
		// prev_start = start;

		lv_tick_inc(5);
		lv_timer_handler();
		sleep_ms(5);
	}
}
*/
uint16_t get_bat_charge_percent();
uint16_t get_remaining_bat_capacity_mAh();
uint16_t get_full_bat_capacity_mAh();
uint16_t read_voltage_mV();

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
    uint16_t bat_percent = get_bat_charge_percent();
	uint16_t remaining_cap = get_remaining_bat_capacity_mAh();
	uint16_t full_cap = get_full_bat_capacity_mAh();
	uint16_t voltage_mV = read_voltage_mV();

	lv_obj_t *percent = lv_label_create(list);
	char percent_text[64];  // increased size for multiple lines
    snprintf(percent_text, sizeof(percent_text),
            "%d%% %d/%dmAh %.2fV",
            bat_percent, remaining_cap, full_cap, voltage_mV / 1000.0);
	lv_label_set_text(percent, percent_text);
	lv_obj_set_style_text_font(percent, LV_FONT_DEFAULT, 0);
	lv_obj_align(percent, LV_ALIGN_BOTTOM_MID, 0, 5);

    // --- Time and date ---
    struct tm now = draft_tm;
    // if (!mcp7940n_get_tm(RTC_I2C_PORT, &now)) return;

    const char *weekdays[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};

    lv_color_t normal_color = lv_color_hex(0x202020);
    lv_color_t highlight_color = lv_color_hex(0x33CC66);

    int y_offset = 10;
    int spacing = 6;

    // weekday
    lv_obj_t *weekday_label = lv_label_create(list);
    lv_label_set_text_fmt(weekday_label, "%s", weekdays[now.tm_wday]);
    lv_obj_set_style_text_color(weekday_label,
        (selected_date_value == 0) ? highlight_color : normal_color, 0);
    lv_obj_align(weekday_label, LV_ALIGN_TOP_MID, 0, y_offset);

    // --- Date: YYYY/MM/DD ---
    char date_text[16];
    snprintf(date_text, sizeof(date_text), "%04d/%02d/%02d",
             now.tm_year + 1900, now.tm_mon + 1, now.tm_mday);
    lv_obj_t *date_label = lv_label_create(list);
    lv_label_set_text(date_label, date_text);
    lv_obj_set_style_text_font(date_label, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(date_label, normal_color, 0);
    lv_obj_align_to(date_label, weekday_label, LV_ALIGN_OUT_BOTTOM_MID, 0, spacing);

    if (selected_date_value >= 1 && selected_date_value <= 3) {
        // Highlight specific part of the date
        // We’ll overlay smaller labels with green for whichever field is selected
        lv_obj_t *highlight_label = lv_label_create(list);
        lv_obj_set_style_text_color(highlight_label, highlight_color, 0);
        lv_obj_set_style_text_font(highlight_label, LV_FONT_DEFAULT, 0);

        if (selected_date_value == 1)
            lv_label_set_text_fmt(highlight_label, "%04d", now.tm_year + 1900);
        else if (selected_date_value == 2)
            lv_label_set_text_fmt(highlight_label, "%02d", now.tm_mon + 1);
        else if (selected_date_value == 3)
            lv_label_set_text_fmt(highlight_label, "%02d", now.tm_mday);

        lv_obj_align_to(highlight_label, date_label, LV_ALIGN_CENTER, 0, 0);
    }

    // --- Time: HH:MM:SS (24h) ---
    char time_text[16];
    snprintf(time_text, sizeof(time_text), "%02d:%02d:%02d",
             now.tm_hour, now.tm_min, now.tm_sec);
    lv_obj_t *time_label = lv_label_create(list);
    lv_label_set_text(time_label, time_text);
    lv_obj_set_style_text_font(time_label, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(time_label, normal_color, 0);
    lv_obj_align_to(time_label, date_label, LV_ALIGN_OUT_BOTTOM_MID, 0, spacing);

    if (selected_date_value >= 4 && selected_date_value <= 6) {
        // --- Highlight which time field is selected ---
        lv_obj_t *time_highlight = lv_label_create(list);
        lv_obj_set_style_text_color(time_highlight, highlight_color, 0);
        lv_obj_set_style_text_font(time_highlight, LV_FONT_DEFAULT, 0);

        if (selected_date_value == 4)
            lv_label_set_text_fmt(time_highlight, "%02d", now.tm_hour);
        else if (selected_date_value == 5)
            lv_label_set_text_fmt(time_highlight, "%02d", now.tm_min);
        else if (selected_date_value == 6)
            lv_label_set_text_fmt(time_highlight, "%02d", now.tm_sec);

        lv_obj_align_to(time_highlight, time_label, LV_ALIGN_CENTER, 0, 0);
    }

    // --- B: Save Time label at the bottom ---
    lv_obj_t *save_label = lv_label_create(list);
    lv_label_set_text(save_label, "B: Save Time");
    lv_obj_set_style_text_font(save_label, LV_FONT_DEFAULT, 0);
    lv_obj_set_style_text_color(save_label, normal_color, 0);
    lv_obj_align(save_label, LV_ALIGN_BOTTOM_MID, 0, -spacing);

}
void draw_rom_list(lv_obj_t *list, char filenames[][256], uint16_t num_file, uint16_t selected, uint16_t page_start) {
    lv_obj_clean(list);
    if (show_settings) {
        draw_settings(list);
        return;
    }
    for (uint16_t i = 0; i < VISIBLE_ITEMS && (i + page_start) < num_file; i++) {
        lv_obj_t *list_title = lv_list_add_text(list, filenames[i + page_start]);
        lv_obj_set_style_text_font(list_title, LV_FONT_DEFAULT, 0);
        lv_obj_set_style_text_color(list_title, lv_color_black(), 0);
        lv_obj_set_style_bg_color(list_title, lv_color_hex(0xFFFFFF), 0);

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

void rom_file_selector() {	
    // Create list
    lv_init();

    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, lv_buf1, NULL, DISP_HOR_RES * LV_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_VER_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;

	lv_disp_t * disp = lv_disp_drv_register(&disp_drv);
	///

	num_file = rom_file_selector_display_page(filename, num_page);

    // Create a container to hold UI
	lv_obj_t *cont = lv_obj_create(lv_scr_act());
	lv_obj_set_size(cont, DISP_HOR_RES, DISP_VER_RES);
	lv_obj_center(cont);
    lv_obj_set_style_bg_color(cont, lv_color_hex(0xFFFFFF), 0);  // dark gray background
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xFFFFFF), 0);  // match your container

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

    draw_rom_list(list, filename, num_file, selected, page_start);
        
	uint16_t bat_percent = get_bat_charge_percent();
	// uint16_t remaining_cap = get_remaining_bat_capacity_mAh();
	// uint16_t full_cap = get_full_bat_capacity_mAh();
	// uint16_t voltage_mV = read_voltage_mV();

	// lv_obj_t *percent = lv_label_create(cont);
	// char percent_text[64];  // increased size for multiple lines
    // snprintf(percent_text, sizeof(percent_text),
    //         "%d%% %d/%dmAh %.2fV",
    //         bat_percent, remaining_cap, full_cap, voltage_mV / 1000.0);
	// lv_label_set_text(percent, percent_text);
	// lv_obj_set_style_text_font(percent, LV_FONT_DEFAULT, 0);
	// lv_obj_align(percent, LV_ALIGN_BOTTOM_MID, 0, 5);
	
    // === Top bar ===
    lv_obj_t *top_bar = lv_obj_create(cont);
    lv_obj_set_size(top_bar, DISP_HOR_RES, 20);
    lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, -15);
    lv_obj_set_style_bg_color(top_bar, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_border_width(top_bar, 0, 0);
    lv_obj_set_scrollbar_mode(top_bar, LV_SCROLLBAR_MODE_OFF);

    // Top bar bottom line
    lv_obj_set_style_border_width(top_bar, 1, 0);
    lv_obj_set_style_border_side(top_bar, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_border_color(top_bar, lv_color_hex(0x000000), 0); // 0xC0C0C0

    // Left label (title)
    lv_obj_t *title_label = lv_label_create(top_bar);
    lv_label_set_text(title_label, "Pico Pal");
    lv_obj_set_style_text_font(title_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(title_label, lv_color_hex(0x202020), 0);
    lv_obj_set_style_bg_color(title_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(title_label, LV_ALIGN_LEFT_MID, 5, 0);

    // Right label (time + battery)
    struct tm now;
    mcp7940n_get_tm(RTC_I2C_PORT, &now);  // example RTC read function
    int hour12 = now.tm_hour % 12;
    if (hour12 == 0) hour12 = 12;
    const char *ampm = (now.tm_hour >= 12) ? "PM" : "AM";

    lv_obj_t *status_label = lv_label_create(top_bar);
    char status_text[32];
    snprintf(status_text, sizeof(status_text), "%d:%02d%s  %d%%",
         hour12, now.tm_min, ampm, bat_percent);

    lv_label_set_text(status_label, status_text);
    lv_obj_set_style_text_font(status_label, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(status_label, lv_color_hex(0x202020), 0);
    lv_obj_set_style_bg_color(status_label, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(status_label, LV_ALIGN_RIGHT_MID, -5, 0);

    // === Bottom hint bar ===
    lv_obj_t *hint_bar = lv_obj_create(cont);
    lv_obj_set_size(hint_bar, DISP_HOR_RES, 20);
    lv_obj_align(hint_bar, LV_ALIGN_BOTTOM_MID, 0, 15);
    lv_obj_set_style_bg_color(hint_bar, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_border_width(hint_bar, 0, 0);
    lv_obj_set_scrollbar_mode(hint_bar, LV_SCROLLBAR_MODE_OFF);

    // Bottom bar top line
    lv_obj_set_style_border_width(hint_bar, 1, 0);
    lv_obj_set_style_border_side(hint_bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_color(hint_bar, lv_color_hex(0x000000), 0);

    // Left label
    lv_obj_t *hint_label_left = lv_label_create(hint_bar);
    lv_label_set_text(hint_label_left, "A: Load");
    lv_obj_set_style_text_font(hint_label_left, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(hint_label_left, lv_color_hex(0x202020), 0);
    lv_obj_set_style_bg_color(hint_label_left, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(hint_label_left, LV_ALIGN_LEFT_MID, 5, 0);

    // Right label
    lv_obj_t *hint_label_right = lv_label_create(hint_bar);
    lv_label_set_text(hint_label_right, "Start: Recent");
    lv_obj_set_style_text_font(hint_label_right, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(hint_label_right, lv_color_hex(0x202020), 0);
    lv_obj_set_style_bg_color(hint_label_right, lv_color_hex(0xE0E0E0), 0);
    lv_obj_align(hint_label_right, LV_ALIGN_RIGHT_MID, -5, 0);


	// input loop
    bool up, down, left, right, a, b, select, start;
	// Keep previous states (initialize all to "released" = true, since buttons are active-low)
	static bool prev_up = true, prev_down = true, prev_left = true, prev_right = true;
	static bool prev_a = true, prev_b = true, prev_start = true, prev_select = true;

	// sleep_ms(3000);
	print_memory_usage();

	while (true) {
#if ENABLE_BAT_MONITORING
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
            // memset(front_fb->data, 0, sizeof(front_fb->data));
            // printf("Recent game\n");
			/* re-start the last game (no need to reprogram flash) */
			break;
		}
		if (!a && prev_a) {
			/* copy the rom from the SD card to flash and start the game */
            // memset(front_fb->data, 0, sizeof(front_fb->data));
			printf("LOADING\n");
			load_cart_rom_file(filename[selected]);
			break;
		}
        if (!b && prev_b) {
            if (show_settings) {
                mcp7940n_set_tm(RTC_I2C_PORT, &draft_tm);
                show_settings = false;
                draw_rom_list(list, filename, num_file, selected, page_start);
            }
        }
		// if (!down && prev_down) {
		// 	/* select the next rom */
		// 	selected++;
		// 	if(selected>=num_file) selected=0;
		// 	printf("%s\n", filename[selected]);
		// 	lv_label_set_text(title, filename[selected]);	
		// }
		// if(!up && prev_up) {
		// 	/* select the previous rom */
		// 	if(selected==0) {
		// 		selected=num_file-1;
		// 	} else {
		// 		selected--;
		// 	}
		// 	lv_label_set_text(title, filename[selected]);
		// }
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
		sleep_ms(5);
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
                          uint8_t *wash_out_level) {
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

    // Save to global
    g_saved_settings = s;

    return true;
}

void save_system_settings(uint8_t lcd_brightness, uint8_t button_brightness,
                          uint8_t power_brightness, int8_t selected_palette,
                          uint8_t wash_out_level) {
    system_settings_t s = {
        .magic = SYSTEM_MAGIC,
        .lcd_brightness = lcd_brightness,
        .button_brightness = button_brightness,
        .power_brightness = power_brightness,
        .selected_palette = selected_palette,
        .wash_out_level = wash_out_level
    };

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
}

static inline bool system_settings_equal(const system_settings_t *a, const system_settings_t *b) {
    return a->lcd_brightness     == b->lcd_brightness &&
           a->button_brightness  == b->button_brightness &&
           a->power_brightness   == b->power_brightness &&
           a->selected_palette   == b->selected_palette &&
           a->wash_out_level     == b->wash_out_level;
}

void save_system_settings_if_changed(uint8_t lcd_brightness, uint8_t button_brightness,
                                     uint8_t power_brightness, int8_t selected_palette,
                                     uint8_t wash_out_level) {

	system_settings_t current = {
        .magic = SYSTEM_MAGIC,
        .lcd_brightness = lcd_brightness,
        .button_brightness = button_brightness,
        .power_brightness = power_brightness,
        .selected_palette = selected_palette,
        .wash_out_level = wash_out_level
    };

    if (!system_settings_equal(&g_saved_settings, &current)) {
        printf("Settings changed, saving...\n");
        save_system_settings(lcd_brightness,
                             button_brightness,
                             power_brightness,
                             selected_palette,
                             wash_out_level);
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
