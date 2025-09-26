#if ENABLE_SDCARD
/**
 * Load a save file from the SD card
 */

void set_sd_busy(bool is_sd_busy) {
	sd_busy = is_sd_busy;
	__sev();
}
void read_cart_ram_file(struct gb_s *gb) {
	char filename[16];
	uint_fast32_t save_size;
	UINT br;
	
	gb_get_rom_name(gb,filename);
	save_size=gb_get_save_size(gb);
	if(save_size>0) {
		// set_sd_busy(true);

		sd_card_t *pSD=sd_get_by_num(0);
		FRESULT fr=f_mount(&pSD->fatfs,pSD->pcName,1);
		if (FR_OK!=fr) {
			printf("E f_mount error: %s (%d)\n",FRESULT_str(fr),fr);
			return;
		}

		FIL fil;
		fr=f_open(&fil,filename,FA_READ);
		if (fr==FR_OK) {
			f_read(&fil,ram,f_size(&fil),&br);
		} else {
			printf("E f_open(%s) error: %s (%d)\n",filename,FRESULT_str(fr),fr);
		}
		
		fr=f_close(&fil);
		if(fr!=FR_OK) {
			printf("E f_close error: %s (%d)\n", FRESULT_str(fr), fr);
		}
		f_unmount(pSD->pcName);	

		// set_sd_busy(false);
	}
	printf("I read_cart_ram_file(%s) COMPLETE (%lu bytes)\n",filename,save_size);
}

/**
 * Write a save file to the SD card
 */
void write_cart_ram_file(struct gb_s *gb) {
	char filename[16];
	uint_fast32_t save_size;
	UINT bw;
	
	gb_get_rom_name(gb,filename);
	save_size=gb_get_save_size(gb);
	if(save_size>0) {
		// set_sd_busy(true);

		sd_card_t *pSD=sd_get_by_num(0);
		FRESULT fr=f_mount(&pSD->fatfs,pSD->pcName,1);
		if (FR_OK!=fr) {
			printf("E f_mount error: %s (%d)\n",FRESULT_str(fr),fr);
			return;
		}

		FIL fil;
		fr=f_open(&fil,filename,FA_CREATE_ALWAYS | FA_WRITE);
		if (fr==FR_OK) {
			f_write(&fil,ram,save_size,&bw);
		} else {
			printf("E f_open(%s) error: %s (%d)\n",filename,FRESULT_str(fr),fr);
		}
		
		fr=f_close(&fil);
		if(fr!=FR_OK) {
			printf("E f_close error: %s (%d)\n", FRESULT_str(fr), fr);
		}
		f_unmount(pSD->pcName);

		// set_sd_busy(false);
	}
	printf("I write_cart_ram_file(%s) COMPLETE (%lu bytes)\n",filename,save_size);
}

/**
 * Load a .gb rom file in flash from the SD card 
 */ 
void load_cart_rom_file(char *filename) {
	UINT br;
	uint8_t buffer[FLASH_SECTOR_SIZE];
	// bool mismatch=false;
	sd_card_t *pSD=sd_get_by_num(0);
	FRESULT fr=f_mount(&pSD->fatfs,pSD->pcName,1);
	if (FR_OK!=fr) {
		printf("E f_mount error: %s (%d)\n",FRESULT_str(fr),fr);
		return;
	}
	FIL fil;
	fr=f_open(&fil,filename,FA_READ);
	if (fr==FR_OK) {
		uint32_t flash_target_offset=FLASH_TARGET_OFFSET;
		set_sd_busy(true);
		for(;;) {
			#warning "DMA/FLASH/RESOURCE CONTENTION WITH SD AND SCANVIDEO, SLEEPING FIXES"
			sleep_ms(1);
			f_read(&fil,buffer,sizeof buffer,&br);
			if(br==0) break; /* end of file */

			uint32_t ints = save_and_disable_interrupts();

			// printf("I Erasing target region...\n");
			flash_range_erase(flash_target_offset,FLASH_SECTOR_SIZE);
			// printf("I Programming target region...\n");
			flash_range_program(flash_target_offset,buffer,FLASH_SECTOR_SIZE);

			restore_interrupts (ints);
			
			/* Read back target region and check programming */
			// printf("I Done. Reading back target region...\n");
			// for(uint32_t i=0;i<FLASH_SECTOR_SIZE;i++) {
			// 	if(rom[flash_target_offset+i]!=buffer[i]) {
			// 		mismatch=true;
			// 	}
			// }

			/* Next sector */
			flash_target_offset+=FLASH_SECTOR_SIZE;
			#warning "Implement this soon"
			// CHATGPT OPTIMIZATION
			/*
			#define BLOCK_SIZE 65536  // 64KB
			uint8_t buffer[BLOCK_SIZE];  // Needs enough RAM (check stack usage)

			for (;;) {
				f_read(&fil, buffer, BLOCK_SIZE, &br);
				if (br == 0) break;

				// Erase all sectors in this block
				for (uint32_t i = 0; i < br; i += FLASH_SECTOR_SIZE)
					flash_range_erase(flash_target_offset + i, FLASH_SECTOR_SIZE);

				// Program in 256-byte chunks (page size)
				for (uint32_t i = 0; i < br; i += FLASH_PAGE_SIZE)
					flash_range_program(flash_target_offset + i, buffer + i, FLASH_PAGE_SIZE);

				flash_target_offset += br;
			}
			*/
		
			/*64KB Sector erase FUKYES
			void load_cart_rom_file(char *filename) {
				UINT br;
				const uint32_t BLOCK_SIZE = 64 * 1024;  // 64KB
				const uint32_t PAGE_SIZE = 256;
				uint8_t buffer[BLOCK_SIZE];  // Ensure you have enough stack or use malloc
				sd_card_t *pSD = sd_get_by_num(0);
				FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
				if (FR_OK != fr) {
					printf("E f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
					return;
				}

				FIL fil;
				fr = f_open(&fil, filename, FA_READ);
				if (fr == FR_OK) {
					uint32_t flash_target_offset = FLASH_TARGET_OFFSET;

					for (;;) {
						f_read(&fil, buffer, BLOCK_SIZE, &br);
						if (br == 0) break;

						uint32_t ints = save_and_disable_interrupts();

						// Erase one 64KB block
						flash_do_cmd_addr(FUNC_XIP_SSI, 0xD8, flash_target_offset);
						// Wait until erase is complete (WIP bit in status reg cleared)
						while (flash_get_cmd(FUNC_XIP_SSI, 0x05) & 0x01);

						// Program in 256-byte chunks (page size)
						for (uint32_t i = 0; i < br; i += PAGE_SIZE) {
							flash_range_program(flash_target_offset + i, buffer + i, PAGE_SIZE);
						}

						restore_interrupts(ints);
						flash_target_offset += br;
					}
				} else {
					printf("E f_open(%s) error: %s (%d)\n", filename, FRESULT_str(fr), fr);
				}

				fr = f_close(&fil);
				if (fr != FR_OK) {
					printf("E f_close error: %s (%d)\n", FRESULT_str(fr), fr);
				}
				f_unmount(pSD->pcName);
				printf("I load_cart_rom_file(%s) COMPLETE\n", filename);
			}

			*/
		}
		// if(mismatch) {
	    //     printf("I Programming successful!\n");
		// } else {
		// 	printf("E Programming failed!\n");
		// }
		set_sd_busy(false);
	} else {
		printf("E f_open(%s) error: %s (%d)\n",filename,FRESULT_str(fr),fr);
	}
	
	fr=f_close(&fil);
	if(fr!=FR_OK) {
		printf("E f_close error: %s (%d)\n", FRESULT_str(fr), fr);
	}
	f_unmount(pSD->pcName);

	printf("I load_cart_rom_file(%s) COMPLETE (%lu bytes)\n",filename,br);
}

/**
 * Function used by the rom file selector to display one page of .gb rom files
 */
uint16_t rom_file_selector_display_page(char filename[22][256],uint16_t num_page) {
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
	for(uint8_t ifile=0;ifile<22;ifile++) {
		strcpy(filename[ifile],"");
	}

    /* search *.gb files */
	#warning "This is a problem for .gba"
	uint16_t num_file=0;
	fr=f_findfirst(&dj, &fno, "", "*.gb*");

	/* skip the first N pages */
	if(num_page>0) {
		while(num_file<num_page*22 && fr == FR_OK && fno.fname[0]) {
			if (fno.fname[0] != '.') {
				num_file++;
			}
			fr=f_findnext(&dj, &fno);
		}
	}

	/* store the filenames of this page */
	num_file=0;
    while(num_file<22 && fr == FR_OK && fno.fname[0]) {
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

#define MAX_FILES 22

static lv_obj_t *rom_list;
static uint16_t num_page = 0;
static uint16_t num_file = 0;
static char filename[MAX_FILES][256];
static uint16_t selected = 0;

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

	// Title label
	lv_obj_t *title = lv_label_create(cont);
	lv_label_set_text(title, filename[selected]);
	lv_obj_set_style_text_font(title, &lv_font_montserrat_10, 0);
	lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);

	uint16_t bat_percent = get_bat_charge_percent();
	lv_obj_t *percent = lv_label_create(cont);
	char percent_text[32];
	snprintf(percent_text, sizeof(percent_text), "Battery: %d%%", bat_percent);
	lv_label_set_text(percent, percent_text);
	lv_obj_set_style_text_font(percent, &lv_font_montserrat_10, 0);
	lv_obj_align(percent, LV_ALIGN_BOTTOM_MID, 0, 5);
	

	// input loop
    bool up, down, left, right, a, b, select, start;
	// Keep previous states (initialize all to "released" = true, since buttons are active-low)
	static bool prev_up = true, prev_down = true, prev_left = true, prev_right = true;
	static bool prev_a = true, prev_b = true, prev_start = true;

	// sleep_ms(3000);
	print_memory_usage();

	while (true) {

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
			/* re-start the last game (no need to reprogram flash) */
			break;
		}
		if ((!a && prev_a) || (!b && prev_b)) {
			/* copy the rom from the SD card to flash and start the game */
			printf("LOADING\n");
			load_cart_rom_file(filename[selected]);
			break;
		}
		if (!down && prev_down) {
			/* select the next rom */
			selected++;
			if(selected>=num_file) selected=0;
			printf("%s\n", filename[selected]);
			lv_label_set_text(title, filename[selected]);	
		}
		if(!up && prev_up) {
			/* select the previous rom */
			if(selected==0) {
				selected=num_file-1;
			} else {
				selected--;
			}
			lv_label_set_text(title, filename[selected]);
		}
		if (!right && prev_right) {
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
			lv_label_set_text(title, filename[selected]);
			//mk_ili9225_text(filename[selected],0,selected*8,0xFFFF,0xF800);
		}
		if (!left && prev_left && num_page > 0) {
			/* select the previous page */
			num_page--;
			num_file=rom_file_selector_display_page(filename,num_page);
			/* select the first file */
			selected=0;
			lv_label_set_text(title, filename[selected]);
			//mk_ili9225_text(filename[selected],0,selected*8,0xFFFF,0xF800);
		}
		// Save states for edge detection
		prev_up = up;
		prev_down = down;
		prev_left = left;
		prev_right = right;
		prev_a = a;
		prev_b = b;
		prev_start = start;

		// in a 1ms timer interrupt or loop
		lv_tick_inc(5);
		lv_timer_handler();
		sleep_ms(5);
	}
}

#endif