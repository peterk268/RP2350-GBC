#if ENABLE_SDCARD
/**
 * Load a save file from the SD card
 */
void read_cart_ram_file(struct gb_s *gb) {
	char filename[16];
	uint_fast32_t save_size;
	UINT br;
	
	gb_get_rom_name(gb,filename);
	save_size=gb_get_save_size(gb);
	if(save_size>0) {
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
	}
	printf("I write_cart_ram_file(%s) COMPLETE (%lu bytes)\n",filename,save_size);
}

/**
 * Load a .gb rom file in flash from the SD card 
 */ 
void load_cart_rom_file(char *filename) {
	UINT br;
	uint8_t buffer[FLASH_SECTOR_SIZE];
	bool mismatch=false;
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
		for(;;) {
			f_read(&fil,buffer,sizeof buffer,&br);
			if(br==0) break; /* end of file */

			uint32_t ints = save_and_disable_interrupts();
			printf("I Erasing target region...\n");
			flash_range_erase(flash_target_offset,FLASH_SECTOR_SIZE);
			printf("I Programming target region...\n");
			flash_range_program(flash_target_offset,buffer,FLASH_SECTOR_SIZE);

			restore_interrupts (ints);
			
			/* Read back target region and check programming */
			printf("I Done. Reading back target region...\n");
			for(uint32_t i=0;i<FLASH_SECTOR_SIZE;i++) {
				if(rom[flash_target_offset+i]!=buffer[i]) {
					mismatch=true;
				}
			}

			/* Next sector */
			flash_target_offset+=FLASH_SECTOR_SIZE;
		}
		if(mismatch) {
	        printf("I Programming successful!\n");
		} else {
			printf("E Programming failed!\n");
		}
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
	mk_ili9225_fill(0x0000);
	for(uint8_t ifile=0;ifile<num_file;ifile++) {
		mk_ili9225_text(filename[ifile],0,ifile*8,0xFFFF,0x0000);
    }
	return num_file;
}

/**
 * The ROM selector displays pages of up to 22 rom files
 * allowing the user to select which rom file to start
 * Copy your *.gb rom files to the root directory of the SD card
 */
void rom_file_selector() {
    uint16_t num_page;
	char filename[22][256];
	uint16_t num_file;
	
	/* display the first page with up to 22 rom files */
	num_file=rom_file_selector_display_page(filename,num_page);

	/* select the first rom */
	uint8_t selected=0;
	mk_ili9225_text(filename[selected],0,selected*8,0xFFFF,0xF800);

	/* get user's input */
	bool up,down,left,right,a,b,select,start;
	while(true) {
		
		// Read GPIO states for joypad buttons
		read_io_expander_states(0);
		up      = gpio_read(IOX_B_UP);
		down    = gpio_read(IOX_B_DOWN);
		left    = gpio_read(IOX_B_LEFT);
		right   = gpio_read(IOX_B_RIGHT);
		a       = gpio_read(IOX_B_A);
		b       = gpio_read(IOX_B_B);
		select  = gpio_read(GPIO_B_SELECT);
		start   = gpio_read(IOX_B_START);

		if(!start) {
			/* re-start the last game (no need to reprogram flash) */
			break;
		}
		if(!a | !b) {
			/* copy the rom from the SD card to flash and start the game */
			load_cart_rom_file(filename[selected]);
			break;
		}
		if(!down) {
			/* select the next rom */
			mk_ili9225_text(filename[selected],0,selected*8,0xFFFF,0x0000);
			selected++;
			if(selected>=num_file) selected=0;
			mk_ili9225_text(filename[selected],0,selected*8,0xFFFF,0xF800);
			sleep_ms(150);
		}
		if(!up) {
			/* select the previous rom */
			mk_ili9225_text(filename[selected],0,selected*8,0xFFFF,0x0000);
			if(selected==0) {
				selected=num_file-1;
			} else {
				selected--;
			}
			mk_ili9225_text(filename[selected],0,selected*8,0xFFFF,0xF800);
			sleep_ms(150);
		}
		if(!right) {
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
			mk_ili9225_text(filename[selected],0,selected*8,0xFFFF,0xF800);
			sleep_ms(150);
		}
		if((!left) && num_page>0) {
			/* select the previous page */
			num_page--;
			num_file=rom_file_selector_display_page(filename,num_page);
			/* select the first file */
			selected=0;
			mk_ili9225_text(filename[selected],0,selected*8,0xFFFF,0xF800);
			sleep_ms(150);
		}
		tight_loop_contents();
	}
}

#endif