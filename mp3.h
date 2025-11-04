#define PCM_FRAME_COUNT 750
#define PCM_BUFFER_SIZE  (PCM_FRAME_COUNT * 2 * sizeof(int16_t))
#define PSRAM_BUFFER_SIZE (6*1024*1024) // 6 MB

typedef enum {
    AUDIO_HP_ONLY = 0,
    AUDIO_SPK_ONLY,
    AUDIO_BOTH
} audio_output_mode_t;

static audio_output_mode_t audio_mode = AUDIO_HP_ONLY;

void hp_on()  { dac_i2c_write(1, 0x1F, 0b11010100); }
void hp_off() { dac_i2c_write(1, 0x1F, 0x00); }

void spk_on()  { dac_i2c_write(1, 0x20, 0b11000110); }
void spk_off() { dac_i2c_write(1, 0x20, 0x00); }

void apply_audio_mode() {
    switch (audio_mode) {
        case AUDIO_HP_ONLY:
            hp_on();
            spk_off();
            break;

        case AUDIO_SPK_ONLY:
            spk_on();
            hp_off();
            break;

        case AUDIO_BOTH:
            hp_on();
            spk_on();
            break;
    }
}

bool is_pressed_now(bool curr, bool prev) {
    return (prev == false && curr == true);   // rising edge
}

typedef struct {
    uint8_t *buffer;   // PSRAM buffer
    size_t buffer_size; 
    size_t file_pos;   // file position of start of buffer
    size_t buf_filled; // bytes currently in buffer
    FIL *file;         // SD file handle
    bool eof;
} mp3_psram_ctx_t;

// --- dr_mp3 read callback ---
static size_t mp3_psram_read(void *pUserData, void *pBufferOut, size_t bytesToRead) {
    mp3_psram_ctx_t *ctx = (mp3_psram_ctx_t *)pUserData;
    if (!ctx || !pBufferOut) return 0;

    size_t bytesCopied = 0;
    while (bytesCopied < bytesToRead) {
        if (ctx->buf_filled == 0 && ctx->eof) break;

        size_t available = ctx->buf_filled;
        size_t toCopy = (bytesToRead - bytesCopied < available) ? bytesToRead - bytesCopied : available;

        memcpy((uint8_t*)pBufferOut + bytesCopied, ctx->buffer, toCopy);

        // Slide buffer forward
        memmove(ctx->buffer, ctx->buffer + toCopy, ctx->buf_filled - toCopy);
        ctx->buf_filled -= toCopy;
        ctx->file_pos += toCopy;
        bytesCopied += toCopy;

        // Refill buffer if empty
        if (ctx->buf_filled < ctx->buffer_size && !ctx->eof) {
            UINT br;
            size_t space = ctx->buffer_size - ctx->buf_filled;
            f_lseek(ctx->file, ctx->file_pos + ctx->buf_filled);
            f_read(ctx->file, ctx->buffer + ctx->buf_filled, space, &br);
            if (br == 0) ctx->eof = true;
            ctx->buf_filled += br;
        }
    }
    return bytesCopied;
}

// --- dr_mp3 seek callback (optional, can disable) ---
static drmp3_bool32 mp3_psram_seek(void *pUserData, int offset, drmp3_seek_origin origin) {
    // Seeking is complex for sliding PSRAM buffer; simplest: disallow
    return DRMP3_FALSE;
}

static drmp3_uint64 mp3_psram_tell(void *pUserData) { return 0; }

void play_mp3_from_psram(const char *filename) {
    FIL fil;
    FRESULT fr;
    UINT br;
    sd_card_t *pSD = sd_get_by_num(0);

    fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) { printf("Mount fail %d\n", fr); return; }

    fr = f_open(&fil, filename, FA_READ);
    if (fr != FR_OK) { printf("Open fail %d\n", fr); f_unmount(pSD->pcName); return; }

    uint32_t file_size = f_size(&fil);
    printf("Loading MP3 file (%lu bytes)...\n", file_size);

    uint8_t *psram_buf = malloc(file_size);
    if (!psram_buf) { printf("malloc fail\n"); return; }

    fr = f_read(&fil, psram_buf, file_size, &br);
    if (fr != FR_OK || br != file_size) {
        printf("read fail\n"); 
        free(psram_buf);
        return;
    }
    f_close(&fil);
    f_unmount(pSD->pcName);
    printf("File loaded successfully!\n");

    // --- dr_mp3 init from memory ---
    drmp3 mp3;
    if (!drmp3_init_memory(&mp3, psram_buf, file_size, NULL)) {
        printf("E drmp3_init_memory failed\n");
        free(psram_buf);
        return;
    }

    printf("MP3: %d Hz, %d ch\n", mp3.sampleRate, mp3.channels);
    i2s_set_sample_freq(&i2s_config, mp3.sampleRate, false);

    int16_t *pcm = malloc(PCM_FRAME_COUNT * mp3.channels * sizeof(int16_t));
    if (!pcm) {
        printf("pcm malloc fail\n");
        drmp3_uninit(&mp3);
        free(psram_buf);
        return;
    }
    audio_mode = headphones_present ? AUDIO_HP_ONLY : AUDIO_SPK_ONLY;
    dac_i2c_write(1, 0x21, 0x06); // setting headphone power on ramp to 100ms
    while (1) {
        drmp3_uint64 frames = drmp3_read_pcm_frames_s16(&mp3, PCM_FRAME_COUNT, pcm);
        if (frames == 0) {
            drmp3_seek_to_pcm_frame(&mp3, 0);
            continue;
        }
        
        read_volume(&i2s_config);

        // Wait until those samples finish playing
        // sleep_us((frames * 1000000ULL) / mp3.sampleRate);
        // sleep_ms(22);
		bool iox_nint = gpio_read(GPIO_IOX_nINT);
		if (!iox_nint) {
			// Read IOX port 0
			read_io_expander_states(0);

            // --- Handle Audio Output Switching ---
            // previous button states
            static bool prev_btn_a = false;
            static bool prev_btn_b = false;

            // Read your buttons directly here
            bool btn_a = gpio_read(IOX_B_A);
            bool btn_b = gpio_read(IOX_B_B);

            // --- A button: cycle through HP → SPK → BOTH ---
            if (!prev_btn_a && btn_a) {         // rising edge
                // audio_mode++;
                // if (audio_mode > AUDIO_BOTH)
                audio_mode = AUDIO_BOTH;

                apply_audio_mode();
            }

            // --- B button: swap HP <-> SPK ---
            if (!prev_btn_b && btn_b) {         // rising edge
                if (audio_mode == AUDIO_HP_ONLY) {
                    audio_mode = AUDIO_SPK_ONLY;
                } else if (audio_mode == AUDIO_SPK_ONLY) {
                    audio_mode = AUDIO_HP_ONLY;
                } else {
                    // If BOTH → choose the physically meaningful one
                    audio_mode = headphones_present ? AUDIO_HP_ONLY : AUDIO_SPK_ONLY;
                }

                apply_audio_mode();
            }

            // update prev states
            prev_btn_a = btn_a;
            prev_btn_b = btn_b;
        }

        i2s_dma_write(&i2s_config, pcm);
    }

    free(pcm);
    drmp3_uninit(&mp3);
    free(psram_buf);
}
