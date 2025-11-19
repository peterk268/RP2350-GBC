#include "dr_mp3.h"
#include "ff.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

// Keep your existing PCM_FRAME_COUNT definition
#ifndef PCM_FRAME_COUNT
#define PCM_FRAME_COUNT 738
#endif

#define MP3_STREAM_BUF_SIZE   (16 * 1024)   // 16 KB ring buffer

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

typedef struct {
    FIL file;                               // SD file handle
    uint8_t buf[MP3_STREAM_BUF_SIZE];       // ring buffer memory
    uint32_t rd;                            // read pointer
    uint32_t wr;                            // write pointer
    uint32_t count;                         // bytes currently in buffer
    bool eof;
} mp3_stream_t;


// ---------------------------------------------------------
// Refill ring buffer from SD (SD → RAM, NO memmove)
// ---------------------------------------------------------
static void mp3_refill(mp3_stream_t *s) {
    if (s->eof || s->count >= MP3_STREAM_BUF_SIZE)
        return;

    UINT br;
    uint32_t free_space = MP3_STREAM_BUF_SIZE - s->count;
    uint32_t write_pos  = s->wr;

    uint32_t max_chunk = MP3_STREAM_BUF_SIZE - write_pos;
    uint32_t to_read   = free_space < max_chunk ? free_space : max_chunk;

    FRESULT fr = f_read(&s->file, &s->buf[write_pos], to_read, &br);
    if (fr != FR_OK) {
        // On read error, just mark EOF so we stop cleanly
        s->eof = true;
        return;
    }

    s->wr = (write_pos + br) % MP3_STREAM_BUF_SIZE;
    s->count += br;

    if (br < to_read) {
        // Hit end of file
        s->eof = true;
    }
}


// ---------------------------------------------------------
// dr_mp3 read callback – matches your drmp3_init signature
// size_t (*drmp3_read_proc)(void* pUserData, void* pBufferOut, size_t bytesToRead);
// ---------------------------------------------------------
static size_t mp3_stream_read(void *pUserData, void *pBufferOut, size_t bytesToRead) {
    mp3_stream_t *s = (mp3_stream_t *)pUserData;
    uint8_t *out = (uint8_t *)pBufferOut;
    size_t copied = 0;

    while (copied < bytesToRead) {
        if (s->count == 0) {
            if (s->eof) break;
            mp3_refill(s);
            if (s->count == 0) break;
        }

        size_t chunk = MP3_STREAM_BUF_SIZE - s->rd;
        if (chunk > s->count) chunk = s->count;
        if (chunk > (bytesToRead - copied)) chunk = bytesToRead - copied;

        memcpy(out + copied, &s->buf[s->rd], chunk);

        s->rd = (s->rd + chunk) % MP3_STREAM_BUF_SIZE;
        s->count -= chunk;
        copied += chunk;
    }

    return copied;
}


// ---------------------------------------------------------
// We don't support seek / tell / meta here → pass NULL
// drmp3_seek_proc, drmp3_tell_proc, drmp3_meta_proc can be NULL
// ---------------------------------------------------------
static drmp3_bool32 mp3_stream_seek(void *pUserData, int offset, drmp3_seek_origin origin) {
    // Not used (we pass NULL into drmp3_init), just here if you want later
    (void)pUserData;
    (void)offset;
    (void)origin;
    return DRMP3_FALSE;
}


// ---------------------------------------------------------
// play_mp3_stream
// ---------------------------------------------------------
void play_mp3_stream(const char *filename) {
    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr;
    UINT br;

    fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) {
        printf("Mount fail %d\n", fr);
        return;
    }

    mp3_stream_t stream = {0};

    fr = f_open(&stream.file, filename, FA_READ);
    if (fr != FR_OK) {
        printf("Open fail %d\n", fr);
        f_unmount(pSD->pcName);
        return;
    }

    // Optional: just log size
    DWORD file_size = f_size(&stream.file);
    printf("Streaming MP3 file (%lu bytes)...\n", (unsigned long)file_size);

    // Initial fill so dr_mp3 has data to parse headers
    mp3_refill(&stream);

    drmp3 mp3;

    // Your drmp3_init signature:
    // drmp3_bool32 drmp3_init(
    //     drmp3 *pMP3,
    //     drmp3_read_proc onRead,
    //     drmp3_seek_proc onSeek,
    //     drmp3_tell_proc onTell,
    //     drmp3_meta_proc onMeta,
    //     void *pUserData,
    //     const drmp3_allocation_callbacks *pAllocationCallbacks);

    if (!drmp3_init(&mp3,
                    mp3_stream_read,
                    NULL,          // no seek
                    NULL,          // no tell
                    NULL,          // no meta
                    &stream,
                    NULL)) {       // default allocators
        printf("E drmp3_init failed\n");
        f_close(&stream.file);
        f_unmount(pSD->pcName);
        return;
    }

    printf("MP3: %d Hz, %d ch\n", mp3.sampleRate, mp3.channels);
    i2s_set_sample_freq(&i2s_config, mp3.sampleRate, false);

    int16_t *pcm = malloc(PCM_FRAME_COUNT * mp3.channels * sizeof(int16_t));
    if (!pcm) {
        printf("pcm malloc fail\n");
        drmp3_uninit(&mp3);
        f_close(&stream.file);
        f_unmount(pSD->pcName);
        return;
    }

    // Audio mode at start
    audio_mode = headphones_present ? AUDIO_HP_ONLY : AUDIO_SPK_ONLY;
    apply_audio_mode();

    // Shorter HP ramp
    dac_i2c_write(1, 0x21, 0x06); // 100ms driver power-on / ramp

    printf("Starting MP3 stream...\n");

    // Previous button state for edge detection
    static bool prev_btn_a = false;
    static bool prev_btn_b = false;

    // ---------------- MAIN LOOP ----------------
    while (1) {
        mp3_refill(&stream);

        drmp3_uint64 frames = drmp3_read_pcm_frames_s16(&mp3, PCM_FRAME_COUNT, pcm);
        if (frames == 0) {
            if (stream.eof) {
                // Reached end of file → stop playback for now
                printf("MP3 EOF reached.\n");
                break;
            }
            // Not EOF but no frames → try again
            continue;
        }

        // Your volume logic
        read_volume(&i2s_config);

        // --- IOX / button handling (same as your original semantics) ---
        bool iox_nint = gpio_read(GPIO_IOX_nINT);
        if (!iox_nint) {
            read_io_expander_states(0);

            bool btn_a = gpio_read(IOX_B_A);
            bool btn_b = gpio_read(IOX_B_B);

            // A: always force BOTH
            if (!prev_btn_a && btn_a) {
                audio_mode = AUDIO_BOTH;
                apply_audio_mode();
            }

            // B: toggle HP <-> SPK, or resolve BOTH → one side
            if (!prev_btn_b && btn_b) {
                if (audio_mode == AUDIO_HP_ONLY) {
                    audio_mode = AUDIO_SPK_ONLY;
                } else if (audio_mode == AUDIO_SPK_ONLY) {
                    audio_mode = AUDIO_HP_ONLY;
                } else {
                    audio_mode = headphones_present ? AUDIO_HP_ONLY : AUDIO_SPK_ONLY;
                }
                apply_audio_mode();
            }

            prev_btn_a = btn_a;
            prev_btn_b = btn_b;
        }

        // Send frames to I2S
        i2s_dma_write(&i2s_config, pcm);
    }

    // Cleanup
    free(pcm);
    drmp3_uninit(&mp3);
    f_close(&stream.file);
    f_unmount(pSD->pcName);
}
