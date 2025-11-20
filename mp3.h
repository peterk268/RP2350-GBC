// ===============================================================
// Pico Pal MP3 Streaming Player (TRIPLE PCM BUFFER VERSION)
// - Uses your original working streaming code
// - PCM_FRAME_COUNT = 738
// - 16 KB MP3 ring buffer
// - Triple PCM buffers
// - Small, repeated SD refills to reduce stutter
// ===============================================================

#include "dr_mp3.h"
#include "ff.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef PCM_FRAME_COUNT
#define PCM_FRAME_COUNT 2048
#endif

#define MP3_STREAM_BUF_SIZE   (16 * 1024)   // 16 KB ring buffer
#define MP3_REFILL_CHUNK      (4 * 1024)    // max bytes per SD read


// ===================================================================
// Audio mode logic (unchanged)
// ===================================================================
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


// ===================================================================
// SD ring buffer logic
// ===================================================================
typedef struct {
    FIL file;
    uint8_t buf[MP3_STREAM_BUF_SIZE];
    uint32_t rd;
    uint32_t wr;
    uint32_t count;
    bool eof;
} mp3_stream_t;

// SD → ring buffer refill, but capped to MP3_REFILL_CHUNK
static void mp3_refill(mp3_stream_t *s) {
    if (s->eof || s->count >= MP3_STREAM_BUF_SIZE)
        return;

    UINT br;
    uint32_t free_space = MP3_STREAM_BUF_SIZE - s->count;
    uint32_t write_pos  = s->wr;

    uint32_t max_chunk = MP3_STREAM_BUF_SIZE - write_pos;
    uint32_t to_read   = (free_space < max_chunk) ? free_space : max_chunk;

    // Cap each SD read to avoid long stalls
    if (to_read > MP3_REFILL_CHUNK)
        to_read = MP3_REFILL_CHUNK;

    if (to_read == 0)
        return;

    FRESULT fr = f_read(&s->file, s->buf + write_pos, to_read, &br);
    if (fr != FR_OK) {
        s->eof = true;
        return;
    }

    s->wr = (write_pos + br) % MP3_STREAM_BUF_SIZE;
    s->count += br;

    if (br < to_read)
        s->eof = true;
}

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

        memcpy(out + copied, s->buf + s->rd, chunk);
        s->rd = (s->rd + chunk) % MP3_STREAM_BUF_SIZE;
        s->count -= chunk;
        copied += chunk;
    }

    return copied;
}

static drmp3_bool32 mp3_stream_seek(void *pUserData, int offset, drmp3_seek_origin origin) {
    (void)pUserData;
    (void)offset;
    (void)origin;
    return DRMP3_FALSE;
}


// ===================================================================
// TRIPLE BUFFERED PCM + “gentle” SD prefill
// ===================================================================
void play_mp3_stream(const char *filename) {
    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr;

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

    DWORD file_size = f_size(&stream.file);
    printf("Streaming MP3 file (%lu bytes)...\n", (unsigned long)file_size);

    // Initial small prefill
    for (int i = 0; i < 4 && !stream.eof; i++) {
        mp3_refill(&stream);
    }

    drmp3 mp3;
    if (!drmp3_init(&mp3,
                    mp3_stream_read,
                    NULL,
                    NULL,
                    NULL,
                    &stream,
                    NULL))
    {
        printf("E drmp3_init failed\n");
        f_close(&stream.file);
        f_unmount(pSD->pcName);
        return;
    }

    printf("MP3: %d Hz, %d ch\n", mp3.sampleRate, mp3.channels);
    i2s_set_sample_freq(&i2s_config, mp3.sampleRate, false);

    int channels = mp3.channels;
    int sample_count = PCM_FRAME_COUNT * channels;

    // ================================================================
    // TRIPLE PCM BUFFERS (PCM_FRAME_COUNT stays 738)
// ================================================================
    int16_t *pcmA = malloc(sample_count * sizeof(int16_t));
    int16_t *pcmB = malloc(sample_count * sizeof(int16_t));
    int16_t *pcmC = malloc(sample_count * sizeof(int16_t));

    if (!pcmA || !pcmB || !pcmC) {
        printf("pcm malloc fail\n");
        if (pcmA) free(pcmA);
        if (pcmB) free(pcmB);
        if (pcmC) free(pcmC);
        drmp3_uninit(&mp3);
        f_close(&stream.file);
        f_unmount(pSD->pcName);
        return;
    }

    int16_t *buf_play  = pcmA;
    int16_t *buf_ready = pcmB;
    int16_t *buf_fill  = pcmC;

    // ================================================================
    // INITIAL DECODE + START
    // ================================================================

    // Some extra prefill before first decode
    for (int i = 0; i < 4 && !stream.eof; i++) {
        mp3_refill(&stream);
    }

    // First buffer: decode & start DMA
    drmp3_read_pcm_frames_s16(&mp3, PCM_FRAME_COUNT, buf_play);
    i2s_dma_write(&i2s_config, buf_play);

    // A bit more prefill before decoding second buffer
    for (int i = 0; i < 4 && !stream.eof; i++) {
        mp3_refill(&stream);
    }

    // Decode second buffer into buf_ready
    drmp3_read_pcm_frames_s16(&mp3, PCM_FRAME_COUNT, buf_ready);

    // buf_fill is free for the next decode

    // ================================================================
    // BEGIN PLAYBACK LOOP
    // ================================================================
    audio_mode = headphones_present ? AUDIO_HP_ONLY : AUDIO_SPK_ONLY;
    apply_audio_mode();
    dac_i2c_write(1, 0x21, 0x06);

    printf("Starting MP3 stream...\n");

    static bool prev_btn_a = false;
    static bool prev_btn_b = false;

    bool decoded_next_chunk = false;

    while (1) {

        // ============================================================
        // While DMA is still sending the previous buffer,
        // do *useful work*: SD refills + MP3 decoding + input handling.
        // ============================================================

        while (!i2s_dma_write_non_blocking(&i2s_config, buf_ready)) {

            // ---- SD mini-refills (keeps stream fed) ----
            for (int i = 0; i < 2 && stream.count < (MP3_STREAM_BUF_SIZE * 3 / 4) && !stream.eof; i++) {
                mp3_refill(&stream);
                printf("RE\n");
            }

            // ---- Decode *into buf_fill* only if we haven’t decoded yet ----
            if (!decoded_next_chunk) {
                drmp3_uint64 frames = drmp3_read_pcm_frames_s16(&mp3, PCM_FRAME_COUNT, buf_fill);

                if (frames == 0) {
                    if (stream.eof) {
                        printf("MP3 EOF reached.\n");
                        goto END_PLAYBACK;
                    }
                    // Otherwise, try again in next cycle
                } else {
                    decoded_next_chunk = true;
                }
            }

            // ---- Buttons + volume ----
            read_volume(&i2s_config);

            bool iox_nint = gpio_read(GPIO_IOX_nINT);
            if (!iox_nint) {
                read_io_expander_states(0);

                bool btn_a = gpio_read(IOX_B_A);
                bool btn_b = gpio_read(IOX_B_B);

                if (!prev_btn_a && btn_a) {
                    audio_mode = AUDIO_BOTH;
                    apply_audio_mode();
                }

                if (!prev_btn_b && btn_b) {
                    if (audio_mode == AUDIO_HP_ONLY) audio_mode = AUDIO_SPK_ONLY;
                    else if (audio_mode == AUDIO_SPK_ONLY) audio_mode = AUDIO_HP_ONLY;
                    else audio_mode = headphones_present ? AUDIO_HP_ONLY : AUDIO_SPK_ONLY;
                    apply_audio_mode();
                }

                prev_btn_a = btn_a;
                prev_btn_b = btn_b;
            }
        }

        // ============================================================
        // If we get here, DMA accepted buf_ready and started playing it.
        // Now rotate buffers and mark we need to decode again.
        // ============================================================

        int16_t *old = buf_play;
        buf_play  = buf_ready;   // buffered PCM now playing
        buf_ready = buf_fill;    // next PCM ready
        buf_fill  = old;         // old play buffer is now the empty fill buffer

        decoded_next_chunk = false;  // must decode next chunk again
    }

    // ================================================================
    // CLEANUP
    // ================================================================
    END_PLAYBACK:
    free(pcmA);
    free(pcmB);
    free(pcmC);
    drmp3_uninit(&mp3);
    f_close(&stream.file);
    f_unmount(pSD->pcName);
}
