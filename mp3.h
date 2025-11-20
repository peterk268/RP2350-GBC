// ===============================================================
// Pico Pal MP3 Streaming Player (TRIPLE PCM BUFFER VERSION)
// - Streaming from SD via 16 KB ring buffer
// - PCM_FRAME_COUNT = 2048
// - Triple PCM buffers
// - Non-blocking I2S DMA
// - Controls:
//      A       -> Play / Pause
//      B       -> Repeat mode (OFF -> ONE -> INFINITE -> OFF)
//      SELECT  -> Shuffle toggle (future menu / GUI)
//      LEFT    -> Tap: prev track (TODO), Hold: seek -5s
//      RIGHT   -> Tap: next track (TODO), Hold: seek +5s
//      UP/DOWN -> Reserved for GUI navigation
//      START   -> Reserved for future menu
// ===============================================================

#include "dr_mp3.h"
#include "ff.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico/time.h"   // for time_us_64()

#ifndef PCM_FRAME_COUNT
#define PCM_FRAME_COUNT 2048
#endif

#define MP3_STREAM_BUF_SIZE       (16 * 1024)   // 16 KB ring buffer
#define MP3_REFILL_CHUNK          (4 * 1024)    // max bytes per SD read

// ---- Seek behaviour tuning ----
#define SEEK_HOLD_TIME_MS         250           // Hold LEFT/RIGHT this long to start seeking
#define SEEK_REPEAT_INTERVAL_MS   150           // Seek step repeat while held
#define SEEK_STEP_SECONDS         2             // Approx seek +/- 5s each step


// ===================================================================
// Audio output mode (headphones / speaker / both)
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

// SD → ring buffer refill, but capped to MP3_REFILL_CHUNK to avoid stalls
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
// Repeat / shuffle modes
// ===================================================================
typedef enum {
    REPEAT_OFF = 0,
    REPEAT_ONE,        // play once more then stop (single extra loop)
    REPEAT_INFINITE    // loop forever
} repeat_mode_t;

// ============================================================
// C helper: approximate seek by jumping compressed bytes
// ============================================================
static bool seek_relative_seconds(
    mp3_stream_t *stream,
    drmp3 *mp3,
    int seconds,
    int sample_rate,
    bool *decoded_next_chunk
){
    DWORD pos = f_tell(&stream->file);

    // Rough byte estimate: sample_rate * 2 bytes per sample
    int64_t delta_bytes = (int64_t)sample_rate * 2 * seconds;

    // Compute new position
    int64_t new_pos = (int64_t)pos + delta_bytes;

    if (new_pos < 0) new_pos = 0;
    if (new_pos > (int64_t)f_size(&stream->file))
        new_pos = f_size(&stream->file);

    // Perform the actual file seek
    f_lseek(&stream->file, (DWORD)new_pos);

    // Reset ring buffer
    stream->rd = 0;
    stream->wr = 0;
    stream->count = 0;
    stream->eof = false;

    // Re-init decoder
    drmp3_uninit(mp3);
    if (!drmp3_init(mp3,
                    mp3_stream_read,
                    NULL,
                    NULL,
                    NULL,
                    stream,
                    NULL))
    {
        printf("E drmp3_init after seek\n");
        return false;
    }

    *decoded_next_chunk = false;
    return true;
}

// ===================================================================
// TRIPLE BUFFERED PCM + “gentle” SD prefill + controls
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
                    NULL,   // no seek callback
                    NULL,   // no tell
                    NULL,   // no meta
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
    // TRIPLE PCM BUFFERS (PCM_FRAME_COUNT = 2048)
    // ================================================================
    int16_t *pcmA = malloc(sample_count * sizeof(int16_t));
    int16_t *pcmB = malloc(sample_count * sizeof(int16_t));
    int16_t *pcmC = malloc(sample_count * sizeof(int16_t));
    int16_t *silence_buf = calloc(sample_count, sizeof(int16_t));  // all zeros

    if (!pcmA || !pcmB || !pcmC || !silence_buf) {
        printf("pcm malloc fail\n");
        if (pcmA) free(pcmA);
        if (pcmB) free(pcmB);
        if (pcmC) free(pcmC);
        if (silence_buf) free(silence_buf);
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
    // PLAYBACK STATE + CONTROLS
    // ================================================================
    audio_mode = headphones_present ? AUDIO_HP_ONLY : AUDIO_SPK_ONLY;
    apply_audio_mode();
    dac_i2c_write(1, 0x21, 0x06);

    printf("Starting MP3 stream...\n");

    bool decoded_next_chunk = false;
    bool paused = false;
    repeat_mode_t repeat_mode = REPEAT_OFF;
    bool shuffle_enabled = false;

    static bool prev_btn_a       = false;
    static bool prev_btn_b       = false;
    static bool prev_btn_select  = false;
    static bool prev_btn_up      = false;
    static bool prev_btn_down    = false;
    static bool prev_btn_left    = false;
    static bool prev_btn_right   = false;
    static bool prev_btn_start   = false;

    // For LEFT/RIGHT hold detection
    uint64_t left_press_us  = 0;
    uint64_t right_press_us = 0;
    uint64_t left_last_seek_us  = 0;
    uint64_t right_last_seek_us = 0;
    bool left_held_seek  = false;
    bool right_held_seek = false;

    // ================================================================
    // MAIN PLAYBACK LOOP
    // ================================================================
    while (1) {

        // ============================================================
        // While DMA is still sending the previous buffer,
        // do *useful work*: SD refills + MP3 decoding + input handling.
        // ============================================================

        while (!i2s_dma_write_non_blocking(&i2s_config, paused ? silence_buf : buf_ready)) {

            // ---- SD mini-refills (keeps stream fed) ----
            for (int i = 0;
                 i < 2 && stream.count < (MP3_STREAM_BUF_SIZE * 3 / 4) && !stream.eof;
                 i++) {
                mp3_refill(&stream);
            }

            // ---- Decode *into buf_fill* only if we haven’t decoded yet ----
            if (!decoded_next_chunk && !paused) {
                drmp3_uint64 frames =
                    drmp3_read_pcm_frames_s16(&mp3, PCM_FRAME_COUNT, buf_fill);

                if (frames == 0) {
                    if (stream.eof) {
                        // Handle repeat logic
                        if (repeat_mode == REPEAT_OFF) {
                            printf("MP3 EOF reached (no repeat).\n");
                            goto END_PLAYBACK;
                        }

                        // Restart track
                        printf("MP3 EOF reached -> restarting track (repeat).\n");
                        f_lseek(&stream.file, 0);
                        stream.rd = stream.wr = stream.count = 0;
                        stream.eof = false;

                        drmp3_uninit(&mp3);
                        if (!drmp3_init(&mp3, mp3_stream_read,
                                        NULL, NULL, NULL, &stream, NULL)) {
                            printf("E drmp3_init after repeat\n");
                            goto END_PLAYBACK;
                        }

                        // If REPEAT_ONE, consume this one extra loop then go OFF
                        if (repeat_mode == REPEAT_ONE) {
                            repeat_mode = REPEAT_OFF;
                            printf("Repeat ONE complete -> Repeat OFF\n");
                        }

                        decoded_next_chunk = false;
                        continue;
                    }
                    // Not EOF but no frames – try again next cycle
                } else {
                    decoded_next_chunk = true;
                }
            }

            // =======================================================
            // BUTTONS + CONTROLS
            // =======================================================
            read_volume(&i2s_config);

            bool iox_nint    = gpio_read(GPIO_IOX_nINT);
            bool select_btn  = !gpio_read(GPIO_B_SELECT);

            bool btn_a     = false;
            bool btn_b     = false;
            bool btn_up    = false;
            bool btn_down  = false;
            bool btn_left  = false;
            bool btn_right = false;
            bool btn_start = false;

            if (!iox_nint) {
                read_io_expander_states(0);

                btn_a     = !gpio_read(IOX_B_A);
                btn_b     = !gpio_read(IOX_B_B);
                btn_up    = !gpio_read(IOX_B_UP);
                btn_down  = !gpio_read(IOX_B_DOWN);
                btn_left  = !gpio_read(IOX_B_LEFT);
                btn_right = !gpio_read(IOX_B_RIGHT);
                btn_start = !gpio_read(IOX_B_START);
            }

            uint64_t now_us = time_us_64();

            // ---- A → Play / Pause ----
            if (!prev_btn_a && btn_a) {
                paused = !paused;
                printf(paused ? "Paused\n" : "Playing\n");
            }

            // ---- B → Repeat mode cycle ----
            if (!prev_btn_b && btn_b) {
                static repeat_mode_t last_mode = REPEAT_OFF;
                repeat_mode = (repeat_mode_t)((repeat_mode + 1) % 3);

                if (repeat_mode == REPEAT_OFF) {
                    printf("Repeat: OFF\n");
                } else if (repeat_mode == REPEAT_ONE) {
                    printf("Repeat: ONE (play once more)\n");
                } else if (repeat_mode == REPEAT_INFINITE) {
                    printf("Repeat: INFINITE\n");
                }
                last_mode = repeat_mode;
            }

            // ---- SELECT → Shuffle toggle + future menu / HP-SPK toggle ----
            if (!prev_btn_select && select_btn) {
                shuffle_enabled = !shuffle_enabled;
                printf("Shuffle: %s\n", shuffle_enabled ? "ON" : "OFF");

                // FUTURE: you can move headphone/speaker toggle here in GUI:
                //
                // if (audio_mode == AUDIO_HP_ONLY)      audio_mode = AUDIO_SPK_ONLY;
                // else if (audio_mode == AUDIO_SPK_ONLY) audio_mode = AUDIO_BOTH;
                // else                                   audio_mode = AUDIO_HP_ONLY;
                // apply_audio_mode();
            }

            // ---- LEFT / RIGHT tap vs hold for seek / track skip ----
            // LEFT handling
            if (btn_left) {
                if (!prev_btn_left) {
                    // New press
                    left_press_us = now_us;
                    left_last_seek_us = now_us;
                    left_held_seek = false;
                } else {
                    // Held
                    uint64_t held_ms =
                        (now_us - left_press_us) / 1000;
                    if (!left_held_seek && held_ms >= SEEK_HOLD_TIME_MS && !paused) {
                        left_held_seek = true;
                        printf("Start rewind (hold LEFT)\n");
                    }
                    if (left_held_seek && !paused) {
                        uint64_t since_last_ms =
                            (now_us - left_last_seek_us) / 1000;
                        if (since_last_ms >= SEEK_REPEAT_INTERVAL_MS) {
                            printf("Rewind -%ds\n", SEEK_STEP_SECONDS);
                            seek_relative_seconds(&stream, &mp3, -SEEK_STEP_SECONDS, mp3.sampleRate, &decoded_next_chunk);
                            left_last_seek_us = now_us;
                        }
                    }
                }
            } else {
                if (prev_btn_left) {
                    // Released
                    if (!left_held_seek) {
                        // Short tap -> previous track (TODO)
                        printf("<< Previous track (TODO)\n");
                    }
                }
                left_held_seek = false;
            }

            // RIGHT handling
            if (btn_right) {
                if (!prev_btn_right) {
                    // New press
                    right_press_us = now_us;
                    right_last_seek_us = now_us;
                    right_held_seek = false;
                } else {
                    // Held
                    uint64_t held_ms =
                        (now_us - right_press_us) / 1000;
                    if (!right_held_seek && held_ms >= SEEK_HOLD_TIME_MS && !paused) {
                        right_held_seek = true;
                        printf("Start fast-forward (hold RIGHT)\n");
                    }
                    if (right_held_seek && !paused) {
                        uint64_t since_last_ms =
                            (now_us - right_last_seek_us) / 1000;
                        if (since_last_ms >= SEEK_REPEAT_INTERVAL_MS) {
                            printf("Fast-forward +%ds\n", SEEK_STEP_SECONDS);
                            seek_relative_seconds(&stream, &mp3, SEEK_STEP_SECONDS, mp3.sampleRate, &decoded_next_chunk);
                            right_last_seek_us = now_us;
                        }
                    }
                }
            } else {
                if (prev_btn_right) {
                    if (!right_held_seek) {
                        // Short tap -> next track (TODO)
                        printf(">> Next track (TODO)\n");
                    }
                }
                right_held_seek = false;
            }

            // ---- UP / DOWN reserved for GUI / navigation ----
            if (!prev_btn_up && btn_up) {
                // TODO: integrate with GUI navigation
                // printf("UP (GUI TODO)\n");
            }
            if (!prev_btn_down && btn_down) {
                // TODO: integrate with GUI navigation
                // printf("DOWN (GUI TODO)\n");
            }

            // ---- START reserved for future menu ----
            if (!prev_btn_start && btn_start) {
                printf("[MENU] (TODO)\n");
            }

            // Save previous button states
            prev_btn_a      = btn_a;
            prev_btn_b      = btn_b;
            prev_btn_up     = btn_up;
            prev_btn_down   = btn_down;
            prev_btn_left   = btn_left;
            prev_btn_right  = btn_right;
            prev_btn_start  = btn_start;
            prev_btn_select = select_btn;
        }

        // ============================================================
        // If we get here, DMA accepted buf_ready and started playing it.
        // Now rotate buffers and mark we need to decode again.
        // ============================================================

        int16_t *old = buf_play;
        buf_play  = buf_ready;   // buffered PCM now playing
        buf_ready = buf_fill;    // next PCM ready
        buf_fill  = old;         // old play buffer becomes the new fill target

        decoded_next_chunk = false;  // must decode next chunk again
    }

    // ================================================================
    // CLEANUP
    // ================================================================
END_PLAYBACK:
    free(pcmA);
    free(pcmB);
    free(pcmC);
    free(silence_buf);
    drmp3_uninit(&mp3);
    f_close(&stream.file);
    f_unmount(pSD->pcName);
}
