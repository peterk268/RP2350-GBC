// ===============================================================
// Pico Pal MP3 Streaming Player (PLAYLIST + TRIPLE PCM BUFFER)
// - Scans "/music" for .mp3; if none, falls back to root
// - PCM_FRAME_COUNT = 6144 (high for long buffers in PSRAM)
// - Triple PCM buffers
// - Non-blocking I2S DMA (i2s_dma_write_non_blocking must be implemented)
// - Controls:
//      A       -> Play / Pause
//      B       -> Repeat mode (OFF -> ONE -> INFINITE -> OFF)
//      SELECT  -> Shuffle toggle
//      LEFT    -> Tap: previous track, Hold: seek backwards
//      RIGHT   -> Tap: next track,     Hold: seek forwards
//      UP/DOWN -> Reserved for GUI navigation (future)
//      START   -> Reserved for menu (future)
// ===============================================================

#include "dr_mp3.h"
#include "ff.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "pico/time.h"   // time_us_64()

#ifndef PCM_FRAME_COUNT
#define PCM_FRAME_COUNT 6144
#endif

#define MP3_STREAM_BUF_SIZE       (16 * 1024)   // 16 KB ring buffer (in PSRAM via malloc)
#define MP3_REFILL_CHUNK          (4 * 1024)    // max bytes per SD read

// Directory used for music; fallback is root ("")
#define MUSIC_DIR                 "music"

// Adaptive seek curve (seconds)
#define SEEK_STEP_1S          1
#define SEEK_STEP_2S          2
#define SEEK_STEP_3S          3
#define SEEK_STEP_5S          5
#define SEEK_STEP_10S        10

// Hold thresholds
#define SEEK_STAGE_1_MS     1500
#define SEEK_STAGE_2_MS     3000
#define SEEK_STAGE_3_MS     5000
#define SEEK_STAGE_4_MS     10000

// Base repeat interval
#define SEEK_REPEAT_INTERVAL_MS  200
#define SEEK_HOLD_TIME_MS        350

// Maximum playlist size and path length
#define MP3_MAX_TRACKS            4096
#define MP3_MAX_PATH_LEN          256


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

void apply_audio_mode(void) {
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
// Repeat / shuffle modes (global so they persist across tracks)
// ===================================================================
typedef enum {
    REPEAT_OFF = 0,
    REPEAT_ONE,        // play once more then stop (single extra loop)
    REPEAT_INFINITE    // loop forever
} repeat_mode_t;

static repeat_mode_t g_repeat_mode = REPEAT_OFF;
static bool          g_shuffle_enabled = false;

// ===================================================================
// Saving MP3 State and settings
// ===================================================================
typedef struct {
    uint32_t magic;           // for corruption check
    int      track_index;     // playlist index
    uint32_t position_ms;     // timestamp inside track
    bool     shuffle;
    uint8_t  repeat_mode;
} mp3_resume_t;

#define RESUME_MAGIC 0x504D3352   // "PM3R"
#define RESUME_FILE  "mp3_resume.bin"

static mp3_resume_t g_resume = {0};

static void mp3_save_resume(int track_index, uint32_t position_ms) {
    FIL wf;
    UINT bw;

    g_resume.magic       = RESUME_MAGIC;
    g_resume.track_index = track_index;
    g_resume.position_ms = position_ms;
    g_resume.shuffle     = g_shuffle_enabled;
    g_resume.repeat_mode = (uint8_t)g_repeat_mode;

    if (f_open(&wf, RESUME_FILE, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
        f_write(&wf, &g_resume, sizeof(g_resume), &bw);
        f_close(&wf);
        printf("Resume saved: track=%d pos=%ums\n",
               track_index, position_ms);
    }
}

// ===================================================================
// SD ring buffer logic (HEAP-BASED, in PSRAM via malloc)
// ===================================================================
typedef struct {
    FIL file;
    uint8_t *buf;                // was: uint8_t buf[MP3_STREAM_BUF_SIZE];
    uint32_t rd;
    uint32_t wr;
    uint32_t count;
    bool eof;
} mp3_stream_t;

// Reset ring buffer state after an external seek
static inline void mp3_stream_flush(mp3_stream_t *s) {
    if (!s) return;
    s->rd = 0;
    s->wr = 0;
    s->count = 0;
    s->eof = false;
}

// SD → ring buffer refill, capped to MP3_REFILL_CHUNK to avoid stalls
static void mp3_refill(mp3_stream_t *s) {
    if (!s || !s->buf) return;

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

    if (!s || !s->buf) return 0;

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

static drmp3_bool32 mp3_stream_seek(void *pUserData, int offset, drmp3_seek_origin origin)
{
    mp3_stream_t *s = (mp3_stream_t *)pUserData;
    if (!s) return DRMP3_FALSE;

    // We ignore origin and treat 'offset' as absolute byte offset from start.
    // This matches how dr_mp3 uses it for MP3 files in practice.
    if (offset < 0) offset = 0;

    if (f_lseek(&s->file, (DWORD)offset) != FR_OK)
        return DRMP3_FALSE;

    // Invalidate ring buffer because file position changed.
    mp3_stream_flush(s);

    (void)origin; // avoid unused warning
    return DRMP3_TRUE;
}


// ===================================================================
// Playlist storage (HEAP-BASED, in PSRAM via malloc)
// ===================================================================

// Pointer to N×MP3_MAX_PATH_LEN block
static char (*g_playlist)[MP3_MAX_PATH_LEN] = NULL;
static int   g_track_count  = 0;
static int   g_playlist_cap = 0;

// Lowercase helper
static char tolower_ascii(char c) {
    if (c >= 'A' && c <= 'Z') return (char)(c + 32);
    return c;
}

// Check if name ends with ".mp3" (case-insensitive)
static bool has_mp3_extension(const char *name) {
    size_t len = strlen(name);
    if (len < 4) return false;
    const char *ext = name + len - 4;
    return (tolower_ascii(ext[0]) == '.' &&
            tolower_ascii(ext[1]) == 'm' &&
            tolower_ascii(ext[2]) == 'p' &&
            tolower_ascii(ext[3]) == '3');
}

// Get basename from path ("music/rock.mp3" -> "rock.mp3")
static const char* basename_from_path(const char *path) {
    const char *slash = strrchr(path, '/');
    return slash ? (slash + 1) : path;
}

// Allocate playlist array in PSRAM
static bool mp3_playlist_init(void) {
    if (g_playlist) {
        return true; // already allocated
    }

    g_playlist = (char (*)[MP3_MAX_PATH_LEN])malloc(MP3_MAX_TRACKS * MP3_MAX_PATH_LEN);
    if (!g_playlist) {
        printf("playlist malloc fail\n");
        g_playlist_cap = 0;
        return false;
    }

    g_playlist_cap = MP3_MAX_TRACKS;
    memset(g_playlist, 0, MP3_MAX_TRACKS * MP3_MAX_PATH_LEN);
    return true;
}

// Scan a directory ("music" or "") and append .mp3 files to playlist
static int mp3_scan_dir(const char *dir) {
    DIR dir_obj;
    FILINFO fno;
    int added = 0;

    if (!g_playlist || g_playlist_cap == 0) {
        return 0;
    }

    const char *open_path = (dir && dir[0] != '\0') ? dir : "";

    FRESULT fr = f_opendir(&dir_obj, open_path);
    if (fr != FR_OK) {
        printf("f_opendir('%s') failed: %d\n", open_path, fr);
        return 0;
    }

    for (;;) {
        fr = f_readdir(&dir_obj, &fno);
        if (fr != FR_OK || fno.fname[0] == '\0')
            break;

        // Skip directories
        if (fno.fattrib & AM_DIR)
            continue;

        const char *name = fno.fname;

        // Skip hidden/metadata files (like ._whatever)
        if (name[0] == '.')
            continue;

        if (!has_mp3_extension(name))
            continue;

        if (g_track_count >= g_playlist_cap)
            break;

        // Build path
        if (dir && dir[0] != '\0')
            snprintf(g_playlist[g_track_count], MP3_MAX_PATH_LEN, "%s/%s", dir, name);
        else
            snprintf(g_playlist[g_track_count], MP3_MAX_PATH_LEN, "%s", name);

        printf("Found track: %s\n", g_playlist[g_track_count]);
        g_track_count++;
        added++;
    }

    f_closedir(&dir_obj);
    return added;
}

// Build playlist: try MUSIC_DIR, then fallback to root
static void mp3_build_playlist(void) {
    if (!mp3_playlist_init()) {
        printf("Playlist init failed\n");
        g_track_count = 0;
        return;
    }

    g_track_count = 0;

    int from_music = mp3_scan_dir(MUSIC_DIR);
    if (from_music > 0) {
        printf("Playlist: %d tracks found in '%s'\n", g_track_count, MUSIC_DIR);
        return;
    }

    // Fallback to root
    mp3_scan_dir("");
    printf("Playlist: %d tracks found in root\n", g_track_count);
}


// ===================================================================
// Seek helper: approximate seek by jumping compressed bytes
// ===================================================================

static inline int adaptive_seek_step_ms(uint64_t held_ms) {
    if (held_ms < SEEK_STAGE_1_MS)
        return SEEK_STEP_1S;
    if (held_ms < SEEK_STAGE_2_MS)
        return SEEK_STEP_2S;
    if (held_ms < SEEK_STAGE_3_MS)
        return SEEK_STEP_3S;
    if (held_ms < SEEK_STAGE_4_MS)
        return SEEK_STEP_5S;
    return SEEK_STEP_10S;
}

static bool seek_relative_seconds_pcm(
    mp3_stream_t *stream,
    drmp3* mp3,
    int seconds,
    uint64_t* played_frames,
    bool *decoded_next_chunk
){
    if (!stream || !mp3 || !played_frames) return false;

    // 1) Compute target frame
    int64_t delta_frames = (int64_t)seconds * mp3->sampleRate;
    int64_t target       = (int64_t)(*played_frames) + delta_frames;
    if (target < 0) target = 0;

    // 2) Flush buffered data and let dr_mp3 handle internal reset on seek.
    mp3_stream_flush(stream);

    // 3) Accurate PCM seek
    if (!drmp3_seek_to_pcm_frame(mp3, (drmp3_uint64)target)) {
        return false;
    }

    *played_frames = (uint64_t)target;
    *decoded_next_chunk = false;
    return true;
}

// Smart seek that chooses between fast byte-jump + PCM seek
static bool seek_relative_seconds(
    mp3_stream_t *stream,
    drmp3 *mp3,
    int seconds,
    uint64_t *played_frames,
    int sample_rate,
    bool *decoded_next_chunk
){
    if (!stream || !mp3 || !played_frames) return false;

    int abs_sec = (seconds < 0 ? -seconds : seconds);

    // Rough bytes-per-second estimate using current playback position.
    int64_t cur_pos = f_tell(&stream->file);
    int64_t cur_size = f_size(&stream->file);
    int64_t bytes_per_sec = 16000; // fallback ~128 kbps
    if (sample_rate > 0 && *played_frames > 0 && cur_pos > 0) {
        int64_t bytes_per_frame_est = cur_pos / (int64_t)(*played_frames);
        if (bytes_per_frame_est > 0) {
            int64_t est = bytes_per_frame_est * sample_rate;
            if (est > 0) bytes_per_sec = est;
        }
    }

    int64_t delta_frames = (int64_t)seconds * sample_rate;
    int64_t target_frames = (int64_t)(*played_frames) + delta_frames;
    if (target_frames < 0) target_frames = 0;

    // ----------------------------------------------------------
    // SMALL SEEK (<5s): fast byte jump only (no PCM realign)
    // ----------------------------------------------------------
    if (abs_sec < 5) {
        int64_t byte_jump = (int64_t)seconds * bytes_per_sec;
        int64_t target = cur_pos + byte_jump;

        if (target < 0) target = 0;
        if (target > cur_size) target = cur_size;

        f_lseek(&stream->file, (DWORD)target);
        mp3_stream_flush(stream);

        *played_frames = (uint64_t)target_frames;
        *decoded_next_chunk = false;
        return true;
    }

    // ----------------------------------------------------------
    // LARGE SEEK (>=5s):
    // 1) Fast byte jump (approx)
    // 2) Flush ring buffer (keep decoder alive)
    // 3) Accurate PCM seek to correct frame
    // ----------------------------------------------------------

    // Approximate skip (fast) using runtime bitrate estimate
    int64_t byte_jump = (int64_t)seconds * bytes_per_sec;
    int64_t target = cur_pos + byte_jump;

    if (target < 0) target = 0;
    if (target > cur_size)
        target = cur_size;

    f_lseek(&stream->file, (DWORD)target);

    // Reset ring buffer fully so the decoder reads from the new position.
    mp3_stream_flush(stream);

    // Accurate PCM seek for perfect alignment without tearing down the decoder
    if (!drmp3_seek_to_pcm_frame(mp3, (drmp3_uint64)target_frames)) {
        return false;
    }

    *played_frames = (uint64_t)target_frames;
    *decoded_next_chunk = false;
    return true;
}

// ===================================================================
// Single-track player: returns what the playlist should do next
// ===================================================================
typedef enum {
    PLAY_RESULT_STOP = 0,
    PLAY_RESULT_NEXT,
    PLAY_RESULT_PREV
} play_result_t;

// Non-blocking MP3 playback for a single file
static play_result_t mp3_play_single_track(const char *filepath,
                                           uint32_t resume_position_ms,
                                           int current_track_index) {

    uint64_t played_frames = 0;

    FRESULT fr;
    play_result_t result = PLAY_RESULT_STOP;

    // --- Allocate stream structure + ring buffer in PSRAM ---
    mp3_stream_t *stream = (mp3_stream_t *)calloc(1, sizeof(mp3_stream_t));
    if (!stream) {
        printf("mp3_stream_t malloc fail\n");
        return PLAY_RESULT_NEXT;
    }

    stream->buf = (uint8_t *)malloc(MP3_STREAM_BUF_SIZE);
    if (!stream->buf) {
        printf("mp3_stream_t buf malloc fail\n");
        free(stream);
        return PLAY_RESULT_NEXT;
    }

    fr = f_open(&stream->file, filepath, FA_READ);
    if (fr != FR_OK) {
        printf("Open fail %d for '%s'\n", fr, filepath);
        result = PLAY_RESULT_NEXT;
        goto CLEANUP_STREAM_ONLY;
    }

    DWORD file_size = f_size(&stream->file);
    printf("Streaming MP3 file (%lu bytes)...\n", (unsigned long)file_size);

    // Initial small prefill
    for (int i = 0; i < 4 && !stream->eof; i++) {
        mp3_refill(stream);
    }

    drmp3 mp3;
    if (!drmp3_init(&mp3,
                    mp3_stream_read,
                    mp3_stream_seek,   // no seek callback
                    NULL,   // no tell
                    NULL,   // no meta
                    stream, // pUserData
                    NULL)) {
        printf("E drmp3_init failed\n");
        result = PLAY_RESULT_NEXT;
        goto CLEANUP_FILE;
    }

    printf("MP3: %d Hz, %d ch\n", mp3.sampleRate, mp3.channels);
    i2s_set_sample_freq(&i2s_config, mp3.sampleRate, false);

    int channels = mp3.channels;
    int sample_count = PCM_FRAME_COUNT * channels;

    // Triple PCM buffers + silence buffer for pause (all in PSRAM)
    int16_t *pcmA        = (int16_t *)malloc(sample_count * sizeof(int16_t));
    int16_t *pcmB        = (int16_t *)malloc(sample_count * sizeof(int16_t));
    int16_t *pcmC        = (int16_t *)malloc(sample_count * sizeof(int16_t));
    int16_t *silence_buf = (int16_t *)calloc(sample_count, sizeof(int16_t));

    if (!pcmA || !pcmB || !pcmC || !silence_buf) {
        printf("pcm malloc fail\n");
        result = PLAY_RESULT_NEXT;
        goto CLEANUP_MP3;
    }

    int16_t *buf_play  = pcmA;
    int16_t *buf_ready = pcmB;
    int16_t *buf_fill  = pcmC;

    // Extra prefill before first decode
    for (int i = 0; i < 4 && !stream->eof; i++) {
        mp3_refill(stream);
    }

    // Resume position (if any)
    if (resume_position_ms > 0) {
        uint32_t seconds = resume_position_ms / 1000;
        bool dummy = false;
        seek_relative_seconds(stream, &mp3, seconds, &played_frames, mp3.sampleRate, &dummy);
    }


    // First buffer: decode & start DMA
    drmp3_read_pcm_frames_s16(&mp3, PCM_FRAME_COUNT, buf_play);
    i2s_dma_write(&i2s_config, (const uint16_t *)buf_play);

    // More prefill before second buffer
    for (int i = 0; i < 4 && !stream->eof; i++) {
        mp3_refill(stream);
    }

    // Decode second buffer into buf_ready
    drmp3_read_pcm_frames_s16(&mp3, PCM_FRAME_COUNT, buf_ready);

    // Playback state
    // audio_mode = headphones_present ? AUDIO_HP_ONLY : AUDIO_SPK_ONLY;
    // apply_audio_mode();
    // dac_i2c_write(1, 0x21, 0x06);   // 100ms ramp

    printf("Starting MP3 stream...\n");

    set_sd_busy(true);

    bool decoded_next_chunk = false;
    bool paused = false;

    bool next_track_requested = false;
    bool prev_track_requested = false;

    bool prev_btn_a       = false;
    bool prev_btn_b       = false;
    bool prev_btn_select  = false;
    bool prev_btn_up      = false;
    bool prev_btn_down    = false;
    bool prev_btn_left    = false;
    bool prev_btn_right   = false;
    bool prev_btn_start   = false;

    // For LEFT/RIGHT hold detection
    uint64_t left_press_us      = 0;
    uint64_t right_press_us     = 0;
    uint64_t left_last_seek_us  = 0;
    uint64_t right_last_seek_us = 0;
    bool left_held_seek  = false;
    bool right_held_seek = false;

    // ================================================================
    // MAIN PLAYBACK LOOP
    // ================================================================
    while (1) {

        // While DMA is still sending previous buffer, do useful work
        while (!i2s_dma_write_non_blocking(&i2s_config,
                                           (const uint16_t *)(paused ? silence_buf : buf_ready))) {

            watchdog_update();
            if (!gpio_read(GPIO_SW_OUT)) {
                uint32_t final_position_ms =
                (played_frames * 1000ULL) / mp3.sampleRate;

                // Save resume info
                mp3_save_resume(current_track_index, final_position_ms);

                release_power();
            }

            // SD mini-refills
            for (int i = 0;
                 i < 2 && stream->count < (MP3_STREAM_BUF_SIZE * 3 / 4) && !stream->eof;
                 i++) {
                mp3_refill(stream);
            }

            // Decode into buf_fill only if we haven’t yet and not paused
            if (!decoded_next_chunk && !paused) {
                drmp3_uint64 frames =
                    drmp3_read_pcm_frames_s16(&mp3, PCM_FRAME_COUNT, buf_fill);

                if (frames > 0)
                    played_frames += frames;

                if (frames == 0) {
                    if (stream->eof) {
                        // Handle repeat logic
                        if (g_repeat_mode == REPEAT_OFF) {
                            printf("MP3 EOF reached (no repeat) -> next track\n");
                            next_track_requested = true;
                            result = PLAY_RESULT_NEXT;
                            goto END_PLAYBACK;
                        }

                        // Restart same track
                        printf("MP3 EOF reached -> restarting track (repeat)\n");
                        f_lseek(&stream->file, 0);
                        stream->rd = stream->wr = stream->count = 0;
                        stream->eof = false;

                        drmp3_uninit(&mp3);
                        if (!drmp3_init(&mp3, mp3_stream_read,
                                        mp3_stream_seek, NULL, NULL, stream, NULL)) {
                            printf("E drmp3_init after repeat\n");
                            result = PLAY_RESULT_NEXT;
                            goto END_PLAYBACK;
                        }

                        // REPEAT_ONE: do only one extra loop, then OFF
                        if (g_repeat_mode == REPEAT_ONE) {
                            g_repeat_mode = REPEAT_OFF;
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

            // ================== BUTTONS + CONTROLS ====================
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

            // A → Play / Pause
            if (!prev_btn_a && btn_a) {
                paused = !paused;
                printf(paused ? "Paused\n" : "Playing\n");
            }

            // B → Repeat mode cycle (global)
            if (!prev_btn_b && btn_b) {
                g_repeat_mode = (repeat_mode_t)((g_repeat_mode + 1) % 3);

                if (g_repeat_mode == REPEAT_OFF) {
                    printf("Repeat: OFF\n");
                } else if (g_repeat_mode == REPEAT_ONE) {
                    printf("Repeat: ONE (play once more)\n");
                } else if (g_repeat_mode == REPEAT_INFINITE) {
                    printf("Repeat: INFINITE\n");
                }
            }

            // SELECT → Shuffle toggle (global)
            if (!prev_btn_select && select_btn) {
                g_shuffle_enabled = !g_shuffle_enabled;
                printf("Shuffle: %s\n", g_shuffle_enabled ? "ON" : "OFF");

                // FUTURE GUI:
                // if (audio_mode == AUDIO_HP_ONLY)      audio_mode = AUDIO_SPK_ONLY;
                // else if (audio_mode == AUDIO_SPK_ONLY) audio_mode = AUDIO_BOTH;
                // else                                   audio_mode = AUDIO_HP_ONLY;
                // apply_audio_mode();
            }

            // LEFT: tap = prev track, hold = seek backward
            if (btn_left) {
                if (!prev_btn_left) {
                    left_press_us     = now_us;
                    left_last_seek_us = now_us;
                    left_held_seek    = false;
                } else {
                    uint64_t held_ms = (now_us - left_press_us) / 1000;

                    if (!left_held_seek && held_ms >= SEEK_HOLD_TIME_MS && !paused) {
                        left_held_seek = true;
                        printf("Start rewind (adaptive)\n");
                    }

                    if (left_held_seek && !paused) {
                        uint64_t since_last_ms = (now_us - left_last_seek_us) / 1000;
                        if (since_last_ms >= SEEK_REPEAT_INTERVAL_MS) {

                            int step = adaptive_seek_step_ms(held_ms);
                            printf("Rewind -%ds (held %llums)\n", step, (unsigned long long)held_ms);

                            if (seek_relative_seconds(stream, &mp3, -step, &played_frames, mp3.sampleRate, &decoded_next_chunk)) {
                                // nothing else needed
                            }

                            left_last_seek_us = now_us;
                        }
                    }
                }
            } else {
                if (prev_btn_left && !left_held_seek) {
                    printf("<< Previous track\n");
                    prev_track_requested = true;
                    result = PLAY_RESULT_PREV;
                    goto END_PLAYBACK;
                }
                left_held_seek = false;
            }

            // RIGHT: tap = next track, hold = seek forward
            if (btn_right) {
                if (!prev_btn_right) {
                    right_press_us     = now_us;
                    right_last_seek_us = now_us;
                    right_held_seek    = false;
                } else {
                    uint64_t held_ms = (now_us - right_press_us) / 1000;

                    if (!right_held_seek && held_ms >= SEEK_HOLD_TIME_MS && !paused) {
                        right_held_seek = true;
                        printf("Start fast-forward (adaptive)\n");
                    }

                    if (right_held_seek && !paused) {
                        uint64_t since_last_ms = (now_us - right_last_seek_us) / 1000;
                        if (since_last_ms >= SEEK_REPEAT_INTERVAL_MS) {

                            int step = adaptive_seek_step_ms(held_ms);
                            printf("Fast-forward +%ds (held %llums)\n", step, (unsigned long long)held_ms);

                            if (seek_relative_seconds(stream, &mp3, step, &played_frames, mp3.sampleRate, &decoded_next_chunk)) {
                                // nothing else needed
                            }

                            right_last_seek_us = now_us;
                        }
                    }
                }
            } else {
                if (prev_btn_right && !right_held_seek) {
                    printf(">> Next track\n");
                    next_track_requested = true;
                    result = PLAY_RESULT_NEXT;
                    goto END_PLAYBACK;
                }
                right_held_seek = false;
            }

            // UP / DOWN reserved
            if (!prev_btn_up && btn_up) {
                // TODO: integrate with GUI navigation
            }
            if (!prev_btn_down && btn_down) {
                // TODO: integrate with GUI navigation
            }

            // START reserved
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

            // BATTERY MONITORING //
            minimal_battery_monitoring_cb();
        }

        // DMA accepted buf_ready and started playing it.
        int16_t *old = buf_play;
        buf_play  = buf_ready;   // now playing
        buf_ready = buf_fill;    // next ready
        buf_fill  = old;         // to be filled

        decoded_next_chunk = false;  // decode again on next loop
    }

END_PLAYBACK:
    free(pcmA);
    free(pcmB);
    free(pcmC);
    free(silence_buf);

CLEANUP_MP3:
    drmp3_uninit(&mp3);

CLEANUP_FILE:
    f_close(&stream->file);

CLEANUP_STREAM_ONLY:
    if (stream) {
        if (stream->buf) free(stream->buf);
        free(stream);
    }

    return result;
}


// ===================================================================
// Playlist-level player
// ===================================================================

void play_mp3_stream(const char *start_filename) {
    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) {
        printf("Mount fail %d\n", fr);
        return;
    }

    watchdog_enable(WATCHDOG_TIMEOUT_MS*12, true); // 4 second timeout, pause-on-debug = true
#if ENABLE_SAVE_ON_POWER_OFF
	hold_power(); // keep power on for saving mp3 state
#endif

    // Load resume info
    memset(&g_resume, 0, sizeof(g_resume));

    FIL rf;
    UINT br;
    if (f_open(&rf, RESUME_FILE, FA_READ) == FR_OK) {
        if (f_read(&rf, &g_resume, sizeof(g_resume), &br) == FR_OK &&
            br == sizeof(g_resume) &&
            g_resume.magic == RESUME_MAGIC) {

            printf("Resume loaded: track=%d pos=%ums\n",
                g_resume.track_index, g_resume.position_ms);
        }
        f_close(&rf);
    }

    // Build playlist from /music, then root fallback
    mp3_build_playlist();

    if (g_track_count == 0) {
        printf("No MP3 files found on SD.\n");
        f_unmount(pSD->pcName);
        return;
    }

    // Choose initial track index
    int current_index = 0;
    uint32_t resume_position_ms = 0;

    // Priority: explicit filename → saved resume → default 0
    if (start_filename && start_filename[0] != '\0') {
        for (int i = 0; i < g_track_count; i++) {
            const char *base = basename_from_path(g_playlist[i]);
            if (strcmp(base, start_filename) == 0) {
                current_index = i;
                break;
            }
        }
    }
    else if (g_resume.magic == RESUME_MAGIC &&
            g_resume.track_index >= 0 &&
            g_resume.track_index < g_track_count) {

        current_index = g_resume.track_index;
        resume_position_ms = g_resume.position_ms * 0;
        g_shuffle_enabled = g_resume.shuffle;
        g_repeat_mode    = (repeat_mode_t)g_resume.repeat_mode;

        printf("Resuming track %d at %u ms\n",
            current_index, resume_position_ms);
    }

    // Optional: seed shuffle from time
    srand((unsigned)time_us_64());

    while (1) {
        const char *path = g_playlist[current_index];
        const char *base = basename_from_path(path);

        printf("\n--- Now playing [%d/%d]: %s ---\n",
               current_index + 1, g_track_count, base);

        play_result_t r = mp3_play_single_track(path, resume_position_ms, current_index);

        // reset it for next tracks now that we're done with it.
        resume_position_ms = 0;

        if (r == PLAY_RESULT_STOP) {
            // Stop playback (e.g., user quits in future menu)
            break;
        } else if (r == PLAY_RESULT_PREV) {
            // Previous track (linear)
            current_index--;
            if (current_index < 0)
                current_index = g_track_count - 1;
        } else if (r == PLAY_RESULT_NEXT) {
            // Next track (respect shuffle)
            if (g_shuffle_enabled && g_track_count > 1) {
                int new_idx = current_index;
                while (new_idx == current_index) {
                    new_idx = rand() % g_track_count;
                }
                current_index = new_idx;
            } else {
                current_index++;
                if (current_index >= g_track_count)
                    current_index = 0;
            }
        }
    }

    f_unmount(pSD->pcName);
    // If you ever want to free playlist:
    // if (g_playlist) { free(g_playlist); g_playlist = NULL; g_playlist_cap = 0; }
}
