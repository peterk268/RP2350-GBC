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
#define PCM_FRAME_COUNT 6144 // 16384 takes too long to load next
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

#define MP3_INACTIVE_TIMEOUT_US   (3000000ULL)   // 3 seconds

// === MP3 UI Global Objects ===
static lv_obj_t *mp3_list_obj         = NULL;
static lv_obj_t *mp3_hint_left_obj    = NULL;
static lv_obj_t *mp3_hint_right_obj   = NULL;
static lv_obj_t *mp3_status_label_obj = NULL;

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

static repeat_mode_t g_repeat_mode       = REPEAT_OFF;
static bool          g_shuffle_enabled   = false;
static bool          g_mp3_inactive      = false;
static bool          paused              = false;
// This flag means "shuffle state changed while a track was playing;
// rebuild the shuffle order once the track returns to the playlist layer".
static bool          g_shuffle_needs_rebuild = false;

// uint8_t saved_lcd_brightness = 0;
uint8_t saved_button_brightness = 0;

// bool mp3_led_faded = false;

// void fade_out_leds_mp3_inactive(void) {
//     if (mp3_led_faded) return;

//     saved_lcd_brightness    = lcd_led_duty_cycle;
//     saved_button_brightness = button_led_duty_cycle;

//     // Fade-out DOWN using the correct timer
//     fade_out_leds_powerdown();

//     mp3_led_faded = true;
// }
// void fade_in_leds_mp3_restore(void) {
//     if (!mp3_led_faded) return;

//     lcd_target_brightness    = saved_lcd_brightness;
//     button_target_brightness = saved_button_brightness;

//     // DO NOT touch pwr_target_brightness (power LED stays on always)

//     fade_in_leds_startup(); // fades towards new target brightness levels

//     mp3_led_faded = false;
// }

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

static drmp3_bool32 mp3_stream_seek(void *pUserData, int offset, drmp3_seek_origin origin) {
    (void)pUserData;
    (void)offset;
    (void)origin;
    return DRMP3_FALSE;
}


// ===================================================================
// Playlist storage (HEAP-BASED, in PSRAM via malloc)
// ===================================================================

// Pointer to N×MP3_MAX_PATH_LEN block
static char (*g_playlist)[MP3_MAX_PATH_LEN] = NULL;
static int   g_track_count  = 0;
static int   g_playlist_cap = 0;

// ===================================================================
// Playlist shuffle order (full playlist shuffling)
// ===================================================================

// Stores a permutation of [0..g_track_count-1]
static int *g_shuffle_order = NULL;
// Index into g_shuffle_order for the *current* track when shuffle is ON
static int  g_shuffle_pos   = 0;

static int g_selected_file = 0;

// Build shuffle order using Fisher–Yates, and ensure current_track is first
static void build_shuffle_order(int current_track) {
    if (g_track_count <= 0)
        return;

    if (!g_shuffle_order) {
        g_shuffle_order = (int *)malloc(sizeof(int) * g_track_count);
        if (!g_shuffle_order) {
            printf("shuffle malloc fail\n");
            return;
        }
    }

    // Fill 0..N-1
    for (int i = 0; i < g_track_count; i++) {
        g_shuffle_order[i] = i;
    }

    // Fisher–Yates shuffle
    for (int i = g_track_count - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int tmp = g_shuffle_order[i];
        g_shuffle_order[i] = g_shuffle_order[j];
        g_shuffle_order[j] = tmp;
    }

    // Ensure current track is at position 0
    for (int i = 0; i < g_track_count; i++) {
        if (g_shuffle_order[i] == current_track) {
            int tmp = g_shuffle_order[0];
            g_shuffle_order[0] = g_shuffle_order[i];
            g_shuffle_order[i] = tmp;
            break;
        }
    }

    g_shuffle_pos = 0;
}

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
static bool seek_relative_seconds(
    mp3_stream_t *stream,
    drmp3 *mp3,
    int seconds,
    int sample_rate,
    bool *decoded_next_chunk
){
    if (!stream) return false;

    DWORD pos = f_tell(&stream->file);

    // Rough byte estimate: sample_rate * 2 bytes/sample * seconds
    int64_t delta_bytes = (int64_t)sample_rate * 2 * seconds;
    int64_t new_pos = (int64_t)pos + delta_bytes;

    if (new_pos < 0) new_pos = 0;
    if (new_pos > (int64_t)f_size(&stream->file))
        new_pos = f_size(&stream->file);

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
                    NULL)) {
        printf("E drmp3_init after seek\n");
        return false;
    }

    *decoded_next_chunk = false;
    return true;
}


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

// ===================================================================
// Single-track player: returns what the playlist should do next
// ===================================================================
typedef enum {
    PLAY_RESULT_STOP = 0,
    PLAY_RESULT_NEXT,
    PLAY_RESULT_PREV,
    PLAY_RESULT_SELECTED
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
                    NULL,   // no seek callback
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
        printf("Seeking to resume position %u ms...\n", resume_position_ms);

        if (mp3.sampleRate > 0) {
            drmp3_uint64 target_frames =
                (drmp3_uint64)resume_position_ms * mp3.sampleRate / 1000;

            if (drmp3_seek_to_pcm_frame(&mp3, target_frames)) {
                played_frames = (uint64_t)target_frames;
            } else {
                printf("drmp3_seek_to_pcm_frame failed for resume\n");
                played_frames = 0;
            }
        }
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

    bool decoded_next_chunk = false;

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

    // Inactivity tracking
    uint64_t last_interaction_us = time_us_64();

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
                                        NULL, NULL, NULL, stream, NULL)) {
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


            #define ANY_BUTTON_PRESSED \
                (btn_a || btn_b || btn_up || btn_down || btn_left || btn_right || btn_start || select_btn)

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

            // -------------- MP3 Inactivity Detection --------------
            if (g_mp3_inactive && ANY_BUTTON_PRESSED) {
                // Wake up immediately
                g_mp3_inactive = false;
                last_interaction_us = now_us;
            }

            if (ANY_BUTTON_PRESSED) {
                last_interaction_us = now_us;
            }

            if (!g_mp3_inactive &&
                (now_us - last_interaction_us >= MP3_INACTIVE_TIMEOUT_US)) {

                g_mp3_inactive = true;
                printf("MP3 → inactive mode\n");
            }

            // A → Play Selected Track
            if (!prev_btn_a && btn_a) {
                result = PLAY_RESULT_SELECTED;
                goto END_PLAYBACK;
            }

            // B → Play / Pause
            if (!prev_btn_b && btn_b) {
                paused = !paused;
                printf(paused ? "Paused\n" : "Playing\n");
                update_mp3_bottom_bar_shuffle_repeat(mp3_hint_right_obj, g_repeat_mode, g_shuffle_enabled, paused);
            }

            // SELECT → Shuffle toggle (global flag ONLY; playlist will rebuild order)
            if (!prev_btn_select && select_btn) {
                g_shuffle_enabled       = !g_shuffle_enabled;
                g_shuffle_needs_rebuild = true;   // handled in playlist-level loop
                printf("Shuffle: %s\n", g_shuffle_enabled ? "ON" : "OFF");
                update_mp3_bottom_bar_shuffle_repeat(mp3_hint_right_obj, g_repeat_mode, g_shuffle_enabled, paused);
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

                            seek_relative_seconds(stream, &mp3, -step, mp3.sampleRate, &decoded_next_chunk);
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

                            seek_relative_seconds(stream, &mp3, step, mp3.sampleRate, &decoded_next_chunk);
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

            // UP
            if (!prev_btn_up && btn_up) {
                g_selected_file--;
                if (g_selected_file < 0)
                    g_selected_file = g_track_count - 1;

                draw_rom_list(
                    mp3_list_obj,
                    g_playlist,
                    g_track_count,
                    g_selected_file,
                    g_selected_file
                );
            }

            // DOWN
            if (!prev_btn_down && btn_down) {
                g_selected_file++;
                if (g_selected_file >= g_track_count)
                    g_selected_file = 0;

                draw_rom_list(
                    mp3_list_obj,
                    g_playlist,
                    g_track_count,
                    g_selected_file,
                    g_selected_file
                );
            }

            // START → Repeat mode cycle (global)
            if (!prev_btn_start && btn_start) {
                g_repeat_mode = (repeat_mode_t)((g_repeat_mode + 1) % 3);

                if (g_repeat_mode == REPEAT_OFF) {
                    printf("Repeat: OFF\n");
                } else if (g_repeat_mode == REPEAT_ONE) {
                    printf("Repeat: ONE (play once more)\n");
                } else if (g_repeat_mode == REPEAT_INFINITE) {
                    printf("Repeat: INFINITE\n");
                }
                update_mp3_bottom_bar_shuffle_repeat(mp3_hint_right_obj, g_repeat_mode, g_shuffle_enabled, paused);
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

        // ==================================================
        // LVGL UPDATE (safe spot - DMA just accepted buffer)
        // ==================================================
        static uint64_t last_lvgl_update = 0;
        uint64_t now = time_us_64();
        uint64_t time_since_last_update = now - last_lvgl_update;
        if (time_since_last_update >= 10000 && !g_mp3_inactive) {   // 10 ms (100 Hz UI updates)
            lv_tick_inc(time_since_last_update);
            lv_timer_handler();
            last_lvgl_update = now;
        }
        // ==================================================

        // BATTERY MONITORING //
        bool timer_task_flagged = minimal_battery_monitoring_cb();
        if (timer_task_flagged) update_status_label(mp3_status_label_obj);

        static bool prev_inactive = false;

        if (g_mp3_inactive && !prev_inactive) {

            // Fade out ONLY LCD + button LEDs
            // fade_out_leds_mp3_inactive();
            saved_button_brightness = button_led_duty_cycle;
            decrease_button_brightness(MAX_BRIGHTNESS);

            // Dim LCD via SD busy if applicable
            set_sd_busy(true);

            prev_inactive = true;
        }
        else if (!g_mp3_inactive && prev_inactive) {

            // Remove SD busy right before restoring LEDs
            set_sd_busy(false);

            // Fade LCD + button LEDs back in
            // fade_in_leds_mp3_restore();
            increase_button_brightness(saved_button_brightness);

            prev_inactive = false;
        }

        // DMA accepted buf_ready and started playing it.
        int16_t *old = buf_play;
        buf_play  = buf_ready;   // now playing
        buf_ready = buf_fill;    // next ready
        buf_fill  = old;         // to be filled

        decoded_next_chunk = false;  // decode again on next loop
    }

END_PLAYBACK:
    // Push a short silence to let the output settle before freeing buffers
    i2s_dma_write(&i2s_config, (const uint16_t *)silence_buf);
    i2s_dma_write(&i2s_config, (const uint16_t *)silence_buf);

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
#define EMPTY_TRACK_STRING "-------- - --------------.mp3"
#define NOW_PLAYING_SCROLL 0
const char *loading_msgs[VISIBLE_ITEMS] = {
    "Loading.mp3",
    "Scanning SD Card.mp3",
    "Almost There.mp3",
    "Lots of Music Here.mp3",
    "Organizing Tracks.mp3",
    "Tuning Audio Chip.mp3",
    "Warming Up Speakers.mp3",
    "One Second.mp3",
    "Pico Pal Almost Ready.mp3"
};

lv_obj_t *create_mp3_bottom_bar(lv_obj_t *parent,
                                lv_obj_t **left_out,
                                lv_obj_t **right_out)
{
    lv_obj_t *bottom_bar = lv_obj_create(parent);
    lv_obj_set_size(bottom_bar, DISP_HOR_RES, 20);
    lv_obj_align(bottom_bar, LV_ALIGN_BOTTOM_MID, 0, 15);
    lv_obj_set_style_bg_color(bottom_bar, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_border_width(bottom_bar, 0, 0);
    lv_obj_set_scrollbar_mode(bottom_bar, LV_SCROLLBAR_MODE_OFF);

    // Top border
    lv_obj_set_style_border_width(bottom_bar, 1, 0);
    lv_obj_set_style_border_side(bottom_bar, LV_BORDER_SIDE_TOP, 0);
    lv_obj_set_style_border_color(bottom_bar, lv_color_hex(0x000000), 0);

    // Right label first (so we know how much space it takes)
    lv_obj_t *right = lv_label_create(bottom_bar);
    lv_label_set_text(right, "R1/S/P");
    lv_obj_set_style_text_font(right, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(right, lv_color_hex(0x202020), 0);
    lv_obj_align(right, LV_ALIGN_RIGHT_MID, -5, 0);

    // Left label (scrolling)
    lv_obj_t *left = lv_label_create(bottom_bar);
    lv_label_set_text(left, "Now Playing");

    // Fix width so it cannot expand over R/S
    int left_width = DISP_HOR_RES - 77;   // 60px reserved for right label margin
    lv_obj_set_width(left, left_width);

    // Only scroll *after* width is fixed
    lv_label_set_long_mode(left, NOW_PLAYING_SCROLL ? LV_LABEL_LONG_SCROLL_CIRCULAR : LV_LABEL_LONG_CLIP);
    lv_obj_set_style_anim_speed(left, 7, 0); // adjust speed

    lv_obj_set_style_text_font(left, &lv_font_montserrat_10, 0);
    lv_obj_set_style_text_color(left, lv_color_hex(0x202020), 0);
    lv_obj_align(left, LV_ALIGN_LEFT_MID, 5, 0);

    if (left_out)  *left_out = left;
    if (right_out) *right_out = right;
    return bottom_bar;
}
void update_mp3_bottom_bar(lv_obj_t *left, lv_obj_t *right, const char *left_string, const char *right_string) {
    lv_label_set_text(left,  left_string);
    lv_label_set_text(right, right_string);
}
void update_mp3_bottom_bar_shuffle_repeat(lv_obj_t *right,
                                          uint8_t repeat_state,
                                          bool shuffle, 
                                          bool paused)
{
    char buf[8] = {0};
    uint8_t idx = 0;

    // Repeat status
    switch (repeat_state) {
        case REPEAT_ONE:
            buf[idx++] = 'R';
            buf[idx++] = '1';
            break;

        case REPEAT_INFINITE:
            buf[idx++] = 'R';
            break;

        case REPEAT_OFF:
        default:
            break;
    }

    // Space between repeat + shuffle if both exist
    if ((repeat_state != REPEAT_OFF) && shuffle) {
        buf[idx++] = '/';
    }

    // Shuffle status
    if (shuffle) {
        buf[idx++] = 'S';
    }

    // Slash between shuffle and paused
    if (shuffle && paused) {
        buf[idx++] = '/';
    }

    // Slash between repeat and paused when NO shuffle
    if ((repeat_state != REPEAT_OFF) && !shuffle && paused) {
        buf[idx++] = '/';
    }

    // --- Pause status ---
    if (paused) {
        buf[idx++] = 'P';
    }

    buf[idx] = '\0';   // null terminate

    lv_label_set_text(right, buf);
}
void update_mp3_bottom_bar_left(lv_obj_t *left, const char *text)
{
    lv_label_set_text(left, basename_from_path(text));
}


void play_mp3_stream(const char *start_filename) {
    // Create list
    lv_init();

    lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, lv_buf1, NULL, DISP_HOR_RES * LV_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_VER_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;

	lv_disp_t * disp = lv_disp_drv_register(&disp_drv);
	///

    // Create a container to hold UI
	lv_obj_t *cont = lv_obj_create(lv_scr_act());
	lv_obj_set_size(cont, DISP_HOR_RES, DISP_VER_RES);
	lv_obj_center(cont);
    lv_obj_set_style_bg_color(cont, lv_color_hex(0xFFFFFF), 0);  // dark gray background
    lv_obj_set_scrollbar_mode(cont, LV_SCROLLBAR_MODE_OFF);

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0xFFFFFF), 0);  // match your container

    // === Music List ===
    lv_obj_t *list = lv_list_create(cont);
    lv_obj_set_size(list, DISP_HOR_RES, DISP_VER_RES - 20);
    lv_obj_align(list, LV_ALIGN_TOP_MID, 0, 7);
    lv_obj_set_style_bg_color(list, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_color(list, lv_color_black(), 0);
    lv_obj_set_style_border_width(list, 0, 0);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_OFF);

    // Skeleton tracks until loaded.
    if (!mp3_playlist_init()) {
        printf("Playlist init failed\n");
        g_track_count = 0;
        return;
    }
    for (uint8_t i = 0; i < VISIBLE_ITEMS; i++) {
        snprintf(g_playlist[i], MP3_MAX_PATH_LEN, "%s", loading_msgs[i]);
    }

    draw_rom_list(list, g_playlist, VISIBLE_ITEMS, 0, 0);
        
    lv_obj_t *status_label;
    lv_obj_t *top_bar = create_top_bar(cont, &status_label);

    // Initial update
    update_status_label(status_label);

    // === Bottom hint bar ===
    lv_obj_t *hint_left;
    lv_obj_t *hint_right;
    lv_obj_t *hint_bar = create_mp3_bottom_bar(cont, &hint_left, &hint_right);

    lv_tick_inc(1);
    lv_timer_handler();

    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) {
        printf("Mount fail %d\n", fr);
        return;
    }

    watchdog_enable(WATCHDOG_TIMEOUT_MS*2, true); // 4 second timeout, pause-on-debug = true
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

        current_index        = g_resume.track_index;
        resume_position_ms   = g_resume.position_ms * 0;
        g_shuffle_enabled    = g_resume.shuffle;
        g_repeat_mode        = (repeat_mode_t)g_resume.repeat_mode;

        printf("Resuming track %d at %u ms\n",
            current_index, resume_position_ms);

        if (g_shuffle_enabled) {
            // Rebuild shuffle order based on resumed track
            build_shuffle_order(current_index);
        }
    }

    // Optional: seed shuffle from time
    srand((unsigned)time_us_64());

    draw_rom_list(list, g_playlist, g_track_count, current_index, current_index);
    update_mp3_bottom_bar_shuffle_repeat(hint_right, g_repeat_mode, g_shuffle_enabled, false);
    update_mp3_bottom_bar_left(hint_left, g_playlist[current_index]);

    mp3_list_obj = list;
    mp3_status_label_obj = status_label;
    mp3_hint_left_obj  = hint_left;
    mp3_hint_right_obj = hint_right;
    g_selected_file = current_index;
    
    lv_tick_inc(1);
    lv_timer_handler();

    while (1) {
        const char *path = g_playlist[current_index];
        const char *base = basename_from_path(path);

        printf("\n--- Now playing [%d/%d]: %s ---\n",
               current_index + 1, g_track_count, base);

        play_result_t r = mp3_play_single_track(path, resume_position_ms, current_index);

        // Any resume was only for the first track
        resume_position_ms = 0;

        // If shuffle toggle happened during this track, rebuild order now
        if (g_shuffle_needs_rebuild) {
            g_shuffle_needs_rebuild = false;
            if (g_shuffle_enabled) {
                build_shuffle_order(current_index);
            }
            // If shuffle was turned OFF, we just leave things linear;
            // current_index already matches the just-played track.
        }

        if (r == PLAY_RESULT_STOP) {
            // Stop playback (e.g., future menu/quit)
            break;
        } else if (r == PLAY_RESULT_PREV) {
            // Previous track (linear or shuffled)
            if (g_shuffle_enabled && g_shuffle_order && g_track_count > 0) {
                g_shuffle_pos--;
                if (g_shuffle_pos < 0)
                    g_shuffle_pos = g_track_count - 1;
                current_index = g_shuffle_order[g_shuffle_pos];
            } else {
                current_index--;
                if (current_index < 0)
                    current_index = g_track_count - 1;
            }

        } else if (r == PLAY_RESULT_NEXT) {
            // Next track (playlist shuffle or linear)
            if (g_shuffle_enabled && g_shuffle_order && g_track_count > 0) {
                g_shuffle_pos++;
                if (g_shuffle_pos >= g_track_count)
                    g_shuffle_pos = 0;
                current_index = g_shuffle_order[g_shuffle_pos];
            } else {
                current_index++;
                if (current_index >= g_track_count)
                    current_index = 0;
            }
        } else if (r == PLAY_RESULT_SELECTED) {
            current_index = g_selected_file;
        }
        update_mp3_bottom_bar_left(mp3_hint_left_obj, g_playlist[current_index]);
    }

    f_unmount(pSD->pcName);
    // If you ever want to free playlist:
    // if (g_playlist) { free(g_playlist); g_playlist = NULL; g_playlist_cap = 0; }
    // if (g_shuffle_order) { free(g_shuffle_order); g_shuffle_order = NULL; g_shuffle_pos = 0; }
}
