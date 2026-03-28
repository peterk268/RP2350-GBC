// ===============================================================
// Pico Pal MP3 Streaming Player (PLAYLIST + TRIPLE PCM BUFFER)
// - Scans "/music" for .mp3; if none, falls back to root
// - PCM_FRAME_COUNT = 6144, 16 KB ring buffer
// - Triple PCM buffers (buf_play, buf_ready, buf_fill)
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
#include "dr_wav.h"
#include "dr_flac.h"
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

#define MP3_STREAM_BUF_SIZE       (16 * 1024)   // 16 KB ring buffer
#define MP3_REFILL_CHUNK          (4 * 1024)    // max bytes per SD read

// FLAC dynamic seek cache (built while playing for files without an embedded seektable)
#define FLAC_SEEK_CACHE_ENTRIES    512           // 512 * 10s = ~85 min coverage at 44100 Hz
#define FLAC_SEEK_INTERVAL_FRAMES  441000ULL     // one seekpoint every ~10 seconds
#define FLAC_READ_AHEAD_BYTES      4096          // dr_flac internal buffer size; subtract from f_tell

// Directory used for music; fallback is root ("")
#define MUSIC_DIR                 "music"

// Adaptive seek curve (seconds)
#define SEEK_STEP_1S          1
#define SEEK_STEP_2S          3
#define SEEK_STEP_3S          6
#define SEEK_STEP_5S         10
#define SEEK_STEP_10S        30

// Hold thresholds
#define SEEK_STAGE_1_MS     1500
#define SEEK_STAGE_2_MS     3000
#define SEEK_STAGE_3_MS     5000
#define SEEK_STAGE_4_MS     10000

// Base repeat interval
#define SEEK_REPEAT_INTERVAL_MS  200
#define SEEK_HOLD_TIME_MS        350

// How long you need to hold before paging instead of 1-by-1
#define NAV_PAGE_HOLD_TIME_MS       200   // tweak to taste
#define NAV_PAGE_REPEAT_INTERVAL_MS 2000   // how often to page while held

// Set to 1 to skip +/-4 fast-paging after hold; always step by 1
#define NAV_FAST_PAGE_ENABLED       0

// Maximum playlist size and path length
#define MP3_MAX_TRACKS            4096
#define MP3_MAX_PATH_LEN          256

#define MP3_INACTIVE_TIMEOUT_US    (9000000ULL)   // 9 seconds
#define MP3_NOW_PLAYING_TIMEOUT_US (7000000ULL)   // 2 seconds on now playing

#define MP3_PROGRESS_BAR_RANGE     (DISP_HOR_RES - 26)  // 1 step per pixel — matches the bar's actual pixel width

#define BLUE_COLA
// #define GREEN
// #define GOLD
#ifdef BLUE_COLA
// Blue Cola
#define ACCENT_COLOR  0x0094E1
#define ACCENT_COLOR2 0x0076B4
#elifdef GREEN
// Green 
#define ACCENT_COLOR  0x33CC66
#define ACCENT_COLOR2 0x00A000
#elifdef GOLD
// Gold
#define ACCENT_COLOR  0xFAB71F
#define ACCENT_COLOR2 0xC89219
#endif

// === MP3 UI Global Objects ===
static lv_obj_t *mp3_list_obj         = NULL;
static lv_obj_t *mp3_hint_left_obj    = NULL;
static lv_obj_t *mp3_hint_right_obj   = NULL;
static lv_obj_t *mp3_status_label_obj = NULL;
static lv_obj_t *mp3_top_bar          = NULL;
static lv_obj_t *mp3_bottom_bar       = NULL;

// === Now-Playing progress widgets (updated each playback loop) ===
static lv_obj_t *g_now_playing_time_label  = NULL;
static lv_obj_t *g_now_playing_bar         = NULL;
static uint32_t  g_total_duration_ms       = 0;
static uint32_t  g_mp3_audio_start         = 0;  // byte offset of first audio frame (past ID3)
// Seek-correction for played_frames-based current time tracking.
// After a seek: base = played_frames at seek time, offset = new logical ms.
static uint64_t  g_time_display_base_frames = 0;
static int64_t   g_time_display_offset_ms   = 0;
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
static bool          show_now_playing    = true;
// This flag means "shuffle state changed while a track was playing;
// rebuild the shuffle order once the track returns to the playlist layer".
static bool          g_shuffle_needs_rebuild = false;
static int current_index = 0;

static uint32_t g_byte_offset = 0;

static bool g_buttons_locked   = false;   // START+SELECT toggle
static bool select_was_combo   = false;   // tracks if SELECT was used in a combo

// uint8_t saved_lcd_brightness = 0;

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
    uint32_t byte_offset;     // resume point in file
} mp3_resume_t;

#define RESUME_MAGIC 0x504D3352   // "PM3R"
#define RESUME_FILE  "mp3_resume.bin"

static mp3_resume_t g_resume = {0};

static void mp3_save_resume(int track_index, uint32_t position_ms, bool hold_sd_busy, uint32_t byte_offset) {
    shutdown_lcd(false, false);

    FIL wf;
    UINT bw;

    g_resume.magic       = RESUME_MAGIC;
    g_resume.track_index = track_index;
    g_resume.position_ms = position_ms;
    g_resume.shuffle     = g_shuffle_enabled;
    g_resume.repeat_mode = (uint8_t)g_repeat_mode;
    g_resume.byte_offset = byte_offset;

    if (f_open(&wf, RESUME_FILE, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
        f_write(&wf, &g_resume, sizeof(g_resume), &bw);
        f_close(&wf);
        printf("Resume saved: track=%d pos=%ums\n",
               track_index, position_ms);
    }

    if (!hold_sd_busy)
        start_lcd(false, false);
}

typedef struct {
    uint32_t magic;    // "SHU2"
    int32_t  count;    // number of tracks
    int32_t  pos;      // last g_shuffle_pos (optional)
    uint32_t seed;     // PRNG seed for this permutation
} mp3_shuffle_hdr_t;

// New magic so old files are ignored and we rebuild cleanly
#define SHUFFLE_MAGIC 0x53485532   // 'SHU2'
#define SHUFFLE_FILE  "mp3_shuffle.bin"

void toggle_speakers_if_paused() {
    audio_pause_active = paused;

    if (audio_pause_active) {
        mute_dac();
    } else if (!is_muted) {
        unmute_dac();
    }
    if (!headphones_present) audio_pause_active ? spk_off() : spk_on();
}

void draw_track_list(lv_obj_t *list,
                     char filenames[][256],
                     uint16_t num_file,
                     uint16_t selected,
                     uint16_t page_start)
{
    lv_obj_clean(list);

    if (show_settings) {
        draw_settings(list);
        return;
    }

    if (num_file == 0) {
        return;
    }

    // Start from the selected track (your "page start")
    uint16_t start = selected % num_file;

    // Base number of slots we want on screen
    uint16_t base_slots = (num_file < VISIBLE_ITEMS)
                            ? num_file
                            : VISIBLE_ITEMS;

    // Does this window cross the end of the list?
    bool will_wrap = (start + base_slots) > num_file;

    // If we wrap, reserve one slot for the separator
    uint16_t to_show = base_slots;
    if (will_wrap && to_show > 0) {
        to_show -= 1;
    }

    bool inserted_separator = false;

    for (uint16_t i = 0; i < to_show; i++) {

        uint16_t raw = start + i;

        // If we wrap in the middle of the visible window, drop the separator HERE
        if (will_wrap && !inserted_separator && raw >= num_file) {
            lv_obj_t *sep = lv_list_add_text(list, "-------------------------------------");
            lv_obj_set_style_text_font(sep, LV_FONT_DEFAULT, 0);
            lv_obj_set_style_bg_color(sep, lv_color_hex(0xFFFFFF), 0);
            lv_obj_set_style_text_color(sep, lv_color_black(), 0);
            lv_obj_set_width(sep, DISP_HOR_RES - 30);
            lv_label_set_long_mode(sep, LV_LABEL_LONG_CLIP);

            inserted_separator = true;
        }

        uint16_t idx = raw % num_file;

        // Actual item
        lv_obj_t *item = lv_list_add_text(
            list,
            basename_from_path(filenames[idx])
        );

        lv_obj_set_style_text_font(item, LV_FONT_DEFAULT, 0);
        lv_obj_set_style_text_color(item, lv_color_black(), 0);
        lv_obj_set_style_bg_color(item, lv_color_hex(0xFFFFFF), 0);

        if (idx == selected) {
            lv_obj_set_style_bg_color(item, lv_color_hex(ACCENT_COLOR), LV_PART_MAIN);
            lv_obj_set_style_text_color(item, lv_color_hex(0xFFFFFF), 0);

            lv_label_set_long_mode(item, LV_LABEL_LONG_SCROLL_CIRCULAR);
            lv_obj_set_style_anim_speed(item, 20, 0);
        } else {
            lv_obj_set_width(item, DISP_HOR_RES - 30);
            lv_label_set_long_mode(item, LV_LABEL_LONG_CLIP);
        }
    }

    // Edge case: we *know* the window wraps, but the wrap point
    // landed just after the last visible item in the loop.
    // Use the reserved slot now and put the separator at the bottom.
    if (will_wrap && !inserted_separator) {
        lv_obj_t *sep = lv_list_add_text(list, "-------------------------------------");
        lv_obj_set_style_text_font(sep, LV_FONT_DEFAULT, 0);
        lv_obj_set_style_bg_color(sep, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_text_color(sep, lv_color_black(), 0);
        lv_obj_set_width(sep, DISP_HOR_RES - 30);
        lv_label_set_long_mode(sep, LV_LABEL_LONG_CLIP);
    }
}

typedef struct {
    char title[96];
    char artist[96];
    char album[96];
} mp3_tags_t;

mp3_tags_t *g_mp3_tags = NULL;
static bool mp3_tags_ensure_alloc(void) {
    if (g_mp3_tags) return true;
    g_mp3_tags = (mp3_tags_t *)calloc(1, sizeof(mp3_tags_t));
    return (g_mp3_tags != NULL);
}

typedef struct {
    FIL file;
    uint8_t *buf;                // was: uint8_t buf[MP3_STREAM_BUF_SIZE];
    uint32_t rd;
    uint32_t wr;
    uint32_t count;
    bool eof;

    mp3_tags_t tags;
} mp3_stream_t;

static uint32_t id3_syncsafe_u32(const uint8_t b[4]) {
    return ((uint32_t)(b[0] & 0x7F) << 21) |
           ((uint32_t)(b[1] & 0x7F) << 14) |
           ((uint32_t)(b[2] & 0x7F) << 7)  |
           ((uint32_t)(b[3] & 0x7F) << 0);
}

static uint32_t be32_u32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | ((uint32_t)p[3] << 0);
}

static void trim_trailing_spaces(char *s) {
    size_t n = strlen(s);
    while (n > 0 && (s[n-1] == ' ' || s[n-1] == '\0')) {
        s[n-1] = '\0';
        n--;
    }
}

static void copy_text_frame(char *dst, size_t dstsz, const uint8_t *payload, size_t payload_sz) {
    if (!dst || dstsz == 0) return;
    dst[0] = '\0';
    if (!payload || payload_sz < 1) return;

    uint8_t enc = payload[0];
    const uint8_t *txt = payload + 1;
    size_t txt_sz = payload_sz - 1;

    // enc: 0=ISO-8859-1, 1=UTF-16 w/BOM, 2=UTF-16BE, 3=UTF-8
    if (enc == 0 || enc == 3) {
        // ISO-8859-1 or UTF-8: copy bytes, stop at first '\0'
        size_t n = 0;
        while (n + 1 < dstsz && n < txt_sz && txt[n] != '\0') {
            dst[n] = (char)txt[n];
            n++;
        }
        dst[n] = '\0';
        trim_trailing_spaces(dst);
        return;
    }

    // Best-effort UTF-16: if mostly ASCII, pull every other byte.
    // Handle BOM if present.
    bool le = true; // default
    if (txt_sz >= 2) {
        if (txt[0] == 0xFF && txt[1] == 0xFE) { le = true;  txt += 2; txt_sz -= 2; }
        else if (txt[0] == 0xFE && txt[1] == 0xFF) { le = false; txt += 2; txt_sz -= 2; }
    }

    size_t n = 0;
    for (size_t i = 0; i + 1 < txt_sz && n + 1 < dstsz; i += 2) {
        uint8_t lo = le ? txt[i] : txt[i+1];
        uint8_t hi = le ? txt[i+1] : txt[i];

        if (lo == 0 && hi == 0) break;

        // ASCII if hi==0
        if (hi == 0) dst[n++] = (char)lo;
        else dst[n++] = '?';
    }
    dst[n] = '\0';
    trim_trailing_spaces(dst);
}

static void parse_id3v1(const uint8_t *tag128, size_t sz, mp3_tags_t *out) {
    if (!tag128 || sz < 128 || !out) return;
    if (memcmp(tag128, "TAG", 3) != 0) return;

    // ID3v1: title[30], artist[30], album[30]
    char tmp[64];

    memset(tmp, 0, sizeof(tmp));
    memcpy(tmp, tag128 + 3, 30);
    tmp[30] = '\0';
    strncpy(out->title, tmp, sizeof(out->title)-1);

    memset(tmp, 0, sizeof(tmp));
    memcpy(tmp, tag128 + 33, 30);
    tmp[30] = '\0';
    strncpy(out->artist, tmp, sizeof(out->artist)-1);

    memset(tmp, 0, sizeof(tmp));
    memcpy(tmp, tag128 + 63, 30);
    tmp[30] = '\0';
    strncpy(out->album, tmp, sizeof(out->album)-1);

    trim_trailing_spaces(out->title);
    trim_trailing_spaces(out->artist);
    trim_trailing_spaces(out->album);
}

static void parse_id3v2(const uint8_t *raw, size_t raw_sz, mp3_tags_t *out) {
    if (!raw || raw_sz < 10 || !out) return;
    if (memcmp(raw, "ID3", 3) != 0) return;

    uint8_t ver = raw[3];          // 3 = ID3v2.3, 4 = ID3v2.4
    uint8_t flags = raw[5];
    uint32_t tag_size = id3_syncsafe_u32(raw + 6); // size excludes 10-byte header
    size_t total = 10 + (size_t)tag_size;
    if (total > raw_sz) total = raw_sz;

    size_t off = 10;

    // Skip extended header if present (best-effort; differs between v2.3 and v2.4)
    if (flags & 0x40) {
        if (off + 4 <= total) {
            uint32_t extsz = (ver == 4) ? id3_syncsafe_u32(raw + off) : be32_u32(raw + off);
            off += 4;
            if (extsz <= (total - off)) off += extsz;
            if (off > total) return;
        }
    }

    while (off + 10 <= total) {
        const uint8_t *fh = raw + off;

        // Padding/end
        if (fh[0] == 0 && fh[1] == 0 && fh[2] == 0 && fh[3] == 0) break;

        char id[5] = { (char)fh[0], (char)fh[1], (char)fh[2], (char)fh[3], 0 };
        uint32_t fsz = (ver == 4) ? id3_syncsafe_u32(fh + 4) : be32_u32(fh + 4);
        // uint16_t fflags = ((uint16_t)fh[8] << 8) | fh[9];

        off += 10;
        if (fsz == 0) continue;
        if (off + fsz > total) break;

        const uint8_t *payload = raw + off;

        if (strcmp(id, "TIT2") == 0) {
            if (out->title[0] == '\0') copy_text_frame(out->title, sizeof(out->title), payload, fsz);
        } else if (strcmp(id, "TPE1") == 0) {
            if (out->artist[0] == '\0') copy_text_frame(out->artist, sizeof(out->artist), payload, fsz);
        } else if (strcmp(id, "TALB") == 0) {
            if (out->album[0] == '\0') copy_text_frame(out->album, sizeof(out->album), payload, fsz);
        }

        off += fsz;

        // Early exit if we got everything
        if (out->title[0] && out->artist[0] && out->album[0]) break;
    }
}

/* dr_mp3 metadata callback.
   NOTE: for drmp3_init(), pUserData passed to onMeta is the same pUserData you pass to init,
   so we keep tags inside mp3_stream_t. */
static void drmp3_on_meta(void *pUserData, const drmp3_metadata *m) {
    mp3_stream_t *s = (mp3_stream_t *)pUserData;
    if (!s || !m || !m->pRawData || m->rawDataSize == 0) return;

    if (m->type == DRMP3_METADATA_TYPE_ID3V2) {
        parse_id3v2((const uint8_t *)m->pRawData, m->rawDataSize, &s->tags);
    } else if (m->type == DRMP3_METADATA_TYPE_ID3V1) {
        parse_id3v1((const uint8_t *)m->pRawData, m->rawDataSize, &s->tags);
    }
}

// ===================================================================
// SD ring buffer logic (HEAP-BASED, in PSRAM via malloc)
// ===================================================================

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
// WAV direct FatFS I/O callbacks (no ring buffer needed for WAV)
// ===================================================================
static size_t wav_fatfs_read(void *pUserData, void *pBufferOut, size_t bytesToRead) {
    FIL *f = (FIL *)pUserData;
    UINT br = 0;
    f_read(f, pBufferOut, (UINT)bytesToRead, &br);
    return (size_t)br;
}

static drwav_bool32 wav_fatfs_seek(void *pUserData, int offset, drwav_seek_origin origin) {
    FIL *f = (FIL *)pUserData;
    FSIZE_t new_pos;
    if (origin == DRWAV_SEEK_SET) {
        new_pos = (FSIZE_t)offset;
    } else if (origin == DRWAV_SEEK_END) {
        new_pos = f_size(f) + (FSIZE_t)offset;  // offset is typically 0 when dr_wav measures file size
    } else {
        new_pos = f_tell(f) + (FSIZE_t)offset;  // SEEK_CUR; unsigned wrap handles negative offsets
    }
    return (f_lseek(f, new_pos) == FR_OK) ? DRWAV_TRUE : DRWAV_FALSE;
}

static drwav_bool32 wav_fatfs_tell(void *pUserData, drwav_int64 *pCursor) {
    FIL *f = (FIL *)pUserData;
    *pCursor = (drwav_int64)f_tell(f);
    return DRWAV_TRUE;
}

static void wav_estimate_track_duration(drwav *wav) {
    g_total_duration_ms = 0;
    if (!wav || wav->sampleRate == 0 || wav->totalPCMFrameCount == 0) return;
    g_total_duration_ms = (uint32_t)((uint64_t)wav->totalPCMFrameCount * 1000 / wav->sampleRate);
    printf("WAV duration: %u ms (%llu frames @ %u Hz)\n",
           g_total_duration_ms, (unsigned long long)wav->totalPCMFrameCount, wav->sampleRate);
}

// ===================================================================
// FLAC direct FatFS I/O callbacks
// ===================================================================
static size_t flac_fatfs_read(void *pUserData, void *pBufferOut, size_t bytesToRead) {
    FIL *f = (FIL *)pUserData;
    UINT br = 0;
    f_read(f, pBufferOut, (UINT)bytesToRead, &br);
    return (size_t)br;
}

static drflac_bool32 flac_fatfs_seek(void *pUserData, int offset, drflac_seek_origin origin) {
    FIL *f = (FIL *)pUserData;
    FSIZE_t new_pos = (origin == DRFLAC_SEEK_SET) ? (FSIZE_t)offset : f_tell(f) + (FSIZE_t)offset;
    return (f_lseek(f, new_pos) == FR_OK) ? DRFLAC_TRUE : DRFLAC_FALSE;
}

static drflac_bool32 flac_fatfs_tell(void *pUserData, drflac_int64 *pCursor) {
    FIL *f = (FIL *)pUserData;
    *pCursor = (drflac_int64)f_tell(f);
    return DRFLAC_TRUE;
}

// Copy at most (dst_max-1) chars from a non-null-terminated vorbis comment value.
// Finds "KEY=value" format; key comparison is case-insensitive.
static void flac_vorbis_copy_tag(const char *comment, drflac_uint32 len,
                                 const char *key, char *dst, size_t dst_max) {
    size_t klen = strlen(key);
    if (len <= klen || comment[klen] != '=') return;
    // Case-insensitive key match
    for (size_t i = 0; i < klen; i++) {
        if ((comment[i] | 0x20) != (key[i] | 0x20)) return;
    }
    const char *val = comment + klen + 1;
    drflac_uint32 vlen = len - (drflac_uint32)(klen + 1);
    if (vlen >= (drflac_uint32)dst_max) vlen = (drflac_uint32)dst_max - 1;
    memcpy(dst, val, vlen);
    dst[vlen] = '\0';
}

static void flac_meta_callback(void *pUserData, drflac_metadata *pMetadata) {
    (void)pUserData;
    if (pMetadata->type != DRFLAC_METADATA_BLOCK_TYPE_VORBIS_COMMENT) return;
    if (!g_mp3_tags) return;

    drflac_vorbis_comment_iterator it;
    drflac_init_vorbis_comment_iterator(&it,
        pMetadata->data.vorbis_comment.commentCount,
        pMetadata->data.vorbis_comment.pComments);

    drflac_uint32 clen;
    const char *comment;
    while ((comment = drflac_next_vorbis_comment(&it, &clen)) != NULL) {
        flac_vorbis_copy_tag(comment, clen, "title",  g_mp3_tags->title,  sizeof(g_mp3_tags->title));
        flac_vorbis_copy_tag(comment, clen, "artist", g_mp3_tags->artist, sizeof(g_mp3_tags->artist));
        flac_vorbis_copy_tag(comment, clen, "album",  g_mp3_tags->album,  sizeof(g_mp3_tags->album));
    }
}

static void flac_estimate_track_duration(drflac *flac) {
    g_total_duration_ms = 0;
    if (!flac || flac->sampleRate == 0 || flac->totalPCMFrameCount == 0) return;
    g_total_duration_ms = (uint32_t)((uint64_t)flac->totalPCMFrameCount * 1000 / flac->sampleRate);
    printf("FLAC duration: %u ms (%llu frames @ %u Hz)\n",
           g_total_duration_ms, (unsigned long long)flac->totalPCMFrameCount, flac->sampleRate);
}

static inline void mp3_zero_pcm_tail(int16_t *pcm,
                                     drmp3_uint64 frames_decoded,
                                     uint32_t frame_capacity,
                                     uint8_t channels) {
    if (!pcm || frames_decoded >= frame_capacity) return;

    size_t start = (size_t)frames_decoded * channels;
    size_t total = (size_t)frame_capacity * channels;
    memset(pcm + start, 0, (total - start) * sizeof(int16_t));
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

// Shuffle state
static uint32_t g_shuffle_seed = 0;

// Simple local PRNG so we don't depend on global rand()
static uint32_t shuffle_prng_next(uint32_t *state) {
    // LCG: X_{n+1} = aX_n + c  (Numerical Recipes constants)
    *state = (*state * 1664525u) + 1013904223u;
    return *state;
}

static uint32_t shuffle_generate_seed(void) {
    uint32_t seed = (uint32_t)time_us_64();

    struct tm now_tm;
    if (mcp7940n_get_tm(RTC_I2C_PORT, &now_tm)) {
        time_t epoch = mktime(&now_tm);
        if (epoch != (time_t)-1) {
            seed ^= (uint32_t)epoch;
        }
    }

    if (seed == 0) seed = 1;  // avoid zero state
    return seed;
}

static void build_shuffle_order_from_seed(uint32_t seed) {
    if (g_track_count <= 0)
        return;

    if (!g_shuffle_order) {
        g_shuffle_order = (int *)malloc(sizeof(int) * g_track_count);
        if (!g_shuffle_order) {
            printf("shuffle malloc fail\n");
            return;
        }
    }

    // Start with 0..N-1
    for (int i = 0; i < g_track_count; i++) {
        g_shuffle_order[i] = i;
    }

    uint32_t state = (seed == 0) ? 1u : seed;

    // Fisher–Yates using our local PRNG
    for (int i = g_track_count - 1; i > 0; --i) {
        uint32_t r = shuffle_prng_next(&state);
        int j = (int)(r % (uint32_t)(i + 1));

        int tmp              = g_shuffle_order[i];
        g_shuffle_order[i]   = g_shuffle_order[j];
        g_shuffle_order[j]   = tmp;
    }
}

// Build shuffle order using Fisher–Yates, and ensure current_track is first
static void build_shuffle_order(int current_track_index) {
    if (g_track_count <= 0)
        return;

    // Fresh seed for a new shuffle permutation
    g_shuffle_seed = shuffle_generate_seed();

    // Build permutation from that seed
    build_shuffle_order_from_seed(g_shuffle_seed);

    // Default to start at index 0
    g_shuffle_pos = 0;

    // If current track is valid, move shuffle_pos to where that track landed
    if (current_track_index >= 0 && current_track_index < g_track_count) {
        for (int i = 0; i < g_track_count; i++) {
            if (g_shuffle_order[i] == current_track_index) {
                g_shuffle_pos = i;
                break;
            }
        }
    }
}

static void shuffle_save_state(bool hold_sd_busy) {
    if (!g_shuffle_order || g_track_count <= 0 || g_shuffle_seed == 0)
        return;

    shutdown_lcd(false, false);

    FIL f;
    UINT bw;

    mp3_shuffle_hdr_t hdr;
    hdr.magic = SHUFFLE_MAGIC;
    hdr.count = g_track_count;
    hdr.pos   = g_shuffle_pos;
    hdr.seed  = g_shuffle_seed;

    if (f_open(&f, SHUFFLE_FILE, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
        f_write(&f, &hdr, sizeof(hdr), &bw);
        f_close(&f);
    }

    if (!hold_sd_busy)
        start_lcd(false, false);
}

static bool shuffle_load_state(void) {
    FIL f;
    UINT br;
    mp3_shuffle_hdr_t hdr;

    if (f_open(&f, SHUFFLE_FILE, FA_READ) != FR_OK)
        return false;

    if (f_read(&f, &hdr, sizeof(hdr), &br) != FR_OK ||
        br != sizeof(hdr) ||
        hdr.magic != SHUFFLE_MAGIC) {

        f_close(&f);
        return false;
    }

    // Track count mismatch → discard old shuffle state
    if (hdr.count <= 0 || hdr.count != g_track_count) {
        f_close(&f);
        return false;
    }

    if (hdr.pos < 0 || hdr.pos >= hdr.count) {
        f_close(&f);
        return false;
    }

    if (hdr.seed == 0) {
        f_close(&f);
        return false;
    }

    f_close(&f);

    // Restore seed and rebuild permutation
    g_shuffle_seed = hdr.seed;
    build_shuffle_order_from_seed(g_shuffle_seed);

    // Restore last known pos (resume code will re-adjust for current_index anyway)
    g_shuffle_pos = hdr.pos;

    return true;
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

// Check if name ends with ".wav" (case-insensitive)
static bool has_wav_extension(const char *name) {
    size_t len = strlen(name);
    if (len < 4) return false;
    const char *ext = name + len - 4;
    return (tolower_ascii(ext[0]) == '.' &&
            tolower_ascii(ext[1]) == 'w' &&
            tolower_ascii(ext[2]) == 'a' &&
            tolower_ascii(ext[3]) == 'v');
}

// Check if name ends with ".flac" (case-insensitive)
static bool has_flac_extension(const char *name) {
    size_t len = strlen(name);
    if (len < 5) return false;
    const char *e = name + len - 5;
    return (e[0]=='.' &&
            (e[1]=='f'||e[1]=='F') && (e[2]=='l'||e[2]=='L') &&
            (e[3]=='a'||e[3]=='A') && (e[4]=='c'||e[4]=='C'));
}

static bool has_audio_extension(const char *name) {
    return has_mp3_extension(name) || has_wav_extension(name) || has_flac_extension(name);
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

        if (!has_audio_extension(name))
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

// Build playlist: scan MUSIC_DIR and root, combining all results
static void mp3_build_playlist(void) {
    if (!mp3_playlist_init()) {
        printf("Playlist init failed\n");
        g_track_count = 0;
        return;
    }

    g_track_count = 0;

    int from_music = mp3_scan_dir(MUSIC_DIR);
    int from_root  = mp3_scan_dir("");
    printf("Playlist: %d from '%s', %d from root, %d total\n",
           from_music, MUSIC_DIR, from_root, g_track_count);
}


// ===================================================================
// Persistent buffer for drmp3's internal pData — keeps it in SRAM.
// Without this, drmp3_init allocates 64KB via malloc which lands in PSRAM (5x slower).
// ===================================================================
static uint8_t *s_drmp3_pdata = NULL;
static size_t   s_drmp3_pdata_cap = 0;
#define DRMP3_PDATA_INIT_SIZE (DRMP3_DATA_CHUNK_SIZE)  // 64KB

static void *drmp3_sram_malloc(size_t sz, void *pUserData) {
    (void)pUserData;
    if (!s_drmp3_pdata) {
        size_t alloc_sz = (sz > DRMP3_PDATA_INIT_SIZE) ? sz : DRMP3_PDATA_INIT_SIZE;
        s_drmp3_pdata = (uint8_t *)malloc(alloc_sz);
        s_drmp3_pdata_cap = s_drmp3_pdata ? alloc_sz : 0;
    }
    if (sz <= s_drmp3_pdata_cap) return s_drmp3_pdata;
    uint8_t *p = (uint8_t *)realloc(s_drmp3_pdata, sz);
    if (p) { s_drmp3_pdata = p; s_drmp3_pdata_cap = sz; }
    return p;
}

static void *drmp3_sram_realloc(void *p, size_t sz, void *pUserData) {
    (void)pUserData;
    if (p != s_drmp3_pdata) return realloc(p, sz);
    if (sz <= s_drmp3_pdata_cap) return s_drmp3_pdata;
    uint8_t *np = (uint8_t *)realloc(s_drmp3_pdata, sz);
    if (np) { s_drmp3_pdata = np; s_drmp3_pdata_cap = sz; }
    return np;
}

static void drmp3_sram_free(void *p, void *pUserData) {
    (void)pUserData; (void)p;
}

static const drmp3_allocation_callbacks s_drmp3_alloc = {
    .pUserData = NULL,
    .onMalloc  = drmp3_sram_malloc,
    .onRealloc = drmp3_sram_realloc,
    .onFree    = drmp3_sram_free,
};

// ===================================================================
// Seek helper: approximate seek by jumping compressed bytes
// ===================================================================
static bool seek_relative_seconds(
    mp3_stream_t *stream,
    drmp3 *mp3,
    int seconds,
    int sample_rate,
    bool *decoded_next_chunk,
    uint32_t *seeked_byte_pos,   // out: byte position seeked to (before drmp3_init)
    uint64_t played_frames_ref   // current played_frames for time calculation
){
    if (!stream) return false;

    DWORD fsize = f_size(&stream->file);

    // Compute target byte from known current display time + seek delta.
    // Using f_tell as a base is wrong because drmp3's internal buffer keeps
    // f_tell far ahead of actual playback; g_time_display_* tracks real position.
    int64_t new_pos;
    if (g_total_duration_ms > 0 && fsize > 0) {
        int64_t cur_ms = (int64_t)((played_frames_ref - g_time_display_base_frames) * 1000 / sample_rate)
                         + g_time_display_offset_ms;
        int64_t target_ms = cur_ms + (int64_t)seconds * 1000;
        if (target_ms < 0) target_ms = 0;
        if (target_ms > (int64_t)g_total_duration_ms) target_ms = (int64_t)g_total_duration_ms;
        // Map target_ms into the audio region only (skip the ID3 header)
        int64_t audio_start = (int64_t)g_mp3_audio_start;
        int64_t audio_bytes = (int64_t)fsize - audio_start;
        if (audio_bytes < 0) audio_bytes = 0;
        new_pos = audio_start + (audio_bytes > 0
                  ? audio_bytes * target_ms / (int64_t)g_total_duration_ms
                  : 0);
    } else {
        // Fallback: use f_tell when duration unknown; clamp to audio start
        new_pos = (int64_t)f_tell(&stream->file) + (int64_t)16000 * seconds;
        if (new_pos < (int64_t)g_mp3_audio_start) new_pos = (int64_t)g_mp3_audio_start;
    }
    if (new_pos < 0) new_pos = 0;
    if (new_pos > (int64_t)fsize) new_pos = (int64_t)fsize;

    f_lseek(&stream->file, (DWORD)new_pos);
    if (seeked_byte_pos) *seeked_byte_pos = (uint32_t)new_pos;

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
                    drmp3_on_meta,
                    stream,
                    &s_drmp3_alloc)) {
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

static inline void mp3_select_relative(int delta,
                                       lv_obj_t *mp3_list_obj,
                                       char (*g_playlist)[MP3_MAX_PATH_LEN],
                                       int g_track_count)
{
    if (g_track_count <= 0) return;

    int new_index = g_selected_file + delta;

    // Wrap around nicely
    if (new_index < 0) {
        new_index = (new_index % g_track_count + g_track_count) % g_track_count;
    } else if (new_index >= g_track_count) {
        new_index = new_index % g_track_count;
    }

    g_selected_file = new_index;

    draw_track_list(
        mp3_list_obj,
        g_playlist,
        g_track_count,
        g_selected_file,
        g_selected_file
    );
}

#define MP3_RESUME_SEEKBACK_BYTES (56u * 1024u)

static uint32_t mp3_get_resume_offset(const mp3_stream_t *s) {
    uint32_t tell = (uint32_t)f_tell((FIL *)&s->file);
    uint32_t off  = (tell >= s->count) ? (tell - s->count) : 0;
    return off;
}
// Seek past ID3v2 tag so drmp3_init starts at audio frames immediately.
// Also parses text metadata into stream->tags using stream->buf as scratch
// (ring buffer must be empty when called). Returns audio start byte offset.
static uint32_t mp3_skip_id3v2(FIL *f, mp3_stream_t *stream) {
    uint8_t hdr[10];
    UINT br;
    if (f_lseek(f, 0) != FR_OK) return 0;
    if (f_read(f, hdr, 10, &br) != FR_OK || br < 10) { f_lseek(f, 0); return 0; }
    if (hdr[0] != 'I' || hdr[1] != 'D' || hdr[2] != '3') { f_lseek(f, 0); return 0; }

    uint32_t tag_sz = ((uint32_t)(hdr[6] & 0x7F) << 21)
                    | ((uint32_t)(hdr[7] & 0x7F) << 14)
                    | ((uint32_t)(hdr[8] & 0x7F) <<  7)
                    |  (uint32_t)(hdr[9] & 0x7F);
    uint32_t skip_to = 10 + tag_sz;
    if (hdr[5] & 0x10) skip_to += 10; // ID3v2 footer

    // Parse text frames (title/artist/album) from the first chunk of the tag.
    // Use stream->buf as scratch — ring buffer is empty here, no extra malloc needed.
    // Text frames are always near the front; artwork (APIC) is typically last,
    // so MP3_STREAM_BUF_SIZE bytes captures all the text we need.
    if (stream && stream->buf && tag_sz > 0) {
        uint32_t read_sz = (10 + tag_sz <= MP3_STREAM_BUF_SIZE)
                         ? (10 + tag_sz) : MP3_STREAM_BUF_SIZE;
        UINT tag_br = 0;
        if (f_lseek(f, 0) == FR_OK)
            f_read(f, stream->buf, read_sz, &tag_br);
        memset(&stream->tags, 0, sizeof(stream->tags));
        if (tag_br >= 10)
            parse_id3v2(stream->buf, tag_br, &stream->tags);
        // Ring buffer is now dirty with tag data — reset it
        stream->rd = stream->wr = stream->count = 0;
        stream->eof = false;
    }

    f_lseek(f, skip_to);
    printf("ID3v2 skip: %lu bytes, title='%s'\n", (unsigned long)skip_to,
           (stream && stream->tags.title[0]) ? stream->tags.title : "(none)");
    return skip_to;
}

bool mp3_resume_open(mp3_stream_t *s, uint32_t byte_offset) {
    if (!s) return false;

    uint32_t start = (byte_offset > MP3_RESUME_SEEKBACK_BYTES)
                   ? (byte_offset - MP3_RESUME_SEEKBACK_BYTES)
                   : 0;

    if (f_lseek(&s->file, start) != FR_OK) return false;

    s->rd = s->wr = 0;
    s->count = 0;
    s->eof = false;
    return true;
}

void mp3_save_shutdown(int current_track_index, uint64_t played_frames, drmp3_uint32 sampleRate, const mp3_stream_t *s) {
    uint32_t final_position_ms =
    (played_frames * 1000ULL) / sampleRate;

    uint8_t temp_lcd_led = g_mp3_inactive ? sd_prev_lcd_led_duty_cycle : lcd_led_duty_cycle;
    uint8_t temp_button_led = g_mp3_inactive ? saved_button_brightness : button_led_duty_cycle;

    // Save resume info
    mp3_save_resume(current_track_index, final_position_ms, true, mp3_get_resume_offset(s));

    // Save shuffle order
    shuffle_save_state(true);

#if TIE_PWR_LED_TO_LCD
    pwr_led_duty_cycle = temp_lcd_led;
#endif
    save_system_settings_if_changed(temp_lcd_led, temp_button_led, low_power ? prev_pwr_led_duty_cycle : pwr_led_duty_cycle, manual_palette_selected, wash_out_level, last_filename_raw, auto_load_state, crt_mode, true);
}
// ===================================================================
// Now-playing progress helpers (must precede mp3_play_single_track)
// ===================================================================
static void format_time_ms(char *buf, size_t bufsz, uint32_t ms) {
    uint32_t s = ms / 1000;
    uint32_t m = s / 60;
    s %= 60;
    snprintf(buf, bufsz, "%u:%02u", m, s);
}

static void mp3_update_progress(uint32_t current_ms, uint32_t total_ms) {
    if (!show_now_playing) return;
    if (g_now_playing_time_label) {
        char cur[8], tot[8], combined[18];
        format_time_ms(cur, sizeof(cur), current_ms);
        format_time_ms(tot, sizeof(tot), total_ms);
        snprintf(combined, sizeof(combined), "%s / %s", cur, tot);
        lv_label_set_text(g_now_playing_time_label, combined);
    }
    if (g_now_playing_bar && total_ms > 0) {
        int32_t pct = (int32_t)((uint64_t)current_ms * MP3_PROGRESS_BAR_RANGE / total_ms);
        if (pct > MP3_PROGRESS_BAR_RANGE) pct = MP3_PROGRESS_BAR_RANGE;
        lv_bar_set_value(g_now_playing_bar, pct, LV_ANIM_OFF);
    }
}

// ===================================================================
// Track duration estimator (Xing/VBRI/CBR fallback)
// Sets g_total_duration_ms. Call after drmp3_init.
// ===================================================================
static void mp3_estimate_track_duration(mp3_stream_t *stream, drmp3 *mp3, DWORD file_sz) {
    g_total_duration_ms = 0;
    if (!stream || !mp3 || file_sz == 0 || mp3->sampleRate == 0) return;

    static const uint16_t br_mpeg1d[16] = {0,32,40,48,56,64,80,96,112,128,160,192,224,256,320,0};
    static const uint16_t br_mpeg2d[16] = {0, 8,16,24,32,40,48,56, 64, 80, 96,112,128,144,160,0};

    // audio_start: skip ID3v2 header if present
    uint32_t audio_start = 0;
    {
        uint8_t id3h[10];
        UINT id3_br = 0;
        FSIZE_t sp = f_tell(&stream->file);
        if (f_lseek(&stream->file, 0) == FR_OK) {
            f_read(&stream->file, id3h, sizeof(id3h), &id3_br);
            f_lseek(&stream->file, sp);
        }
        if (id3_br >= 10 && id3h[0]=='I' && id3h[1]=='D' && id3h[2]=='3') {
            uint32_t id3_size = ((uint32_t)(id3h[6] & 0x7F) << 21)
                              | ((uint32_t)(id3h[7] & 0x7F) << 14)
                              | ((uint32_t)(id3h[8] & 0x7F) <<  7)
                              |  (uint32_t)(id3h[9] & 0x7F);
            audio_start = 10 + id3_size;
            if (id3h[5] & 0x10) audio_start += 10;
        }
    }
    g_mp3_audio_start = audio_start;

    uint8_t hdr[512];
    UINT hdr_br = 0;
    {
        FSIZE_t sp = f_tell(&stream->file);
        if (f_lseek(&stream->file, audio_start) == FR_OK) {
            f_read(&stream->file, hdr, sizeof(hdr), &hdr_br);
            f_lseek(&stream->file, sp);
        }
    }

    for (uint32_t i = 0; i + 3 < hdr_br; i++) {
        if (hdr[i] != 0xFF || (hdr[i+1] & 0xE0) != 0xE0) continue;
        int mpeg_ver = (hdr[i+1] >> 3) & 3;
        int layer    = (hdr[i+1] >> 1) & 3;
        int br_idx   = (hdr[i+2] >> 4) & 0xF;
        int ch_mode  = (hdr[i+3] >> 6) & 3;
        if (layer != 1 || br_idx == 0 || br_idx == 15) continue;
        int br_kbps = (mpeg_ver == 3) ? br_mpeg1d[br_idx] : br_mpeg2d[br_idx];
        if (br_kbps <= 0) continue;

        uint32_t side_sz = (mpeg_ver == 3) ? ((ch_mode == 3) ? 17u : 32u)
                                           : ((ch_mode == 3) ?  9u : 17u);
        uint32_t xoff = i + 4 + side_sz;

        if (xoff + 11 < hdr_br) {
            bool is_xing = (hdr[xoff]=='X' && hdr[xoff+1]=='i' && hdr[xoff+2]=='n' && hdr[xoff+3]=='g') ||
                           (hdr[xoff]=='I' && hdr[xoff+1]=='n' && hdr[xoff+2]=='f' && hdr[xoff+3]=='o');
            bool is_vbri = (hdr[xoff]=='V' && hdr[xoff+1]=='B' && hdr[xoff+2]=='R' && hdr[xoff+3]=='I');
            if (is_xing && (hdr[xoff+7] & 0x01)) {
                uint32_t tf = ((uint32_t)hdr[xoff+8]  << 24) | ((uint32_t)hdr[xoff+9]  << 16)
                            | ((uint32_t)hdr[xoff+10] <<  8) |  hdr[xoff+11];
                uint32_t spf = (mpeg_ver == 3) ? 1152u : 576u;
                if (tf > 0) {
                    g_total_duration_ms = (uint32_t)((uint64_t)tf * spf * 1000u / mp3->sampleRate);
                    printf("MP3 duration (Xing %u frames): %u ms\n", tf, g_total_duration_ms);
                }
            } else if (is_vbri && xoff + 17 < hdr_br) {
                uint32_t tf = ((uint32_t)hdr[xoff+14] << 24) | ((uint32_t)hdr[xoff+15] << 16)
                            | ((uint32_t)hdr[xoff+16] <<  8) |  hdr[xoff+17];
                uint32_t spf = (mpeg_ver == 3) ? 1152u : 576u;
                if (tf > 0) {
                    g_total_duration_ms = (uint32_t)((uint64_t)tf * spf * 1000u / mp3->sampleRate);
                    printf("MP3 duration (VBRI %u frames): %u ms\n", tf, g_total_duration_ms);
                }
            }
        }

        if (g_total_duration_ms == 0) {
            uint32_t audio_bytes = (file_sz > audio_start) ? (file_sz - audio_start) : file_sz;
            g_total_duration_ms = (uint32_t)((uint64_t)audio_bytes * 8u / (uint32_t)br_kbps);
            printf("MP3 duration (CBR %d kbps): %u ms\n", br_kbps, g_total_duration_ms);
        }
        break;
    }
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

    // Persistent heap allocations — allocated once on the first MP3 track, reused forever.
    // Avoids heap fragmentation from free+realloc between tracks, which can push these
    // time-critical buffers into PSRAM (5x slower) on subsequent tracks.
    static drmp3    *s_mp3_state    = NULL;
    static uint8_t  *s_mp3_ring_buf = NULL;
    // PCM decode buffers — persistent to prevent PSRAM spill on subsequent tracks.
    // Always allocate max size (stereo) so mono tracks can reuse without realloc.
    static int16_t  *s_pcmA = NULL;
    static int16_t  *s_pcmB = NULL;
    static int16_t  *s_pcmC = NULL;
    static const uint32_t PCM_BUF_MAX = PCM_FRAME_COUNT * 2; // stereo max


    uint64_t played_frames = 0;
    g_total_duration_ms = 0;
    g_mp3_audio_start   = 0;
    g_time_display_base_frames = 0;
    g_time_display_offset_ms   = 0;

    FRESULT fr;
    play_result_t result = PLAY_RESULT_STOP;
    bool is_wav  = has_wav_extension(filepath);
    bool is_flac = has_flac_extension(filepath);

    mp3_stream_t *stream = (mp3_stream_t *)calloc(1, sizeof(mp3_stream_t));
    if (!stream) {
        printf("mp3_stream_t malloc fail\n");
        return PLAY_RESULT_NEXT;
    }

    // Allocate ring buffer once; reuse on subsequent tracks to keep it in SRAM.
    if (!s_mp3_ring_buf) {
        s_mp3_ring_buf = (uint8_t *)malloc(MP3_STREAM_BUF_SIZE);
        if (!s_mp3_ring_buf) {
            printf("mp3 ring buf malloc fail\n");
            free(stream);
            return PLAY_RESULT_NEXT;
        }
    }
    stream->buf = s_mp3_ring_buf;

    fr = f_open(&stream->file, filepath, FA_READ);
    if (fr != FR_OK) {
        printf("Open fail %d for '%s'\n", fr, filepath);
        result = PLAY_RESULT_NEXT;
        goto CLEANUP_STREAM_ONLY;
    }

    DWORD file_size = f_size(&stream->file);

    // ---- WAV decoder ----
    drwav  *wav  = NULL;
    // ---- FLAC decoder + dynamic seek cache ----
    drflac          *flac            = NULL;
    drflac_seekpoint *flac_seekcache = NULL;
    uint32_t         flac_seekcache_count = 0;
    // ---- MP3 decoder ----
    drmp3  *mp3  = NULL;

    uint32_t track_sample_rate = 0;
    uint8_t  track_channels    = 0;

    if (is_wav) {
        printf("Streaming WAV file (%lu bytes)...\n", (unsigned long)file_size);

        wav = (drwav *)calloc(1, sizeof(drwav));
        if (!wav) { result = PLAY_RESULT_NEXT; goto CLEANUP_FILE; }

        if (!drwav_init(wav, wav_fatfs_read, wav_fatfs_seek, wav_fatfs_tell, &stream->file, NULL)) {
            printf("E drwav_init failed for '%s'\n", filepath);
            free(wav); wav = NULL;
            result = PLAY_RESULT_NEXT;
            goto CLEANUP_FILE;
        }

        track_sample_rate = wav->sampleRate;
        track_channels    = (uint8_t)wav->channels;

        // Populate tags from filename (WAV rarely carries metadata)
        if (mp3_tags_ensure_alloc()) {
            memset(g_mp3_tags, 0, sizeof(*g_mp3_tags));
            const char *base = basename_from_path(filepath);
            size_t blen = strlen(base);
            // Strip ".wav" extension for display
            size_t copy_len = (blen > 4) ? (blen - 4) : blen;
            if (copy_len >= sizeof(g_mp3_tags->title)) copy_len = sizeof(g_mp3_tags->title) - 1;
            memcpy(g_mp3_tags->title, base, copy_len);
            g_mp3_tags->title[copy_len] = '\0';
        }

        wav_estimate_track_duration(wav);

        // Resume: seek to approximate byte offset if requested
        if (g_byte_offset > 0 && track_sample_rate > 0 && g_total_duration_ms > 0) {
            uint64_t target_frame = (uint64_t)wav->totalPCMFrameCount * g_byte_offset / file_size;
            if (drwav_seek_to_pcm_frame(wav, target_frame)) {
                played_frames = target_frame;
                g_time_display_base_frames = played_frames;
                g_time_display_offset_ms   = (int64_t)(target_frame * 1000 / track_sample_rate);
            }
        }

    } else if (is_flac) {
        printf("Streaming FLAC file (%lu bytes)...\n", (unsigned long)file_size);

        // Alloc + clear tags before open so flac_meta_callback can write into them
        if (mp3_tags_ensure_alloc()) {
            memset(g_mp3_tags, 0, sizeof(*g_mp3_tags));
        }

        flac = drflac_open_with_metadata(flac_fatfs_read, flac_fatfs_seek, flac_fatfs_tell,
                                         flac_meta_callback, &stream->file, NULL);
        if (!flac) {
            printf("E drflac_open failed for '%s'\n", filepath);
            result = PLAY_RESULT_NEXT;
            goto CLEANUP_FILE;
        }

        track_sample_rate = flac->sampleRate;
        track_channels    = (uint8_t)flac->channels;

        // Fallback: use filename if vorbis TITLE tag was absent
        if (g_mp3_tags && g_mp3_tags->title[0] == '\0') {
            const char *base = basename_from_path(filepath);
            size_t blen = strlen(base);
            size_t copy_len = (blen > 5) ? (blen - 5) : blen;
            if (copy_len >= sizeof(g_mp3_tags->title)) copy_len = sizeof(g_mp3_tags->title) - 1;
            memcpy(g_mp3_tags->title, base, copy_len);
            g_mp3_tags->title[copy_len] = '\0';
        }

        flac_estimate_track_duration(flac);

        // Build a dynamic seek cache for files without an embedded seektable.
        // dr_flac's pSeekpoints/seekpointCount are NOT separately freed by drflac_close
        // (they live inside the pFlac allocation when present, or NULL). We can safely inject
        // our own buffer and restore NULL before close.
        if (flac->pSeekpoints == NULL || flac->seekpointCount == 0) {
            flac_seekcache = (drflac_seekpoint *)calloc(FLAC_SEEK_CACHE_ENTRIES, sizeof(drflac_seekpoint));
            if (flac_seekcache) {
                // Anchor at frame 0 = start of audio data
                flac_seekcache[0].firstPCMFrame  = 0;
                flac_seekcache[0].flacFrameOffset = 0;
                flac_seekcache[0].pcmFrameCount   = flac->maxBlockSizeInPCMFrames ? flac->maxBlockSizeInPCMFrames : 4096;
                flac_seekcache_count = 1;
                flac->pSeekpoints   = flac_seekcache;
                flac->seekpointCount = flac_seekcache_count;
                printf("FLAC: no seektable, building dynamic seek cache\n");
            }
        }

        // Resume: seek to approximate frame offset if requested
        if (g_byte_offset > 0 && track_sample_rate > 0 && g_total_duration_ms > 0 && flac->totalPCMFrameCount > 0) {
            uint64_t target_frame = (uint64_t)flac->totalPCMFrameCount * g_byte_offset / file_size;
            if (drflac_seek_to_pcm_frame(flac, target_frame)) {
                played_frames = target_frame;
                g_time_display_base_frames = played_frames;
                g_time_display_offset_ms   = (int64_t)(target_frame * 1000 / track_sample_rate);
            }
        }

    } else {
        printf("Streaming MP3 file (%lu bytes)...\n", (unsigned long)file_size);

        // Seek from power down state; otherwise skip past ID3 tag for fast init
        if (g_byte_offset > 0) {
            if (!mp3_resume_open(stream, g_byte_offset)) {
                f_lseek(&stream->file, 0);
                stream->rd = stream->wr = 0;
                stream->count = 0;
                stream->eof = false;
            }
        } else {
            mp3_skip_id3v2(&stream->file, stream); // parses tags + seeks past ID3
        }

        // Initial small prefill
        for (int i = 0; i < 4 && !stream->eof; i++) {
            mp3_refill(stream);
        }

        // Allocate drmp3 state once; reuse on subsequent tracks to keep it in SRAM.
        if (!s_mp3_state) {
            s_mp3_state = (drmp3 *)malloc(sizeof(drmp3));
            if (!s_mp3_state) { result = PLAY_RESULT_NEXT; goto CLEANUP_FILE; }
        }
        mp3 = s_mp3_state;
        memset(mp3, 0, sizeof(drmp3));

        if (!drmp3_init(mp3,
                        mp3_stream_read,
                        NULL,
                        NULL,
                        drmp3_on_meta,
                        stream,
                        &s_drmp3_alloc)) {
            printf("E drmp3_init failed\n");
            result = PLAY_RESULT_NEXT;
            goto CLEANUP_FILE;
        }

        track_sample_rate = mp3->sampleRate;
        track_channels    = (uint8_t)mp3->channels;

        if (mp3_tags_ensure_alloc()) {
            *g_mp3_tags = stream->tags;
        }

        mp3_estimate_track_duration(stream, mp3, file_size);

        // Seed display time from byte offset on resume
        if (g_byte_offset > 0 && g_total_duration_ms > 0 && file_size > 0) {
            uint32_t audio_start = 0;
            {
                uint8_t id3h[10]; UINT id3_br = 0;
                FSIZE_t sp = f_tell(&stream->file);
                if (f_lseek(&stream->file, 0) == FR_OK) {
                    f_read(&stream->file, id3h, sizeof(id3h), &id3_br);
                    f_lseek(&stream->file, sp);
                }
                if (id3_br >= 10 && id3h[0]=='I' && id3h[1]=='D' && id3h[2]=='3') {
                    uint32_t id3_size = ((uint32_t)(id3h[6]&0x7F)<<21)|((uint32_t)(id3h[7]&0x7F)<<14)
                                      | ((uint32_t)(id3h[8]&0x7F)<< 7)| (uint32_t)(id3h[9]&0x7F);
                    audio_start = 10 + id3_size;
                    if (id3h[5] & 0x10) audio_start += 10;
                }
            }
            if (file_size > audio_start) {
                uint32_t audio_bytes     = file_size - audio_start;
                uint32_t offset_in_audio = (g_byte_offset > audio_start) ? (g_byte_offset - audio_start) : 0;
                uint32_t resume_ms = (uint32_t)((uint64_t)offset_in_audio * g_total_duration_ms / audio_bytes);
                g_time_display_base_frames = played_frames;
                g_time_display_offset_ms   = (int64_t)resume_ms;
            }
        }
    }

    if (show_now_playing) {
        draw_now_playing(mp3_list_obj);
    }

    printf("%s: %u Hz, %u ch\n", is_wav ? "WAV" : is_flac ? "FLAC" : "MP3", track_sample_rate, track_channels);
#if ALLOW_48KHz_PERFECT_PITCH
    // Switch sys clock for optimal integer I2S divider: 300MHz for 48kHz, 340MHz for everything else.
    // No-ops if already at the right clock. Handles I2S stop/restart + DAC mute internally.
    switch_to_48khz_clock(track_sample_rate == 48000, track_sample_rate, is_muted);
#endif
    i2s_set_sample_freq(&i2s_config, track_sample_rate, false);

    uint8_t channels = track_channels;
    // Persistent decode buffers — allocated once at max (stereo) size, reused across tracks.
    if (!s_pcmA) s_pcmA = (int16_t *)malloc(PCM_BUF_MAX * sizeof(int16_t));
    if (!s_pcmB) s_pcmB = (int16_t *)malloc(PCM_BUF_MAX * sizeof(int16_t));
    if (!s_pcmC) s_pcmC = (int16_t *)malloc(PCM_BUF_MAX * sizeof(int16_t));
    // Pre-allocate drmp3 pData buffer to guarantee SRAM placement.
    if (!s_drmp3_pdata) {
        s_drmp3_pdata = (uint8_t *)malloc(DRMP3_PDATA_INIT_SIZE);
        s_drmp3_pdata_cap = s_drmp3_pdata ? DRMP3_PDATA_INIT_SIZE : 0;
    }

    // Zero decode buffers so new track starts silent (no stale audio).
    if (s_pcmA) memset(s_pcmA, 0, PCM_BUF_MAX * sizeof(int16_t));
    if (s_pcmB) memset(s_pcmB, 0, PCM_BUF_MAX * sizeof(int16_t));
    if (s_pcmC) memset(s_pcmC, 0, PCM_BUF_MAX * sizeof(int16_t));

    int16_t *pcmA = s_pcmA;
    int16_t *pcmB = s_pcmB;
    int16_t *pcmC = s_pcmC;

    if (!pcmA || !pcmB || !pcmC) {
        printf("pcm malloc fail\n");
        result = PLAY_RESULT_NEXT;
        goto CLEANUP_MP3;
    }

    int16_t *buf_play  = pcmA;
    int16_t *buf_ready = pcmB;
    int16_t *buf_fill  = pcmC;

    // Extra prefill for MP3 ring buffer (WAV/FLAC reads directly from FatFS)
    if (!is_wav && !is_flac) {
        for (int i = 0; i < 4 && !stream->eof; i++) {
            mp3_refill(stream);
        }
    }

#define DECODE_FRAMES(dst) \
    (is_wav  ? (uint64_t)drwav_read_pcm_frames_s16(wav,   PCM_FRAME_COUNT, (dst)) \
     : is_flac ? (uint64_t)drflac_read_pcm_frames_s16(flac, PCM_FRAME_COUNT, (drflac_int16 *)(dst)) \
               : (uint64_t)drmp3_read_pcm_frames_s16(mp3,  PCM_FRAME_COUNT, (dst)))

    printf("Starting %s stream...\n", is_wav ? "WAV" : is_flac ? "FLAC" : "MP3");

    (void)mp3_get_resume_offset(stream); // position snapshot (MP3 byte tracking)
    uint64_t frames_play = DECODE_FRAMES(buf_play);
    if (frames_play > 0) played_frames += frames_play;
    mp3_zero_pcm_tail(buf_play, frames_play, PCM_FRAME_COUNT, channels);
    if (paused) i2s_dma_write_silence(&i2s_config);
    else        i2s_dma_write(&i2s_config, (const uint16_t *)buf_play);

    if (!is_wav && !is_flac) {
        for (int i = 0; i < 4 && !stream->eof; i++) mp3_refill(stream);
    }

    uint64_t frames_ready = DECODE_FRAMES(buf_ready);
    if (frames_ready > 0) played_frames += frames_ready;
    mp3_zero_pcm_tail(buf_ready, frames_ready, PCM_FRAME_COUNT, channels);

    uint64_t frames_fill = DECODE_FRAMES(buf_fill);
    if (frames_fill > 0) played_frames += frames_fill;
    mp3_zero_pcm_tail(buf_fill, frames_fill, PCM_FRAME_COUNT, channels);

    bool ready_chunk_valid = (frames_ready > 0);
    bool decoded_next_chunk = (frames_fill > 0);
    uint32_t replayed_chunk_count = 0;

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

    // UP/DOWN hold state
    static uint64_t up_press_us        = 0;
    static uint64_t up_last_page_us    = 0;
    static bool     up_held_paging     = false;

    static uint64_t down_press_us      = 0;
    static uint64_t down_last_page_us  = 0;
    static bool     down_held_paging   = false;

    // Inactivity tracking
    uint64_t last_interaction_us = time_us_64();
    uint64_t last_nav_us         = last_interaction_us; // UP/DOWN/A (for now-playing auto)

    bool prev_inactive = g_mp3_inactive;

    // ================================================================
    // MAIN PLAYBACK LOOP
    // ================================================================
    while (1) {
            watchdog_update();

            if (!gpio_read(GPIO_SW_OUT)) {
                mp3_save_shutdown(current_track_index, played_frames, track_sample_rate, stream);

                release_power();

                // if we reach here, something went wrong
                sleep_ms(100);
                watchdog_reboot(0, 0, 0); // Force reboot
            }

            // SD mini-refills (MP3 ring buffer only; WAV/FLAC reads directly)
            // 3 passes keep the ring buffer near-full at high bitrates, reducing
            // inline SD reads inside drmp3_read_pcm_frames_s16 at VBR spikes.
            if (!is_wav && !is_flac) {
                for (int i = 0;
                     i < 3 && stream->count < (MP3_STREAM_BUF_SIZE - MP3_REFILL_CHUNK) && !stream->eof;
                     i++) {
                    mp3_refill(stream);
                }
            }

            static uint64_t last_i2s = 0;
            uint64_t i2s_now = time_us_64();

            // If the fill buffer is decoded while ready is empty, promote it.
            if (!ready_chunk_valid && decoded_next_chunk) {
                int16_t *tmp = buf_ready;
                buf_ready = buf_fill;
                buf_fill = tmp;
                ready_chunk_valid = true;
                decoded_next_chunk = false;
            }

            bool rotate_audio_buffers = false;
            bool replay_previous_chunk = false;
            bool dma_accepted;
            if (paused) {
                dma_accepted = i2s_dma_write_silence_non_blocking(&i2s_config);
            } else {
                const uint16_t *dma_src;
                if (ready_chunk_valid) {
                    dma_src = (const uint16_t *)buf_ready;
                    rotate_audio_buffers = true;
                } else {
                    // Decoder fell behind: replay last known-good audio.
                    dma_src = (const uint16_t *)buf_play;
                    replay_previous_chunk = true;
                }
                dma_accepted = i2s_dma_write_non_blocking(&i2s_config, dma_src);
            }

            if (dma_accepted) {
                if (rotate_audio_buffers) {
                    // DMA accepted buf_ready and started playing it.
                    int16_t *old = buf_play;
                    buf_play  = buf_ready;   // now playing
                    buf_ready = buf_fill;    // next ready
                    buf_fill  = old;         // to be filled

                    ready_chunk_valid = decoded_next_chunk;
                    decoded_next_chunk = false;  // decode again on next loop
                }
                last_i2s = i2s_now;

                if (replay_previous_chunk) {
                    replayed_chunk_count++;
                    // Prioritize ring-buffer refill and decode catch-up over UI work on underrun.
                    if (!is_wav && !is_flac) {
                        for (int i = 0; i < 4 && stream->count < (MP3_STREAM_BUF_SIZE / 2) && !stream->eof; i++) {
                            mp3_refill(stream);
                        }
                    }
                    continue;
                }
            }

            // Decode into buf_fill only when we actually need another chunk.
            if (!decoded_next_chunk && !paused) {
                uint64_t frames = DECODE_FRAMES(buf_fill);

                if (frames > 0) {
                    played_frames += frames;
                    mp3_zero_pcm_tail(buf_fill, frames, PCM_FRAME_COUNT, channels);

                    // Record a seekpoint every ~10 seconds for FLAC files without a seektable.
                    // f_tell is up to FLAC_READ_AHEAD_BYTES ahead of the actual decode position;
                    // we subtract that to ensure the stored offset is before the true frame boundary
                    // so the binary search can always find the frame by scanning forward.
                    if (is_flac && flac_seekcache && flac_seekcache_count < FLAC_SEEK_CACHE_ENTRIES) {
                        drflac_uint64 next_sp = flac_seekcache[flac_seekcache_count - 1].firstPCMFrame
                                                + FLAC_SEEK_INTERVAL_FRAMES;
                        if (played_frames >= next_sp) {
                            uint64_t fp = (uint64_t)f_tell(&stream->file);
                            drflac_uint64 off = (fp > flac->firstFLACFramePosInBytes + FLAC_READ_AHEAD_BYTES)
                                                ? (drflac_uint64)(fp - flac->firstFLACFramePosInBytes - FLAC_READ_AHEAD_BYTES)
                                                : 0;
                            flac_seekcache[flac_seekcache_count].firstPCMFrame  = played_frames;
                            flac_seekcache[flac_seekcache_count].flacFrameOffset = off;
                            flac_seekcache[flac_seekcache_count].pcmFrameCount  =
                                flac->maxBlockSizeInPCMFrames ? flac->maxBlockSizeInPCMFrames : 4096;
                            flac_seekcache_count++;
                            flac->seekpointCount = flac_seekcache_count;
                        }
                    }
                }

                bool at_eof = (is_wav || is_flac) ? (frames == 0) : (frames == 0 && stream->eof);

                if (frames == 0) {
                    if (at_eof) {
                        // Handle repeat logic
                        if (g_repeat_mode == REPEAT_OFF) {
                            printf("EOF reached (no repeat) -> next track\n");
                            next_track_requested = true;
                            result = PLAY_RESULT_NEXT;
                            goto END_PLAYBACK;
                        }

                        // Restart same track
                        printf("EOF reached -> restarting track (repeat)\n");
                        if (is_wav) {
                            drwav_seek_to_pcm_frame(wav, 0);
                        } else if (is_flac) {
                            drflac_seek_to_pcm_frame(flac, 0);
                        } else {
                            f_lseek(&stream->file, 0);
                            stream->rd = stream->wr = stream->count = 0;
                            stream->eof = false;
                            drmp3_uninit(mp3);
                            if (!drmp3_init(mp3, mp3_stream_read,
                                            NULL, NULL, drmp3_on_meta, stream, &s_drmp3_alloc)) {
                                printf("E drmp3_init after repeat\n");
                                result = PLAY_RESULT_NEXT;
                                goto END_PLAYBACK;
                            }
                        }

                        // REPEAT_ONE: do only one extra loop, then OFF
                        if (g_repeat_mode == REPEAT_ONE) {
                            g_repeat_mode = REPEAT_OFF;
                            update_mp3_bottom_bar_shuffle_repeat(mp3_hint_right_obj, g_repeat_mode, g_shuffle_enabled, paused);
                            printf("Repeat ONE complete -> Repeat OFF\n");
                        }

                        ready_chunk_valid = false;
                        decoded_next_chunk = false;
                        // Reset seek correction: time restarts from 0
                        g_time_display_base_frames = played_frames;
                        g_time_display_offset_ms   = 0;
                        continue;
                    }
                    // Not EOF but no frames – try again next cycle
                } else {
                    decoded_next_chunk = true;
                }
            }

            // Skip controls if too little time remains before the next DMA deadline, or too soon
            // since the last controls pass. Use a fresh time_us_64() here (not the stale i2s_now
            // captured at the top of the loop) so that decode time (~30ms with double buffering)
            // is accounted for in the remaining-budget calculation.
            // Budget: DMA period (~139ms 44.1k / ~128ms 48k) - 90ms guard = ~49ms window for decode+controls.
            static uint64_t last_controls_us = 0;
            uint64_t now_guard = time_us_64();
            if (now_guard - last_i2s > (PCM_FRAME_COUNT*1000000ULL / track_sample_rate) - 80*1000 || now_guard - last_controls_us < 10*1000) {
                continue;
            }
            last_controls_us = now_guard;
            // ================== BUTTONS + CONTROLS ====================
            read_volume(&i2s_config);

            bool iox_nint    = gpio_read(GPIO_IOX_nINT);
            bool select_btn  = !gpio_read(GPIO_B_SELECT);

            static bool btn_a     = false;
            static bool btn_b     = false;
            static bool btn_up    = false;
            static bool btn_down  = false;
            static bool btn_left  = false;
            static bool btn_right = false;
            static bool btn_start = false;


            #define ANY_BUTTON_PRESSED \
                (btn_a || btn_b || btn_up || btn_down || btn_left || btn_right || btn_start || select_btn)

            #define NAV_BUTTON_PRESSED \
                (btn_up || btn_down || btn_a)

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
            bool woke_from_inactive_this_tick = false;

            if (ANY_BUTTON_PRESSED && !g_buttons_locked) {
                last_interaction_us = now_us;
            }

            if (!g_mp3_inactive &&
                (now_us - last_interaction_us >= MP3_INACTIVE_TIMEOUT_US)) {

                g_mp3_inactive = true;
                printf("MP3 → inactive mode\n");
            }

            // ==================================================================
            // NEW: SELECT combos (brightness + lock) + SELECT tap handling
            // ==================================================================

            // If SELECT participates in any combo this time it's pressed,
            // we mark it so we DON'T treat it as a repeat-mode tap later.
            if (!select_btn) {
                // SELECT released: if it was NOT part of a combo, treat as repeat tap
                if (prev_btn_select && !select_was_combo) {
                    if (!g_mp3_inactive) {
                        g_repeat_mode = (repeat_mode_t)((g_repeat_mode + 1) % 3);

                        if (g_repeat_mode == REPEAT_OFF) {
                            printf("Repeat: OFF\n");
                        } else if (g_repeat_mode == REPEAT_ONE) {
                            printf("Repeat: ONE (play once more)\n");
                        } else if (g_repeat_mode == REPEAT_INFINITE) {
                            printf("Repeat: INFINITE\n");
                        }
                        update_mp3_bottom_bar_shuffle_repeat(mp3_hint_right_obj,
                                                            g_repeat_mode,
                                                            g_shuffle_enabled,
                                                            paused);
                    }
                }
                // Reset combo flag once SELECT is up
                select_was_combo = false;
            } else {
                // SELECT is held: check for combos

                // 1) START + SELECT → lock/unlock all buttons
                if (btn_start && !(prev_btn_start && prev_btn_select)) {
                    g_buttons_locked = !g_buttons_locked;
                    select_was_combo = true;
                    // sleep mp3
                    g_mp3_inactive = g_buttons_locked; 
                    if (!g_buttons_locked) last_interaction_us = now_us;
                    printf("Buttons %s\n", g_buttons_locked ? "LOCKED" : "UNLOCKED");
                }

                // Only allow brightness combos when NOT locked
                if (!g_buttons_locked) {
                    // SELECT + UP   → LCD brighter
                    if (btn_up && (!prev_btn_up || !prev_btn_select)) {
                        if (!low_power) {
#if TIE_PWR_LED_TO_LCD
                            pwr_led_duty_cycle = lcd_led_duty_cycle;
#endif
                            step_pwr_brightness(true);
                        }
                        step_lcd_brightness(true);
                        select_was_combo = true;
                    }

                    // SELECT + DOWN → LCD dimmer
                    if (btn_down && (!prev_btn_down || !prev_btn_select)) {
                        if (!low_power) {
#if TIE_PWR_LED_TO_LCD
                            pwr_led_duty_cycle = lcd_led_duty_cycle;
#endif
                            step_pwr_brightness(false);
                        }
                        step_lcd_brightness(false);
                        select_was_combo = true;
                    }

                    // SELECT + RIGHT → button LEDs brighter
                    if (btn_right && (!prev_btn_right || !prev_btn_select)) {
                        step_button_brightness(true); 
                        select_was_combo = true;
                    }

                    // SELECT + LEFT → button LEDs dimmer
                    if (btn_left && (!prev_btn_left || !prev_btn_select)) {
                        step_button_brightness(false);
                        select_was_combo = true;
                    }

                    // SELECT + A → something
                    if (btn_a && (!prev_btn_a || !prev_btn_select)) {
                        alternate_eq();
                        select_was_combo = true;
                    }

                    // SELECT + B → something
                    if (btn_b && (!prev_btn_b || !prev_btn_select)) {
                        select_was_combo = true;
                    }

                }
            }

            // If buttons are locked, ignore everything EXCEPT the lock combo above.
            // (We still got here, so START+SELECT can always unlock.)
            if (g_buttons_locked || select_was_combo) {
                goto STORE_PREV_BUTTONS;
            }

            // -------------- MP3 Inactivity Detection --------------
            if (g_mp3_inactive && ANY_BUTTON_PRESSED) {
                // Wake LCD immediately, but continue processing this same button press.
                g_mp3_inactive = false;
                last_interaction_us = now_us;
                woke_from_inactive_this_tick = true;
                if (prev_inactive) {
                    start_lcd(true, false);
                    prev_inactive = false;
                }
            }

            // --------- "Now Playing" auto swap (ONLY UP/DOWN/A) ---------
            if (NAV_BUTTON_PRESSED) {
                last_nav_us = now_us;
            }

            if (!g_mp3_inactive &&
                !show_now_playing &&
                (now_us - last_nav_us) >= MP3_NOW_PLAYING_TIMEOUT_US) {

                show_now_playing = true;
                printf("MP3 → now playing\n");
            }


            // A → Play Selected Track
            if (!prev_btn_a && btn_a) {
                // If A was used to wake from inactive this tick, consume it as wake-only.
                if (!woke_from_inactive_this_tick) {
                    // only if mp3 is active can we toggle now playing and play the selected track.
                    if (!g_mp3_inactive && !show_now_playing) {
                        show_now_playing = true;
                        paused = false;
                        toggle_speakers_if_paused();
                        update_mp3_bottom_bar_shuffle_repeat(mp3_hint_right_obj, g_repeat_mode, g_shuffle_enabled, paused);
                        result = PLAY_RESULT_SELECTED;
                        goto END_PLAYBACK;
                    }
                    if (!g_mp3_inactive && show_now_playing) {
                        show_now_playing = false;
                    }
                }
            }

            // B → Play / Pause
            if (!prev_btn_b && btn_b) {
                paused = !paused;
                toggle_speakers_if_paused();
                printf(paused ? "Paused\n" : "Playing\n");
                update_mp3_bottom_bar_shuffle_repeat(mp3_hint_right_obj, g_repeat_mode, g_shuffle_enabled, paused);
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

                            bool seek_ok = false;
                            if (is_wav || is_flac) {
                                int64_t cur_ms = (int64_t)((played_frames - g_time_display_base_frames) * 1000 / track_sample_rate) + g_time_display_offset_ms;
                                int64_t target_ms = cur_ms - (int64_t)step * 1000;
                                if (target_ms < 0) target_ms = 0;
                                uint64_t target_frame = (uint64_t)target_ms * track_sample_rate / 1000;
                                bool seeked = is_wav ? drwav_seek_to_pcm_frame(wav, (drwav_uint64)target_frame)
                                                     : drflac_seek_to_pcm_frame(flac, (drflac_uint64)target_frame);
                                if (seeked) {
                                    ready_chunk_valid = false;
                                    decoded_next_chunk = false;
                                    played_frames              = target_frame;
                                    g_time_display_base_frames = target_frame;
                                    g_time_display_offset_ms   = target_ms;
                                    seek_ok = true;
                                }
                            } else {
                                uint32_t seeked_pos = 0;
                                if (seek_relative_seconds(stream, mp3, -step, track_sample_rate, &decoded_next_chunk, &seeked_pos, played_frames)) {
                                    ready_chunk_valid = false;
                                    if (g_total_duration_ms > 0 && file_size > g_mp3_audio_start) {
                                        int64_t audio_bytes = (int64_t)file_size - (int64_t)g_mp3_audio_start;
                                        int64_t off = (int64_t)seeked_pos - (int64_t)g_mp3_audio_start;
                                        if (off < 0) off = 0;
                                        int64_t new_ms = off * (int64_t)g_total_duration_ms / audio_bytes;
                                        g_time_display_base_frames = played_frames;
                                        g_time_display_offset_ms   = new_ms;
                                    }
                                    seek_ok = true;
                                }
                            }
                            (void)seek_ok;
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

                            bool seek_ok = false;
                            if (is_wav || is_flac) {
                                int64_t cur_ms = (int64_t)((played_frames - g_time_display_base_frames) * 1000 / track_sample_rate) + g_time_display_offset_ms;
                                int64_t target_ms = cur_ms + (int64_t)step * 1000;
                                if (target_ms < 0) target_ms = 0;
                                if (g_total_duration_ms > 0 && target_ms > (int64_t)g_total_duration_ms) target_ms = (int64_t)g_total_duration_ms;
                                uint64_t target_frame = (uint64_t)target_ms * track_sample_rate / 1000;
                                bool seeked = is_wav ? drwav_seek_to_pcm_frame(wav, (drwav_uint64)target_frame)
                                                     : drflac_seek_to_pcm_frame(flac, (drflac_uint64)target_frame);
                                if (seeked) {
                                    ready_chunk_valid = false;
                                    decoded_next_chunk = false;
                                    played_frames              = target_frame;
                                    g_time_display_base_frames = target_frame;
                                    g_time_display_offset_ms   = target_ms;
                                    seek_ok = true;
                                }
                            } else {
                                uint32_t seeked_pos = 0;
                                if (seek_relative_seconds(stream, mp3, step, track_sample_rate, &decoded_next_chunk, &seeked_pos, played_frames)) {
                                    ready_chunk_valid = false;
                                    if (g_total_duration_ms > 0 && file_size > g_mp3_audio_start) {
                                        int64_t audio_bytes = (int64_t)file_size - (int64_t)g_mp3_audio_start;
                                        int64_t off = (int64_t)seeked_pos - (int64_t)g_mp3_audio_start;
                                        if (off < 0) off = 0;
                                        int64_t new_ms = off * (int64_t)g_total_duration_ms / audio_bytes;
                                        g_time_display_base_frames = played_frames;
                                        g_time_display_offset_ms   = new_ms;
                                    }
                                    seek_ok = true;
                                }
                            }
                            (void)seek_ok;
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

            // ===================== UP (single step + page on hold) =====================
            if (btn_up) {
                if (!prev_btn_up) {
                    // First press
                    up_press_us     = now_us;
                    up_last_page_us = now_us;
                    up_held_paging  = false;

                    if (show_now_playing) {
                        // First UP just exits Now Playing
                        show_now_playing = false;
                    } else {
                        // Tap immediately moves 1 item for "short press"
                        mp3_select_relative(-1, mp3_list_obj, g_playlist, g_track_count);
                    }
                } else {
                    // Still holding UP
                    if (!show_now_playing) {  // only page when list is visible
                        uint64_t held_ms = (now_us - up_press_us) / 1000;

                        if (held_ms >= NAV_PAGE_HOLD_TIME_MS) {
                            if (!up_held_paging) {
                                // First time we detect a "hold":
                                // we already moved -1 on press, so move -(VISIBLE_ITEMS-1)
                                // to get a total of -VISIBLE_ITEMS from the original.
                                up_held_paging = true;
                                mp3_select_relative(-1, mp3_list_obj, g_playlist, g_track_count);
                                up_last_page_us = now_us;
                            } else {
                                // Optional auto-repeat paging while held
                                uint64_t since_last_ms = (now_us - up_last_page_us) / 1000;
                                if (NAV_FAST_PAGE_ENABLED && since_last_ms >= NAV_PAGE_REPEAT_INTERVAL_MS) {
                                    mp3_select_relative(-4, mp3_list_obj, g_playlist, g_track_count);
                                    // up_last_page_us = now_us;
                                } else {
                                    mp3_select_relative(-1, mp3_list_obj, g_playlist, g_track_count);
                                }
                            }
                        }
                    }
                }
            } else {
                // Released
                up_held_paging = false;
            }

            // =================== DOWN (single step + page on hold) =====================
            if (btn_down) {
                if (!prev_btn_down) {
                    // First press
                    down_press_us     = now_us;
                    down_last_page_us = now_us;
                    down_held_paging  = false;

                    if (show_now_playing) {
                        // First DOWN just exits Now Playing
                        show_now_playing = false;
                    } else {
                        // Tap immediately moves 1 item for "short press"
                        mp3_select_relative(+1, mp3_list_obj, g_playlist, g_track_count);
                    }
                } else {
                    // Still holding DOWN
                    if (!show_now_playing) {
                        uint64_t held_ms = (now_us - down_press_us) / 1000;

                        if (held_ms >= NAV_PAGE_HOLD_TIME_MS) {
                            if (!down_held_paging) {
                                // First "hold" page:
                                // already moved +1, so add +(VISIBLE_ITEMS-1)
                                // -> net +VISIBLE_ITEMS from original.
                                down_held_paging = true;
                                // mp3_select_relative(+(VISIBLE_ITEMS - 1),
                                //                     mp3_list_obj, g_playlist, g_track_count);
                                mp3_select_relative(+1, mp3_list_obj, g_playlist, g_track_count);
                                down_last_page_us = now_us;
                            } else {
                                // Auto-repeat pages while held
                                uint64_t since_last_ms = (now_us - down_last_page_us) / 1000;
                                if (NAV_FAST_PAGE_ENABLED && since_last_ms >= NAV_PAGE_REPEAT_INTERVAL_MS) {
                                    // mp3_select_relative(+VISIBLE_ITEMS,
                                                        // mp3_list_obj, g_playlist, g_track_count);
                                    mp3_select_relative(+4, mp3_list_obj, g_playlist, g_track_count);
                                    // down_last_page_us = now_us;
                                } else {
                                    mp3_select_relative(+1, mp3_list_obj, g_playlist, g_track_count);
                                }
                            }
                        }
                    }
                }
            } else {
                down_held_paging = false;
            }

            // START → Shuffle toggle (global flag ONLY; playlist will rebuild order)
            if (!prev_btn_start && btn_start) {
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

            // -----------------------------------------------------------------
            // Label used for the lock early-exit above:
            // -----------------------------------------------------------------
            STORE_PREV_BUTTONS:
            // Save previous button states
            prev_btn_a      = btn_a;
            prev_btn_b      = btn_b;
            prev_btn_up     = btn_up;
            prev_btn_down   = btn_down;
            prev_btn_left   = btn_left;
            prev_btn_right  = btn_right;
            prev_btn_start  = btn_start;
            prev_btn_select = select_btn;


        // Now Playing shift
        static bool prev_now_playing = false;
        if (show_now_playing && !prev_now_playing) {
            // switch to now playing ui
            mp3_apply_now_playing_theme();
            draw_now_playing(mp3_list_obj);
            prev_now_playing = true;
        }
        else if (!show_now_playing && prev_now_playing) {
            // switch to track list ui
            mp3_apply_now_playing_theme();
            draw_track_list(
                mp3_list_obj,
                g_playlist,
                g_track_count,
                g_selected_file,
                g_selected_file
            );
            prev_now_playing = false;
        }

        // ==================================================
        // LVGL UPDATE (safe spot - DMA just accepted buffer)
        // ==================================================
        static uint64_t last_lvgl_update = 0;
        uint64_t now = time_us_64();
        uint64_t time_since_last_update = now - last_lvgl_update;
        if (/*time_since_last_update >= 10000 && */!g_mp3_inactive) {   // 10 ms (100 Hz UI updates)
            // printf("%llu us\n", (unsigned long long)time_since_last_update);
            uint32_t inc_ms = (uint32_t)(time_since_last_update / 1000);
            if (inc_ms > 300) inc_ms = 0;   // clamp to avoid anim jump on initial load of file.. otherwise increments range from 10ms-88ms
            lv_tick_inc(inc_ms);
            lv_timer_handler();
            last_lvgl_update = now;

            // Update now-playing time label and progress bar.
            // Use played_frames with seek correction for smooth ~1-second updates.
            {
                uint32_t cur_ms = 0;
                if (track_sample_rate > 0) {
                    int64_t raw_ms = (int64_t)((played_frames - g_time_display_base_frames) * 1000 / track_sample_rate)
                                     + g_time_display_offset_ms;
                    if (raw_ms > 0) cur_ms = (uint32_t)raw_ms;
                    if (g_total_duration_ms > 0 && cur_ms > g_total_duration_ms)
                        cur_ms = g_total_duration_ms;
                }
                mp3_update_progress(cur_ms, g_total_duration_ms);
            }
        }
        // ==================================================

        // BATTERY MONITORING //
        static bool timer_task_flagged = false;
        static bool prev_timer_task_flagged = false;

        // minimal battery monitoring callback function code extracted with the addition of the mp3 save shutdown function.
#if ENABLE_BAT_MONITORING
        bool was_task_flagged = false;
        // Check battery status periodically
        if (battery_task_flag) {
            battery_task_flag = false;
            process_bat_percent();
            was_task_flagged = true;
        }
        if (low_power_shutdown) {
            shutdown_screen(1500);
            mp3_save_shutdown(current_track_index, played_frames, track_sample_rate, stream);

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
        timer_task_flagged = was_task_flagged;
#else
        // Battery monitoring disabled: still consume the flag so the edge-detect
        // below can trigger update_status_label (and its RTC sync) each timer tick.
        if (battery_task_flag) {
            battery_task_flag = false;
            timer_task_flagged = true;
        } else {
            timer_task_flagged = false;
        }
#endif
        
        // we typically don't want to check the battery and update the rtc at the same time
        // this is due to both using i2c which we don't want to block for too long with audio running.
        // so we are going to do the rtc update only when the timer task flag goes from true to false
        // thus meaning the battery was checked in the previous iteration but not this one.
        if (!timer_task_flagged && prev_timer_task_flagged) {
            update_status_label(mp3_status_label_obj);
        }
        prev_timer_task_flagged = timer_task_flagged;

        // ================= MP3 Inactivity Handling =================
        if (g_mp3_inactive && !prev_inactive) {
            shutdown_lcd(true, false);
            // underclock_cpu(true);
            // i2s_set_sample_freq(&i2s_config, mp3->sampleRate, false);
            prev_inactive = true;
        }
        else if (!g_mp3_inactive && prev_inactive) {
            // underclock_cpu(false);
            // i2s_set_sample_freq(&i2s_config, mp3->sampleRate, false);
            start_lcd(true, false);
            prev_inactive = false;
        }

        sleep_us(10); // small delay for timers
        tight_loop_contents();
    }

END_PLAYBACK:
    if (replayed_chunk_count > 0) {
        printf("MP3 underrun recovery events: %lu\n", (unsigned long)replayed_chunk_count);
    }

    // If we exited the loop before hitting inactivity restore, bring LCD back now.
    if (!g_mp3_inactive && prev_inactive) {
        start_lcd(true, false);
        prev_inactive = false;
    }

    // Push silence to let the output settle before freeing buffers
    i2s_dma_write_silence(&i2s_config);

    // pcmA/B/C are persistent (s_pcmA/B/C) — don't free them.
    // drmp3 pData is persistent (s_drmp3_pdata) — don't free it.

CLEANUP_MP3:
    if (wav) {
        drwav_uninit(wav);
        free(wav);
        wav = NULL;
    }
    if (flac) {
        if (flac_seekcache) {
            // Restore original NULL state before close; drflac_close frees the pFlac block,
            // not pSeekpoints separately, but be explicit to avoid any future confusion.
            flac->pSeekpoints  = NULL;
            flac->seekpointCount = 0;
            free(flac_seekcache);
            flac_seekcache = NULL;
        }
        drflac_close(flac);
        flac = NULL;
    }
    if (mp3) {
        drmp3_uninit(mp3);
        // Don't free — s_mp3_state keeps this allocation alive for the next track.
        mp3 = NULL;
    }

CLEANUP_FILE:
    f_close(&stream->file);

CLEANUP_STREAM_ONLY:
    if (stream) {
        // Don't free stream->buf — it points to s_mp3_ring_buf which persists.
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
    lv_label_set_text(right, "R/S/P");
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
    char buf[8];
    uint8_t idx = 0;
    bool need_sep = false;

    // ---- Repeat ----
    if (repeat_state != REPEAT_OFF) {
        buf[idx++] = 'R';
        if (repeat_state == REPEAT_ONE)
            buf[idx++] = '1';

        need_sep = true;
    }

    // ---- Shuffle ----
    if (shuffle) {
        if (need_sep) buf[idx++] = '/';
        buf[idx++] = 'S';
        need_sep = true;
    }

    // ---- Paused ----
    if (paused) {
        if (need_sep) buf[idx++] = '/';
        buf[idx++] = 'P';
    }

    buf[idx] = '\0';

    lv_label_set_text(right, buf);
}
void update_mp3_bottom_bar_left(lv_obj_t *left, const char *text)
{
    lv_label_set_text(left, basename_from_path(text));
}

void mp3_apply_now_playing_theme() {
    lv_color_t col = show_now_playing ? lv_color_hex(0xFFFFFF)
                                : lv_color_hex(0xE0E0E0);
    // bottom bar bg
    if (mp3_bottom_bar) {
        lv_obj_set_style_bg_color(mp3_bottom_bar, col, 0);
    }
    // top bar bg
    if (mp3_top_bar) {
        lv_obj_set_style_bg_color(mp3_top_bar, col, 0);
    }
    if (show_now_playing) {
        update_mp3_bottom_bar_left(mp3_hint_left_obj, "Now Playing");
    } else {
        update_mp3_bottom_bar_left(mp3_hint_left_obj, g_playlist[current_index]);
    }
}

void draw_now_playing(lv_obj_t *parent)
{
    if (!parent) return;

    lv_obj_clean(parent);

    lv_color_t txt_color           = lv_color_hex(0x202020);
    lv_color_t highlight_color     = lv_color_hex(ACCENT_COLOR);
    lv_color_t txt_highlight_color = lv_color_hex(ACCENT_COLOR);
    int spacing = 6;

    const char *filename = g_playlist[current_index];

    // NEW: prefer ID3 title if available, else your filename fallback
    const char *display_title =
        (g_mp3_tags && g_mp3_tags->title[0]) ? g_mp3_tags->title : basename_from_path(filename);

    const char *artist = (g_mp3_tags && g_mp3_tags->artist[0]) ? g_mp3_tags->artist : "Unknown";
    const char *album  = (g_mp3_tags && g_mp3_tags->album[0])  ? g_mp3_tags->album  : "Unknown";

    // ============================================================
    // TRACK TITLE
    // ============================================================
    lv_obj_t *title_label = lv_label_create(parent);
    lv_obj_set_style_text_color(title_label, txt_highlight_color, 0);
    lv_obj_set_width(title_label, DISP_HOR_RES - 26);
    lv_label_set_long_mode(title_label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_anim_speed(title_label, 20, 0);
    lv_label_set_text_fmt(title_label, "%s\n", display_title);
    lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 13, 13);

    // ============================================================
    // TRACK NUMBER   (e.g. "3 / 25")
    // ============================================================
    lv_obj_t *tracknum_label = lv_label_create(parent);
    lv_obj_set_style_text_color(tracknum_label, txt_color, 0);
    lv_label_set_text_fmt(tracknum_label, "%d / %d", current_index + 1, g_track_count);
    lv_obj_align_to(tracknum_label, title_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, spacing);

    // ============================================================
    // ARTIST
    // ============================================================
    lv_obj_t *artist_label = lv_label_create(parent);
    lv_obj_set_style_text_color(artist_label, txt_color, 0);
    lv_label_set_text_fmt(artist_label, "Artist: %s", artist);
    lv_obj_align_to(artist_label, tracknum_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, spacing);

    // ============================================================
    // ALBUM
    // ============================================================
    lv_obj_t *album_label = lv_label_create(parent);
    lv_obj_set_style_text_color(album_label, txt_color, 0);
    lv_label_set_text_fmt(album_label, "Album: %s\n\n", album);
    lv_obj_align_to(album_label, artist_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, spacing);

    // ============================================================
    // TIME  (0:00 / 0:00)
    // ============================================================
    lv_obj_t *time_label = lv_label_create(parent);
    lv_obj_set_style_text_color(time_label, lv_color_hex(0x888888), 0);
    lv_obj_set_width(time_label, DISP_HOR_RES - 26);
    lv_obj_set_style_text_align(time_label, LV_TEXT_ALIGN_RIGHT, 0);
    g_now_playing_time_label = time_label;
    {
        char cur_buf[8], tot_buf[8], combined[18];
        format_time_ms(cur_buf, sizeof(cur_buf), 0);
        format_time_ms(tot_buf, sizeof(tot_buf), g_total_duration_ms);
        snprintf(combined, sizeof(combined), "%s / %s", cur_buf, tot_buf);
        lv_label_set_text(time_label, combined);
    }
    lv_obj_align_to(time_label, album_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, spacing);

    // ============================================================
    // PROGRESS BAR
    // ============================================================
    lv_obj_t *bar = lv_bar_create(parent);
    lv_obj_set_size(bar, DISP_HOR_RES - 26, 8);
    lv_obj_align_to(bar, time_label, LV_ALIGN_OUT_BOTTOM_LEFT, 0, spacing);

    // bar background
    lv_obj_set_style_bg_color(bar, lv_color_hex(0xD0D0D0), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_MAIN);

    // highlight fill
    lv_obj_set_style_bg_color(bar, highlight_color, LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(bar, LV_OPA_COVER, LV_PART_INDICATOR);

    lv_bar_set_range(bar, 0, MP3_PROGRESS_BAR_RANGE);
    g_now_playing_bar = bar;
    lv_bar_set_value(bar, 0, LV_ANIM_OFF);
}

static void mp3_show_message(lv_obj_t *list,
                             lv_obj_t *hint_left,
                             lv_obj_t *hint_right,
                             const char *msg)
{
    if (!list || !msg) return;

    char one_row[1][MP3_MAX_PATH_LEN];
    snprintf(one_row[0], MP3_MAX_PATH_LEN, "%s", msg);
    draw_track_list(list, one_row, 1, 0, 0);

    if (hint_left)  update_mp3_bottom_bar_left(hint_left, msg);
    if (hint_right) update_mp3_bottom_bar_shuffle_repeat(hint_right, REPEAT_OFF, false, false);

    lv_tick_inc(1);
    lv_timer_handler();
}

static void mp3_message_idle_loop(void)
{
    release_power();
    watchdog_disable();

    while (1) {
        __wfi();
        tight_loop_contents();
    }
}

void play_mp3_stream(const char *start_filename) {
    // Create list
    lv_init();

    // DO NOT EVER CHANGE THIS
    // We don't need the lvgl framebuffers for mp3 or imu dash
    // because the fb swaps that mess up lvgl only happen 
    // with gbc gameplay for the triple buffering.
    // But when having the lvgl fb buffers allocated,
    // we waste memory and also crash with malloc failures.
    lvgl_free_buffers();
    // Free the spare GBC frame buffer (~46 KB)
	// front_fb->data = lvgl_fb and write_fb->data = lv_buf1, so those must stay.
	if (free_fb) { free(free_fb); free_fb = NULL; }
    lvgl_fb = front_fb->data;
    lv_buf1 = (lv_color_t *)write_fb->data;

    static lv_disp_draw_buf_t draw_buf;
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

    lv_obj_t *status_label;
    mp3_top_bar = create_top_bar(cont, &status_label);
    update_status_label(status_label);

    // === Bottom hint bar ===
    lv_obj_t *hint_left;
    lv_obj_t *hint_right;
    mp3_bottom_bar = create_mp3_bottom_bar(cont, &hint_left, &hint_right);

    // Skeleton tracks until loaded.
    if (!mp3_playlist_init()) {
        printf("Playlist init failed\n");
        g_track_count = 0;
        mp3_show_message(list, hint_left, hint_right, "Playlist init failed");
        mp3_message_idle_loop();
    }
    for (uint8_t i = 0; i < VISIBLE_ITEMS; i++) {
        snprintf(g_playlist[i], MP3_MAX_PATH_LEN, "%s", loading_msgs[i]);
    }

    draw_track_list(list, g_playlist, VISIBLE_ITEMS, 0, 0);

    lv_tick_inc(1);
    lv_timer_handler();

    sd_card_t *pSD = sd_get_by_num(0);
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);
    if (fr != FR_OK) {
        printf("Mount fail %d\n", fr);
        mp3_show_message(list, hint_left, hint_right, "SD mount failed");
        mp3_message_idle_loop();
    }

    watchdog_enable(WATCHDOG_TIMEOUT_MS*2, true); // 4 second timeout, pause-on-debug = true
#if ENABLE_SAVE_ON_POWER_OFF
	hold_power(); // keep power on for saving mp3 state
#endif

    // Load resume info
    memset(&g_resume, 0, sizeof(g_resume));

    static FIL rf;
    static UINT br;
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
        mp3_show_message(list, hint_left, hint_right, "No MP3 files found on SD :P");
        f_unmount(pSD->pcName);
        mp3_message_idle_loop();
    }

    // Choose initial track index
    static uint32_t resume_position_ms = 0;

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
        g_byte_offset        = g_resume.byte_offset;
        g_shuffle_enabled    = g_resume.shuffle;
        g_repeat_mode        = (repeat_mode_t)g_resume.repeat_mode;

        printf("Resuming track %d at %u ms\n",
            current_index, resume_position_ms);

        if (g_shuffle_enabled) {
            // Try to restore existing shuffle order from disk
            if (!shuffle_load_state()) {
                // Fallback: build fresh order anchored on current_index
                build_shuffle_order(current_index);
            } else {
                // Make sure current_index exists in the loaded permutation
                int found = -1;
                for (int i = 0; i < g_track_count; i++) {
                    if (g_shuffle_order[i] == current_index) {
                        found = i;
                        break;
                    }
                }

                if (found >= 0) {
                    g_shuffle_pos = found;
                } else {
                    // Playlist changed; rebuild + save
                    build_shuffle_order(current_index);
                }
            }
        }
    }

    draw_track_list(list, g_playlist, g_track_count, current_index, current_index);
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
        g_byte_offset      = 0;

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
            if (show_now_playing) g_selected_file = current_index;

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
            if (show_now_playing) g_selected_file = current_index;

        } else if (r == PLAY_RESULT_SELECTED) {
            current_index = g_selected_file;
        }
        if (!show_now_playing) {
            update_mp3_bottom_bar_left(mp3_hint_left_obj, g_playlist[current_index]);
        } else {
            draw_now_playing(mp3_list_obj);
        }
    }

    f_unmount(pSD->pcName);
    // If you ever want to free playlist:
    // if (g_playlist) { free(g_playlist); g_playlist = NULL; g_playlist_cap = 0; }
    // if (g_shuffle_order) { free(g_shuffle_order); g_shuffle_order = NULL; g_shuffle_pos = 0; }
}
