#include <time.h>

// Global variable to store the current time in milliseconds
uint64_t current_time_ms = 0;

// Example: Default value to indicate unset RTC
#define RTC_UNSET_VALUE 0

// Example: Default value to indicate unset RTC
// For pokemon silver this is the time since North American release date (Oct 14, 2000)
// plus a day - 10 minutes
#warning "Make rtc a flash variable or idk store it in rtc"
#define RTC_DEFAULT_VALUE 767232000000 + 85800000

// Function to check if the RTC timer was set
bool is_rtc_set() {
    uint64_t rtc_time = powman_timer_get_ms();
    return (rtc_time != RTC_UNSET_VALUE);
}

// Function to initialize RTC timer if unset
void initialize_rtc(uint64_t initial_time) {
    if (!is_rtc_set()) {
        powman_timer_start();
        powman_timer_set_ms(initial_time);
    }
}

// Function to read the AON Timer's counter and update the global time variable
void update_current_time() {
    // Read the 64-bit counter value from the AON Timer
    uint64_t aon_timer_count = powman_timer_get_ms();

    // Update the global time variable
    current_time_ms = aon_timer_count;
}

// Function to convert milliseconds to struct tm
void ms_to_struct_tm(uint64_t ms, struct tm *time_info) {
    time_t seconds = ms / 1000;
    gmtime_r(&seconds, time_info);
    // printf("Converted time: Year=%d, Mon=%d, Day=%d, Hour=%d, Min=%d, Sec=%d\n",
    //     time_info->tm_year + 1900, time_info->tm_mon + 1, time_info->tm_mday,
    //     time_info->tm_hour, time_info->tm_min, time_info->tm_sec);
}

// Function to synchronize the Game Boy RTC with the current time
void synchronize_gb_rtc(struct gb_s *gb) {
    // Update the current time
    update_current_time();

    // Convert milliseconds to struct tm
    struct tm time_info;
    ms_to_struct_tm(current_time_ms, &time_info);

    // Set the Game Boy RTC
    gb_set_rtc(gb, &time_info);
}

// Function used after gameplay is done. Since rp2350 timer is not capable of understanding the frameskip feature??
// uint64_t gb_rtc_to_ms(const struct gb_s *gb) {
//     struct tm time_info = {0};

//     // Retrieve the RTC values from the Game Boy structure
//     time_info.tm_sec = gb->cart_rtc[0];
//     time_info.tm_min = gb->cart_rtc[1];
//     time_info.tm_hour = gb->cart_rtc[2];
//     time_info.tm_yday = (gb->cart_rtc[4] << 8) | gb->cart_rtc[3];

//     // Assume the year starts from 0 (tm_year should be relative to 1900)
//     // Customize if your system requires a specific base year
//     time_info.tm_year = 70; // 1970 as the Unix epoch
//     time_info.tm_mday = 1;  // Start day of the year

//     // Convert tm structure to time_t (seconds since the Unix epoch)
//     time_t seconds_since_epoch = mktime(&time_info);

//     if (seconds_since_epoch == -1) {
//         // Handle mktime failure
//         return 0;
//     }

//     // Convert seconds to milliseconds
//     return (uint64_t)seconds_since_epoch * 1000;
// }


// MARK: HARDWARE RTC
#define MCP7940N_I2C_ADDR 0x6F

#define MCP7940N_REG_SECONDS   0x00
#define MCP7940N_REG_MINUTES   0x01
#define MCP7940N_REG_HOURS     0x02
#define MCP7940N_REG_WEEKDAY   0x03
#define MCP7940N_REG_DATE      0x04
#define MCP7940N_REG_MONTH     0x05
#define MCP7940N_REG_YEAR      0x06
#define MCP7940N_REG_CONTROL   0x07
#define MCP7940N_REG_OSCTRIM   0x08

#define VBATEN_BIT   (1 << 3)
#define VBAT_BIT     (1 << 4) // set by hardware when running on VBAT

static inline uint8_t bcd_to_dec(uint8_t val) {
    return ((val >> 4) * 10) + (val & 0x0F);
}

static inline uint8_t dec_to_bcd(uint8_t val) {
    return ((val / 10) << 4) | (val % 10);
}

bool mcp7940n_write_register(i2c_inst_t *i2c, uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_write_blocking(i2c, MCP7940N_I2C_ADDR, buf, 2, false) == 2;
}

bool mcp7940n_read_register(i2c_inst_t *i2c, uint8_t reg, uint8_t *data) {
    if (i2c_write_blocking(i2c, MCP7940N_I2C_ADDR, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(i2c, MCP7940N_I2C_ADDR, data, 1, false) == 1;
}

// Enable VBAT (preserve other bits in RTCWKDAY)
bool mcp7940n_enable_vbat(i2c_inst_t *i2c) {
    uint8_t wkday;
    if (!mcp7940n_read_register(i2c, MCP7940N_REG_WEEKDAY, &wkday)) return false;
    wkday |= VBATEN_BIT;
    return mcp7940n_write_register(i2c, MCP7940N_REG_WEEKDAY, wkday);
}

// Check if VBATEN is set
bool mcp7940n_is_vbaten_enabled(i2c_inst_t *i2c, bool *out_enabled) {
    uint8_t wkday;
    if (!mcp7940n_read_register(i2c, MCP7940N_REG_WEEKDAY, &wkday)) return false;
    *out_enabled = !!(wkday & VBATEN_BIT);
    return true;
}

// Optional: check if the device currently reports it is on VBAT (bit set by hardware)
bool mcp7940n_is_running_on_vbat(i2c_inst_t *i2c, bool *out_on_vbat) {
    uint8_t wkday;
    if (!mcp7940n_read_register(i2c, MCP7940N_REG_WEEKDAY, &wkday)) return false;
    *out_on_vbat = !!(wkday & VBAT_BIT);
    return true;
}

#define MCP7940N_BIT_OSCON   (1 << 5)  // Oscillator running
#define MCP7940N_BIT_VBATEN  (1 << 3)  // Battery backup enabled

bool mcp7940n_time_is_valid(i2c_inst_t *i2c, bool *valid) {
    uint8_t wkday;
    if (!mcp7940n_read_register(i2c, MCP7940N_REG_WEEKDAY, &wkday)) {
        return false; // I2C failed
    }

    // If OSCON = 1 → time is valid (oscillator running)
    *valid = (wkday & MCP7940N_BIT_OSCON) != 0;
    return true;
}

bool mcp7940n_init(i2c_inst_t *i2c) {
    uint8_t seconds;
    if (!mcp7940n_read_register(i2c, MCP7940N_REG_SECONDS, &seconds)) return false;
    
    mcp7940n_enable_vbat(RTC_I2C_PORT);
    // Set ST (start oscillator) bit (bit 7)
    seconds |= 0x80;
    return mcp7940n_write_register(i2c, MCP7940N_REG_SECONDS, seconds);
}
typedef enum {
    RTC_SUNDAY    = 1,
    RTC_MONDAY    = 2,
    RTC_TUESDAY   = 3,
    RTC_WEDNESDAY = 4,
    RTC_THURSDAY  = 5,
    RTC_FRIDAY    = 6,
    RTC_SATURDAY  = 7
} rtc_weekday_t;

typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;     // 24-hour
    rtc_weekday_t weekday;   // 1=Sunday, 7=Saturday
    uint8_t date;
    uint8_t month;
    uint8_t year;      // 0-99
} rtc_time_t;

bool mcp7940n_set_time(i2c_inst_t *i2c, rtc_time_t *t) {
    uint8_t buf[8];
    buf[0] = MCP7940N_REG_SECONDS;
    buf[1] = dec_to_bcd(t->seconds) | 0x80; // ST bit
    buf[2] = dec_to_bcd(t->minutes);
    buf[3] = dec_to_bcd(t->hours);
    buf[4] = dec_to_bcd(t->weekday);
    buf[5] = dec_to_bcd(t->date);
    buf[6] = dec_to_bcd(t->month);
    buf[7] = dec_to_bcd(t->year);
    
    bool success = i2c_write_blocking(i2c, MCP7940N_I2C_ADDR, buf, 8, false) == 8;
    mcp7940n_init(RTC_I2C_PORT);
    return success;
}
#warning "Seems like it takes a couple of seconds to set in so keep in mind"

bool mcp7940n_set_time_if_unset(i2c_inst_t *i2c, rtc_time_t *t) {
    bool valid;
    if (!mcp7940n_time_is_valid(i2c, &valid)) return false;
    if (valid) return true; // Time is already valid, do nothing
    printf("RTC was unset, setting to default time\n");
    return mcp7940n_set_time(i2c, t);
}

void tm_to_rtc(const struct tm *tm_val, rtc_time_t *rtc) {
    rtc->seconds = tm_val->tm_sec;
    rtc->minutes = tm_val->tm_min;
    rtc->hours   = tm_val->tm_hour;
    rtc->date    = tm_val->tm_mday;
    rtc->month   = tm_val->tm_mon + 1;      // tm_mon: 0-11
    rtc->year    = tm_val->tm_year - 100;   // tm_year: since 1900, RTC uses 0-99
    rtc->weekday = (tm_val->tm_wday == 0) ? 1 : tm_val->tm_wday; // RTC: 1=Sun
}

bool mcp7940n_set_tm(i2c_inst_t *i2c, struct tm *in_tm) {
    rtc_time_t rtc_val;

    // Convert struct tm to RTC format
    tm_to_rtc(in_tm, &rtc_val);

    // Write to RTC
    if (!mcp7940n_set_time(i2c, &rtc_val)) {
        printf("Failed to set RTC time\n");
        return false;
    }
    return true;
}

bool mcp7940n_get_time(i2c_inst_t *i2c, rtc_time_t *t) {
    uint8_t buf[7];
    if (i2c_write_blocking(i2c, MCP7940N_I2C_ADDR, (uint8_t[]){MCP7940N_REG_SECONDS}, 1, true) != 1) return false;
    if (i2c_read_blocking(i2c, MCP7940N_I2C_ADDR, buf, 7, false) != 7) return false;

    t->seconds = bcd_to_dec(buf[0] & 0x7F);
    t->minutes = bcd_to_dec(buf[1]);
    t->hours   = bcd_to_dec(buf[2] & 0x3F);
    t->weekday = bcd_to_dec(buf[3] & 0x07);
    t->date    = bcd_to_dec(buf[4]);
    t->month   = bcd_to_dec(buf[5] & 0x1F);
    t->year    = bcd_to_dec(buf[6]);

    return true;
}

// Function to convert rtc_time_t -> struct tm
void rtc_to_tm(const rtc_time_t *rtc, struct tm *out_tm) {
    out_tm->tm_sec  = rtc->seconds;
    out_tm->tm_min  = rtc->minutes;
    out_tm->tm_hour = rtc->hours;
    out_tm->tm_mday = rtc->date;
    out_tm->tm_mon  = rtc->month - 1;       // tm_mon: 0-11
    out_tm->tm_year = rtc->year + 100;      // tm_year: years since 1900
    out_tm->tm_wday = rtc->weekday % 7;     // tm_wday: 0=Sunday, 6=Saturday

    // Normalize and calculate tm_yday, etc.
    mktime(out_tm);
}

// Updated function
bool mcp7940n_get_tm(i2c_inst_t *i2c, struct tm *out_tm) {
    rtc_time_t now;
    if (!mcp7940n_get_time(i2c, &now)) {
        return false; // Failed to read RTC
    }

    // Convert RTC time to struct tm
    rtc_to_tm(&now, out_tm);

    // Optional: print for debugging
    printf("RTC Time: %04d-%02d-%02d %02d:%02d:%02d (Day of year: %d)\n",
           out_tm->tm_year + 1900,
           out_tm->tm_mon + 1,
           out_tm->tm_mday,
           out_tm->tm_hour,
           out_tm->tm_min,
           out_tm->tm_sec,
           out_tm->tm_yday);

    return true;
}

struct tm adjust_tm_for_gbc(const struct tm *input) {
    struct tm t = *input; // Copy original

    // Ensure tm_yday is valid
    if (t.tm_yday > 365) t.tm_yday = 0;

    // Day of year → GB RTC day counter (9-bit)
    uint16_t day_counter = t.tm_yday & 0x1FF;

    // Adjust weekday: GB RTC expects 0 = Saturday
    uint8_t weekday_gb = (day_counter + 6) % 7;
    t.tm_wday = weekday_gb;

    // Hours: ensure 24-hour mode
    t.tm_hour &= 0x3F;

    // Print the adjusted time for debugging
    printf("Adjusted TM: %04d-%02d-%02d %02d:%02d:%02d, yday=%d, wday=%d\n",
           t.tm_year + 1900,
           t.tm_mon + 1,
           t.tm_mday,
           t.tm_hour,
           t.tm_min,
           t.tm_sec,
           t.tm_yday,
           t.tm_wday);



    return t;
}


void sync_gb_rtc(struct gb_s *gb) {
	struct tm t;
	if (mcp7940n_get_tm(RTC_I2C_PORT, &t)) {
		gb_set_rtc(gb, &t); // Feed it directly
		printf("GBC RTC Set\n");
	} else {
		printf("Failed to read RTC\n");
	}
}