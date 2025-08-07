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
