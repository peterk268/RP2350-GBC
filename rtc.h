#include <time.h>

// Global variable to store the current time in milliseconds
uint64_t current_time_ms = 0;

// Example: Default value to indicate unset RTC
#define RTC_UNSET_VALUE 0

// Example: Default value to indicate unset RTC
#define RTC_DEFAULT_VALUE 1736050180000

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
