#ifndef GMETER_DASHBOARD_H
#define GMETER_DASHBOARD_H

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------------------------------------
// Display defaults (override from your build system if needed)
// ----------------------------------------------------
#ifndef DISP_HOR_RES
#define DISP_HOR_RES 160
#endif

#ifndef DISP_VER_RES
#define DISP_VER_RES 144
#endif

// ----------------------------------------------------
// G-meter configuration
// ----------------------------------------------------
#ifndef GMETER_ACCENT_COLOR_HEX
// Default accent: blue
#define GMETER_ACCENT_COLOR_HEX 0x007BFF
#endif

#define GMETER_RING_COLOR        lv_color_hex(0x1F50FF)
#define GMETER_BG_COLOR          lv_color_hex(0x000000)
#define GMETER_ACCENT_COLOR      lv_color_hex(0xFFFFFF)
#define GMETER_TEXT_COLOR        lv_color_hex(0xFFFFFF)

// Outer ring represents ±1 g
#define GMETER_MAX_G             0.6f
// Update period: 20ms → 50 FPS
#define GMETER_UPDATE_MS         20
// Simple low-pass smoothing
#define GMETER_SMOOTH_ALPHA      0.30f

#define ACCEL_BAR_FULLSCALE_G   0.6f   // or whatever max g you want

// ----------------------------------------------------
// UI state
// ----------------------------------------------------
typedef struct {
    lv_obj_t  *screen;

    lv_obj_t  *map_bg;       // Square map background
    lv_obj_t  *ring_05g;
    lv_obj_t  *ring_075g;
    lv_obj_t  *ring_10g;

    lv_obj_t  *ball;

    lv_obj_t  *label_values; // "X:+0.12g  Z:-0.34g  Y:+0.98g"

    lv_coord_t map_size;
    lv_coord_t map_center_x;
    lv_coord_t map_center_y;
    lv_coord_t ball_radius;

    lv_obj_t *accel_bar_bg;   // outline
    lv_obj_t *accel_bar_fill; // fill rectangle
    lv_coord_t accel_bar_width;
    lv_coord_t accel_bar_height;
    lv_coord_t accel_bar_center_x;

    lv_obj_t *trail_layer;   // container for trail ghost dots

    #define TRAIL_COUNT 20

    lv_obj_t *trail_dots[TRAIL_COUNT];
    uint8_t trail_opacity[TRAIL_COUNT];
    uint8_t trail_index;
    lv_color_t trail_color[TRAIL_COUNT];

    lv_obj_t *g_indicator;   // horizontal g-force line

} gmeter_ui_t;

// Single global instance
static gmeter_ui_t g_gmeter_ui = {0};

// ----------------------------------------------------
// Internal helpers
// ----------------------------------------------------

// Create the LVGL screen and all objects
static void gmeter_create_screen(void) {
    lv_init();

    // Display driver setup
    static lv_disp_draw_buf_t draw_buf;
    lv_disp_draw_buf_init(&draw_buf, lv_buf1, NULL, DISP_HOR_RES * LV_BUF_LINES);

    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISP_HOR_RES;
    disp_drv.ver_res = DISP_VER_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
	///

    gmeter_ui_t *ui = &g_gmeter_ui;

    // Create a fresh screen
    ui->screen = lv_obj_create(NULL);
    lv_obj_set_size(ui->screen, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_set_style_bg_color(ui->screen, GMETER_BG_COLOR, 0);
    lv_obj_set_style_bg_opa(ui->screen, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ui->screen, 0, 0);

    // Decide map size: leave some space at bottom for text (about 24 px)
    lv_coord_t max_square = DISP_VER_RES - 24;
    if (max_square > DISP_HOR_RES) {
        max_square = DISP_HOR_RES;
    }
    ui->map_size = max_square - 8; // small margin inside

    // Map background (square)
    ui->map_bg = lv_obj_create(ui->screen);
    lv_obj_set_size(ui->map_bg, ui->map_size, ui->map_size);
    lv_obj_set_style_bg_color(ui->map_bg, GMETER_BG_COLOR, 0);
    lv_obj_set_style_bg_opa(ui->map_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ui->map_bg, 1, 0);
    lv_obj_set_style_border_color(ui->map_bg, GMETER_BG_COLOR, 0);
    lv_obj_set_style_radius(ui->map_bg, 0, 0); // keep square
    lv_obj_set_scrollbar_mode(ui->map_bg, LV_SCROLLBAR_MODE_OFF);

    // Align map near top center with small margin
    lv_obj_align(ui->map_bg, LV_ALIGN_TOP_MID, 0, 4);

    ui->map_center_x = DISP_HOR_RES / 2;
    ui->map_center_y = 4 + (ui->map_size / 2);

    // Outer radius is 1g ring
    lv_coord_t outer_r = (ui->map_size / 2) - 2;
    lv_coord_t r05  = (lv_coord_t)((float)outer_r * 0.5f);
    lv_coord_t r075 = (lv_coord_t)((float)outer_r * 0.75f);
    lv_coord_t r10  = outer_r;

    // Utility lambda-ish macro
#define GMETER_MAKE_RING(dst, radius)                      \
    do {                                                   \
        dst = lv_obj_create(ui->screen);                   \
        lv_obj_set_size(dst, (radius) * 2, (radius) * 2);  \
        lv_obj_set_style_bg_opa(dst, LV_OPA_TRANSP, 0);    \
        lv_obj_set_style_border_width(dst, 1, 0);          \
        lv_obj_set_style_border_color(dst, GMETER_RING_COLOR, 0); \
        lv_obj_set_style_radius(dst, LV_RADIUS_CIRCLE, 0); \
        lv_obj_set_scrollbar_mode(dst, LV_SCROLLBAR_MODE_OFF); \
        lv_obj_set_pos(dst,                                \
                       ui->map_center_x - (radius),        \
                       ui->map_center_y - (radius));       \
    } while (0)

    GMETER_MAKE_RING(ui->ring_05g,  r05);
    GMETER_MAKE_RING(ui->ring_075g, r075);
    GMETER_MAKE_RING(ui->ring_10g,  r10);

#undef GMETER_MAKE_RING

    // --- Axis lines (Ford GT style) ---
    lv_obj_t *axis_h = lv_obj_create(ui->screen);
    lv_obj_set_size(axis_h, outer_r * 2, 1);
    lv_obj_set_style_bg_color(axis_h, GMETER_RING_COLOR, 0);
    lv_obj_set_style_bg_opa(axis_h, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(axis_h, 0, 0);
    lv_obj_set_pos(axis_h, ui->map_center_x - outer_r, ui->map_center_y);

    lv_obj_t *axis_v = lv_obj_create(ui->screen);
    lv_obj_set_size(axis_v, 1, outer_r * 2);
    lv_obj_set_style_bg_color(axis_v, GMETER_RING_COLOR, 0);
    lv_obj_set_style_bg_opa(axis_v, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(axis_v, 0, 0);
    lv_obj_set_pos(axis_v, ui->map_center_x, ui->map_center_y - outer_r);


    // --- Gravity / vertical G indicator line ---
    ui->g_indicator = lv_obj_create(ui->screen);

    // Half the 0.5g diameter = r05
    lv_coord_t g_len = r05;   // r05 is radius of the 0.5g ring

    lv_obj_set_size(ui->g_indicator, g_len, 2);
    lv_obj_set_style_bg_color(ui->g_indicator, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_bg_opa(ui->g_indicator, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ui->g_indicator, 0, 0);
    lv_obj_set_style_radius(ui->g_indicator, 1, 0);

    // Start centered (neutral G)
    lv_obj_set_pos(
        ui->g_indicator,
        ui->map_center_x - (g_len / 2),
        ui->map_center_y - 1
    );

    // // --- Tick marks at former 0.75g ring location ---
    // lv_coord_t rtick = r075;  // where the tick ring used to be

    // // RIGHT tick (+X)
    // lv_obj_t *tick_r = lv_obj_create(ui->screen);
    // lv_obj_set_size(tick_r, 1, 6);  // vertical, longer
    // lv_obj_set_style_bg_color(tick_r, GMETER_RING_COLOR, 0);
    // lv_obj_set_style_bg_opa(tick_r, LV_OPA_COVER, 0);
    // lv_obj_set_pos(tick_r,
    //     ui->map_center_x + rtick - 1,
    //     ui->map_center_y - 3
    // );

    // // LEFT tick (−X)
    // lv_obj_t *tick_l = lv_obj_create(ui->screen);
    // lv_obj_set_size(tick_l, 1, 6);  // vertical
    // lv_obj_set_style_bg_color(tick_l, GMETER_RING_COLOR, 0);
    // lv_obj_set_style_bg_opa(tick_l, LV_OPA_COVER, 0);
    // lv_obj_set_pos(tick_l,
    //     ui->map_center_x - rtick,
    //     ui->map_center_y - 3
    // );

    // // UP tick (+Z)
    // lv_obj_t *tick_u = lv_obj_create(ui->screen);
    // lv_obj_set_size(tick_u, 6, 1);  // horizontal
    // lv_obj_set_style_bg_color(tick_u, GMETER_RING_COLOR, 0);
    // lv_obj_set_style_bg_opa(tick_u, LV_OPA_COVER, 0);
    // lv_obj_set_pos(tick_u,
    //     ui->map_center_x - 3,
    //     ui->map_center_y - rtick - 1
    // );

    // // DOWN tick (−Z)
    // lv_obj_t *tick_d = lv_obj_create(ui->screen);
    // lv_obj_set_size(tick_d, 6, 1);  // horizontal
    // lv_obj_set_style_bg_color(tick_d, GMETER_RING_COLOR, 0);
    // lv_obj_set_style_bg_opa(tick_d, LV_OPA_COVER, 0);
    // lv_obj_set_pos(tick_d,
    //     ui->map_center_x - 3,
    //     ui->map_center_y + rtick - 1
    // );

    // Trail layer (transparent container)
    ui->trail_layer = lv_obj_create(ui->screen);
    lv_obj_set_size(ui->trail_layer, DISP_HOR_RES, DISP_VER_RES);
    lv_obj_set_style_bg_opa(ui->trail_layer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ui->trail_layer, 0, 0);
    lv_obj_set_scrollbar_mode(ui->trail_layer, LV_SCROLLBAR_MODE_OFF);
    lv_obj_move_foreground(ui->trail_layer); // put above rings, under ball

    ui->trail_index = 0;

    for (int i = 0; i < TRAIL_COUNT; i++) {
        ui->trail_dots[i] = lv_obj_create(ui->trail_layer);
        lv_obj_set_size(ui->trail_dots[i], 3, 3);
        lv_obj_set_style_radius(ui->trail_dots[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(ui->trail_dots[i], 0, 0);
        lv_obj_set_style_bg_color(ui->trail_dots[i], GMETER_ACCENT_COLOR, 0);
        lv_obj_set_style_bg_opa(ui->trail_dots[i], LV_OPA_TRANSP, 0);
        lv_obj_set_pos(ui->trail_dots[i], -10, -10); // off-screen
        ui->trail_opacity[i] = 0;
    }

    // G-ball
    ui->ball = lv_obj_create(ui->screen);
    ui->ball_radius = 5; // 10px ball
    lv_coord_t ball_d = ui->ball_radius * 2;

    lv_obj_set_size(ui->ball, ball_d, ball_d);
    lv_obj_set_style_bg_color(ui->ball, GMETER_ACCENT_COLOR, 0);
    lv_obj_set_style_bg_opa(ui->ball, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ui->ball, 0, 0);
    lv_obj_set_style_radius(ui->ball, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_scrollbar_mode(ui->ball, LV_SCROLLBAR_MODE_OFF);

    // Start ball at center
    lv_obj_set_pos(ui->ball,
                   ui->map_center_x - ui->ball_radius,
                   ui->map_center_y - ui->ball_radius);

    // Bottom label for numeric values (X/Z/Y g's)
    ui->label_values = lv_label_create(ui->screen);
    lv_obj_set_style_text_color(ui->label_values, GMETER_TEXT_COLOR, 0);
    lv_obj_set_style_text_font(ui->label_values, LV_FONT_DEFAULT, 0);
    lv_label_set_text(ui->label_values, "X:+0.00g  Z:+0.00g  Y:+0.00g");
    lv_obj_align(ui->label_values, LV_ALIGN_BOTTOM_MID, 0, -2);


    // --- Acceleration Bar (BMW M style) ---
    ui->accel_bar_width = DISP_HOR_RES - 20;   // full width minus 10px margins
    ui->accel_bar_height = 6;                  // slim sport bar
    ui->accel_bar_center_x = DISP_HOR_RES / 2;

    ui->accel_bar_bg = lv_obj_create(ui->screen);
    lv_obj_set_size(ui->accel_bar_bg, ui->accel_bar_width, ui->accel_bar_height);
    lv_obj_set_style_bg_opa(ui->accel_bar_bg, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(ui->accel_bar_bg, 1, 0);
    lv_obj_set_style_border_color(ui->accel_bar_bg, GMETER_RING_COLOR, 0);
    lv_obj_set_style_radius(ui->accel_bar_bg, 2, 0);

    // position it slightly above the text line
    lv_obj_align(ui->accel_bar_bg, LV_ALIGN_BOTTOM_MID, 0, -14);

    // bar fill (starts centered)
    ui->accel_bar_fill = lv_obj_create(ui->screen);
    lv_obj_set_style_bg_color(ui->accel_bar_fill, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_bg_opa(ui->accel_bar_fill, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(ui->accel_bar_fill, 0, 0);
    lv_obj_set_style_radius(ui->accel_bar_fill, 2, 0);

    // Start at zero width
    lv_obj_set_size(ui->accel_bar_fill, 0, ui->accel_bar_height);
    lv_obj_set_pos(ui->accel_bar_fill, 
                ui->accel_bar_center_x, 
                lv_obj_get_y(ui->accel_bar_bg));

    // --- ACCEL / BRAKE labels ---
    lv_obj_t *label_accel = lv_label_create(ui->screen);
    lv_label_set_text(label_accel, "ACCEL");
    lv_obj_set_style_text_color(label_accel, lv_color_hex(0x00FF00), 0);
    lv_obj_set_style_text_font(label_accel, LV_FONT_DEFAULT, 0);
    lv_obj_align_to(label_accel, ui->accel_bar_bg, LV_ALIGN_OUT_TOP_RIGHT, -2, -2);

    lv_obj_t *label_brake = lv_label_create(ui->screen);
    lv_label_set_text(label_brake, "BRAKE");
    lv_obj_set_style_text_color(label_brake, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_text_font(label_brake, LV_FONT_DEFAULT, 0);
    lv_obj_align_to(label_brake, ui->accel_bar_bg, LV_ALIGN_OUT_TOP_LEFT, 2, -2);

    // Load this screen
    lv_scr_load(ui->screen);
}

static void gmeter_update_trail(lv_coord_t x, lv_coord_t y, lv_color_t c) {
    gmeter_ui_t *ui = &g_gmeter_ui;

    uint8_t i = ui->trail_index;
    ui->trail_index = (ui->trail_index + 1) % TRAIL_COUNT;

    // spawn new ghost
    ui->trail_opacity[i] = 200;
    ui->trail_color[i] = c;

    lv_obj_set_pos(ui->trail_dots[i], x, y);
    lv_obj_set_style_bg_color(ui->trail_dots[i], c, 0);
    lv_obj_set_style_bg_opa(ui->trail_dots[i], 200, 0);

    // fade existing ghosts
    for (int j = 0; j < TRAIL_COUNT; j++) {
        if (ui->trail_opacity[j] > 0) {
            ui->trail_opacity[j] -= 7; // fade rate
            lv_obj_set_style_bg_opa(ui->trail_dots[j], ui->trail_opacity[j], 0);
        }
    }
}


// Update ball position + numeric label
// gx and gz drive the ball; gy is numeric-only
static void gmeter_update_ui(float gx, float gz, float gy) {
    gmeter_ui_t *ui = &g_gmeter_ui;

    if (!ui->screen) {
        return;
    }

    // Update numeric label before capping
    char buf[64];
    // Show two decimals in g
    snprintf(buf, sizeof(buf),
             "X:%+0.2fg  Z:%+0.2fg  Y:%+0.2fg",
             (double)gx, (double)gz, (double)gy);
    lv_label_set_text(ui->label_values, buf);

    // Clamp magnitude to max ring
    float mag = sqrtf(gx * gx + gz * gz);
    if (mag > GMETER_MAX_G && mag > 0.0f) {
        float scale = GMETER_MAX_G / mag;
        gx *= scale;
        gz *= scale;
    }

    // Convert g's to pixels inside outer ring
    lv_coord_t outer_r = (ui->map_size / 2) - 2;
    float px_per_g = (float)outer_r / GMETER_MAX_G;

    float dx = gx * px_per_g;
    float dz = gz * px_per_g;

    // --- Vertical G indicator (uses gy = gravity/up-down) ---
    {
        // Clamp gy to ±GMETER_MAX_G so it stays inside the outer ring
        float gy_clamped = gy;
        if (gy_clamped > GMETER_MAX_G)  gy_clamped = GMETER_MAX_G;
        if (gy_clamped < -GMETER_MAX_G) gy_clamped = -GMETER_MAX_G;

        // Convert gy to pixels (same scale as ball)
        float gy_px = gy_clamped * px_per_g;

        // LVGL Y axis grows downward, so subtract to move "up" for +gy
        lv_coord_t gy_pos = ui->map_center_y - (lv_coord_t)lrintf(gy_px);

        // Half-length of indicator (we set it in create_screen)
        lv_coord_t g_len = lv_obj_get_width(ui->g_indicator);

        // Center the line horizontally, slide it vertically
        lv_obj_set_pos(
            ui->g_indicator,
            ui->map_center_x - (g_len / 2),
            gy_pos - 1
        );
    }

    // LVGL Y axis grows downward, so subtract dz
    lv_coord_t ball_center_x = ui->map_center_x + (lv_coord_t)lrintf(dx);
    lv_coord_t ball_center_y = ui->map_center_y - (lv_coord_t)lrintf(dz);

    lv_color_t trail_c;

    // Decide color based on corrected Gs
    if (gz > 0.05f) {
        // Forward acceleration → GREEN
        trail_c = lv_color_hex(0x00FF00);
    }
    else if (gz < -0.05f) {
        // Braking → RED
        trail_c = lv_color_hex(0xFF0000);
    }
    else if (gx > 0.05f) {
        // Right lateral → BLUE
        trail_c = lv_color_hex(0x3388FF);
    }
    else if (gx < -0.05f) {
        // Left lateral → BLUE (same tone)
        trail_c = lv_color_hex(0x3388FF);
    }
    else {
        // Neutral zone → WHITE
        trail_c = lv_color_hex(0xFFFFFF);
    }

    // Spawn ghost at previous ball position
    gmeter_update_trail(
        ball_center_x - ui->ball_radius - 9,
        ball_center_y - ui->ball_radius - 9,
        trail_c
    );

    lv_obj_set_pos(ui->ball,
                   ball_center_x - ui->ball_radius,
                   ball_center_y - ui->ball_radius);

    // --- Acceleration Bar Update ---
    lv_coord_t max_w = g_gmeter_ui.accel_bar_width / 2;

    // Limit to ±0.6g (map scale)
    float z = gz / ACCEL_BAR_FULLSCALE_G;   // Normalize
    if (z > 1.0f) z = 1.0f;
    if (z < -1.0f) z = -1.0f;

    lv_coord_t bar_w = (lv_coord_t)(max_w * fabsf(z));

    // Choose position depending on sign
    lv_coord_t bg_y = lv_obj_get_y(g_gmeter_ui.accel_bar_bg);
    lv_coord_t center_x = g_gmeter_ui.accel_bar_center_x;

    // Move the bar either left or right
    if (z >= 0) {
        // accelerating → right, green
        lv_obj_set_style_bg_color(g_gmeter_ui.accel_bar_fill, lv_color_hex(0x00FF00), 0);

        lv_obj_set_pos(g_gmeter_ui.accel_bar_fill,
                    center_x,
                    bg_y);
    } else {
        // braking → left, red
        lv_obj_set_style_bg_color(g_gmeter_ui.accel_bar_fill, lv_color_hex(0xFF0000), 0);

        lv_obj_set_pos(g_gmeter_ui.accel_bar_fill,
                    center_x - bar_w,
                    bg_y);
    }

    // Set fill size
    lv_obj_set_size(g_gmeter_ui.accel_bar_fill, bar_w, g_gmeter_ui.accel_bar_height);

}

// ----------------------------------------------------
// Public API
// ----------------------------------------------------

// This takes over execution and runs the G-meter forever
// (like your MP3 example). Reset device to exit.
static inline void run_gmeter_dashboard(void) {

    // Init IMU (safe to call even if already done elsewhere)
    if (!imu_init()) {
        printf("IMU init failed, aborting G-meter dashboard.\n");
        return;
    }

    // Accel configuration
    imu_set_accel(IMU_ODR_104_HZ, IMU_ACCEL_SCALE_4G);

    gmeter_create_screen();

    float filt_gx = 0.0f;
    float filt_gy = 0.0f;
    float filt_gz = 0.0f;
    bool  first_sample = true;

    // OFFSETS FOR CALIBRATION (these are for the BALL axes)
    float offset_gy = 0.0f;   // Ball X movement axis
    float offset_gz = 0.0f;   // Ball Y movement axis
    float offset_gx = 0.0f;   // Ball Z movement axis
    bool prev_select = true;

    while (1) {
        watchdog_update();

        // ------------------------------
        // Handle SELECT calibration (active-low)
        // ------------------------------
        bool select_now = gpio_get(GPIO_B_SELECT);

        if (!select_now && prev_select) {
            // Save offsets on button press
            offset_gy = filt_gy;
            offset_gz = filt_gz;
            offset_gx = filt_gx;
            printf("G-meter calibrated!\n");
        }
        prev_select = select_now;

        // ------------------------------
        // Read IMU
        // ------------------------------
        float ax_ms2, ay_ms2, az_ms2;
        imu_read_accel_ms2(&ax_ms2, &ay_ms2, &az_ms2, IMU_ACCEL_SCALE_4G);

        // Convert to g units
        const float ONE_G = 9.80665f;
        float gx = ax_ms2 / ONE_G;  // used only for display
        float gy = ay_ms2 / ONE_G;  // THIS is your ball X axis
        float gz = az_ms2 / ONE_G;  // THIS is your ball Y axis

        // ------------------------------
        // Low-pass filter
        // ------------------------------
        if (first_sample) {
            filt_gx = gx;
            filt_gy = gy;
            filt_gz = gz;
            first_sample = false;
        } else {
            filt_gx += GMETER_SMOOTH_ALPHA * (gx - filt_gx);
            filt_gy += GMETER_SMOOTH_ALPHA * (gy - filt_gy);
            filt_gz += GMETER_SMOOTH_ALPHA * (gz - filt_gz);
        }

        // ------------------------------
        // Apply calibration offsets
        // ------------------------------
        float gx_corrected = filt_gy - offset_gy;   // horizontal ball motion
        float gz_corrected = -(filt_gz - offset_gz);   // vertical ball motion
        float gy_display   = filt_gx - offset_gx;   // numeric value (gravity axis)


        // Change ball color depending on acceleration direction
        if (gz_corrected > 0.05f) {
            // Accelerating → GREEN
            lv_obj_set_style_bg_color(g_gmeter_ui.ball, lv_color_hex(0x00FF00), 0);
        } 
        else if (gz_corrected < -0.05f) {
            // Braking → RED
            lv_obj_set_style_bg_color(g_gmeter_ui.ball, lv_color_hex(0xFF0000), 0);
        }
        else {
            // Neutral → WHITE (or blue if you prefer)
            lv_obj_set_style_bg_color(g_gmeter_ui.ball, GMETER_ACCENT_COLOR, 0);
        }

        // ------------------------------
        // Update UI (ball + numbers)
        // ------------------------------
        gmeter_update_ui(gx_corrected, gz_corrected, gy_display);

        // ------------------------------
        // Run LVGL manually
        // ------------------------------
        lv_tick_inc(GMETER_UPDATE_MS);
        lv_timer_handler();
        sleep_ms(GMETER_UPDATE_MS);
    }
}

#ifdef __cplusplus
}
#endif

#endif // GMETER_DASHBOARD_H
