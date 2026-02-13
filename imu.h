#ifndef IMU_H
#define IMU_H

/*
 *
 * - Device address: 0x6B (your define)
 * - Port: IMU_I2C_PORT (your define, default i2c1)
 */

#ifndef IMU_I2C_ADDRESS
#define IMU_I2C_ADDRESS 0x6B
#endif

#ifndef IMU_I2C_PORT
#define IMU_I2C_PORT i2c1
#endif

/* ---------- Register map (subset) ---------- */
#define IMU_REG_FUNC_CFG_ACCESS   0x01
#define IMU_REG_PIN_CTRL          0x02
#define IMU_REG_FIFO_CTRL1        0x07
#define IMU_REG_FIFO_CTRL2        0x08
#define IMU_REG_FIFO_CTRL3        0x09
#define IMU_REG_FIFO_CTRL4        0x0A
#define IMU_REG_COUNTER_BDR_REG1  0x0B
#define IMU_REG_COUNTER_BDR_REG2  0x0C
#define IMU_REG_INT1_CTRL         0x0D
#define IMU_REG_INT2_CTRL         0x0E
#define IMU_REG_WHO_AM_I          0x0F
#define IMU_REG_CTRL1_XL          0x10
#define IMU_REG_CTRL2_G           0x11
#define IMU_REG_CTRL3_C           0x12
#define IMU_REG_CTRL4_C           0x13
#define IMU_REG_CTRL5_C           0x14
#define IMU_REG_CTRL6_C           0x15
#define IMU_REG_CTRL7_G           0x16
#define IMU_REG_CTRL8_XL          0x17
#define IMU_REG_CTRL9_XL          0x18
#define IMU_REG_CTRL10_C          0x19

#define IMU_REG_ALL_INT_SRC       0x1A
#define IMU_REG_WAKE_UP_SRC       0x1B
#define IMU_REG_TAP_SRC           0x1C
#define IMU_REG_D6D_SRC           0x1D
#define IMU_REG_STATUS_REG        0x1E

#define IMU_REG_OUT_TEMP_L        0x20
#define IMU_REG_OUT_TEMP_H        0x21
#define IMU_REG_OUTX_L_G          0x22
#define IMU_REG_OUTX_H_G          0x23
#define IMU_REG_OUTY_L_G          0x24
#define IMU_REG_OUTY_H_G          0x25
#define IMU_REG_OUTZ_L_G          0x26
#define IMU_REG_OUTZ_H_G          0x27
#define IMU_REG_OUTX_L_A          0x28
#define IMU_REG_OUTX_H_A          0x29
#define IMU_REG_OUTY_L_A          0x2A
#define IMU_REG_OUTY_H_A          0x2B
#define IMU_REG_OUTZ_L_A          0x2C
#define IMU_REG_OUTZ_H_A          0x2D

#define IMU_REG_TAP_CFG0          0x56
#define IMU_REG_TAP_CFG1          0x57
#define IMU_REG_TAP_CFG2          0x58
#define IMU_REG_TAP_THS_6D        0x59
#define IMU_REG_INT_DUR2          0x5A
#define IMU_REG_WAKE_UP_THS       0x5B
#define IMU_REG_WAKE_UP_DUR       0x5C
#define IMU_REG_FREE_FALL         0x5D
#define IMU_REG_MD1_CFG           0x5E
#define IMU_REG_MD2_CFG           0x5F

#define IMU_REG_X_OFS_USR         0x73
#define IMU_REG_Y_OFS_USR         0x74
#define IMU_REG_Z_OFS_USR         0x75

/* ---------- WHO_AM_I ---------- */
#define IMU_WHO_AM_I_EXPECTED     0x6C

/* ---------- WAKE_UP_SRC bits (Table in AN / datasheet) ---------- */
/* b5 SLEEP_STATE, b4 FF_IA, b3 WU_IA, b2 Z_WU, b1 Y_WU, b0 X_WU */
#define IMU_WAKE_X_WU             (1u << 0)
#define IMU_WAKE_Y_WU             (1u << 1)
#define IMU_WAKE_Z_WU             (1u << 2)
#define IMU_WAKE_WU_IA            (1u << 3)
#define IMU_WAKE_FF_IA            (1u << 4)
#define IMU_WAKE_SLEEP_STATE      (1u << 5)

/* ---------- TAP_SRC bits (Table 33 in AN) ---------- */
/* b6 TAP_IA, b5 SINGLE_TAP, b4 DOUBLE_TAP, b3 TAP_SIGN, b2 X_TAP, b1 Y_TAP, b0 Z_TAP */
#define IMU_TAP_Z_TAP             (1u << 0)
#define IMU_TAP_Y_TAP             (1u << 1)
#define IMU_TAP_X_TAP             (1u << 2)
#define IMU_TAP_SIGN              (1u << 3)
#define IMU_TAP_DOUBLE_TAP        (1u << 4)
#define IMU_TAP_SINGLE_TAP        (1u << 5)
#define IMU_TAP_TAP_IA            (1u << 6)

/* ---------- Simple scale enums ---------- */
typedef enum {
    IMU_ACCEL_SCALE_4G  = 0b00,  // FS_XL = ±4 g
    IMU_ACCEL_SCALE_8G  = 0b10,  // ±8 g
    IMU_ACCEL_SCALE_16G = 0b11,  // ±16 g
    IMU_ACCEL_SCALE_32G = 0b01   // ±32 g (LSM6DSO32-only)
} imu_accel_scale_t;

typedef enum {
    IMU_GYRO_SCALE_250DPS  = 0b00,
    IMU_GYRO_SCALE_500DPS  = 0b01,
    IMU_GYRO_SCALE_1000DPS = 0b10,
    IMU_GYRO_SCALE_2000DPS = 0b11
} imu_gyro_scale_t;

typedef enum {
    IMU_ODR_POWER_DOWN = 0b0000,
    IMU_ODR_12_5_HZ    = 0b0001,
    IMU_ODR_26_HZ      = 0b0010,
    IMU_ODR_52_HZ      = 0b0011,
    IMU_ODR_104_HZ     = 0b0100,
    IMU_ODR_208_HZ     = 0b0101,
    IMU_ODR_416_HZ     = 0b0110,
    IMU_ODR_833_HZ     = 0b0111,
    IMU_ODR_1667_HZ    = 0b1000,
    IMU_ODR_3333_HZ    = 0b1001,
    IMU_ODR_6667_HZ    = 0b1010,
    IMU_ODR_1_6_HZ     = 0b1011  // accel only
} imu_odr_t;

/* ---------- Motion event summary ---------- */
typedef struct {
    bool wake_event;
    bool wake_x;
    bool wake_y;
    bool wake_z;
    bool free_fall;
    bool single_tap;
    bool double_tap;
    bool tap_x;
    bool tap_y;
    bool tap_z;
} imu_motion_event_t;

/* ============================================================
 * Low-level I2C helpers
 * ============================================================ */

static inline int imu_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, buf, 2, false);
}

static inline int imu_write_regs(uint8_t reg, const uint8_t *data, size_t len) {
    uint8_t tmp[16];
    if (len + 1 > sizeof(tmp)) return PICO_ERROR_GENERIC;
    tmp[0] = reg;
    for (size_t i = 0; i < len; ++i) tmp[i + 1] = data[i];
    return i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, tmp, len + 1, false);
}

static inline int imu_read_reg(uint8_t reg, uint8_t *value) {
    int r = i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, &reg, 1, true);
    if (r < 0) return r;
    return i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, value, 1, false);
}

static inline int imu_read_regs(uint8_t reg, uint8_t *buf, size_t len) {
    int r = i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, &reg, 1, true);
    if (r < 0) return r;
    return i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, buf, len, false);
}

/* ============================================================
 * Core setup / basic config
 * ============================================================ */

/* Read WHO_AM_I, return true if it matches 0x6C */
static inline bool imu_check_whoami(void) {
    uint8_t v = 0;
    if (imu_read_reg(IMU_REG_WHO_AM_I, &v) < 0) return false;
    return v == IMU_WHO_AM_I_EXPECTED;
}

/*
 * Basic “run” configuration:
 * - accel: 104 Hz, ±4 g
 * - gyro : 104 Hz, ±2000 dps
 * - auto-increment, BDU set, I2C enabled
 */
static inline void imu_basic_config(void) {
    // CTRL3_C: BDU=1, IF_INC=1, I2C enabled, little-endian, 4-wire SPI
    // bit7 BOOT=0, bit6 BDU=1, bit5 H_LACTIVE=0, bit4 PP_OD=0,
    // bit3 SIM=0, bit2 IF_INC=1, bit1 I2C_DISABLE=0, bit0 SW_RESET=0
    imu_write_reg(IMU_REG_CTRL3_C, 0b01000100);

    // accel: ODR=104 Hz, FS=±4g (2 bits), LPF2 disabled
    uint8_t ctrl1_xl = (IMU_ODR_104_HZ << 4) | (IMU_ACCEL_SCALE_4G << 2);
    imu_write_reg(IMU_REG_CTRL1_XL, ctrl1_xl);

    // gyro: ODR=104 Hz, FS=±2000 dps
    uint8_t ctrl2_g = (IMU_ODR_104_HZ << 4) | (IMU_GYRO_SCALE_2000DPS << 2);
    imu_write_reg(IMU_REG_CTRL2_G, ctrl2_g);

    // Some sane defaults for filters (you can tweak later)
    imu_write_reg(IMU_REG_CTRL4_C, 0x00);
    imu_write_reg(IMU_REG_CTRL5_C, 0x00);
    imu_write_reg(IMU_REG_CTRL6_C, 0x00);
    imu_write_reg(IMU_REG_CTRL7_G, 0x00);
    imu_write_reg(IMU_REG_CTRL8_XL, 0x00);
    imu_write_reg(IMU_REG_CTRL9_XL, 0x00);
    imu_write_reg(IMU_REG_CTRL10_C, 0x00);
}

/*
 * Top-level init:
 * - assumes I2C pins + IMU_I2C_PORT already initialized.
 * - returns true on success.
 */
static inline bool imu_init(void) {
    if (!imu_check_whoami()) {
        return false;
    }
    imu_basic_config();
    return true;
}

/* ============================================================
 * Scale configuration + conversion helpers
 * ============================================================ */

/* Set accelerometer ODR and scale (FS_XL). */
static inline void imu_set_accel(imu_odr_t odr, imu_accel_scale_t scale) {
    uint8_t v = 0;
    imu_read_reg(IMU_REG_CTRL1_XL, &v);
    v &= 0b00001110;                 // keep LPF2 bit + reserved
    v |= ((uint8_t)odr << 4) | ((uint8_t)scale << 2);
    imu_write_reg(IMU_REG_CTRL1_XL, v);
}

/* Set gyro ODR and scale (FS_G). */
static inline void imu_set_gyro(imu_odr_t odr, imu_gyro_scale_t scale) {
    uint8_t v = 0;
    imu_read_reg(IMU_REG_CTRL2_G, &v);
    v &= 0b00001100;                 // keep reserved + 125dps bit
    v |= ((uint8_t)odr << 4) | ((uint8_t)scale << 2);
    imu_write_reg(IMU_REG_CTRL2_G, v);
}

/* Get accel LSB → m/s² scale for given FS */
static inline float imu_accel_lsb_to_ms2(imu_accel_scale_t scale) {
    // Datasheet sensitivity in mg/LSB, then * 9.80665 / 1000
    switch (scale) {
        case IMU_ACCEL_SCALE_4G:  return 0.00980665f * 0.122f;  // 122 mg/LSB
        case IMU_ACCEL_SCALE_8G:  return 0.00980665f * 0.244f;
        case IMU_ACCEL_SCALE_16G: return 0.00980665f * 0.488f;
        case IMU_ACCEL_SCALE_32G: return 0.00980665f * 0.976f;
        default:                  return 1.0f;
    }
}

/* Get gyro LSB → dps scale */
static inline float imu_gyro_lsb_to_dps(imu_gyro_scale_t scale) {
    // Sensitivity in mdps/LSB → dps/LSB
    switch (scale) {
        case IMU_GYRO_SCALE_250DPS:  return 8.75f  / 1000.0f;
        case IMU_GYRO_SCALE_500DPS:  return 17.5f  / 1000.0f;
        case IMU_GYRO_SCALE_1000DPS: return 35.0f  / 1000.0f;
        case IMU_GYRO_SCALE_2000DPS: return 70.0f  / 1000.0f;
        default:                     return 1.0f;
    }
}

/* ============================================================
 * Raw data reads
 * ============================================================ */

static inline void imu_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buf[6];
    imu_read_regs(IMU_REG_OUTX_L_A, buf, 6);
    *ax = (int16_t)((buf[1] << 8) | buf[0]);
    *ay = (int16_t)((buf[3] << 8) | buf[2]);
    *az = (int16_t)((buf[5] << 8) | buf[4]);
}

static inline void imu_read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[6];
    imu_read_regs(IMU_REG_OUTX_L_G, buf, 6);
    *gx = (int16_t)((buf[1] << 8) | buf[0]);
    *gy = (int16_t)((buf[3] << 8) | buf[2]);
    *gz = (int16_t)((buf[5] << 8) | buf[4]);
}

static inline int16_t imu_read_temp_raw(void) {
    uint8_t buf[2];
    imu_read_regs(IMU_REG_OUT_TEMP_L, buf, 2);
    return (int16_t)((buf[1] << 8) | buf[0]);
}

/* Simple conversions using current hard-coded scales (if you keep defaults) */
static inline void imu_read_accel_ms2(float *ax, float *ay, float *az,
                                      imu_accel_scale_t scale) {
    int16_t rax, ray, raz;
    imu_read_accel_raw(&rax, &ray, &raz);
    float s = imu_accel_lsb_to_ms2(scale);
    *ax = rax * s;
    *ay = ray * s;
    *az = raz * s;
}

static inline void imu_read_gyro_dps(float *gx, float *gy, float *gz,
                                     imu_gyro_scale_t scale) {
    int16_t rgx, rgy, rgz;
    imu_read_gyro_raw(&rgx, &rgy, &rgz);
    float s = imu_gyro_lsb_to_dps(scale);
    *gx = rgx * s;
    *gy = rgy * s;
    *gz = rgz * s;
}

/* ============================================================
 * Interrupt + motion-detection helpers
 * Uses ST app-note canned settings.
 * All of these **reconfigure the accelerometer** (ODR/FS/filter).
 * ============================================================ */

/* Read & clear ALL_INT_SRC (good generic clear) */
static inline uint8_t imu_read_all_int_src(void) {
    uint8_t v = 0;
    imu_read_reg(IMU_REG_ALL_INT_SRC, &v);
    return v;
}

/* Read & clear WAKE_UP_SRC */
static inline uint8_t imu_read_wake_src(void) {
    uint8_t v = 0;
    imu_read_reg(IMU_REG_WAKE_UP_SRC, &v);
    return v;
}

/* Read & clear TAP_SRC */
static inline uint8_t imu_read_tap_src(void) {
    uint8_t v = 0;
    imu_read_reg(IMU_REG_TAP_SRC, &v);
    return v;
}

/*
 * Decode motion events from WAKE_UP_SRC + TAP_SRC.
 * NOTE: this **clears** latched events because it reads those registers.
 */
static inline void imu_get_motion_events(imu_motion_event_t *ev) {
    uint8_t wake = imu_read_wake_src();
    uint8_t tap  = imu_read_tap_src();

    ev->wake_event = (wake & IMU_WAKE_WU_IA) != 0;
    ev->wake_x     = (wake & IMU_WAKE_X_WU)  != 0;
    ev->wake_y     = (wake & IMU_WAKE_Y_WU)  != 0;
    ev->wake_z     = (wake & IMU_WAKE_Z_WU)  != 0;
    ev->free_fall  = (wake & IMU_WAKE_FF_IA) != 0;

    ev->single_tap = (tap & IMU_TAP_SINGLE_TAP) != 0;
    ev->double_tap = (tap & IMU_TAP_DOUBLE_TAP) != 0;
    ev->tap_x      = (tap & IMU_TAP_X_TAP) != 0;
    ev->tap_y      = (tap & IMU_TAP_Y_TAP) != 0;
    ev->tap_z      = (tap & IMU_TAP_Z_TAP) != 0;
}

/* ---------- INT1/INT2 routing helpers (MD1_CFG / MD2_CFG) ---------- */
/* Bits in MD1/MD2_CFG (from AN):
 * b7 INTx_SLEEP_CHANGE
 * b6 INTx_SINGLE_TAP
 * b5 INTx_WU
 * b4 INTx_FF
 * b3 INTx_DOUBLE_TAP
 * b2 INTx_6D
 * b1 INTx_EMB_FUNC
 * b0 INTx_TIMESTAMP
 */

#define IMU_MD_INT_SLEEP_CHANGE  (1u << 7)
#define IMU_MD_INT_SINGLE_TAP    (1u << 6)
#define IMU_MD_INT_WU            (1u << 5)
#define IMU_MD_INT_FF            (1u << 4)
#define IMU_MD_INT_DOUBLE_TAP    (1u << 3)
#define IMU_MD_INT_6D            (1u << 2)
#define IMU_MD_INT_EMB_FUNC      (1u << 1)
#define IMU_MD_INT_TIMESTAMP     (1u << 0)

/* Set exactly which sources go to INT1 (MD1_CFG) */
static inline void imu_set_md1_cfg(uint8_t bits) {
    imu_write_reg(IMU_REG_MD1_CFG, bits);
}

/* Set exactly which sources go to INT2 (MD2_CFG) */
static inline void imu_set_md2_cfg(uint8_t bits) {
    imu_write_reg(IMU_REG_MD2_CFG, bits);
}

/* ============================================================
 * Preset: Wake-up on motion (INT1)
 *
 * From AN example:
 * 1. CTRL1_XL = 0x60  -> ODR_XL = 417 Hz, FS_XL = ±2 g
 * 2. TAP_CFG0 = 0x51  -> latch, reset-on-read, slope HPF
 * 3. TAP_CFG2 = 0x80  -> INTERRUPTS_ENABLE = 1
 * 4. WAKE_UP_DUR = 0x00
 * 5. WAKE_UP_THS = 0x02 (approx 62.5 mg threshold)
 * 6. MD1_CFG = 0x20   -> wake-up interrupt on INT1
 *
 * NOTE: this overwrites your accel config.
 * ============================================================ */
static inline void imu_preset_wake_on_motion_int1(void) {
    // 104 Hz, ±4g — MUCH lower noise than 417 Hz ±2g
    imu_write_reg(IMU_REG_CTRL1_XL, 0x44);

    // TAP_CFG0:
    // 0x40 = HP filter, latched interrupts, no tap axis enabled
    imu_write_reg(IMU_REG_TAP_CFG0, 0x40);

    // TAP_CFG2:
    // bit7 = INTERRUPTS_ENABLE, everything else off
    imu_write_reg(IMU_REG_TAP_CFG2, 0x80);

    // Disable tap entirely
    imu_write_reg(IMU_REG_TAP_CFG1, 0x00);
    imu_write_reg(IMU_REG_TAP_THS_6D, 0x00);
    imu_write_reg(IMU_REG_INT_DUR2, 0x00);

    // Wake-up threshold (~500 mg)
    imu_write_reg(IMU_REG_WAKE_UP_THS, 0x08);

    // Debounce duration (~80 ms)
    imu_write_reg(IMU_REG_WAKE_UP_DUR, 0x04);

    // Disable FREE-FALL
    imu_write_reg(IMU_REG_FREE_FALL, 0x00);

    // Route ONLY wake-up to INT1
    imu_write_reg(IMU_REG_MD1_CFG, IMU_MD_INT_WU);
}

/* ============================================================
 * Preset: Free-fall (INT1)
 *
 * From AN example:
 * 1. CTRL1_XL = 0x60  -> 417 Hz, ±2 g
 * 2. TAP_CFG0 = 0x41  -> latch mode + reset on read
 * 3. TAP_CFG2 = 0x80  -> INTERRUPTS_ENABLE
 * 4. WAKE_UP_DUR = 0x00
 * 5. FREE_FALL = 0x33 -> threshold/time (typical)
 * 6. MD1_CFG = 0x10   -> FF interrupt on INT1
 *
 * NOTE: overwrites accel config.
 * ============================================================ */
static inline void imu_preset_free_fall_int1(void) {
    imu_write_reg(IMU_REG_CTRL1_XL, 0x60);  // 417 Hz, ±2 g

    imu_write_reg(IMU_REG_TAP_CFG0, 0x41);  // latch + reset on read
    imu_write_reg(IMU_REG_TAP_CFG2, 0x80);  // enable interrupt function

    imu_write_reg(IMU_REG_WAKE_UP_DUR, 0x00);
    imu_write_reg(IMU_REG_FREE_FALL,   0x33);  // typical FF config (thresh / duration)

    imu_write_reg(IMU_REG_MD1_CFG, IMU_MD_INT_FF);
}

/* ============================================================
 * Preset: Single-tap detection (INT1)
 *
 * From AN example:
 * 1. CTRL1_XL = 0x60  -> 417 Hz, ±2 g
 * 2. TAP_CFG0 = 0x0E  -> enable tap on X,Y,Z
 * 3. TAP_CFG1 = 0x09  -> X threshold + priority
 * 4. TAP_CFG2 = 0x89  -> Y threshold + INTERRUPTS_ENABLE
 * 5. TAP_THS_6D = 0x09 -> Z threshold
 * 6. INT_DUR2 = 0x06  -> Quiet & Shock windows
 * 7. WAKE_UP_THS = 0x00 -> SINGLE_DOUBLE_TAP = 0 => only single-tap
 * 8. MD1_CFG = 0x40   -> single-tap on INT1
 *
 * NOTE: overwrites accel config.
 * ============================================================ */
static inline void imu_preset_single_tap_int1(void) {
    // Accel: 104 Hz, ±4g
    imu_write_reg(IMU_REG_CTRL1_XL, 0x44);

    // Tap detection on X/Y/Z + HP filter
    imu_write_reg(IMU_REG_TAP_CFG0, 0x0E);

    // Thresholds
    imu_write_reg(IMU_REG_TAP_CFG1, 0x03);   // X threshold
    imu_write_reg(IMU_REG_TAP_CFG2, 0x83);   // INTERRUPTS_ENABLE + Y threshold
    imu_write_reg(IMU_REG_TAP_THS_6D, 0x03); // Z threshold

    // Quiet + Shock → prevents false taps
    imu_write_reg(IMU_REG_INT_DUR2, 0x23);

    // Disable wake-up engine
    imu_write_reg(IMU_REG_WAKE_UP_THS, 0x00);
    imu_write_reg(IMU_REG_WAKE_UP_DUR, 0x00);
    imu_write_reg(IMU_REG_FREE_FALL,   0x00);

    // Route single tap only
    imu_write_reg(IMU_REG_MD1_CFG, IMU_MD_INT_SINGLE_TAP);
}

static inline void imu_preset_double_tap_int1(void) {
    // Accel: 104 Hz, ±4g
    imu_write_reg(IMU_REG_CTRL1_XL, 0x44);

    // Tap detection enabled on X/Y/Z
    imu_write_reg(IMU_REG_TAP_CFG0, 0x0E);

    // Thresholds
    imu_write_reg(IMU_REG_TAP_CFG1, 0x04);   // X
    imu_write_reg(IMU_REG_TAP_CFG2, 0x84);   // INTERRUPTS_ENABLE + Y
    imu_write_reg(IMU_REG_TAP_THS_6D, 0x04); // Z

    // Quiet + Shock + DUR tuned for double tap window
    imu_write_reg(IMU_REG_INT_DUR2, 0x46);   // Quiet=4, Shock=6 → very stable

    // Enable DOUBLE TAP (WU_THS bit = 1)
    imu_write_reg(IMU_REG_WAKE_UP_THS, 0x80);

    // Disable wake/freefall
    imu_write_reg(IMU_REG_WAKE_UP_DUR, 0x00);
    imu_write_reg(IMU_REG_FREE_FALL,   0x00);

    // Route double tap to INT1
    imu_write_reg(IMU_REG_MD1_CFG, IMU_MD_INT_DOUBLE_TAP);
}

static inline void imu_preset_shake_to_wake_int1(void) {
    // Accel: 104 Hz, ±4g
    imu_write_reg(IMU_REG_CTRL1_XL, 0x44);

    // HP filtered slope detection
    imu_write_reg(IMU_REG_TAP_CFG0, 0x40);

    // Enable interrupt engine
    imu_write_reg(IMU_REG_TAP_CFG2, 0x80);

    // Threshold: ~500 mg
    imu_write_reg(IMU_REG_WAKE_UP_THS, 0x08);

    // Debounce: ~80ms
    imu_write_reg(IMU_REG_WAKE_UP_DUR, 0x04);

    // Disable tap + freefall
    imu_write_reg(IMU_REG_TAP_CFG1, 0x00);
    imu_write_reg(IMU_REG_TAP_THS_6D, 0x00);
    imu_write_reg(IMU_REG_INT_DUR2, 0x00);
    imu_write_reg(IMU_REG_FREE_FALL, 0x00);

    // Route WU only
    imu_write_reg(IMU_REG_MD1_CFG, IMU_MD_INT_WU);
}

static inline void imu_preset_pick_up_device_int1(void) {
    // 104 Hz, ±4g
    imu_write_reg(IMU_REG_CTRL1_XL, 0x44);

    // Enable slope-based wake
    imu_write_reg(IMU_REG_TAP_CFG0, 0x40);

    // Engine on
    imu_write_reg(IMU_REG_TAP_CFG2, 0x80);

    // Very low threshold: ~250 mg
    imu_write_reg(IMU_REG_WAKE_UP_THS, 0x04);

    // Longer debounce to ignore table vibrations
    imu_write_reg(IMU_REG_WAKE_UP_DUR, 0x08);  // ~160ms

    // Disable tap/freefall
    imu_write_reg(IMU_REG_TAP_CFG1, 0x00);
    imu_write_reg(IMU_REG_TAP_THS_6D, 0x00);
    imu_write_reg(IMU_REG_INT_DUR2, 0x00);
    imu_write_reg(IMU_REG_FREE_FALL,   0x00);

    imu_write_reg(IMU_REG_MD1_CFG, IMU_MD_INT_WU);
}

static inline void imu_preset_pocket_detect_int1(void) {
    // Low-noise mode
    imu_write_reg(IMU_REG_CTRL1_XL, 0x44);

    // Enable HP slope detection
    imu_write_reg(IMU_REG_TAP_CFG0, 0x40);

    // Enable interrupt engine
    imu_write_reg(IMU_REG_TAP_CFG2, 0x80);

    // Threshold small, but debounce long
    imu_write_reg(IMU_REG_WAKE_UP_THS, 0x03);   // ~187 mg
    imu_write_reg(IMU_REG_WAKE_UP_DUR, 0x10);   // ~320 ms

    // Disable tap/freefall
    imu_write_reg(IMU_REG_TAP_CFG1, 0x00);
    imu_write_reg(IMU_REG_TAP_THS_6D, 0x00);
    imu_write_reg(IMU_REG_INT_DUR2, 0x00);
    imu_write_reg(IMU_REG_FREE_FALL, 0x00);

    // Send to INT1
    imu_write_reg(IMU_REG_MD1_CFG, IMU_MD_INT_WU);
}

static inline void imu_preset_device_dropped_int1(void) {
    // 104 Hz ±4g
    imu_write_reg(IMU_REG_CTRL1_XL, 0x44);

    // HP slope + latched
    imu_write_reg(IMU_REG_TAP_CFG0, 0x41);

    // Engine enable
    imu_write_reg(IMU_REG_TAP_CFG2, 0x80);

    // FREE-FALL settings:
    // DUR=3 (≈ 60 ms), THS=3 → stable, low false-positives
    imu_write_reg(IMU_REG_FREE_FALL, 0x33);

    // Disable wake/tap
    imu_write_reg(IMU_REG_WAKE_UP_THS, 0x00);
    imu_write_reg(IMU_REG_WAKE_UP_DUR, 0x00);
    imu_write_reg(IMU_REG_TAP_CFG1,    0x00);
    imu_write_reg(IMU_REG_TAP_THS_6D,  0x00);
    imu_write_reg(IMU_REG_INT_DUR2,    0x00);

    // Route FF to INT1
    imu_write_reg(IMU_REG_MD1_CFG, IMU_MD_INT_FF);
}

/* ============================================================
 * Small helpers for app-level use
 * ============================================================ */

/* Poll if any motion event happened (tap / wake / ff).
 * This ALSO clears latched interrupt flags in the IMU. */
static inline bool imu_poll_motion(imu_motion_event_t *ev) {
    imu_get_motion_events(ev);
    return ev->wake_event || ev->free_fall || ev->single_tap || ev->double_tap;
}

#endif /* IMU_H */


void imu_example(void) {
    imu_init();
    if (!imu_check_whoami()) {
        printf("IMU not found!\n");
        return;
    }

    // ---------------------------------------
    // Choose *ONE* motion preset:
    // ---------------------------------------
    // imu_preset_wake_on_motion_int1();
    // imu_preset_free_fall_int1();
    // imu_preset_single_tap_int1();     // <- example: single tap detection
    // ---------------------------------------

    float ax, ay, az, gx, gy, gz;

    // Current scale settings (from your init)
    imu_accel_scale_t a_scale = IMU_ACCEL_SCALE_4G;
    imu_gyro_scale_t  g_scale = IMU_GYRO_SCALE_2000DPS;

    while (1) {
        watchdog_update();

        // ---------------------------------------
        // 1. Poll IMU motion (tap, wake, free-fall)
        // ---------------------------------------
        imu_motion_event_t ev;
        if (imu_poll_motion(&ev)) {
            if (ev.single_tap) {
                printf("[IMU] Single Tap!\n");
            }
            if (ev.double_tap) {
                printf("[IMU] Double Tap!\n");
            }
            if (ev.wake_event) {
                printf("[IMU] Wake-up motion  X:%d Y:%d Z:%d\n",
                       ev.wake_x, ev.wake_y, ev.wake_z);
            }
            if (ev.free_fall) {
                printf("[IMU] Free-Fall detected!\n");
            }
        }

        // ---------------------------------------
        // 2. Regular accel/gyro readout
        // ---------------------------------------
        imu_read_accel_ms2(&ax, &ay, &az, a_scale);
        imu_read_gyro_dps(&gx, &gy, &gz, g_scale);

        printf("ACCEL: %.2f %.2f %.2f m/s²\n", ax, ay, az);
        // printf("GYRO : %.2f %.2f %.2f dps\n", gx, gy, gz);

        sleep_ms(200);
    }
}

/* ============================================================
 * Sleep / Wake helpers (power down sensors, keep I2C alive)
 * ============================================================ */

typedef struct {
    uint8_t ctrl1_xl;
    uint8_t ctrl2_g;
    uint8_t fifo_ctrl4;   // in your map this is 0x0A (you named FIFO_CTRL4 but it's FIFO_CTRL5 on some docs)
    uint8_t md1_cfg;
    uint8_t md2_cfg;
} imu_saved_state_t;

static imu_saved_state_t g_imu_saved;
static bool g_imu_saved_valid = false;

/* Save current run state (ODR/FS, FIFO ODR, MD routing) */
static inline void imu_save_state(void) {
    imu_read_reg(IMU_REG_CTRL1_XL, &g_imu_saved.ctrl1_xl);
    imu_read_reg(IMU_REG_CTRL2_G,  &g_imu_saved.ctrl2_g);
    imu_read_reg(IMU_REG_FIFO_CTRL4, &g_imu_saved.fifo_ctrl4); // your define = 0x0A
    imu_read_reg(IMU_REG_MD1_CFG,  &g_imu_saved.md1_cfg);
    imu_read_reg(IMU_REG_MD2_CFG,  &g_imu_saved.md2_cfg);
    g_imu_saved_valid = true;
}

/* Power down accel+gyro and stop FIFO clock. Keep I2C enabled. */
static inline void imu_sleep(void) {
    // If we can read the device, capture config so wake restores correctly
    if (imu_check_whoami()) {
        imu_save_state();
    }

    // Optionally stop routing motion interrupts while asleep (prevents noisy INT line)
    imu_write_reg(IMU_REG_MD1_CFG, 0x00);
    imu_write_reg(IMU_REG_MD2_CFG, 0x00);

    // Stop FIFO ODR (ODR_FIFO = 0000). We don't assume other bits, we just write 0.
    // If you rely on FIFO watermark mode etc, save_state() + wake() will restore it.
    imu_write_reg(IMU_REG_FIFO_CTRL4, 0x00);

    // Power-down accel + gyro by setting ODR fields to 0, preserving FS bits.
    uint8_t v;

    // CTRL1_XL: [7:4] ODR_XL, [3:2] FS_XL  -> keep FS, clear ODR
    imu_read_reg(IMU_REG_CTRL1_XL, &v);
    v &= 0x0F;                 // keep low nibble (FS + LPF bits), clear ODR
    imu_write_reg(IMU_REG_CTRL1_XL, v);

    // CTRL2_G: [7:4] ODR_G, [3:2] FS_G -> keep FS, clear ODR
    imu_read_reg(IMU_REG_CTRL2_G, &v);
    v &= 0x0F;
    imu_write_reg(IMU_REG_CTRL2_G, v);

    // NOTE: do NOT set I2C disable bits here (keeps it wakeable over I2C)
}

/* Restore previous configuration (if we saved it), otherwise fall back to your defaults. */
static inline void imu_wake(void) {
    if (g_imu_saved_valid) {
        // Restore FIFO/MD routing first (optional ordering)
        imu_write_reg(IMU_REG_FIFO_CTRL4, g_imu_saved.fifo_ctrl4);
        imu_write_reg(IMU_REG_MD1_CFG,    g_imu_saved.md1_cfg);
        imu_write_reg(IMU_REG_MD2_CFG,    g_imu_saved.md2_cfg);

        // Restore sensors
        imu_write_reg(IMU_REG_CTRL1_XL,   g_imu_saved.ctrl1_xl);
        imu_write_reg(IMU_REG_CTRL2_G,    g_imu_saved.ctrl2_g);
    } else {
        // If we didn't have a saved state, just re-apply your baseline
        imu_basic_config();
    }

    // Optional: clear any latched interrupts after wake
    (void)imu_read_all_int_src();
    (void)imu_read_wake_src();
    (void)imu_read_tap_src();
}