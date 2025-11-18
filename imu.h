#ifndef IMU_LSM6DSO32_H
#define IMU_LSM6DSO32_H

#define IMU_I2C_ADDRESS   0x6B

// -------------------------------
// LSM6DSO32 Registers
// -------------------------------
#define REG_FUNC_CFG       0x01
#define REG_WHO_AM_I       0x0F
#define REG_CTRL1_XL       0x10
#define REG_CTRL2_G        0x11
#define REG_CTRL3_C        0x12
#define REG_OUT_TEMP_L     0x20
#define REG_OUTX_L_G       0x22
#define REG_OUTX_L_A       0x28

// Expected WHO_AM_I value
#define LSM6DSO32_WHOAMI   0x6C

// -------------------------------
// Enum Configurations
// -------------------------------
typedef enum {
    ODR_OFF     = 0b0000,
    ODR_12_5    = 0b0001,
    ODR_26      = 0b0010,
    ODR_52      = 0b0011,
    ODR_104     = 0b0100,
    ODR_208     = 0b0101,
    ODR_416     = 0b0110,
    ODR_833     = 0b0111,
    ODR_1666    = 0b1000
} imu_odr_t;

typedef enum {
    ACCEL_4G  = 0b00,
    ACCEL_8G  = 0b10,
    ACCEL_16G = 0b11,
    ACCEL_32G = 0b01
} imu_accel_scale_t;

typedef enum {
    GYRO_250DPS  = 0b00,
    GYRO_500DPS  = 0b01,
    GYRO_1000DPS = 0b10,
    GYRO_2000DPS = 0b11
} imu_gyro_scale_t;

// -------------------------------
// Low-level I2C helpers
// -------------------------------
static inline void imu_write_reg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = { reg, value };
    i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, data, 2, false);
}

static inline void imu_read_reg(uint8_t reg, uint8_t *value) {
    i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, &reg, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, value, 1, false);
}

static inline void imu_read_regs(uint8_t reg, uint8_t *buf, uint16_t len) {
    i2c_write_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, &reg, 1, true);
    i2c_read_blocking(IMU_I2C_PORT, IMU_I2C_ADDRESS, buf, len, false);
}

// -------------------------------
// Conversion factors
// -------------------------------
static float accel_res = 0.009806f * 0.122f;  // default ±4G
static float gyro_res  = 0.001f * 8.75f;      // default 250 dps

// -------------------------------
// WHO_AM_I Check
// -------------------------------
static inline bool imu_whoami_ok(void) {
    uint8_t id;
    imu_read_reg(REG_WHO_AM_I, &id);
    return (id == LSM6DSO32_WHOAMI);
}

// -------------------------------
// IMU Init (standard accel+gyro on)
// -------------------------------
static inline void imu_init(void) {
    sleep_ms(30);

    // Reset device
    imu_write_reg(REG_CTRL3_C, 0x01);
    sleep_ms(20);

    // Accelerometer: 104Hz, ±4G
    imu_write_reg(REG_CTRL1_XL,
        (ODR_104 << 4) |   // ODR
        (ACCEL_4G << 2) |  // full-scale
        0b00               // anti-aliasing filter
    );

    // Gyroscope: 104Hz, 250 dps
    imu_write_reg(REG_CTRL2_G,
        (ODR_104 << 4) |
        (GYRO_250DPS << 2)
    );

    // Auto-increment + little-endian
    imu_write_reg(REG_CTRL3_C, 0b01000100);
}

// -------------------------------
// Set accelerometer configuration
// -------------------------------
static inline void imu_set_accel(imu_odr_t odr, imu_accel_scale_t scale) {
    uint8_t value =
        (odr << 4) |
        (scale << 2);

    imu_write_reg(REG_CTRL1_XL, value);

    // Update conversion factor
    switch (scale) {
        case ACCEL_4G:  accel_res = 0.009806f * 0.122f; break;
        case ACCEL_8G:  accel_res = 0.009806f * 0.244f; break;
        case ACCEL_16G: accel_res = 0.009806f * 0.488f; break;
        case ACCEL_32G: accel_res = 0.009806f * 0.976f; break;
    }
}

// -------------------------------
// Set gyro configuration
// -------------------------------
static inline void imu_set_gyro(imu_odr_t odr, imu_gyro_scale_t scale) {
    uint8_t value =
        (odr << 4) |
        (scale << 2);

    imu_write_reg(REG_CTRL2_G, value);

    switch (scale) {
        case GYRO_250DPS:  gyro_res = 0.001f * 8.75f; break;
        case GYRO_500DPS:  gyro_res = 0.001f * 17.5f; break;
        case GYRO_1000DPS: gyro_res = 0.001f * 35.0f; break;
        case GYRO_2000DPS: gyro_res = 0.001f * 70.0f; break;
    }
}

// -------------------------------
// Read temperature
// -------------------------------
static inline int16_t imu_read_temp_raw(void) {
    uint8_t buf[2];
    imu_read_regs(REG_OUT_TEMP_L, buf, 2);
    return (int16_t)((buf[1] << 8) | buf[0]);
}

static inline float imu_read_temp_c(void) {
    return (float)imu_read_temp_raw() / 256.0f + 25.0f;
}

// -------------------------------
// Read raw accel
// -------------------------------
static inline void imu_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buf[6];
    imu_read_regs(REG_OUTX_L_A, buf, 6);

    *ax = (int16_t)(buf[1] << 8 | buf[0]);
    *ay = (int16_t)(buf[3] << 8 | buf[2]);
    *az = (int16_t)(buf[5] << 8 | buf[4]);
}

static inline void imu_read_accel_ms2(float *ax, float *ay, float *az) {
    int16_t rx, ry, rz;
    imu_read_accel_raw(&rx, &ry, &rz);

    *ax = rx * accel_res;
    *ay = ry * accel_res;
    *az = rz * accel_res;
}

// -------------------------------
// Read raw gyro
// -------------------------------
static inline void imu_read_gyro_raw(int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buf[6];
    imu_read_regs(REG_OUTX_L_G, buf, 6);

    *gx = (int16_t)(buf[1] << 8 | buf[0]);
    *gy = (int16_t)(buf[3] << 8 | buf[2]);
    *gz = (int16_t)(buf[5] << 8 | buf[4]);
}

static inline void imu_read_gyro_dps(float *gx, float *gy, float *gz) {
    int16_t rx, ry, rz;
    imu_read_gyro_raw(&rx, &ry, &rz);

    *gx = rx * gyro_res;
    *gy = ry * gyro_res;
    *gz = rz * gyro_res;
}

#endif
