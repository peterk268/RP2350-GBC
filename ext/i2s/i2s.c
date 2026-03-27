#include "i2s.h"

// I2S
#define I2S_PIO              pio1
#define I2S_PIO_GPIO         GPIO_FUNC_PIO1

#define GPIO_I2S_MCLK        20
#define GPIO_I2S_BCLK        21
#define GPIO_I2S_LRCLK       22
#define GPIO_I2S_DIN         23

#define I2S_SM_CYCLES_PER_FRAME 64u
#define MCLK_SM_CYCLES_PER_CLOCK 2u
// NEED INTEGER LOCKED CLOCKS OR ELSE HELLA JITTER WILL COME FROM FRACTIONAL DIV
// Fractional DIV - "It acts as a sigma-delta modulator, switching between two integer divisors to achieve a desired, non-integer average frequency"
// Won't be super accurate but it is best to not have the jitter and get artefacts in audio. Integer-locked clocks will at least be stable, even if they are not the exact target frequency.
//
// Integer-locked accuracy (MCLK=256x, sys_clock must be integer multiple of DPI_PCLK=20MHz):
//   sys_clock | 44.1kHz div | 44.1kHz error | 48kHz div | 48kHz error
//   320 MHz   |      14     |    +1.23%      |     13    |   +0.16%
//   340 MHz   |      15     |    +0.39%      |     14    |   -1.18%   <- best balanced
//   360 MHz   |      16     |    -0.35%      |     15    |   -2.34%
//   380 MHz   |      17     |    -1.00%      |     15    |   +3.08%
//   400 MHz   |      18     |    -1.58%      |     16    |   +1.72%
//
// 340MHz is currently used (17x20MHz, DPI-compatible). Best balanced option across both rates.
//
// NOTE: integer-locking MUST round to nearest integer, not floor. See integer-locked code path.
#define I2S_INTEGER_LOCKED_CLOCKS 1

static inline uint32_t i2s_get_divider_x256_for_hz(uint32_t target_sm_hz) {
    if (target_sm_hz == 0) return 256u;

    uint32_t system_clock_hz = clock_get_hz(clk_sys);

    // PIO divider is INT.FRAC with 8 fractional bits.
    uint64_t divider_x256 = (((uint64_t)system_clock_hz) << 8) + (target_sm_hz / 2u);
    divider_x256 /= target_sm_hz;
    if (divider_x256 < 256u) divider_x256 = 256u; // minimum divider = 1.0
    if (divider_x256 > 0xFFFFFFu) divider_x256 = 0xFFFFFFu; // maximum divider = 65535.996

    return (uint32_t)divider_x256;
}

static inline void i2s_set_sm_divider_x256(PIO pio, uint sm, uint32_t divider_x256) {
    pio_sm_set_clkdiv_int_frac(pio, sm,
                               (uint16_t)(divider_x256 >> 8),
                               (uint8_t)(divider_x256 & 0xFFu));
}

static inline void i2s_configure_clocks(i2s_config_t *i2s_config, uint32_t sample_freq) {
    if (i2s_config->mclk_enabled && i2s_config->mclk_mult != 0) {
        uint32_t mclk_sm_hz = i2s_config->mclk_mult * sample_freq * MCLK_SM_CYCLES_PER_CLOCK;

#if I2S_INTEGER_LOCKED_CLOCKS
        // Compute integer divisor directly with round-to-nearest, then shift to Q8 format.
        // This avoids the root cause: i2s_get_divider_x256_for_hz() returns a rounded Q8 value,
        // but masking with & 0xFFFF00 then truncates (floors) the fraction — undoing the rounding
        // when frac >= 0.5. Computing the integer divisor in one step avoids that entirely.
        uint32_t sys_hz = clock_get_hz(clk_sys);
        uint32_t int_div = (sys_hz + mclk_sm_hz / 2u) / mclk_sm_hz; // round(sys_hz / mclk_sm_hz)
        if (int_div < 1u) int_div = 1u;
        if (int_div > 0xFFFFu) int_div = 0xFFFFu;
        uint32_t mclk_div_x256 = int_div << 8;
#else
        uint32_t mclk_div_x256 = i2s_get_divider_x256_for_hz(mclk_sm_hz);
#endif
        i2s_set_sm_divider_x256(i2s_config->pio, i2s_config->sm_mclk, mclk_div_x256);

        // Keep BCLK/LRCLK divider ratio locked to MCLK divider to avoid relative drift.
        uint64_t i2s_div_x256 = (uint64_t)mclk_div_x256 * i2s_config->mclk_mult * MCLK_SM_CYCLES_PER_CLOCK;
        i2s_div_x256 = (i2s_div_x256 + (I2S_SM_CYCLES_PER_FRAME / 2u)) / I2S_SM_CYCLES_PER_FRAME;
        if (i2s_div_x256 < 256u) i2s_div_x256 = 256u;
        if (i2s_div_x256 > 0xFFFFFFu) i2s_div_x256 = 0xFFFFFFu;
        i2s_set_sm_divider_x256(i2s_config->pio, i2s_config->sm, (uint32_t)i2s_div_x256);
    } else {
        uint32_t i2s_sm_hz = sample_freq * I2S_SM_CYCLES_PER_FRAME;
        i2s_set_sm_divider_x256(i2s_config->pio, i2s_config->sm, i2s_get_divider_x256_for_hz(i2s_sm_hz));
    }
}

// #warning "lr needs to be righ tafter btclk"
/**
 * return the default i2s context used to store information about the setup
 */
i2s_config_t i2s_get_default_config(void) {
    i2s_config_t i2s_config = {
		.sample_freq = 44100, 
		.channel_count = 2,
		.data_pin = GPIO_I2S_DIN, 
		.clock_pin_base = GPIO_I2S_BCLK,
        .mclk_pin = GPIO_I2S_MCLK,
		.pio = I2S_PIO,
		.sm = 0,
        .sm_mclk = 1,
        .dma_channel = 0,
        .dma_buf = NULL,
        .dma_trans_count = 0,
        .volume = 0,
        .mclk_enabled = true, // MCLK generation is enabled by default
        .mclk_mult = 256 // Default MCLK multiplier
	};

    return i2s_config;
}

/**
 * Initialize the I2S driver. Must be called before calling i2s_write or i2s_dma_write
 * i2s_config: I2S context obtained by i2s_get_default_config()
 */
void i2s_init(i2s_config_t *i2s_config) {
    gpio_set_function(i2s_config->data_pin, I2S_PIO_GPIO);
    gpio_set_function(i2s_config->clock_pin_base, I2S_PIO_GPIO);
    gpio_set_function(i2s_config->clock_pin_base+1, I2S_PIO_GPIO);
    
    i2s_config->sm = pio_claim_unused_sm(i2s_config->pio, true);

    if(i2s_config->mclk_enabled) {
        gpio_set_function(i2s_config->mclk_pin, I2S_PIO_GPIO);
        i2s_config->sm_mclk = pio_claim_unused_sm(i2s_config->pio, true);
        uint offset_mclk = pio_add_program(i2s_config->pio, &pio_i2s_mclk_program);
        pio_i2s_MCLK_program_init(i2s_config->pio, i2s_config->sm_mclk, offset_mclk, i2s_config->mclk_pin);
    }

    
    uint offset = pio_add_program(i2s_config->pio, &audio_i2s_program);

    audio_i2s_program_init(i2s_config->pio, i2s_config->sm , offset, i2s_config->data_pin , i2s_config->clock_pin_base);
    
    i2s_configure_clocks(i2s_config, i2s_config->sample_freq);

    pio_sm_set_enabled(i2s_config->pio, i2s_config->sm, false);

    /* Allocate memory for the DMA buffer */
    i2s_config->dma_buf=malloc(i2s_config->dma_trans_count*sizeof(uint32_t));

    /* Direct Memory Access setup */
    i2s_config->dma_channel = dma_claim_unused_channel(true);
    
    dma_channel_config dma_config = dma_channel_get_default_config(i2s_config->dma_channel);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    channel_config_set_dreq(&dma_config, pio_get_dreq(i2s_config->pio, i2s_config->sm, true));
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    dma_channel_configure(i2s_config->dma_channel,
                          &dma_config,
                          &(i2s_config->pio->txf[i2s_config->sm]),    // Destination pointer
                          i2s_config->dma_buf,                        // Source pointer
                          i2s_config->dma_trans_count,                // Number of 32 bits words to transfer
                          false                                       // Start immediately
    );

    if (i2s_config->mclk_enabled && i2s_config->mclk_mult != 0) {
        uint32_t sm_mask = (1u << i2s_config->sm) | (1u << i2s_config->sm_mclk);
        pio_enable_sm_mask_in_sync(i2s_config->pio, sm_mask);
    } else {
        pio_sm_set_enabled(i2s_config->pio, i2s_config->sm, true);
    }
}

/**
 * Write samples to I2S directly and wait for completion (blocking)
 * i2s_config: I2S context obtained by i2s_get_default_config()
 *     sample: pointer to an array of len x 32 bits samples
 *             Each 32 bits sample contains 2x16 bits samples, 
 *             one for the left channel and one for the right channel
 *        len: length of sample in 32 bits words
 */
void i2s_write(const i2s_config_t *i2s_config,const int16_t *samples,const size_t len) {
    for(size_t i=0;i<len;i++) {
            pio_sm_put_blocking(i2s_config->pio, i2s_config->sm, (uint32_t)samples[i]);
    }
}

/**
 * Write samples to DMA buffer and initiate DMA transfer (non blocking)
 * i2s_config: I2S context obtained by i2s_get_default_config()
 *     sample: pointer to an array of dma_trans_count x 32 bits samples
 */
void i2s_dma_write(i2s_config_t *i2s_config, const uint16_t *samples) {
    if (!i2s_config || !samples || !i2s_config->dma_buf) return;

    // Wait for previous DMA transfer to finish
    dma_channel_wait_for_finish_blocking(i2s_config->dma_channel);

    memcpy(i2s_config->dma_buf, samples, i2s_config->dma_trans_count * sizeof(uint32_t));
    dma_channel_transfer_from_buffer_now(i2s_config->dma_channel,
                                        i2s_config->dma_buf,
                                        i2s_config->dma_trans_count);
}

/**
 * Write silence (zeros) to DMA buffer and initiate DMA transfer (blocking).
 * Avoids needing a separate silence buffer allocation.
 */
void i2s_dma_write_silence(i2s_config_t *i2s_config) {
    if (!i2s_config || !i2s_config->dma_buf) return;
    dma_channel_wait_for_finish_blocking(i2s_config->dma_channel);
    memset(i2s_config->dma_buf, 0, i2s_config->dma_trans_count * sizeof(uint32_t));
    dma_channel_transfer_from_buffer_now(i2s_config->dma_channel,
                                        i2s_config->dma_buf,
                                        i2s_config->dma_trans_count);
}

/**
 * Non-blocking silence write. Returns true if DMA accepted, false if still busy.
 */
bool i2s_dma_write_silence_non_blocking(i2s_config_t *i2s_config) {
    if (!i2s_config || !i2s_config->dma_buf)
        return true;
    if (dma_channel_is_busy(i2s_config->dma_channel))
        return false;
    memset(i2s_config->dma_buf, 0, i2s_config->dma_trans_count * sizeof(uint32_t));
    dma_channel_transfer_from_buffer_now(i2s_config->dma_channel,
                                        i2s_config->dma_buf,
                                        i2s_config->dma_trans_count);
    return true;
}

/**
 * Non-blocking I2S DMA write.
 *
 * Returns:
 *   true  → DMA transfer started (previous transfer finished)
 *   false → DMA is still busy; call again later
 *
 * Notes:
 *   - Does NOT wait for DMA to finish.
 *   - Caller should retry until true is returned.
 *   - `samples` must contain dma_trans_count 32-bit I2S words.
 */
bool i2s_dma_write_non_blocking(i2s_config_t *i2s_config, const uint16_t *samples) {
    if (!i2s_config || !samples || !i2s_config->dma_buf) 
        return true; // treat invalid state as "ready" to avoid locking

    // If DMA is still transferring the previous buffer, don't block.
    if (dma_channel_is_busy(i2s_config->dma_channel)) {
        return false;
    }

    // DMA is free → copy new data and start next transfer.
    memcpy(i2s_config->dma_buf, samples, i2s_config->dma_trans_count * sizeof(uint32_t));
    dma_channel_transfer_from_buffer_now(i2s_config->dma_channel,
                                        i2s_config->dma_buf,
                                        i2s_config->dma_trans_count);
    return true;
}


/**
 * Adjust the output volume
 * i2s_config: I2S context obtained by i2s_get_default_config()
 *     volume: desired volume between 0 (highest. volume) and 16 (lowest volume)
 */
void i2s_volume(i2s_config_t *i2s_config, float volume) {
    if (volume > 1) volume = 1;
    if (volume < 0) volume = 0;
    i2s_config->volume = volume;
}

void i2s_set_sample_freq(i2s_config_t *i2s_config, uint32_t sample_freq, bool frameskip) {
    if (frameskip) {
        sample_freq *= 2; // Double the sample frequency for faster processing
    }

    i2s_configure_clocks(i2s_config, sample_freq);

    i2s_config->sample_freq = sample_freq;
}

// Stop I2S: abort DMA, disable PIO state machines, clear TX FIFO.
// Call before changing sys clock to prevent BCLK/LRCLK/MCLK glitching.
void i2s_stop(i2s_config_t *i2s_config) {
    dma_channel_abort(i2s_config->dma_channel);
    pio_sm_set_enabled(i2s_config->pio, i2s_config->sm, false);
    if (i2s_config->mclk_enabled)
        pio_sm_set_enabled(i2s_config->pio, i2s_config->sm_mclk, false);
    pio_sm_clear_fifos(i2s_config->pio, i2s_config->sm);
}

// Restart I2S after a clock change. Assumes i2s_set_sample_freq() has already been
// called to update the PIO clock dividers for the new sys clock. Re-enables both SMs
// in sync so MCLK and BCLK/LRCLK start phase-aligned.
void i2s_restart(i2s_config_t *i2s_config) {
    pio_sm_clear_fifos(i2s_config->pio, i2s_config->sm);
    pio_sm_restart(i2s_config->pio, i2s_config->sm);
    if (i2s_config->mclk_enabled) {
        pio_sm_clear_fifos(i2s_config->pio, i2s_config->sm_mclk);
        pio_sm_restart(i2s_config->pio, i2s_config->sm_mclk);
        uint32_t sm_mask = (1u << i2s_config->sm) | (1u << i2s_config->sm_mclk);
        pio_enable_sm_mask_in_sync(i2s_config->pio, sm_mask);
    } else {
        pio_sm_set_enabled(i2s_config->pio, i2s_config->sm, true);
    }
}


// /**
//  * Increases the output volume
//  */
// void i2s_increase_volume(i2s_config_t *i2s_config) {
//     if(i2s_config->volume>0) {
//         i2s_config->volume--;
//     }
// }

// /**
//  * Decreases the output volume
//  */
// void i2s_decrease_volume(i2s_config_t *i2s_config) {
//     if(i2s_config->volume<16) {
//         i2s_config->volume++;
//     }
// }
