#include "i2s.h"

// I2S
#define I2S_PIO              pio1
#define I2S_PIO_GPIO         GPIO_FUNC_PIO1

#define GPIO_I2S_MCLK        20
#define GPIO_I2S_BCLK        21
#define GPIO_I2S_LRCLK       22
#define GPIO_I2S_DIN         23

static inline void i2s_set_sm_clkdiv_for_hz(PIO pio, uint sm, uint32_t sys_hz, uint32_t target_hz) {
    if (!target_hz) return;

    // PIO divider is 16.8 fixed-point. Round to nearest for lower frequency error.
    uint64_t div_x256 = (((uint64_t)sys_hz << 8) + (target_hz / 2u)) / target_hz;

    // Hardware minimum divider is 1.0.
    if (div_x256 < 0x100ull) div_x256 = 0x100ull;

    uint32_t div_int = (uint32_t)(div_x256 >> 8u);
    uint32_t div_frac = (uint32_t)(div_x256 & 0xffu);

    // Clamp to representable 16.8 range.
    if (div_int > 0xffffu) {
        div_int = 0xffffu;
        div_frac = 0xffu;
    }

    pio_sm_set_clkdiv_int_frac(pio, sm, (uint16_t)div_int, (uint8_t)div_frac);
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
    uint8_t func=I2S_PIO_GPIO;
    gpio_set_function(i2s_config->data_pin, I2S_PIO_GPIO);
    gpio_set_function(i2s_config->clock_pin_base, I2S_PIO_GPIO);
    gpio_set_function(i2s_config->clock_pin_base+1, I2S_PIO_GPIO);
    
    i2s_config->sm = pio_claim_unused_sm(i2s_config->pio, true);

    uint32_t system_clock_frequency = clock_get_hz(clk_sys);

    if(i2s_config->mclk_enabled) {
        gpio_set_function(i2s_config->mclk_pin, I2S_PIO_GPIO);
        i2s_config->sm_mclk = pio_claim_unused_sm(i2s_config->pio, true);
        uint offset_mclk = pio_add_program(i2s_config->pio, &pio_i2s_mclk_program);
        pio_i2s_MCLK_program_init(i2s_config->pio, i2s_config->sm_mclk, offset_mclk, i2s_config->mclk_pin);

        uint32_t mclk_sm_hz = i2s_config->mclk_mult * i2s_config->sample_freq * 2u; // edges per clock
        i2s_set_sm_clkdiv_for_hz(i2s_config->pio, i2s_config->sm_mclk, system_clock_frequency, mclk_sm_hz);

        pio_sm_set_enabled(i2s_config->pio, i2s_config->sm_mclk, true);
    }

    
    uint offset = pio_add_program(i2s_config->pio, &audio_i2s_program);

    audio_i2s_program_init(i2s_config->pio, i2s_config->sm , offset, i2s_config->data_pin , i2s_config->clock_pin_base);
    
    /* Audio SM runs at 64*Fs for 16-bit stereo I2S framing. */
    uint32_t audio_sm_hz = i2s_config->sample_freq * 64u;
    i2s_set_sm_clkdiv_for_hz(i2s_config->pio, i2s_config->sm, system_clock_frequency, audio_sm_hz);

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

    pio_sm_set_enabled(i2s_config->pio, i2s_config->sm , true);
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

    uint32_t system_clock_frequency = clock_get_hz(clk_sys);
    uint32_t audio_sm_hz = sample_freq * 64u;
    i2s_set_sm_clkdiv_for_hz(i2s_config->pio, i2s_config->sm, system_clock_frequency, audio_sm_hz);

    if (i2s_config->mclk_enabled) {
        uint32_t mclk_sm_hz = i2s_config->mclk_mult * sample_freq * 2u; // edges per clock
        i2s_set_sm_clkdiv_for_hz(i2s_config->pio, i2s_config->sm_mclk, system_clock_frequency, mclk_sm_hz);
    }

    i2s_config->sample_freq = sample_freq;
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
