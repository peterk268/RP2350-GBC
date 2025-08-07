#pragma once

#include <stdlib.h>
#include <string.h>
#include <hardware/pio.h>
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include "audio_i2s.pio.h"

typedef struct i2s_config 
{
    uint32_t sample_freq;        
    uint16_t channel_count; 
    uint8_t  data_pin;
    uint8_t  clock_pin_base;
    PIO	     pio;
    uint8_t  sm; 
    uint8_t  dma_channel;
    uint16_t dma_trans_count;
    uint16_t *dma_buf;
    float volume;
} i2s_config_t;


i2s_config_t i2s_get_default_config(void);
void i2s_init(i2s_config_t *i2s_config);
void i2s_write(const i2s_config_t *i2s_config,const int16_t *samples,const size_t len);
void i2s_dma_write(i2s_config_t *i2s_config,const int16_t *samples);
void i2s_volume(i2s_config_t *i2s_config, float volume);
void i2s_increase_volume(i2s_config_t *i2s_config);
void i2s_decrease_volume(i2s_config_t *i2s_config);
void i2s_set_sample_freq(i2s_config_t *i2s_config, uint32_t sample_freq, bool frameskip);