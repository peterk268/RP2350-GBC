/**
 * Original source code from https://github.com/carlk3/no-OS-FatFS-SD-SPI-RPi-Pico
 * Copyright 2021 Carl John Kugler III
 * 
 * Licensed under the Apache License, Version 2.0 (the License); you may not use 
 * this file except in compliance with the License. You may obtain a copy of the 
 * License at
 * 
 *    http://www.apache.org/licenses/LICENSE-2.0 
 * Unless required by applicable law or agreed to in writing, software distributed 
 * under the License is distributed on an AS IS BASIS, WITHOUT WARRANTIES OR 
 * CONDITIONS OF ANY KIND, either express or implied. See the License for the 
 * specific language governing permissions and limitations under the License.
 * 
 */

#pragma once

#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <string.h>
#include "my_debug.h"
#include "f_util.h"
#include "ff.h"
#include "sd_card.h"
#include "diskio.h"
#include "rtc.h"

void spi_dma_isr();

static spi_t spis[]={
    {
        .hw_inst=spi1,
        .miso_gpio=12,
        .mosi_gpio=15,
        .sck_gpio=14,
        .baud_rate=12500*1000,
        .dma_isr=spi_dma_isr
    }
};

static sd_card_t sd_cards[]={
    {
        .pcName="0:",
        .spi=&spis[0],
        .ss_gpio=13,
        .use_card_detect=false,
        .m_Status=STA_NOINIT
    }
};

void spi_dma_isr() {
    spi_irq_handler(&spis[0]);
}

size_t sd_get_num() {
    return count_of(sd_cards);
}

sd_card_t *sd_get_by_num(size_t num) {
    if (num<=sd_get_num()) {
        return &sd_cards[num];
    } else {
        return NULL;
    }
}

size_t spi_get_num() {
    return count_of(spis);
}

spi_t *spi_get_by_num(size_t num) {
    if (num <= sd_get_num()) {
        return &spis[num];
    } else {
        return NULL;
    }
}
