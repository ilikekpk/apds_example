/**
 * Copyright (c) 2016 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup tw_scanner main.c
 * @{
 * @ingroup nrf_twi_example
 * @brief TWI Sensor Example main file.
 *
 * This file contains the source code for a sample application using TWI.
 *
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_delay.h"

#define APDS_INT_PIN            14

#define APDS9301_ADDR           0x29

#define CMD                     0x80

#define CTRL_REG                0x0
#define TIMING_REG              0x1
#define THRES_LOW_LOW           0x2    // Interrupt Threshold Register - ADC channel 0 lower byte of the low threshold
#define THRES_LOW_HIGH          0x3    // Interrupt Threshold Register - ADC channel 0 upper byte of the low threshold
#define THRES_HIGH_LOW          0x4    // Interrupt Threshold Register - ADC channel 0 lower byte of the high threshold
#define THRES_HIGH_HIGH         0x5    // Interrupt Threshold Register - ADC channel 0 upper byte of the high threshold
#define INT_CTRL_REG            0x6    // Interrupt Control Register
#define ID_REG                  0xA    // ID Register for identifiation
#define DATA0_LOW               0xC    // ADC Channel Data Register - ADC channel 0 lower byte
#define DATA0_HIGH              0xD    // ADC Channel Data Register - ADC channel 0 upper byte
#define DATA1_LOW               0xE    // ADC Channel Data Register - ADC channel 1 lower byte
#define DATA1_HIGH              0xF    // ADC Channel Data Register - ADC channel 1 upper byte

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
bool m_xfer_donee = false;

uint8_t data0_low = 0;
uint8_t data0_high = 0;
uint8_t data1_low = 0;
uint8_t data1_high = 0;

void TWI_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    uint8_t s_data[1] = {0};
    ret_code_t err_code;
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TX)
            {
                m_xfer_donee = true;
            }

            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_TXRX)
            {
                static uint8_t current_reg = 0;
                nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(0x29, s_data, 1, NULL, 1);;
                switch (current_reg)
                {
                    case 0:
                        s_data[0] = (CMD | DATA0_LOW);
                        xfer.p_secondary_buf = &data0_low;
                        current_reg++;
                        break;
                    case 1:
                        s_data[0] = (CMD | DATA0_HIGH);
                        xfer.p_secondary_buf = &data0_high;
                        current_reg++;
                        break;
                    case 2:
                        s_data[0] = (CMD | DATA1_LOW);
                        xfer.p_secondary_buf = &data1_low;
                        current_reg++;
                        break;
                    case 3:
                        s_data[0] = (CMD | DATA1_HIGH);
                        xfer.p_secondary_buf = &data1_high;
                        current_reg = 0;
                        break;
                    
                }

                
                uint32_t flags = NRF_DRV_TWI_FLAG_TX_NO_STOP;
                err_code = nrf_drv_twi_xfer(&m_twi, &xfer, flags);
            }
            
            break;
        default:
            break;
    }
}

void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = 11,
       .sda                = 13,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, TWI_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}



void apds_read()
{
    ret_code_t err_code;

    //powerup
    uint8_t tx_data[2];
    tx_data[0] = (CMD | CTRL_REG);
    tx_data[1] = 0x03;
    m_xfer_donee = false;
    err_code = nrf_drv_twi_tx(&m_twi, APDS9301_ADDR, tx_data, sizeof(tx_data), false);
    APP_ERROR_CHECK(err_code);
    while (!m_xfer_donee);
    
    // config
    tx_data[0] = (CMD | TIMING_REG);
    tx_data[1] = 0x02;
    m_xfer_donee = false;
    err_code = nrf_drv_twi_tx(&m_twi, 0x29, tx_data, 2, false);
    APP_ERROR_CHECK(err_code);
    while (!m_xfer_donee);

    nrf_delay_ms(500);
    tx_data[0] = (CMD | DATA0_LOW);
    nrf_drv_twi_xfer_desc_t xfer = NRF_DRV_TWI_XFER_DESC_TXRX(0x29, tx_data, 1, &data0_low, 1);
    uint32_t flags = NRF_DRV_TWI_FLAG_TX_NO_STOP;
    err_code = nrf_drv_twi_xfer(&m_twi, &xfer, flags);
}


int main(void)
{
    ret_code_t err_code;
    nrf_gpio_cfg_output(18);
    nrf_gpio_pin_set(18);
    
    twi_init();

    
    nrf_delay_ms(100);
     apds_read();
    //nrf_gpio_pin_clear(18);
    while (true)
    {
       // nrf_delay_ms(1000);
        
        /* Empty loop. */
    }
}

/** @} */
