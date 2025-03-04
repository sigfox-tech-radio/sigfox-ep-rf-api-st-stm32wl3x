/*!*****************************************************************
 * \file    stm32wl3x_rf_api.h
 * \brief   Sigfox STM32WL3X RF API implementation.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#ifndef __STM32WL3X_RF_API_H__
#define __STM32WL3X_RF_API_H__

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "manuf/rf_api.h"

/*** STM32WL3X RF API structures ***/

#ifdef SIGFOX_EP_ERROR_CODES
/*!******************************************************************
 * \enum STM32WL3X_RF_API_status_t
 * \brief RF driver error codes.
 *******************************************************************/
typedef enum {
    STM32WL3X_RF_API_ERROR_NULL_PARAMETER = RF_API_ERROR_LAST,
    STM32WL3X_RF_API_ERROR_REGULATOR_READY,
    STM32WL3X_RF_API_ERROR_STATE_SWITCH_TIMEOUT,
    STM32WL3X_RF_API_ERROR_BUFFER_SIZE,
    STM32WL3X_RF_API_ERROR_MODE,
    STM32WL3X_RF_API_ERROR_FREQUENCY,
    STM32WL3X_RF_API_ERROR_MODULATION,
    STM32WL3X_RF_API_ERROR_GPIO,
    STM32WL3X_RF_API_ERROR_PA_DRIVE_MODE,
    STM32WL3X_RF_API_ERROR_TX_POWER,
    STM32WL3X_RF_API_ERROR_LATENCY_TYPE,
    STM32WL3X_RF_API_ERROR_STATE,
    // Low level errors.
    // Activate the ERROR_STACK flag and use the SIGFOX_EP_API_unstack_error() function to get more details.
    STM32WL3X_RF_API_ERROR_DRIVER_MCU_API,
    STM32WL3X_RF_API_ERROR_DRIVER_STM32WL3X_HW_API,
    // Last index.
    SIGFOX_STM32WL3X_RF_API_ERROR_LAST
} STM32WL3X_RF_API_status_t;
#else
typedef void STM32WL3X_RF_API_status_t;
#endif

/*** STM32WL3X RF API functions ***/

#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_open(RF_API_config_t *rf_api_config)
 * \brief Open the RF driver.
 * \param[in]   rf_api_config: Pointer to the RF API configuration.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_open(RF_API_config_t *rf_api_config);
#endif

#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_close(void)
 * \brief Close the RF driver.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_close(void);
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_process(void)
 * \brief Process RF driver, this function will be call by SIGFOX_EP_API_process just after the process_callback has been sent to process RF interruptions in main context.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_process(void);
#endif

/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_wake_up(void)
 * \brief Wake-up the radio before each overall TX or RX sequence.
*  \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_wake_up(void);

/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_sleep(void)
 * \brief Release the radio after each overall TX or RX sequence.
 * \param[in]   none
 * \param[out]  none
*  \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_sleep(void);

/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_init(RF_API_radio_parameters_t *radio_parameters)
 * \brief Initialize the radio operation before each individual frame transmission or reception.
 * \param[in]   radio_parameters: Pointers to the radio parameters.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_init(RF_API_radio_parameters_t *radio_parameters);

/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_de_init(void)
 * \brief Stop the radio operation after each individual frame transmission or reception.
 * \param[in]   rf_mode: Radio mode.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_de_init(void);

/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_send(RF_API_tx_data_t *tx_data)
 * \brief Sending a bitstream over the air.
 * \brief In blocking mode, this function blocks until the full bitstream is sent.
 * \brief In asynchronous, this function only starts the transmission. End of transmission should be notified through the cplt_cb() callback.
 * \param[in]   tx_data: Pointer to the TX parameters.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_send(RF_API_tx_data_t *tx_data);

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_receive(RF_API_rx_data_t *rx_data)
 * \brief Start downlink reception. Could be called multiple times if several downlink frames are received during the RX window.
 * \brief In blocking mode, this function blocks until a valid downlink data is received or the MCU_API_TIMER_INSTANCE_T_W has elapsed.
 * \brief In asynchronous mode, this function only starts the reception. Data reception should be notified through the rx_data_received() callback.
 * \param[in]   none
 * \param[out]  rx_data: Pointer to the RX parameters.
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_receive(RF_API_rx_data_t *rx_data);
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm)
 * \brief Read DL-PHY content and RSSI received by the radio.
 * \brief In blocking mode, this function will be called only if the data_received parameter of the STM32WL3X_RF_API_receive() function is returned with SFX_TRUE value.
 * \brief in asynchronous mode, this function will be called only if the data_received_cb callback is triggered during reception.
 * \param[in]   dl_phy_content_size: Number of bytes to copy in dl_phy_content.
 * \param[out]  dl_phy_content: Array to be filled with the received DL-PHY content.
 * \param[out]  dl_rssi_dbm: Pointer to 16-bits signed value to be filled with the DL RSSI in dBm.
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm);
#endif

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params)
 * \brief In blocking mode, the function until the LBT condition is met or the MCU_API_TIMER_INSTANCE_LBT has elapsed.
 * \brief In asynchronous mode, this function only starts the carrier sense operation. Channel free event should be notified through the channel_free_cb() callback.
 * \param[in]   carrier_sense_params: Pointer to the carrier sense parameters.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params);
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32 *latency_ms)
 * \brief Read radio latency in milliseconds.
 * \brief This functions is called by the core library to compensate the durations in the MCU_API_timer_start() function.
 * \param[in]   latency_type: Type of latency to get.
 * \param[out]  latency_ms: Pointer to integer that will contain the radio latency in milliseconds.
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32 *latency_ms);
#endif

#ifdef SIGFOX_EP_CERTIFICATION
/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_start_continuous_wave(void)
 * \brief Start continuous wave transmission using radio parameters given in the STM32WL3X_RF_API_init() function.
 * \brief This function is only called by the type approval addon (EP-ADDON-TA). In asynchronous mode, it must not issue any completion callback.
 * \brief The transmission will be stopped by the STM32WL3X_RF_API_de_init() function.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_start_continuous_wave(void);
#endif

#ifdef SIGFOX_EP_VERBOSE
/*!******************************************************************
 * \fn RF_API_status_t STM32WL3X_RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char)
 * \brief Get RF driver version.
 * \param[in]   none
 * \param[out]  version: RF driver version.
 * \param[out]  version_size_char: Pointer that will contain the string size.
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t STM32WL3X_RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char);
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*!******************************************************************
 * \fn void STM32WL3X_RF_API_error(void)
 * \brief Function called by the library if any error occurred during the processing.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void STM32WL3X_RF_API_error(void);
#endif

#endif /* __STM32WL3X_RF_API_H__ */
