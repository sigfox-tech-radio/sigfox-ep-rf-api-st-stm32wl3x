/*!*****************************************************************
 * \file    stm32wl3x_hw_api.h
 * \brief   Sigfox STM32WL3x HW interface.
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

#ifndef __STM32WL3X_HW_API_H__
#define __STM32WL3X_HW_API_H__

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "manuf/rf_api.h"

#include "stm32wl3x_ll_mrsubg.h"

/*** STM32WL3X HW API structures ***/

#ifdef SIGFOX_EP_ERROR_CODES
/*!******************************************************************
 * \enum STM32WL3X_HW_API_status_t
 * \brief STM32WL3X HW driver error codes.
 *******************************************************************/
typedef enum {
    STM32WL3X_HW_API_SUCCESS = 0,
    STM32WL3X_HW_API_ERROR,
    // Additional custom error codes can be added here (up to sfx_u32).
    // They will be logged in the library error stack if the ERROR_STACK flag is defined (SIGFOX_ERROR_SOURCE_HW base).
    // Last index.
    STM32WL3X_HW_API_ERROR_LAST
} STM32WL3X_HW_API_status_t;
#else
typedef void STM32WL3X_HW_API_status_t;
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*!******************************************************************
 * \enum STM32WL3X_HW_API_latency_t
 * \brief STM32WL3X hardware functions latency delay type.
 *******************************************************************/
typedef enum {
    STM32WL3X_HW_API_LATENCY_INIT_TX,
    STM32WL3X_HW_API_LATENCY_DE_INIT_TX,
#ifdef SIGFOX_EP_BIDIRECTIONAL
    STM32WL3X_HW_API_LATENCY_INIT_RX,
    STM32WL3X_HW_API_LATENCY_DE_INIT_RX,
#endif
    STM32WL3X_HW_API_LATENCY_LAST
} STM32WL3X_HW_API_latency_t;
#endif

/*!******************************************************************
 * \struct STM32WL3X_HW_API_config_t
 * \brief STM32WL3X driver configuration structure.
 *******************************************************************/
typedef struct {
    const SIGFOX_rc_t *rc;
} STM32WL3X_HW_API_config_t;

/*!******************************************************************
 * \struct STM32WL3X_radio_parameters_t
 * \brief STM32WL3X radio parameters structure.
 *******************************************************************/
typedef struct {
    RF_API_mode_t rf_mode;
    sfx_s8 expected_tx_power_dbm;
} STM32WL3X_radio_parameters_t;

/*** STM32WL3X HW API functions ***/

/*!******************************************************************
 * \fn STM32WL3X_HW_API_status_t STM32WL3X_HW_API_open(STM32WL3X_HW_API_config_t *hw_api_config)
 * \brief This function is called during the RF_API_open() function to open the STM32WL3X hardware interface.
 * \param[in]   hw_api_config: Pointer to the HW API configuration.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_open(STM32WL3X_HW_API_config_t *hw_api_config);

/*!******************************************************************
 * \fn STM32WL3X_HW_API_status_t STM32WL3X_HW_API_close(void)
 * \brief This function is called during the RF_API_close() function to close the STM32WL3X hardware interface.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_close(void);

/*!******************************************************************
 * \fn STM32WL3X_HW_API_status_t STM32WL3X_HW_API_init(STM32WL3X_radio_parameters_t *radio_parameters)
 * \brief This optional function is called during the RF_API_init() function to configure additional hardware parameters.
 * \param[in]   radio_parameters: Pointers to the radio parameters.
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_init(STM32WL3X_radio_parameters_t *radio_parameters);

/*!******************************************************************
 * \fn STM32WL3X_HW_API_status_t STM32WL3X_HW_API_de_init(void)
 * \brief This optional function is called during the RF_API_de_init() function to release additional hardware parameters.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_de_init(void);

/*!******************************************************************
 * \fn STM32WL3X_HW_API_status_t STM32WL3X_HW_API_get_tx_power(sfx_s8 expected_tx_power_dbm, sfx_s8 *tx_power_dbm, MRSubG_PA_DRVMode *pa_drive_mode)
 * \brief Returns the effective RF output power to program on the STM32WL3X to get the expected value at board level.
 * \brief This function is required when an external gain has to be compensated (typical case of an external PA). Otherwise the STM32WL3X output power equals the expected value.
 * \param[in]   expected_tx_power_dbm: Expected output power in dBm (given by applicative level).
 * \param[out]  tx_power_dbm: Pointer to the effective output power in dBm to program on the STM32WL3X transceiver.
 * \param[out]  pa_drive_mode: Pointer to the PA drive mode to use according to hardware routing.
 * \retval      Function execution status.
 *******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_get_tx_power(sfx_s8 expected_tx_power_dbm, sfx_s8 *tx_power_dbm, MRSubG_PA_DRVMode *pa_drive_mode);

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*!******************************************************************
 * \fn STM32WL3X_HW_API_status_t STM32WL3X_HW_API_get_latency(STM32WL3X_HW_API_latency_t latency_type, sfx_u32 *latency_ms)
 * \brief Read HW functions latency in milliseconds.
 * \param[in]   latency_type: Type of latency to get.
 * \param[out]  latency_ms: Pointer to integer that will contain the latency in milliseconds.
 * \retval      Function execution status.
 *******************************************************************/
STM32WL3X_HW_API_status_t STM32WL3X_HW_API_get_latency(STM32WL3X_HW_API_latency_t latency_type, sfx_u32 *latency_ms);
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*!******************************************************************
 * \fn void STM32WL3X_HW_API_stack_error(void)
 * \brief Generic macro which calls the error stack function for STM32WL3X_HW_API errors (if enabled).
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
#ifdef SIGFOX_EP_ERROR_STACK
#define STM32WL3X_HW_API_stack_error(void) SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_HW_API, STM32WL3X_hw_api_status)
#else
#define STM32WL3X_HW_API_stack_error(void)
#endif
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*!******************************************************************
 * \fn void STM32WL3X_HW_API_check_status(error)
 * \brief Generic macro to check an STM32WL3X_HW_API function status and exit.
 * \param[in]   error: High level error code to rise.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
#define STM32WL3X_HW_API_check_status(error) { if (STM32WL3X_hw_api_status != STM32WL3X_HW_API_SUCCESS) { STM32WL3X_HW_API_stack_error(); SIGFOX_EXIT_ERROR(error) } }
#endif

#endif /* __STM32WL3X_HW_API_H__ */
