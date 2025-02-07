/*!*****************************************************************
 * \file    stm32wl3x_hw_api.c
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

#include "board/stm32wl3x_hw_api.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "manuf/rf_api.h"

/*** STM32WL3X HW API functions ***/

/*******************************************************************/
STM32WL3X_HW_API_status_t __attribute__((weak)) STM32WL3X_HW_API_open(STM32WL3X_HW_API_config_t *hw_api_config) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
    SIGFOX_UNUSED(hw_api_config);
    SIGFOX_RETURN();
}

/*******************************************************************/
STM32WL3X_HW_API_status_t __attribute__((weak)) STM32WL3X_HW_API_close(void) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
   STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
   SIGFOX_RETURN();
}

/*******************************************************************/
STM32WL3X_HW_API_status_t __attribute__((weak))STM32WL3X_HW_API_init(STM32WL3X_radio_parameters_t *radio_parameters) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
   STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
   SIGFOX_UNUSED(radio_parameters);
   SIGFOX_RETURN();
}

/*******************************************************************/
STM32WL3X_HW_API_status_t __attribute__((weak)) STM32WL3X_HW_API_de_init(void) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
   STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
   SIGFOX_RETURN();
}

/*******************************************************************/
STM32WL3X_HW_API_status_t __attribute__((weak)) STM32WL3X_HW_API_get_tx_power(sfx_s8 expected_tx_power_dbm, sfx_s8 *tx_power_dbm, MRSubG_PA_DRVMode *pa_drive_mode) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
   STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
   SIGFOX_UNUSED(expected_tx_power_dbm);
   SIGFOX_UNUSED(tx_power_dbm);
   SIGFOX_UNUSED(pa_drive_mode);
   SIGFOX_RETURN();
}

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
STM32WL3X_HW_API_status_t __attribute__((weak)) STM32WL3X_HW_API_get_latency(STM32WL3X_HW_API_latency_t latency_type, sfx_u32 *latency_ms) {
    /* To be implemented by the device manufacturer */
#ifdef SIGFOX_EP_ERROR_CODES
  STM32WL3X_HW_API_status_t status = STM32WL3X_HW_API_SUCCESS;
#endif
  SIGFOX_UNUSED(latency_type);
  SIGFOX_UNUSED(latency_ms);
  SIGFOX_RETURN();
}
#endif
