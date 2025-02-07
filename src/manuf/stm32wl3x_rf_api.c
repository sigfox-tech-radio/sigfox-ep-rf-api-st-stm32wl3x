/*!*****************************************************************
 * \file    stm32wl3x_rf_api.c
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

#include "manuf/stm32wl3x_rf_api.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"
#include "manuf/mcu_api.h"
#include "manuf/rf_api.h"
// Internal radio driver.
#include "stm32wl3x_hal_mrsubg.h"
#include "stm32wl3x_ll_bus.h"
#include "stm32wl3x_ll_mrsubg.h"
#include "stm32wl3x.h"
// STM32WL3X hardware driver.
#include "board/stm32wl3x_hw_api.h"

/*** RF API local macros ***/

#define STM32WL3X_RF_API_FREQUENCY_MIN_HZ                   826000000
#define STM32WL3X_RF_API_FREQUENCY_MAX_HZ                   958000000

#define STM32WL3X_RF_API_NUMBER_OF_PA_DRIVE_MODE            3

#define STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES          40

#define STM32WL3X_RF_API_POLAR_DATARATE_MULTIPLIER          8

#define STM32WL3X_RF_API_FDEV_NEGATIVE                      0x7F // FDEV * (+1)
#define STM32WL3X_RF_API_FDEV_POSITIVE                      0x81 // FDEV * (-1)

#define STM32WL3X_RF_API_DBM_BUFFER_DEPTH                   2
#define STM32WL3X_RF_API_DBM_BUFFER_SIZE_BYTES              (STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES << 1) // Size is twice to store PA and FDEV values.
#define STM32WL3X_RF_API_DBM_BUFFER_FDEV_IDX                (STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES >> 1) // Index where deviation is performed to invert phase.
#define STM32WL3X_RF_API_DBM_TX_ALMOST_EMPTY_THRESHOLD      (STM32WL3X_RF_API_DBM_BUFFER_SIZE_BYTES >> 1) // Threshold set to the middle of a symbol.

#define STM32WL3X_RF_API_DL_BANDWIDTH_HZ                    2100
#ifdef SIGFOX_EP_BIDIRECTIONAL
#define STM32WL3X_RF_API_DL_PR_SIZE_BITS                    16
#define STM32WL3X_RF_API_DL_RSSI_THRESHOLD_DBM              -139
#define STM32WL3X_RF_API_DL_PHY_CONTENT_BUFFER_SIZE_BYTES   (((SIGFOX_DL_PHY_CONTENT_SIZE_BYTES >> 2) << 2) + 4)
#endif

#define STM32WL3X_RF_API_SEND_LATENCY_MARGIN_US             200

#define STM32WL3X_RF_API_TIMEOUT_COUNT                      1000000

/*** STM32WL3X RF API local structures ***/

/*******************************************************************/
typedef enum {
    STM32WL3X_RF_API_STATE_READY = 0,
    STM32WL3X_RF_API_STATE_TX_RAMP_UP,
    STM32WL3X_RF_API_STATE_TX_BITSTREAM,
    STM32WL3X_RF_API_STATE_TX_RAMP_DOWN,
    STM32WL3X_RF_API_STATE_TX_PADDING_BIT,
    STM32WL3X_RF_API_STATE_TX_END,
#ifdef SIGFOX_EP_BIDIRECTIONAL
    STM32WL3X_RF_API_STATE_RX_START,
    STM32WL3X_RF_API_STATE_RX,
#endif
    STM32WL3X_RF_API_STATE_LAST
} STM32WL3X_RF_API_state_t;

/*******************************************************************/
typedef union {
    struct {
        unsigned radio_irq_enable :1;
        unsigned radio_irq_process :1;
        unsigned tx_almost_empty :1;
        unsigned tx_almost_empty_0 :1;
        unsigned tx_almost_empty_1 :1;
#ifdef SIGFOX_EP_BIDIRECTIONAL
        unsigned rx_ok :1;
#endif
    } field;
    sfx_u8 all;
} STM32WL3X_RF_API_flags_t;

/*******************************************************************/
typedef struct {
    // Common.
    RF_API_config_t config;
    STM32WL3X_RF_API_state_t state;
    volatile STM32WL3X_RF_API_flags_t flags;
    // TX.
    // Warning: ramp and symbol buffers size must be a multiple of 4 because DBM peripheral only performs 32-bits access.
    sfx_u8 tx_bitstream[SIGFOX_UL_BITSTREAM_SIZE_BYTES];
    sfx_u8 tx_bitstream_size_bytes;
    sfx_u8 tx_byte_idx;
    sfx_u8 tx_bit_idx;
    sfx_u8 tx_fdev;
    sfx_u8 tx_ramp_amplitude_profile[STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES];
    sfx_u8 tx_bit0_amplitude_profile[STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES];
    volatile sfx_u8 __attribute__((aligned(32))) tx_dbm_buffer[STM32WL3X_RF_API_DBM_BUFFER_DEPTH][STM32WL3X_RF_API_DBM_BUFFER_SIZE_BYTES];
    volatile sfx_u8 *tx_dbm_buffer_ptr;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    RF_API_tx_cplt_cb_t tx_cplt_cb;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // RX.
    // Warning: buffers size must be a multiple of 4 because DBM peripheral only performs 32-bits access.
    volatile sfx_u8 __attribute__((aligned(32))) rx_dbm_buffer[STM32WL3X_RF_API_DBM_BUFFER_DEPTH][STM32WL3X_RF_API_DL_PHY_CONTENT_BUFFER_SIZE_BYTES];
    volatile sfx_u8 *rx_dbm_buffer_ptr;
    sfx_s16 dl_rssi_dbm;
    sfx_u32 sync_word_u32;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    RF_API_rx_data_received_cb_t rx_data_received_cb;
#endif
#endif
} STM32WL3X_RF_API_context_t;

/*** STM32WL3X RF API local global variables ***/

#ifdef SIGFOX_EP_VERBOSE
static const sfx_u8 STM32WL3X_RF_API_VERSION[] = "v1.0";
#endif
// Maximum output power depending on PA drive mode.
static const sfx_s8 STM32WL3X_RF_API_TX_POWER_MAX_DBM[STM32WL3X_RF_API_NUMBER_OF_PA_DRIVE_MODE] = { 10, 14, 20 };
// Amplitude profile tables for ramp and bit 0 transmission at maximum output power.
static const sfx_u8 STM32WL3X_RF_API_RAMP_AMPLITUDE_PROFILE[STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES] = { 81, 81, 81, 81, 80, 80, 80, 79, 79, 79, 77, 77, 77, 76, 76, 75, 74, 73, 72, 71, 70, 69, 68, 67, 65, 63, 61, 59, 56, 53, 50, 46, 42, 38, 34, 28, 22, 15, 0 };
static const sfx_u8 STM32WL3X_RF_API_BIT0_AMPLITUDE_PROFILE[STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES] = { 81, 81, 81, 80, 80, 79, 79, 78, 76, 74, 71, 68, 63, 59, 55, 51, 45, 34, 22, 0, 0, 22, 34, 45, 51, 55, 59, 63, 68, 71, 74, 76, 78, 79, 79, 80, 80, 81, 81, 81 };
#ifdef SIGFOX_EP_BIDIRECTIONAL
static const sfx_u8 STM32WL3X_RF_API_SIGFOX_DL_FT[SIGFOX_DL_FT_SIZE_BYTES] = SIGFOX_DL_FT;
#endif
#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
// Latency values (for core clock at 32MHz and SPI interface at 8MHz).
static sfx_u32 STM32WL3X_RF_API_LATENCY_MS[RF_API_LATENCY_LAST] = {
    0, // Wake-up (8.6µs + HW latency).
    3, // TX init (3.3ms).
    0, // Send start (depends on bit rate and will be computed during init function).
    0, // Send stop (depends on bit rate and will be computed during init function).
    0, // TX de-init (150µs).
    0, // Sleep (5µs + HW latency).
#ifdef SIGFOX_EP_BIDIRECTIONAL
    4, // RX init (3.4ms).
    1, // Receive start (130µs).
    0, // Receive stop (6µs).
    0, // RX de-init (15µs).
#endif
};
#endif
static STM32WL3X_RF_API_context_t stm32wl3x_rf_api_ctx;

/*** STM32WL3X RF API local functions ***/

/*******************************************************************/
void MR_SUBG_IRQHandler(void) {
    // Check buffer 0 flag.
    if ((MR_SUBG_GLOB_STATUS->RFSEQ_IRQ_STATUS) & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_ALMOST_EMPTY_0_F) {
        // Set local flags if IRQ is enabled.
        stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty_0 = stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable;
        stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty = stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable;
        stm32wl3x_rf_api_ctx.flags.field.radio_irq_process = stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable;
        // Clear peripheral flag.
        MR_SUBG_GLOB_STATUS->RFSEQ_IRQ_STATUS |= MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_ALMOST_EMPTY_0_F;
    }
    // Check buffer 1 flag.
    if ((MR_SUBG_GLOB_STATUS->RFSEQ_IRQ_STATUS) & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_ALMOST_EMPTY_1_F) {
        // Set local flags.
        stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty_1 = stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable;
        stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty = stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable;
        stm32wl3x_rf_api_ctx.flags.field.radio_irq_process = stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable;
        // Clear peripheral flag.
        MR_SUBG_GLOB_STATUS->RFSEQ_IRQ_STATUS |= MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_TX_ALMOST_EMPTY_1_F;
    }
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Check RX done flag.
    if (((MR_SUBG_GLOB_STATUS->RFSEQ_IRQ_STATUS) & MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F) != 0) {
        // Set local flags.
        stm32wl3x_rf_api_ctx.flags.field.rx_ok = stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable;
        stm32wl3x_rf_api_ctx.flags.field.radio_irq_process = stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable;
        // Clear peripheral flag.
        MR_SUBG_GLOB_STATUS->RFSEQ_IRQ_STATUS |= MR_SUBG_GLOB_STATUS_RFSEQ_IRQ_STATUS_RX_OK_F;
    }
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Call Sigfox callback to process IRQ in main context.
    if (stm32wl3x_rf_api_ctx.flags.field.radio_irq_process != 0) {
        // Set local flags.
        stm32wl3x_rf_api_ctx.config.process_cb();
    }
#endif
}

/*******************************************************************/
static void _STM32WL3X_rf_api_update_tx_dbm_buffer(void) {
    // Update pointer.
    if (stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty_0 != 0) {
        // Move local pointer to buffer 1 and clear flag.
        stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr = &(stm32wl3x_rf_api_ctx.tx_dbm_buffer[1][0]);
        stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty_0 = 0;
    }
    if (stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty_1 != 0) {
        // Move local pointer to buffer 0 and clear flag.
        stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr = &(stm32wl3x_rf_api_ctx.tx_dbm_buffer[0][0]);
        stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty_1 = 0;
    }
    // Clear global flag.
    stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty = 0;
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static void _STM32WL3X_rf_api_update_rx_dbm_buffer(void) {
    // Check buffer in use.
    sfx_u8 buffer_in_use = (((MR_SUBG_GLOB_STATUS->DATABUFFER_INFO) >> MR_SUBG_GLOB_STATUS_DATABUFFER_INFO_CURRENT_DATABUFFER_Pos) & 0x00000001);
    // Update local pointer.
    stm32wl3x_rf_api_ctx.rx_dbm_buffer_ptr = (sfx_u8*) stm32wl3x_rf_api_ctx.rx_dbm_buffer[buffer_in_use];
}
#endif

/*******************************************************************/
static void _STM32WL3X_rf_api_reset_context(void) {
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Local variables.
    sfx_u8 idx = 0;
#endif
    // Init context.
    stm32wl3x_rf_api_ctx.state = STM32WL3X_RF_API_STATE_READY;
    stm32wl3x_rf_api_ctx.flags.all = 0;
    stm32wl3x_rf_api_ctx.config.rc = SIGFOX_NULL;
    stm32wl3x_rf_api_ctx.tx_bitstream_size_bytes = 0;
    stm32wl3x_rf_api_ctx.tx_byte_idx = 0;
    stm32wl3x_rf_api_ctx.tx_bit_idx = 0;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    stm32wl3x_rf_api_ctx.config.process_cb = SIGFOX_NULL;
    stm32wl3x_rf_api_ctx.config.error_cb = SIGFOX_NULL;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Build synchronization word in integer format (0xB2270000).
    stm32wl3x_rf_api_ctx.sync_word_u32 = 0;
    for (idx = 0; idx < SIGFOX_DL_FT_SIZE_BYTES; idx++) {
        stm32wl3x_rf_api_ctx.sync_word_u32 |= STM32WL3X_RF_API_SIGFOX_DL_FT[idx] << ((3 - idx) << 3);
    }
#endif
}

/*******************************************************************/
static RF_API_status_t _STM32WL3X_rf_api_wait_for_state_switch(MRSubGFSMState new_state) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    sfx_u32 loop_count = 0;
    // Wait until state is reached.
    while (LL_MRSubG_GetRadioFSMState() != new_state) {
        // Manage timeout.
        loop_count++;
        if (loop_count > STM32WL3X_RF_API_TIMEOUT_COUNT) {
            SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_STATE_SWITCH_TIMEOUT);
        }
    }
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
static void _STM32WL3X_rf_api_compute_amplitude_tables(sfx_s8 stm32wl3x_tx_power_dbm, sfx_u8 pa_drive_mode_index) {
    // Local variables.
    sfx_u8 pa_value_min = STM32WL3X_RF_API_RAMP_AMPLITUDE_PROFILE[STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES - 1];
    sfx_u8 pa_value_max = STM32WL3X_RF_API_RAMP_AMPLITUDE_PROFILE[0];
    sfx_u8 new_pa_value_max = 0;
    sfx_u8 idx = 0;
    // Compute maximum PA value for the given output power.
    if (stm32wl3x_tx_power_dbm >= STM32WL3X_RF_API_TX_POWER_MAX_DBM[pa_drive_mode_index]) {
        new_pa_value_max = pa_value_max;
    }
    else {
        new_pa_value_max = (sfx_u8) (pa_value_max - ((STM32WL3X_RF_API_TX_POWER_MAX_DBM[pa_drive_mode_index] - stm32wl3x_tx_power_dbm) << 1));
    }
    // Sub-symbols loop.
    for (idx = 0; idx < STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES; idx++) {
        // Compute amplitude tables.
        stm32wl3x_rf_api_ctx.tx_ramp_amplitude_profile[idx] = (sfx_u8) (pa_value_min + ((STM32WL3X_RF_API_RAMP_AMPLITUDE_PROFILE[idx] - pa_value_min) * (new_pa_value_max - pa_value_min)) / (pa_value_max - pa_value_min));
        stm32wl3x_rf_api_ctx.tx_bit0_amplitude_profile[idx] = (sfx_u8) (pa_value_min + ((STM32WL3X_RF_API_BIT0_AMPLITUDE_PROFILE[idx] - pa_value_min) * (new_pa_value_max - pa_value_min)) / (pa_value_max - pa_value_min));
    }
}

/*******************************************************************/
static RF_API_status_t _STM32WL3X_rf_api_internal_process(void) {
    // Local variables.
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    // Perform state machine.
    switch (stm32wl3x_rf_api_ctx.state) {
    case STM32WL3X_RF_API_STATE_READY:
        // Nothing to do.
        break;
    case STM32WL3X_RF_API_STATE_TX_RAMP_UP:
        // Fill ramp-up.
        for (idx = 0; idx < STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES; idx++) {
            stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr[(idx << 1) + 0] = 0; // Deviation.
            stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr[(idx << 1) + 1] = stm32wl3x_rf_api_ctx.tx_ramp_amplitude_profile[STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES - idx - 1]; // PA output power.
        }
        // Enable interrupt.
        stm32wl3x_rf_api_ctx.flags.all = 0;
        stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable = 1;
        // Start radio.
        LL_MRSubG_StrobeCommand(CMD_LOCKTX);
#ifdef SIGFOX_EP_ERROR_CODES
        status = _STM32WL3X_rf_api_wait_for_state_switch(STATE_LOCKONTX);
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _STM32WL3X_rf_api_wait_for_state_switch(STATE_LOCKONTX);
#endif
        LL_MRSubG_StrobeCommand(CMD_TX);
#ifdef SIGFOX_EP_ERROR_CODES
        status = _STM32WL3X_rf_api_wait_for_state_switch(STATE_TX);
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _STM32WL3X_rf_api_wait_for_state_switch(STATE_TX);
#endif
        // Update state.
        stm32wl3x_rf_api_ctx.state = STM32WL3X_RF_API_STATE_TX_BITSTREAM;
        break;
    case STM32WL3X_RF_API_STATE_TX_BITSTREAM:
        // Check DBM flag.
        if (stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty != 0) {
            // Update TX buffer pointer.
            _STM32WL3X_rf_api_update_tx_dbm_buffer();
            // Check bit.
            if ((stm32wl3x_rf_api_ctx.tx_bitstream[stm32wl3x_rf_api_ctx.tx_byte_idx] & (1 << (7 - stm32wl3x_rf_api_ctx.tx_bit_idx))) == 0) {
                // Phase shift and amplitude shaping required.
                stm32wl3x_rf_api_ctx.tx_fdev = (stm32wl3x_rf_api_ctx.tx_fdev == STM32WL3X_RF_API_FDEV_NEGATIVE) ? STM32WL3X_RF_API_FDEV_POSITIVE : STM32WL3X_RF_API_FDEV_NEGATIVE; // Toggle deviation.
                for (idx = 0; idx < STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES; idx++) {
                    stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr[(idx << 1) + 0] = (idx == STM32WL3X_RF_API_DBM_BUFFER_FDEV_IDX) ? stm32wl3x_rf_api_ctx.tx_fdev : 0; // Deviation.
                    stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr[(idx << 1) + 1] = stm32wl3x_rf_api_ctx.tx_bit0_amplitude_profile[idx]; // PA output power.
                }
            }
            else {
                // Constant CW.
                for (idx = 0; idx < STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES; idx++) {
                    stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr[(idx << 1) + 0] = 0; // Deviation.
                    stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr[(idx << 1) + 1] = stm32wl3x_rf_api_ctx.tx_bit0_amplitude_profile[0]; // PA output power.
                }
            }
            // Increment bit index..
            stm32wl3x_rf_api_ctx.tx_bit_idx++;
            if (stm32wl3x_rf_api_ctx.tx_bit_idx >= 8) {
                // Reset bit index.
                stm32wl3x_rf_api_ctx.tx_bit_idx = 0;
                // Increment byte index.
                stm32wl3x_rf_api_ctx.tx_byte_idx++;
                // Check end of bitstream.
                if (stm32wl3x_rf_api_ctx.tx_byte_idx >= (stm32wl3x_rf_api_ctx.tx_bitstream_size_bytes)) {
                    stm32wl3x_rf_api_ctx.state = STM32WL3X_RF_API_STATE_TX_RAMP_DOWN;
                }
            }
        }
        break;
    case STM32WL3X_RF_API_STATE_TX_RAMP_DOWN:
        // Check DBM flag.
        if (stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty != 0) {
            // Update TX buffer pointer.
            _STM32WL3X_rf_api_update_tx_dbm_buffer();
            // Fill ramp-down.
            for (idx = 0; idx < STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES; idx++) {
                stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr[(idx << 1) + 0] = 0; // FDEV.
                stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr[(idx << 1) + 1] = stm32wl3x_rf_api_ctx.tx_ramp_amplitude_profile[idx]; // PA output power for ramp-down.
            }
            // Update state.
            stm32wl3x_rf_api_ctx.state = STM32WL3X_RF_API_STATE_TX_PADDING_BIT;
        }
        break;
    case STM32WL3X_RF_API_STATE_TX_PADDING_BIT:
        // Check DBM flag.
        if (stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty != 0) {
            // Update TX buffer pointer.
            _STM32WL3X_rf_api_update_tx_dbm_buffer();
            // Padding bit to ensure last ramp down is completely transmitted.
            for (idx = 0; idx < STM32WL3X_RF_API_DBM_BUFFER_SIZE_BYTES; idx++) {
                stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr[idx] = 0x00;
            }
            // Update state.
            stm32wl3x_rf_api_ctx.state = STM32WL3X_RF_API_STATE_TX_END;
        }
        break;
    case STM32WL3X_RF_API_STATE_TX_END:
        // Check DBM flag.
        if (stm32wl3x_rf_api_ctx.flags.field.tx_almost_empty != 0) {
            // Stop radio.
            LL_MRSubG_StrobeCommand(CMD_SABORT);
            // Disable interrupt.
            stm32wl3x_rf_api_ctx.flags.all = 0;
#ifdef SIGFOX_EP_ASYNCHRONOUS
            // Call TX completion callback.
            stm32wl3x_rf_api_ctx.tx_cplt_cb();
#endif
            // Update state.
            stm32wl3x_rf_api_ctx.state = STM32WL3X_RF_API_STATE_READY;
        }
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case STM32WL3X_RF_API_STATE_RX_START:
        // Enable external GPIO interrupt.
        stm32wl3x_rf_api_ctx.flags.all = 0;
        stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable = 1;
        // Start radio.
        LL_MRSubG_StrobeCommand(CMD_LOCKRX);
#ifdef SIGFOX_EP_ERROR_CODES
        status = _STM32WL3X_rf_api_wait_for_state_switch(STATE_LOCKONRX);
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _STM32WL3X_rf_api_wait_for_state_switch(STATE_LOCKONRX);
#endif
        LL_MRSubG_StrobeCommand(CMD_RX);
#ifdef SIGFOX_EP_ERROR_CODES
        status = _STM32WL3X_rf_api_wait_for_state_switch(STATE_RX);
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _STM32WL3X_rf_api_wait_for_state_switch(STATE_RX);
#endif
        // Update state.
        stm32wl3x_rf_api_ctx.state = STM32WL3X_RF_API_STATE_RX;
        break;
    case STM32WL3X_RF_API_STATE_RX:
        // Check RX data flag.
        if (stm32wl3x_rf_api_ctx.flags.field.rx_ok != 0) {
            // Update DBM pointer and RSSI.
            _STM32WL3X_rf_api_update_rx_dbm_buffer();
            stm32wl3x_rf_api_ctx.dl_rssi_dbm = (sfx_s16) HAL_MRSubG_GetRssidBm();
            // Stop radio.
            LL_MRSubG_StrobeCommand(CMD_SABORT);
            // Clear flag.
            stm32wl3x_rf_api_ctx.flags.field.rx_ok = 0;
            // Disable interrupt.
            stm32wl3x_rf_api_ctx.flags.field.radio_irq_enable = 0;
#ifdef SIGFOX_EP_ASYNCHRONOUS
            // Call RX completion callback.
            stm32wl3x_rf_api_ctx.rx_data_received_cb();
#endif
            // Update state.
            stm32wl3x_rf_api_ctx.state = STM32WL3X_RF_API_STATE_READY;
        }
        break;
#endif /* SIGFOX_EP_BIDIRECTIONAL */
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_STATE);
        break;
    }
errors:
    // Clear process flag.
    stm32wl3x_rf_api_ctx.flags.field.radio_irq_process = 0;
    SIGFOX_RETURN();
}

/*** STM32WL3X RF API functions ***/

#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_open(RF_API_config_t *rf_api_config) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
    STM32WL3X_HW_API_status_t STM32WL3X_hw_api_status = STM32WL3X_HW_API_SUCCESS;
#endif
    STM32WL3X_HW_API_config_t hw_config;
    // Reset context.
    _STM32WL3X_rf_api_reset_context();
    // Save parameters.
    stm32wl3x_rf_api_ctx.config.rc = (rf_api_config->rc);
#ifdef SIGFOX_EP_ASYNCHRONOUS
    stm32wl3x_rf_api_ctx.config.process_cb = (rf_api_config -> process_cb);
    stm32wl3x_rf_api_ctx.config.error_cb = (rf_api_config -> error_cb);
#endif
    // Init board.
    hw_config.rc = (rf_api_config->rc);
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_hw_api_status = STM32WL3X_HW_API_open(&hw_config);
    STM32WL3X_HW_API_check_status((RF_API_status_t) STM32WL3X_RF_API_ERROR_DRIVER_STM32WL3X_HW_API);
#else
    STM32WL3X_HW_API_open(&hw_config);
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_close(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    STM32WL3X_HW_API_status_t STM32WL3X_hw_api_status = STM32WL3X_HW_API_SUCCESS;
#endif
    // Reset context.
    _STM32WL3X_rf_api_reset_context();
    // De-init board.
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_hw_api_status = STM32WL3X_HW_API_close();
    STM32WL3X_HW_API_check_status((RF_API_status_t) STM32WL3X_RF_API_ERROR_DRIVER_STM32WL3X_HW_API);
#else
    STM32WL3X_HW_API_close();
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_process(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    // Run internal process.
    while (stm32wl3x_rf_api_ctx.flags.field.radio_irq_process != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
        status = _STM32WL3X_rf_api_internal_process();
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _STM32WL3X_rf_api_internal_process();
#endif
    }
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_wake_up(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    sfx_u32 loop_count = 0;
    // Turn radio on.
    PWR->CR2 |= PWR_CR2_RFREGEN;
    // Wait for RF regulator to be stable.
    while (((PWR->CR2) & PWR_CR2_RFREGRDY) == 0) {
        loop_count++;
        if (loop_count > STM32WL3X_RF_API_TIMEOUT_COUNT) {
            SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_REGULATOR_READY);
        }
    }
    // Enable peripheral clock.
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_MRSUBG);
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_sleep(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    // Disable peripheral clock.
    LL_APB2_GRP1_DisableClock(LL_APB2_GRP1_PERIPH_MRSUBG);
    // Turn radio off.
    PWR->CR2 &= ~(PWR_CR2_RFREGEN);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_init(RF_API_radio_parameters_t *radio_parameters) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
    STM32WL3X_HW_API_status_t STM32WL3X_hw_api_status = STM32WL3X_HW_API_SUCCESS;
#endif
    STM32WL3X_radio_parameters_t STM32WL3X_hw_radio_parameters;
    SMRSubGConfig radio_config;
    MRSubG_PA_DRVMode pa_drive_mode = PA_DRV_TX;
    sfx_u8 pa_drive_mode_index = 0;
    sfx_s8 expected_tx_power_dbm = 0;
    sfx_s8 tx_power_dbm = 0;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    MRSubG_PcktBasicFields downlink_pkt_init;
#endif
    // Common parameters.
    radio_config.outputPower = 0;
    radio_config.dsssExp = 0;
    radio_config.lBandwidth = STM32WL3X_RF_API_DL_BANDWIDTH_HZ;
    // Frequency.
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if (((radio_parameters->frequency_hz) < STM32WL3X_RF_API_FREQUENCY_MIN_HZ) || ((radio_parameters->frequency_hz) > STM32WL3X_RF_API_FREQUENCY_MAX_HZ)) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_FREQUENCY);
    }
#endif
    radio_config.lFrequencyBase = (radio_parameters->frequency_hz);
    // Modulation and bit rate.
    switch (radio_parameters->modulation) {
    case RF_API_MODULATION_NONE:
        radio_config.xModulationSelect = MOD_CW;
        radio_config.lDatarate = 100; // Unused value but should be > 0 to be able to stop CW with SABORT command.
#ifdef SIGFOX_EP_BIDIRECTIONAL
        radio_config.lFreqDev = (radio_parameters->deviation_hz);
#else
        radio_config.lFreqDev = 0;
#endif
        // Disable PA ramp (workaround to be able to stop CW with SABORT command).
        MR_SUBG_GLOB_STATIC->PA_CONFIG &= ~MR_SUBG_GLOB_STATIC_PA_CONFIG_PA_RAMP_ENABLE;
        MR_SUBG_GLOB_STATIC->PA_CONFIG &= ~MR_SUBG_GLOB_STATIC_PA_CONFIG_PA_RAMP_STEP_WIDTH;
        break;
    case RF_API_MODULATION_DBPSK:
        radio_config.xModulationSelect = MOD_POLAR;
        radio_config.lDatarate = (sfx_u32) (((radio_parameters->bit_rate_bps) * STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES) / (STM32WL3X_RF_API_POLAR_DATARATE_MULTIPLIER));
        radio_config.lFreqDev = (sfx_u32) (((radio_parameters->bit_rate_bps) * STM32WL3X_RF_API_SYMBOL_PROFILE_SIZE_BYTES) >> 1);
        // Enable PA ramp.
        MR_SUBG_GLOB_STATIC->PA_CONFIG |= MR_SUBG_GLOB_STATIC_PA_CONFIG_PA_RAMP_ENABLE;
        break;
    case RF_API_MODULATION_GFSK:
        radio_config.xModulationSelect = MOD_2GFSK1;
        radio_config.lDatarate = (radio_parameters->bit_rate_bps);
#ifdef SIGFOX_EP_BIDIRECTIONAL
        radio_config.lFreqDev = (radio_parameters->deviation_hz);
#else
        radio_config.lFreqDev = 0;
#endif
        // Enable PA ramp.
        MR_SUBG_GLOB_STATIC->PA_CONFIG |= MR_SUBG_GLOB_STATIC_PA_CONFIG_PA_RAMP_ENABLE;
        break;
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_MODULATION);
        break;
    }
    HAL_MRSubG_Init(&radio_config);
    // Configure specific registers.
    switch (radio_parameters->rf_mode) {
    case RF_API_MODE_TX:
        // Enable IRQ of current buffer.
        MR_SUBG_GLOB_DYNAMIC->RFSEQ_IRQ_ENABLE = (MR_SUBG_GLOB_DYNAMIC_RFSEQ_IRQ_ENABLE_TX_ALMOST_EMPTY_0_E | MR_SUBG_GLOB_DYNAMIC_RFSEQ_IRQ_ENABLE_TX_ALMOST_EMPTY_1_E);
        // Set TX mode.
        LL_MRSubG_SetTXMode(TX_DIRECT_BUFFERS);
        LL_MRSUBG_SetPacketLength(0);
        LL_MRSubG_SetAlmostEmptyThresholdTx(STM32WL3X_RF_API_DBM_TX_ALMOST_EMPTY_THRESHOLD);
        LL_MRSubG_SetPAMode(PA_LEGACY);
        LL_MRSubG_SetPALevelMaxIndex(0);
        // Enable PA interpolator and clocking during lock.
        MR_SUBG_GLOB_STATIC->PA_CONFIG |= MR_SUBG_GLOB_STATIC_PA_CONFIG_PA_INTERP_EN;
        MR_SUBG_GLOB_DYNAMIC->MOD0_CONFIG |= MR_SUBG_GLOB_DYNAMIC_MOD0_CONFIG_PA_CLKON_LOCKONTX;
        // Set RAM buffers address and size.
        MR_SUBG_GLOB_STATIC->DATABUFFER0_PTR = (sfx_u32) &(stm32wl3x_rf_api_ctx.tx_dbm_buffer[0][0]);
        MR_SUBG_GLOB_STATIC->DATABUFFER1_PTR = (sfx_u32) &(stm32wl3x_rf_api_ctx.tx_dbm_buffer[1][0]);
        MR_SUBG_GLOB_STATIC->DATABUFFER_SIZE = (sfx_u32) STM32WL3X_RF_API_DBM_BUFFER_SIZE_BYTES;
        // DBM starts with buffer 0.
        stm32wl3x_rf_api_ctx.tx_dbm_buffer_ptr = &(stm32wl3x_rf_api_ctx.tx_dbm_buffer[0][0]);
        // Get effective output power to program.
#ifdef SIGFOX_EP_TX_POWER_DBM_EIRP
        expected_tx_power_dbm = SIGFOX_EP_TX_POWER_DBM_EIRP;
#else
        expected_tx_power_dbm = (radio_parameters->tx_power_dbm_eirp);
#endif
#ifdef SIGFOX_EP_ERROR_CODES
        STM32WL3X_hw_api_status = STM32WL3X_HW_API_get_tx_power(expected_tx_power_dbm, &tx_power_dbm, &pa_drive_mode);
        STM32WL3X_HW_API_check_status((RF_API_status_t) STM32WL3X_RF_API_ERROR_DRIVER_STM32WL3X_HW_API);
#else
        STM32WL3X_HW_API_get_tx_power(expected_tx_power_dbm, &tx_power_dbm, &pa_drive_mode);
#endif
        // Check returned value.
        if ((pa_drive_mode == 0) || (pa_drive_mode > STM32WL3X_RF_API_NUMBER_OF_PA_DRIVE_MODE)) {
            SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_PA_DRIVE_MODE);
        }
        // Convert to index.
        pa_drive_mode_index = (pa_drive_mode - 1);
        // Check power.
        if (tx_power_dbm > STM32WL3X_RF_API_TX_POWER_MAX_DBM[pa_drive_mode_index]) {
            SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_TX_POWER);
        }
        // Set output power.
        if ((radio_parameters->modulation) == RF_API_MODULATION_NONE) {
            HAL_MRSubG_SetPALeveldBm(0, tx_power_dbm, pa_drive_mode);
        }
        else {
            LL_MRSubG_SetPADriveMode(pa_drive_mode);
            _STM32WL3X_rf_api_compute_amplitude_tables(tx_power_dbm, pa_drive_mode_index);
        }
#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
        // Start latency = 1 symbol of ramp-up.
        STM32WL3X_RF_API_LATENCY_MS[RF_API_LATENCY_SEND_START] = ((1000 - STM32WL3X_RF_API_SEND_LATENCY_MARGIN_US) / ((sfx_u32) (radio_parameters->bit_rate_bps)));
        // Stop latency = 1 symbol of ramp-down + half of padding symbol (since IRQ is raised at the middle of the symbol).
        STM32WL3X_RF_API_LATENCY_MS[RF_API_LATENCY_SEND_STOP] = ((1500 - STM32WL3X_RF_API_SEND_LATENCY_MARGIN_US) / ((sfx_u32) (radio_parameters->bit_rate_bps)));
#endif
        break;
#if (defined SIGFOX_EP_BIDIRECTIONAL) || ((defined SIGFOX_EP_REGULATORY && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)))
    case RF_API_MODE_RX:
#ifdef SIGFOX_EP_BIDIRECTIONAL
        // Configure IRQ.
        MR_SUBG_GLOB_DYNAMIC->RFSEQ_IRQ_ENABLE = MR_SUBG_GLOB_DYNAMIC_RFSEQ_IRQ_ENABLE_RX_OK_E;
        // Set RX mode.
        LL_MRSubG_SetRXMode(RX_NORMAL);
        // Set RAM buffers address and size.
        MR_SUBG_GLOB_STATIC->DATABUFFER0_PTR = (sfx_u32) &(stm32wl3x_rf_api_ctx.rx_dbm_buffer[0]);
        MR_SUBG_GLOB_STATIC->DATABUFFER1_PTR = (sfx_u32) &(stm32wl3x_rf_api_ctx.rx_dbm_buffer[1]);
        MR_SUBG_GLOB_STATIC->DATABUFFER_SIZE = (sfx_u32) STM32WL3X_RF_API_DL_PHY_CONTENT_BUFFER_SIZE_BYTES;
        stm32wl3x_rf_api_ctx.rx_dbm_buffer_ptr = (sfx_u8*) &(stm32wl3x_rf_api_ctx.rx_dbm_buffer[0]);
        // RSSI threshold.
        HAL_MRSubG_SetRSSIThreshold(STM32WL3X_RF_API_DL_RSSI_THRESHOLD_DBM);
        // Disable equalization, antenna switching and carrier sense.
        LL_MRSubG_SetISIEqualizer(ISI_EQ_DISABLED);
        // Downlink packet structure.
        downlink_pkt_init.PreambleLength = STM32WL3X_RF_API_DL_PR_SIZE_BITS;
        downlink_pkt_init.PostambleLength = 0;
        downlink_pkt_init.SyncLength = ((SIGFOX_DL_FT_SIZE_BYTES << 3) - 1);
        downlink_pkt_init.SyncWord = stm32wl3x_rf_api_ctx.sync_word_u32;
        downlink_pkt_init.FixVarLength = FIXED;
        downlink_pkt_init.PreambleSequence = PRE_SEQ_1010;
        downlink_pkt_init.PostambleSequence = POST_SEQ_OTHER;
        downlink_pkt_init.CrcMode = PKT_NO_CRC;
        downlink_pkt_init.Coding = CODING_NONE;
        downlink_pkt_init.DataWhitening = DISABLE;
        downlink_pkt_init.LengthWidth = BYTE_LEN_1;
        downlink_pkt_init.SyncPresent = ENABLE;
        HAL_MRSubG_PacketBasicInit(&downlink_pkt_init);
        LL_MRSUBG_SetPacketLength(SIGFOX_DL_PHY_CONTENT_SIZE_BYTES);
        // Disable AFC.
        MR_SUBG_RADIO->AFC2_CONFIG &= ~MR_SUBG_RADIO_AFC2_CONFIG_AFC_EN;
        break;
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#endif
    default:
        SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_MODE);
        break;
    }
    // Optional init on hardware side.
    STM32WL3X_hw_radio_parameters.rf_mode = (radio_parameters->rf_mode);
    STM32WL3X_hw_radio_parameters.expected_tx_power_dbm = expected_tx_power_dbm;
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_hw_api_status = STM32WL3X_HW_API_init(&STM32WL3X_hw_radio_parameters);
    STM32WL3X_HW_API_check_status((RF_API_status_t) STM32WL3X_RF_API_ERROR_DRIVER_STM32WL3X_HW_API);
#else
    STM32WL3X_HW_API_init(&STM32WL3X_hw_radio_parameters);
#endif
    // Go to ready state.
    LL_MRSubG_StrobeCommand(CMD_SABORT);
#ifdef SIGFOX_EP_ERROR_CODES
    status = _STM32WL3X_rf_api_wait_for_state_switch(STATE_IDLE);
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _STM32WL3X_rf_api_wait_for_state_switch(STATE_IDLE);
#endif
    // Enable radio interrupt.
    NVIC_SetPriority(MR_SUBG_IRQn, 0);
    NVIC_EnableIRQ(MR_SUBG_IRQn);
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_de_init(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    RF_API_status_t status = RF_API_SUCCESS;
    STM32WL3X_HW_API_status_t STM32WL3X_hw_api_status = STM32WL3X_HW_API_SUCCESS;
#endif
    // Disable external GPIO interrupt.
    stm32wl3x_rf_api_ctx.flags.all = 0;
    // Stop radio.
    LL_MRSubG_StrobeCommand(CMD_SABORT);
#ifdef SIGFOX_EP_ERROR_CODES
    status = _STM32WL3X_rf_api_wait_for_state_switch(STATE_IDLE);
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _STM32WL3X_rf_api_wait_for_state_switch(STATE_IDLE);
#endif
    // Optional release on hardware side.
#ifdef SIGFOX_EP_ERROR_CODES
    STM32WL3X_hw_api_status = STM32WL3X_HW_API_de_init();
    STM32WL3X_HW_API_check_status((RF_API_status_t) STM32WL3X_RF_API_ERROR_DRIVER_STM32WL3X_HW_API);
#else
    STM32WL3X_HW_API_de_init();
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    // Disable radio interrupt.
    NVIC_DisableIRQ(MR_SUBG_IRQn);
    SIGFOX_RETURN();
}

/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_send(RF_API_tx_data_t *tx_data) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    sfx_u8 idx = 0;
    // Store TX data.
    stm32wl3x_rf_api_ctx.tx_bitstream_size_bytes = (tx_data->bitstream_size_bytes);
    for (idx = 0; idx < (stm32wl3x_rf_api_ctx.tx_bitstream_size_bytes); idx++) {
        stm32wl3x_rf_api_ctx.tx_bitstream[idx] = (tx_data->bitstream)[idx];
    }
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Store callback.
    stm32wl3x_rf_api_ctx.tx_cplt_cb = (tx_data -> cplt_cb);
#endif
    // Init state.
    stm32wl3x_rf_api_ctx.tx_bit_idx = 0;
    stm32wl3x_rf_api_ctx.tx_byte_idx = 0;
    stm32wl3x_rf_api_ctx.flags.all = 0;
    // Trigger TX.
    stm32wl3x_rf_api_ctx.state = STM32WL3X_RF_API_STATE_TX_RAMP_UP;
#ifdef SIGFOX_EP_ERROR_CODES
    status = _STM32WL3X_rf_api_internal_process();
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _STM32WL3X_rf_api_internal_process();
#endif
#ifndef SIGFOX_EP_ASYNCHRONOUS
    // Wait for transmission to complete.
    while (stm32wl3x_rf_api_ctx.state != STM32WL3X_RF_API_STATE_READY) {
        // Wait for radio interrupt.
        while (stm32wl3x_rf_api_ctx.flags.field.radio_irq_process == 0);
        // Call process function.
#ifdef SIGFOX_EP_ERROR_CODES
        status = _STM32WL3X_rf_api_internal_process();
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _STM32WL3X_rf_api_internal_process();
#endif
    }
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_receive(RF_API_rx_data_t *rx_data) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#ifndef SIGFOX_EP_ASYNCHRONOUS
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Store callback.
    stm32wl3x_rf_api_ctx.rx_data_received_cb = (rx_data -> data_received_cb);
#else
    sfx_bool dl_timeout = SIGFOX_FALSE;
    // Reset flag.
    (rx_data->data_received) = SIGFOX_FALSE;
#endif
    // Init state.
    stm32wl3x_rf_api_ctx.flags.all = 0;
    // Trigger RX.
    stm32wl3x_rf_api_ctx.state = STM32WL3X_RF_API_STATE_RX_START;
#ifdef SIGFOX_EP_ERROR_CODES
    status = _STM32WL3X_rf_api_internal_process();
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _STM32WL3X_rf_api_internal_process();
#endif
#ifndef SIGFOX_EP_ASYNCHRONOUS
    // Wait for reception to complete.
    while (stm32wl3x_rf_api_ctx.state != STM32WL3X_RF_API_STATE_READY) {
        // Wait for GPIO interrupt.
        while (stm32wl3x_rf_api_ctx.flags.field.radio_irq_process == 0) {
            // Check timeout.
#ifdef SIGFOX_EP_ERROR_CODES
            mcu_api_status = MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
            MCU_API_check_status((RF_API_status_t) STM32WL3X_RF_API_ERROR_DRIVER_MCU_API);
#else
            MCU_API_timer_status(MCU_API_TIMER_INSTANCE_T_RX, &dl_timeout);
#endif
            // Exit if timeout.
            if (dl_timeout == SIGFOX_TRUE) {
                // Stop radio.
                LL_MRSubG_StrobeCommand(CMD_SABORT);
                goto errors;
            }
        }
        // Call process function.
#ifdef SIGFOX_EP_ERROR_CODES
        status = _STM32WL3X_rf_api_internal_process();
        SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
        _STM32WL3X_rf_api_internal_process();
#endif
    }
    // Update status flag.
    (rx_data->data_received) = SIGFOX_TRUE;
#endif
#if (defined SIGFOX_EP_ERROR_CODES) || !(defined SIGFOX_EP_ASYNCHRONOUS)
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameters.
    if ((dl_phy_content == SIGFOX_NULL) || (dl_rssi_dbm == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_NULL_PARAMETER);
    }
    if (dl_phy_content_size > SIGFOX_DL_PHY_CONTENT_SIZE_BYTES) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_BUFFER_SIZE);
    }
#endif
    // Fill data.
    for (idx = 0; idx < dl_phy_content_size; idx++) {
        dl_phy_content[idx] = stm32wl3x_rf_api_ctx.rx_dbm_buffer_ptr[idx];
    }
    (*dl_rssi_dbm) = (sfx_s16) stm32wl3x_rf_api_ctx.dl_rssi_dbm;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    /* To be implemented by the device manufacturer */
    SIGFOX_UNUSED(carrier_sense_params);
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32 *latency_ms) {
    // Local variables.
    sfx_u32 hw_api_latency[STM32WL3X_HW_API_LATENCY_LAST];
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
    STM32WL3X_HW_API_status_t STM32WL3X_hw_api_status = STM32WL3X_HW_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Reset result.
    (*latency_ms) = 0;
    // Check parameter.
    if (latency_type >= RF_API_LATENCY_LAST) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_LATENCY_TYPE);
    }
#endif
    // Set base latency.
    (*latency_ms) = STM32WL3X_RF_API_LATENCY_MS[latency_type];
    // Read hardware latencies.
    for (idx = 0; idx < STM32WL3X_HW_API_LATENCY_LAST; idx++) {
        // Reset value.
        hw_api_latency[idx] = 0;
#ifdef SIGFOX_EP_ERROR_CODES
        STM32WL3X_hw_api_status = STM32WL3X_HW_API_get_latency(idx, &(hw_api_latency[idx]));
        STM32WL3X_HW_API_check_status((RF_API_status_t) STM32WL3X_RF_API_ERROR_DRIVER_STM32WL3X_HW_API);
#else
        STM32WL3X_HW_API_get_latency(idx, &(hw_api_latency[idx]));
#endif
    }
    // Add HW API latencies.
    if (latency_type == RF_API_LATENCY_INIT_TX) {
        (*latency_ms) += hw_api_latency[STM32WL3X_HW_API_LATENCY_INIT_TX];
    }
    if (latency_type == RF_API_LATENCY_DE_INIT_TX) {
        (*latency_ms) += hw_api_latency[STM32WL3X_HW_API_LATENCY_DE_INIT_TX];
    }
#ifdef SIGFOX_EP_BIDIRECTIONAL
    if (latency_type == RF_API_LATENCY_INIT_RX) {
        (*latency_ms) += hw_api_latency[STM32WL3X_HW_API_LATENCY_INIT_RX];
    }
    if (latency_type == RF_API_LATENCY_DE_INIT_RX) {
        (*latency_ms) += hw_api_latency[STM32WL3X_HW_API_LATENCY_DE_INIT_RX];
    }
#endif
#if (defined SIGFOX_EP_PARAMETERS_CHECK) || (defined SIGFOX_EP_ERROR_CODES)
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_CERTIFICATION
/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_start_continuous_wave(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    // Modulated CW is not supported for now.
    if (HAL_MRSubG_GetModulation() != MOD_CW) {
        SIGFOX_EXIT_ERROR((RF_API_status_t) STM32WL3X_RF_API_ERROR_MODULATION);
    }
    // Start radio.
    LL_MRSubG_StrobeCommand(CMD_LOCKTX);
#ifdef SIGFOX_EP_ERROR_CODES
    status = _STM32WL3X_rf_api_wait_for_state_switch(STATE_LOCKONTX);
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _STM32WL3X_rf_api_wait_for_state_switch(STATE_LOCKONTX);
#endif
    LL_MRSubG_StrobeCommand(CMD_TX);
#ifdef SIGFOX_EP_ERROR_CODES
    status = _STM32WL3X_rf_api_wait_for_state_switch(STATE_TX);
    SIGFOX_CHECK_STATUS(RF_API_SUCCESS);
#else
    _STM32WL3X_rf_api_wait_for_state_switch(STATE_TX);
#endif
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
RF_API_status_t STM32WL3X_RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t status = RF_API_SUCCESS;
#endif
    (*version) = (sfx_u8*) STM32WL3X_RF_API_VERSION;
    (*version_size_char) = (sfx_u8) sizeof(STM32WL3X_RF_API_VERSION);
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
void STM32WL3X_RF_API_error(void) {
    // Turn radio off.
    STM32WL3X_RF_API_de_init();
    STM32WL3X_RF_API_sleep();
}
#endif

/*** STM32WL3X RF API functions mapping ***/

#ifndef SIGFOX_EP_DYNAMIC_RF_API

#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t RF_API_open(RF_API_config_t *rf_api_config) {
    return STM32WL3X_RF_API_open(rf_api_config);
}
#endif

#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t RF_API_close(void) {
    return STM32WL3X_RF_API_close();
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t RF_API_process(void) {
    return STM32WL3X_RF_API_process();
}
#endif

/*******************************************************************/
RF_API_status_t RF_API_wake_up(void) {
    return STM32WL3X_RF_API_wake_up();
}

/*******************************************************************/
RF_API_status_t RF_API_sleep(void) {
    return STM32WL3X_RF_API_sleep();
}

/*******************************************************************/
RF_API_status_t RF_API_init(RF_API_radio_parameters_t *radio_parameters) {
    return STM32WL3X_RF_API_init(radio_parameters);
}

/*******************************************************************/
RF_API_status_t RF_API_de_init(void) {
    return STM32WL3X_RF_API_de_init();
}

/*******************************************************************/
RF_API_status_t RF_API_send(RF_API_tx_data_t *tx_data) {
    return STM32WL3X_RF_API_send(tx_data);
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_receive(RF_API_rx_data_t *rx_data) {
    return STM32WL3X_RF_API_receive(rx_data);
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm) {
    return STM32WL3X_RF_API_get_dl_phy_content_and_rssi(dl_phy_content, dl_phy_content_size, dl_rssi_dbm);
}
#endif

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
    return STM32WL3X_RF_API_carrier_sense(carrier_sense_params);
}
#endif

#if (defined SIGFOX_EP_TIMER_REQUIRED) && (defined SIGFOX_EP_LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32 *latency_ms) {
    return STM32WL3X_RF_API_get_latency(latency_type, latency_ms);
}
#endif

#ifdef SIGFOX_EP_CERTIFICATION
/*******************************************************************/
RF_API_status_t RF_API_start_continuous_wave(void) {
    return STM32WL3X_RF_API_start_continuous_wave();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
    return STM32WL3X_RF_API_get_version(version, version_size_char);
}
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
void RF_API_error(void) {
    STM32WL3X_RF_API_error();
}
#endif

#endif /* DYNAMIC_RF_API */
