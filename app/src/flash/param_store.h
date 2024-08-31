/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
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

#ifndef __PARAM_STORE_H__
#define __PARAM_STORE_H__

#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nordic_common.h"
#include "nrf_pwr_mgmt.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "fds.h"

/* File ID and Key used for the configuration record. */

#define CONFIG_FILE     (0x8010)
// #define CONFIG_REC_KEY  (0x7010)
#define FDS_POLL_MAX    10000000

extern bool fds_op_busy;

#ifdef __cplusplus
extern "C" {
#endif
void fds_evt_handler_C(fds_evt_t const * p_evt);
#ifdef __cplusplus
}
#endif


template <typename T>
class ParamStore {
public:
    ParamStore(const uint16_t cfg_rec_key) :
        config_rec_key(cfg_rec_key)
    {
    }

    ~ParamStore() {}

    void erase(void)
    {
        ret_code_t rc;
        fds_record_desc_t desc = {0};
        fds_find_token_t  tok  = {0};

        rc = fds_record_find(CONFIG_FILE, config_rec_key, &desc, &tok);

        if (rc == NRF_SUCCESS)
        {
            NRF_LOG_INFO("Updating Flash");

            rc = check_for_fds_op_busy();
            APP_ERROR_CHECK(rc);

            /* Erase record . */
            rc = fds_record_delete(&desc);
            APP_ERROR_CHECK(rc);
            NRF_LOG_INFO("Flash Record Erased");
        } else 
	{
            NRF_LOG_INFO("Flash record already erased.");
	}
    }

    void set(T *param_store_data_ptr)
    {
        ret_code_t rc;
        fds_record_desc_t desc = {0};
        fds_find_token_t  tok  = {0};

        rc = fds_record_find(CONFIG_FILE, config_rec_key, &desc, &tok);

        if (rc == NRF_SUCCESS)
        {
            NRF_LOG_INFO("Updating Flash");

            param_store_record.file_id           = CONFIG_FILE;
            param_store_record.key               = config_rec_key;
            param_store_record.data.p_data       = param_store_data_ptr;
            /* The length of a record is always expressed in 4-byte units (words). */
            param_store_record.data.length_words = (sizeof(*param_store_data_ptr) + 3) / sizeof(uint32_t);

            rc = check_for_fds_op_busy();
            APP_ERROR_CHECK(rc);

            /* Write the updated record to flash. */
            rc = fds_record_update(&desc, &param_store_record);
            if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH))
            {
                NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
            }
            else
            {
                APP_ERROR_CHECK(rc);
            }
        } else 
	{
            APP_ERROR_CHECK(rc);
	}

    }

    T get(void)
    {
        return param_data;
    }

    void init(T *param_store_data_ptr)
    {
        ret_code_t rc;
        fds_record_desc_t desc = {0};
        fds_find_token_t  tok  = {0};

        NRF_LOG_INFO("Initializing Flash");

	// Flash event handler callback
        (void) fds_register(fds_evt_handler_C);

        rc = check_for_fds_op_busy();
        APP_ERROR_CHECK(rc);

        rc = fds_init();
        APP_ERROR_CHECK(rc);

        rc = fds_record_find(CONFIG_FILE, config_rec_key, &desc, &tok);
        if (rc == NRF_SUCCESS)
        {
            NRF_LOG_INFO("Flash Record Found");

            /* A config file is in flash. Let's read it. */
            fds_flash_record_t config = {0};

            /* Open the record and read its contents. */
            rc = fds_record_open(&desc, &config);
            APP_ERROR_CHECK(rc);

            /* Copy the configuration from flash into the param_data. */
            memcpy(&param_data, config.p_data, sizeof(T));

            /* Close the record when done reading. */
            rc = fds_record_close(&desc);
            APP_ERROR_CHECK(rc);

        }
        else
        {
            /* System config not found; write a new one. */
            NRF_LOG_INFO("Writing Flash Record");

            param_store_record.file_id           = CONFIG_FILE;
            param_store_record.key               = config_rec_key;
            param_store_record.data.p_data       = param_store_data_ptr;
            /* The length of a record is always expressed in 4-byte units (words). */
            param_store_record.data.length_words = (sizeof(*param_store_data_ptr) + 3) / sizeof(uint32_t);

            rc = check_for_fds_op_busy();
            APP_ERROR_CHECK(rc);

            rc = fds_record_write(&desc, &param_store_record);
            if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH))
            {
                NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
            }
            else
            {
                APP_ERROR_CHECK(rc);
            }

	    param_data = *param_store_data_ptr; 

        }
    }

protected:
    /* parameter data. */
    uint16_t config_rec_key;
    fds_record_t param_store_record;
    T param_data;

    /**@brief Function for handling the idle state (main loop).
     *
     * @details If there is no pending log operation, then sleep until next the next event occurs.
     * FIXME: move this into a separate power management library file.
     */
    void idle_state_handle(void)
    {
        if (NRF_LOG_PROCESS() == false)
        {
            nrf_pwr_mgmt_run();
        }
    }

    /**@brief   Wait for fds write complete . */
    ret_code_t wait_for_fds_op_complete()
    {
	int32_t timeout_counter = 0;
        ret_code_t rc = NRF_SUCCESS;
        while (fds_op_busy && timeout_counter != FDS_POLL_MAX)
        {
            idle_state_handle();
	    timeout_counter++;
        }
	if (fds_op_busy)
	{
	    rc = NRF_ERROR_TIMEOUT;
	}
	return rc;
    }

    /**@brief   check for fds write complete . */
    ret_code_t check_for_fds_op_busy()
    {
        ret_code_t rc = NRF_SUCCESS;
	if (fds_op_busy)
	{
	    rc = wait_for_fds_op_complete();
	} else {
            // indicates flash operation will be underway
	    fds_op_busy = true;
	}
	return rc;
    }

};

#endif



