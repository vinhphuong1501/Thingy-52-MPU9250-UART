/*
  Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
     list of conditions and the following disclaimer.

  2. Redistributions in binary form, except as embedded into a Nordic
     Semiconductor ASA integrated circuit in a product or a software update for
     such product, must reproduce the above copyright notice, this list of
     conditions and the following disclaimer in the documentation and/or other
     materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of its
     contributors may be used to endorse or promote products derived from this
     software without specific prior written permission.

  4. This software, with or without modification, must only be used with a
     Nordic Semiconductor ASA integrated circuit.

  5. Any software provided in binary form under this license must not be reverse
     engineered, decompiled, modified and/or disassembled.

  THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
  OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
  OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "softdevice_handler_appsh.h"
#include "app_timer.h"
#include "fstorage.h"
#include "m_ble.h"
#include "m_ble_flash.h"
#include "drv_nfc.h"
#include "thingy_config.h"
#include "advertiser_beacon.h"
#include "pca20020.h"
#include "nrf_delay.h"
#include "nrf_drv_rng.h"
#include "support_func.h"

#define  NRF_LOG_MODULE_NAME "m_ble         "
#include "nrf_log.h"
#include "macros_common.h"

#ifdef BLE_DFU_APP_SUPPORT
    #include "ble_dfu.h"
#endif // BLE_DFU_APP_SUPPORT


#ifdef BLE_DFU_APP_SUPPORT
    #define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
    #define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
    #define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
    #define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
    #define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

    STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */
#endif // BLE_DFU_APP_SUPPORT

#define RANDOM_VECTOR_DEVICE_ID_SIZE         4                                          /** Length of random ID vector. Must be <= 32. */

//For UART
#include "nrf_ble_gatt.h"
#include "ble_nus.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_log_ctrl.h"
#define CONN_CFG_TAG                    1                                           /**< A tag that refers to the BLE stack configuration we set with @ref sd_ble_cfg_set. Default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */


static uint16_t                         m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;  /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


//END UART

static uint16_t                   m_conn_handle = BLE_CONN_HANDLE_INVALID;              /**< Handle of the current connection. */
static m_ble_evt_handler_t        m_evt_handler = 0;
static m_ble_service_handle_t   * m_service_handles = 0;
static uint32_t                   m_service_num = 0;
static ble_tcs_t                  m_tcs;
static ble_tcs_params_t         * m_ble_config;
static const ble_tcs_params_t     m_ble_default_config = THINGY_CONFIG_DEFAULT;
static ble_tcs_mtu_t              m_mtu;
static bool                       m_flash_disconnect = false;
static bool                       m_major_minor_fw_ver_changed = false;
static char                       m_mac_addr[SUPPORT_FUNC_MAC_ADDR_STR_LEN];            /**< The device MAC address. */
static uint8_t                    m_random_vector_device_id[RANDOM_VECTOR_DEVICE_ID_SIZE];        /**< Device random ID. Used for NFC BLE pairng on iOS. */

#define NRF_BLE_MAX_MTU_SIZE            BLE_GATT_ATT_MTU_DEFAULT*12         /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */

#ifdef BLE_DFU_APP_SUPPORT
    static ble_dfu_t                  m_dfus;                                   /**< Structure used to identify the DFU service. */

    static void ble_dfu_evt_handler(ble_dfu_t * p_dfu, ble_dfu_evt_t * p_evt)
    {
        switch (p_evt->type)
        {
            case BLE_DFU_EVT_INDICATION_DISABLED:
                NRF_LOG_INFO("Indication for BLE_DFU is disabled\r\n");
                break;

            case BLE_DFU_EVT_INDICATION_ENABLED:
                NRF_LOG_INFO("Indication for BLE_DFU is enabled\r\n");
                break;

            case BLE_DFU_EVT_ENTERING_BOOTLOADER:
                NRF_LOG_INFO("Device is entering bootloader mode!\r\n");
                break;
            default:
                NRF_LOG_WARNING("Unknown event from ble_dfu\r\n");
                break;
        }
    }

#endif // BLE_DFU_APP_SUPPORT
#define CONN_CFG_TAG_THINGY 1


/**@brief Check if flash is currently being accessed.
 */
static bool flash_access_ongoing(void)
{
    fs_ret_t err_code;
    uint32_t flash_op_cnt = 0;

    err_code = fs_queued_op_count_get(&flash_op_cnt);
    APP_ERROR_CHECK(err_code);

    return (flash_op_cnt == 0) ? false : true;
}

/**@brief Generate random number.
 */
static uint32_t random_vector_generate(uint8_t * p_buff, uint8_t size)
{
    uint32_t err_code;
    uint8_t  bytes_available = 0;

    nrf_drv_rng_bytes_available(&bytes_available);
    uint8_t retries = 0;
    
    while (bytes_available < size)
    {
        retries++;
        NRF_LOG_WARNING("Too few random bytes available. Trying again \r\n");
        nrf_drv_rng_bytes_available(&bytes_available);
        nrf_delay_ms(5);
        
        if (retries > 5)    // Return after n attempts.
        {
            return NRF_ERROR_TIMEOUT;
        }
    }
    
    NRF_LOG_INFO("Available random bytes: %d \r\n", bytes_available);

    err_code = nrf_drv_rng_rand(p_buff, size);
    RETURN_IF_ERROR(err_code);
    
    NRF_LOG_INFO("Random value (hex): ");
    
    for (uint8_t i = 0; i < size; i++)
    {
        NRF_LOG_RAW_INFO("%02x", p_buff[i]);
    }
    
    NRF_LOG_RAW_INFO("\r\n");

    return NRF_SUCCESS;
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static uint32_t gap_params_init(void)
{

    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
/*
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;    

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          m_ble_config->dev_name.name,
                                          strlen((const char *)m_ble_config->dev_name.name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = m_ble_config->conn_params.min_conn_int;
    gap_conn_params.max_conn_interval = m_ble_config->conn_params.max_conn_int;
    gap_conn_params.slave_latency     = m_ble_config->conn_params.slave_latency;
    gap_conn_params.conn_sup_timeout  = m_ble_config->conn_params.sup_timeout;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    if (err_code == NRF_ERROR_INVALID_PARAM)
    {
        // Use default config
        m_ble_config->conn_params.min_conn_int  = (uint16_t)MSEC_TO_UNITS(MIN_CONN_INTERVAL_MS, UNIT_1_25_MS);
        m_ble_config->conn_params.max_conn_int  = MSEC_TO_UNITS(MAX_CONN_INTERVAL_MS, UNIT_1_25_MS);
        m_ble_config->conn_params.slave_latency = SLAVE_LATENCY;
        m_ble_config->conn_params.sup_timeout   = MSEC_TO_UNITS(CONN_SUP_TIMEOUT_MS, UNIT_10_MS);

        err_code = m_ble_flash_config_store(m_ble_config);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }

        gap_conn_params.min_conn_interval = (uint16_t)MSEC_TO_UNITS(MIN_CONN_INTERVAL_MS, UNIT_1_25_MS);
        gap_conn_params.max_conn_interval = MSEC_TO_UNITS(MAX_CONN_INTERVAL_MS, UNIT_1_25_MS);
        gap_conn_params.slave_latency     = SLAVE_LATENCY;
        gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(CONN_SUP_TIMEOUT_MS, UNIT_10_MS);

        err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }
    else if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }*/

    return NRF_SUCCESS;
}


/**@brief Function for dispatching system events from the SoftDevice.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a
 *          SoftDevice event has been received.
 *
 * @param[in] evt_id  System event id.
 */
static void sys_evt_dispatch(uint32_t evt_id)
{
    fs_sys_event_handler(evt_id);
    app_beacon_on_sys_evt(evt_id);
    ble_advertising_on_sys_evt(evt_id);

    if ( (evt_id == NRF_EVT_FLASH_OPERATION_SUCCESS) ||
         (evt_id == NRF_EVT_FLASH_OPERATION_ERROR) )
    {
        if (!flash_access_ongoing())
        {
            if (m_flash_disconnect)
            {
                m_ble_evt_t  evt;
                evt.evt_type = thingy_ble_evt_disconnected;

                m_evt_handler(&evt);
            }
        }
    }
}

/**@brief Function for handling a BeaconAdvertiser error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void beacon_advertiser_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the beacon timeslot functionality.
 */
static uint32_t timeslot_init(void)
{
    uint32_t err_code;
    static ble_beacon_init_t beacon_init;

    if (m_ble_config->eddystone_url.len < BLE_TCS_BEACON_LEN_MIN)
    {
        app_beacon_stop();
        return NRF_SUCCESS;
    }

    beacon_init.adv_interval  = THINGY_BEACON_ADV_INTERVAL;
    beacon_init.p_data         = m_ble_config->eddystone_url.data;
    beacon_init.data_size      = m_ble_config->eddystone_url.len;
    beacon_init.error_handler = beacon_advertiser_error_handler;

    err_code = sd_ble_gap_addr_get(&beacon_init.beacon_addr);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Increment device address by 2 for beacon advertising.
    beacon_init.beacon_addr.addr[0] += 2;

    app_beacon_init(&beacon_init);
    app_beacon_start();

    return NRF_SUCCESS;
}


/**@brief Function for handling thingy configuration events.
 */
static void tcs_evt_handler (ble_tcs_t        * p_tcs,
                             ble_tcs_evt_type_t evt_type,
                             uint8_t          * p_data,
                             uint16_t           length)
{
    bool update_flash = false;

    switch (evt_type)
    {
        case BLE_TCS_EVT_DEV_NAME:
            if (length <= BLE_TCS_DEVICE_NAME_LEN_MAX)
            {
                memcpy(m_ble_config->dev_name.name, p_data, length);
                m_ble_config->dev_name.name[length] = 0;
                m_ble_config->dev_name.len = length;
                update_flash = true;
            }
            break;
        case BLE_TCS_EVT_ADV_PARAM:
            if (length == sizeof(ble_tcs_adv_params_t))
            {
                memcpy(&m_ble_config->adv_params, p_data, length);

                update_flash = true;
            }
            break;
        case BLE_TCS_EVT_CONN_PARAM:
            if (length == sizeof(ble_tcs_conn_params_t))
            {
                uint32_t              err_code;
                ble_gap_conn_params_t gap_conn_params;

                memcpy(&m_ble_config->conn_params, p_data, length);
                memset(&gap_conn_params, 0, sizeof(gap_conn_params));

                gap_conn_params.min_conn_interval = m_ble_config->conn_params.min_conn_int;
                gap_conn_params.max_conn_interval = m_ble_config->conn_params.max_conn_int;
                gap_conn_params.slave_latency     = m_ble_config->conn_params.slave_latency;
                gap_conn_params.conn_sup_timeout  = m_ble_config->conn_params.sup_timeout;

                err_code = ble_conn_params_change_conn_params(&gap_conn_params);
                APP_ERROR_CHECK(err_code);

                update_flash = true;
            }
            break;
        case BLE_TCS_EVT_BEACON:
            if (length <= BLE_TCS_BEACON_LEN_MAX)
            {
                uint32_t err_code;

                memcpy(m_ble_config->eddystone_url.data, p_data, length);
                m_ble_config->eddystone_url.len = length;
                update_flash = true;

                err_code = timeslot_init();
                APP_ERROR_CHECK(err_code);
            }
            break;
        case BLE_TCS_EVT_CLOUD_TOKEN:
            if (length <= BLE_TCS_CLOUD_LEN_MAX)
            {
                memcpy(m_ble_config->cloud_token.token, p_data, length);
                m_ble_config->cloud_token.len = length;
                update_flash = true;
            }
            break;

        case BLE_TCS_EVT_MTU:
            if (length == sizeof(ble_tcs_mtu_t))
            {
                uint32_t        err_code;
                ble_tcs_mtu_t * p_mtu = (ble_tcs_mtu_t *)p_data;

                if (p_mtu->req == TCS_MTU_REQ_EXCHANGE)
                {
                    NRF_LOG_INFO("tcs_evt_handler: TCS_MTU_REQ_EXCHANGE - %d\r\n", p_mtu->size);
                    err_code = sd_ble_gattc_exchange_mtu_request(m_conn_handle, p_mtu->size);
                    if (err_code == NRF_SUCCESS)
                    {
                        memcpy(&m_mtu, p_data, length);
                    }
                    else
                    {
                        err_code = ble_tcs_mtu_set(&m_tcs, &m_mtu);
                        APP_ERROR_CHECK(err_code);
                    }
                }
                else
                {
                    err_code = ble_tcs_mtu_set(&m_tcs, &m_mtu);
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
    }

    if (update_flash)
    {
        uint32_t err_code;

        err_code = m_ble_flash_config_store(m_ble_config);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Checks the current version of the FW against the previous version stored in flash.
 * If a major or minor FW change is detected, modules must reinitialize their flash storage.
 *
 * @note: If the FW version is changed while erasing all flash, a FW change cannot be detected.
 */
static uint32_t thingy_config_verify(void)
{
    bool update_flash = false;
    uint32_t err_code;
    
    bool fw_version_major_changed = ( m_ble_config->fw_version.major != m_ble_default_config.fw_version.major );
    bool fw_version_minor_changed = ( m_ble_config->fw_version.minor != m_ble_default_config.fw_version.minor );
    bool fw_version_patch_changed = ( m_ble_config->fw_version.patch != m_ble_default_config.fw_version.patch );
    
    ble_tcs_fw_version_t prev_fw_version = m_ble_config->fw_version;

    if ( fw_version_major_changed || fw_version_minor_changed || fw_version_patch_changed)
    {
        m_ble_config->fw_version.major = m_ble_default_config.fw_version.major;
        m_ble_config->fw_version.minor = m_ble_default_config.fw_version.minor;
        m_ble_config->fw_version.patch = m_ble_default_config.fw_version.patch;
        
        update_flash = true;
        
        if(fw_version_major_changed || fw_version_minor_changed)
        {       
            update_flash = false;
            m_major_minor_fw_ver_changed = true;
            
            err_code = m_ble_flash_config_store(&m_ble_default_config);
            APP_ERROR_CHECK(err_code);
        }
    }
    
    NRF_LOG_INFO(NRF_LOG_COLOR_CODE_GREEN"m_ble: Current FW: v%d.%d.%d \r\n",
    m_ble_default_config.fw_version.major, m_ble_default_config.fw_version.minor, m_ble_default_config.fw_version.patch);
    
    if(m_major_minor_fw_ver_changed)
    {
        NRF_LOG_INFO(NRF_LOG_COLOR_CODE_YELLOW"m_ble: Major or minor FW version changed. Prev. FW (from flash): v%d.%d.%d \r\n",
        prev_fw_version.major, prev_fw_version.minor, prev_fw_version.patch);
    }

    // Check Eddystone URL length.
    if (m_ble_config->eddystone_url.len > 17)
    {
        memcpy(m_ble_config->eddystone_url.data, m_ble_default_config.eddystone_url.data, m_ble_default_config.eddystone_url.len);
        m_ble_config->eddystone_url.len = m_ble_default_config.eddystone_url.len;
        update_flash = true;
    }

    // Check Cloud Token length.
    if (m_ble_config->cloud_token.len > BLE_TCS_CLOUD_LEN_MAX)
    {
        memcpy(m_ble_config->cloud_token.token, m_ble_default_config.cloud_token.token, m_ble_default_config.cloud_token.len);
        m_ble_config->cloud_token.len = m_ble_default_config.cloud_token.len;
        update_flash = true;
    }

    if (update_flash)
    {
        err_code = m_ble_flash_config_store(m_ble_config);
        APP_ERROR_CHECK(err_code);
    }
    
    return NRF_SUCCESS;
}


static uint32_t thingy_config_init(void)
{
    ble_tcs_init_t params;
    uint32_t err_code;

    params.p_init_vals = m_ble_config;

    params.evt_handler = tcs_evt_handler;
    err_code = ble_tcs_init(&m_tcs, &params);

    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    return NRF_SUCCESS;
}





uint32_t nfc_init(void)
{
    uint32_t err_code;
    
    err_code = drv_nfc_init();
    RETURN_IF_ERROR(err_code);
    
    err_code = drv_nfc_app_launch_android_record_add(THINGY_NFC_APP_ANDROID_NAME_DEFAULT, THINGY_NFC_APP_ANDROID_NAME_LEN);
    RETURN_IF_ERROR(err_code);
    
    uint8_t NFC_STR_LEN = SUPPORT_FUNC_MAC_ADDR_STR_LEN + (RANDOM_VECTOR_DEVICE_ID_SIZE * 2) + 1; // MAC (with '\0') + space + random vector.
    char nfc_string[NFC_STR_LEN];
    nfc_string[0] = '\0';
    
    strcat(nfc_string, m_mac_addr);
    strcat(nfc_string, " ");
    
    for (uint8_t i = 0; i < RANDOM_VECTOR_DEVICE_ID_SIZE; i++)
    {
        char tmp[3] = {0};
        sprintf(tmp, "%02x", m_random_vector_device_id[i]);
        strcat(nfc_string, tmp);
    }
    
    NRF_LOG_INFO("nfc string: %s \r\n", nrf_log_push(nfc_string));
    
    err_code = drv_nfc_string_record_add(nfc_string, NFC_STR_LEN);
    RETURN_IF_ERROR(err_code);
                                                   
    err_code = drv_nfc_uri_record_add(NFC_URI_HTTP_WWW, THINGY_NFC_URI_DEFAULT, THINGY_NFC_URI_LEN);
    RETURN_IF_ERROR(err_code);
                                                   
    err_code = drv_nfc_enable();
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}


uint32_t m_ble_init(m_ble_init_t * p_params)
{
    uint32_t err_code;

    m_evt_handler     = p_params->evt_handler;
    m_service_handles = p_params->p_service_handles;
    m_service_num     = p_params->service_num;
/*
    err_code = ble_stack_init();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("ble_stack_init failed - %d\r\n", err_code);
        return err_code;
    }
*/
        /* Enable FPU again due to SD issue */
    #if (__FPU_USED == 1)
        SCB->CPACR |= (3UL << 20) | (3UL << 22);
        __DSB();
        __ISB();
    #endif
    
    err_code = random_vector_generate(m_random_vector_device_id, RANDOM_VECTOR_DEVICE_ID_SIZE);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("random_vector_generate failed - %d\r\n", err_code);
        return err_code;
    }
    
    /**@brief Load configuration from flash. */
    /*
    err_code = m_ble_flash_init(&m_ble_default_config, &m_ble_config);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR(" m_ble_flash_init failed - %d\r\n", err_code);
        return err_code;
    }

    err_code = thingy_config_verify();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Thingy_config_verify failed - %d\r\n", err_code);
        return err_code;
    }
*/
    err_code = gap_params_init();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("gap_params_init failed - %d\r\n", err_code);
        return err_code;
    }

    //gatt_init();
/*
    err_code = services_init(m_service_handles, m_service_num);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Thingy_ble_init: services_init failed - %d\r\n", err_code);
        return err_code;
    }

    err_code = advertising_init();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Advertising_init failed - %d\r\n", err_code);
        return err_code;
    }

    err_code = conn_params_init();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("Conn_params_init failed - %d\r\n", err_code);
        return err_code;
    }
*/ 
/*   
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("ble_advertising_start failed - %d\r\n", err_code);
        return err_code;
    }*/
   /* 
    err_code = support_func_ble_mac_address_get(m_mac_addr);

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("mac address get failed - %d\r\n", err_code);
        return err_code;
    }
    
    NRF_LOG_RAW_INFO("MAC addr-> %s \r\n", nrf_log_push(m_mac_addr));
    
    err_code = nfc_init();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("nfc init failed - %d\r\n", err_code);
        return err_code;
    }
    
    nrf_delay_ms (10);

    err_code = timeslot_init();

    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("timeslot_init failed - %d\r\n", err_code);
        return err_code;
    }
    */
    return NRF_SUCCESS;
}
