/*
 *
 *    Copyright (c) 2020 Project CHIP Authors
 *    Copyright (c) 2019 Google LLC.
 *    All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#include <stdbool.h>
#include <stdint.h>

#include "boards.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_usbd.h"
#include "nrf_log.h"
#ifdef SOFTDEVICE_PRESENT
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#endif
#include "nrf_drv_clock.h"
#if NRF_CRYPTO_ENABLED
#include "nrf_crypto.h"
#endif
#include "mem_manager.h"
#if CHIP_ENABLE_OPENTHREAD
extern "C" {
#include "multiprotocol_802154_config.h"
#include "nrf_802154.h"
#include "nrf_cc310_platform_abort.h"
#include "nrf_cc310_platform_mutex.h"
#include <openthread/platform/platform-softdevice.h>
}
#endif // CHIP_ENABLE_OPENTHREAD

#if NRF_LOG_ENABLED
#include "nrf_log_backend_uart.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#endif // NRF_LOG_ENABLED

#include <AppTask.h>
#if CHIP_ENABLE_OPENTHREAD
#include <mbedtls/platform.h>
#include <openthread/cli.h>
#include <openthread/dataset.h>
#include <openthread/error.h>
#include <openthread/heap.h>
#include <openthread/icmp6.h>
#include <openthread/instance.h>
#include <openthread/link.h>
#include <openthread/platform/openthread-system.h>
#include <openthread/tasklet.h>
#include <openthread/thread.h>
#include <platform/CHIPDeviceLayer.h>
#include <support/logging/CHIPLogging.h>
#endif // CHIP_ENABLE_OPENTHREAD

#ifdef __cplusplus
extern "C" {
#endif
#include "app_error.h"
#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_core.h"
#include "app_usbd_serial_num.h"
#include "app_usbd_string_desc.h"
#include "app_util.h"
#ifdef __cplusplus
}
#endif

#include "chipinit.h"

using namespace ::chip;
using namespace ::chip::Inet;
using namespace ::chip::DeviceLayer;

extern "C" size_t GetHeapTotalSize(void);

// ================================================================================
// Logging Support
// ================================================================================

#if NRF_LOG_ENABLED

#if NRF_LOG_USES_TIMESTAMP

uint32_t LogTimestamp(void)
{
    return 0;
}

#define LOG_TIMESTAMP_FUNC LogTimestamp
#define LOG_TIMESTAMP_FREQ 1000

#else // NRF_LOG_USES_TIMESTAMP

#define LOG_TIMESTAMP_FUNC NULL
#define LOG_TIMESTAMP_FREQ 0

#endif // NRF_LOG_USES_TIMESTAMP

#endif // NRF_LOG_ENABLED

// ================================================================================
// SoftDevice Support
// ================================================================================

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT

static void OnSoCEvent(uint32_t sys_evt, void * p_context)
{
#if CHIP_ENABLE_OPENTHREAD
    otSysSoftdeviceSocEvtHandler(sys_evt);
#endif
    UNUSED_PARAMETER(p_context);
}

#endif // defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT

// ================================================================================
// J-Link Monitor Mode Debugging Support
// ================================================================================

#if JLINK_MMD

extern "C" void JLINK_MONITOR_OnExit(void) {}

extern "C" void JLINK_MONITOR_OnEnter(void) {}

extern "C" void JLINK_MONITOR_OnPoll(void) {}

#endif // JLINK_MMD

#ifdef __cplusplus
extern "C" {
#endif

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE 0
#define CDC_ACM_COMM_EPIN NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE 1
#define CDC_ACM_DATA_EPIN NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT NRF_DRV_USBD_EPOUT1

/**
 * @brief CDC_ACM class instance
 * */
APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm, cdc_acm_user_ev_handler, CDC_ACM_COMM_INTERFACE, CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN, CDC_ACM_DATA_EPIN, CDC_ACM_DATA_EPOUT, APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

// Max read-size
#define READ_SIZE 1

static char m_rx_buffer[READ_SIZE];
static char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
static bool m_send_flag = 0;
static bool m_ok_flag   = 0;
/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);
    static bool state                    = false;

    switch (event)
    {
    case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN: {
        // bsp_board_led_on(LED_CDC_ACM_OPEN);

        /*Setup first transfer*/
        ret_code_t ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer, READ_SIZE);
        UNUSED_VARIABLE(ret);
        break;
    }
    case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
        // bsp_board_led_off(LED_CDC_ACM_OPEN);
        break;
    case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
        break;
    case APP_USBD_CDC_ACM_USER_EVT_RX_DONE: {
        ret_code_t ret;
        m_ok_flag = 0;
        do
        {
            /*Get amount of data transfered*/
            size_t size = app_usbd_cdc_acm_rx_size(p_cdc_acm);
            NRF_LOG_INFO("RX: size: %lu char: %c", size, m_rx_buffer[0]);
            if (m_rx_buffer[0] == 'R')
            {
                m_ok_flag = 1;
            }
            else if (m_rx_buffer[0] == 'B')
            {
                LightingManager::Action_t action = (state == false) ? LightingManager::ON_ACTION : LightingManager::OFF_ACTION;
                GetAppTask().PostLightingActionRequest(action);
                state     = !state;
                m_ok_flag = 1;
            }
            /* Fetch data until internal buffer is empty */

            ret = app_usbd_cdc_acm_read(&m_app_cdc_acm, m_rx_buffer, size);
        } while (ret == NRF_SUCCESS);
        m_send_flag = 1;
        break;
    }
    default:
        break;
    }
}

void usb_setup(void)
{
    ret_code_t ret;
    static const app_usbd_config_t usbd_config = {};
    app_usbd_serial_num_generate();
    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);
    NRF_LOG_INFO("USB serial example started.");

    app_usbd_class_inst_t const * class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
    ret                                         = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);
    app_usbd_enable();
    app_usbd_start();
}

// Actuation Server Task
void actuation_server(void *)
{
    usb_setup();
    while (true)
    {
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }

        if (m_send_flag)
        {
            size_t size    = sprintf(m_tx_buffer, m_ok_flag == 1 ? "OK\r\n" : "NA\r\n");
            ret_code_t ret = app_usbd_cdc_acm_write(&m_app_cdc_acm, m_tx_buffer, size);
            APP_ERROR_CHECK(ret);
            m_send_flag = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

#ifdef __cplusplus
}
#endif

// ================================================================================
// Main Code
// ================================================================================

int main(void)
{
    ret_code_t ret;

#if JLINK_MMD
    NVIC_SetPriority(DebugMonitor_IRQn, _PRIO_SD_LOWEST);
#endif

    // Initialize clock driver.
    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_lfclk_request(NULL);

    // Wait for the clock to be ready.
    while (!nrf_clock_lf_is_running())
    {
    }

#if NRF_LOG_ENABLED

    // Initialize logging component
    ret = NRF_LOG_INIT(LOG_TIMESTAMP_FUNC, LOG_TIMESTAMP_FREQ);
    APP_ERROR_CHECK(ret);

    // Initialize logging backends
    NRF_LOG_DEFAULT_BACKENDS_INIT();

#endif

    NRF_LOG_INFO("==================================================");
    NRF_LOG_INFO("chip-nrf52840-lock-example starting");
#if BUILD_RELEASE
    NRF_LOG_INFO("*** PSEUDO-RELEASE BUILD ***");
#else
    NRF_LOG_INFO("*** DEVELOPMENT BUILD ***");
#endif
    NRF_LOG_INFO("==================================================");
    NRF_LOG_FLUSH();

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT

    NRF_LOG_INFO("Enabling SoftDevice");

    ret = nrf_sdh_enable_request();
    if (ret != NRF_SUCCESS)
    {
        NRF_LOG_INFO("nrf_sdh_enable_request() failed");
        APP_ERROR_HANDLER(ret);
    }

    NRF_LOG_INFO("Waiting for SoftDevice to be enabled");

    while (!nrf_sdh_is_enabled())
    {
    }

    // Register a handler for SOC events.
    NRF_SDH_SOC_OBSERVER(m_soc_observer, NRF_SDH_SOC_STACK_OBSERVER_PRIO, OnSoCEvent, NULL);

    NRF_LOG_INFO("SoftDevice enable complete");

#endif // defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT

#if defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT

    {
        uint32_t appRAMStart = 0;

        // Configure the BLE stack using the default settings.
        // Fetch the start address of the application RAM.
        ret = nrf_sdh_ble_default_cfg_set(CHIP_DEVICE_LAYER_BLE_CONN_CFG_TAG, &appRAMStart);
        APP_ERROR_CHECK(ret);

        // Enable BLE stack.
        ret = nrf_sdh_ble_enable(&appRAMStart);
        APP_ERROR_CHECK(ret);
        NRF_LOG_INFO("SoftDevice BLE enabled");
    }

#endif // defined(SOFTDEVICE_PRESENT) && SOFTDEVICE_PRESENT

    ret = ChipInit();
    if (ret != NRF_SUCCESS)
    {
        NRF_LOG_INFO("ChipInit() failed");
        APP_ERROR_HANDLER(ret);
    }

    ret = GetAppTask().StartAppTask();
    if (ret != NRF_SUCCESS)
    {
        NRF_LOG_INFO("GetAppTask().Init() failed");
        APP_ERROR_HANDLER(ret);
    }

    // Activate deep sleep mode
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    {
        struct mallinfo minfo = mallinfo();
        NRF_LOG_INFO("System Heap Utilization: heap size %" PRId32 ", arena size %" PRId32 ", in use %" PRId32 ", free %" PRId32,
                     GetHeapTotalSize(), minfo.arena, minfo.uordblks, minfo.fordblks);
    }

    NRF_LOG_INFO("Starting FreeRTOS scheduler");

    xTaskCreate(actuation_server, "Actuation", 1024, NULL, 3, NULL);
    /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();

    // Should never get here
    NRF_LOG_INFO("vTaskStartScheduler() failed");
    APP_ERROR_HANDLER(0);
}
