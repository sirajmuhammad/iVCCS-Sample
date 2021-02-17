/**
  ******************************************************************************
  * Author             : Siraj Muhammad <sirajmuhammad@ou.edu>
  * File Name          : main.c
  * Description        : Main source file for iVCCS 3G firmware
  ******************************************************************************
  *
  * Copyright (c) 2018 - 2019 The University of Oklahoma - WECAD Center
  * All rights reserved.
  *
  ******************************************************************************
  */

/** @file
* @brief iVCCS 3G Firmware Project.
* @defgroup OU_iVCCS OU iVCCS
*
*/
#define NRF_LOG_MODULE_NAME main   // This is the log prefix of this module

#include <stdbool.h>
#include <stdint.h>

/* Drivers */
#include "boards.h"
#include "nrfx_qspi.h"

/* nRF5 Libraries */
#include "nrfx.h"
#include "sdk_config.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrfx_log.h"
#include "nrf_delay.h"
#include "nrf_pwr_mgmt.h"

/* iVCCS Libraries */
#include "common.h"
#include "detection_alg.h"
#include "fxos87.h"
#include "flash.h"
#include "battery.h"
#include "bluetooth.h"
#include "rtc.h"
#include "gps.h"
#include "configs.h"
#include "cli.h"

NRF_LOG_MODULE_REGISTER();      // For log module to take the prefix effective


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    ret_code_t err_code;
    detect_alg_ret_code_t detect_ret;

    // Initialize the logger
    err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    // Initialize and add a backend
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    /* =============== WDT ================ */
    wdt_init();

    /* =============== Load Sensor Configurations ================ */
    err_code = configs_init();
    APP_ERROR_CHECK(err_code);
    err_code = configs_load();
    APP_ERROR_CHECK(err_code);
    NRFX_LOG_INFO("Hello, World! iVCSS ID: %s", ivccs_config.sensor_name);

    /* =============== Initialize Power Management ================ */
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

    /* =============== Initialize SoftDevice and start Bluetooth ================ */
    // Initialize app_timer for bluetooth module and other timers
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
    bluetooth_init();
    bluetooth_advertising_start();
    
    /* =============== Enable the internal DC/DC Buck Converter ================ */
    sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

    /* =============== Initialize Drivers and Libraries ================ */
    // Initialize GPIO
    gpio_init();
    NRFX_LOG_INFO("GPIOs have been initialized successfully.");

    // Initialize timers
    timers_init();
    NRFX_LOG_INFO("Timers have been initialized successfully.");

    // Initialize the magnetometer TWI/I2C interface
    mgm_twim_init(); 
    NRFX_LOG_INFO("TWI has been initialized successfully.");

    /* =============== Configure Magnetometer ================ */
    odr = ODR_100Hz;
    FXOS_Configure(odr);
    NRFX_LOG_INFO("Magnetometer has been configured successfully.");
    FXOS_Ref_Recalb();

    /* =============== Initialize RTC ================ */
    rtc_twim_init();
    rtc_init();
    
    /* =============== Flash ================ */
    flash_seek();   // prepare global address

    /* =============== GPS ================ */
    gps_enable();
    gps_sync();
    NRFX_LOG_INFO("GPS is in sync mode...");

    /* =============== Battery Gauge ================ */
    bg_enable();
    if (!bg_is_configured())
    {
        bg_config(BG_MODE_ACC);
        NRFX_LOG_INFO("Battery gauge has been configured successfully.");
    }
    if (bg_start() == NRFX_SUCCESS)
        bg_timer_start();


    while (true)
    {
        /* =============== WDT ================ */
        wdt_feed();

        /* =============== CLI ================ */
        cli_process();
        if (cli_do_uninit)
        {
            cli_uninit(); // Uninitialization has to be outside the CLI module
            cli_do_uninit = false;
        }

        /* =============== Detection Algorithm ================ */
        detect_ret = detect_alg_exec();

        /* =============== Misc. Requests ================ */
        if (MGM_Recalb_Rqst)
        {
            FXOS_Ref_Recalb();
            MGM_Recalb_Rqst = 0;
            bluetooth_send("MGM Recalibration done.\r\n");
        }
        
        // Check if GPS has synced and updated RTC
        if (gps_is_synced())
        {
            char str[42] = {0};
            sprintf(str, "%s: GPS has synced and updated RTC.\r\n", ivccs_config.sensor_name);
            bluetooth_send(str);
            gps_disable();
        }

        /* =============== Automatic Bluetooth Advertising ================ */
        if (ble_adv_stop)
        {
            ble_adv_stop = false;

            // Stop any impending connection parameters update
            err_code = ble_conn_params_stop();
            APP_ERROR_CHECK(err_code);
            // Disable SoftDevice
            err_code = nrf_sdh_disable_request();
            APP_ERROR_CHECK(err_code);
            ASSERT(!nrf_sdh_is_enabled());
            // Start the advertising idle timer
            ble_adv_timer_start();
        }

        if (ble_adv_start)
        {
            ble_adv_start = false;
            bluetooth_init();
            bluetooth_advertising_start();
        }


        /* =============== Power Management ================ */
        if (GoTo_SysOff)
        {
            GoTo_SysOff = false;
            nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
        }

        if (!(MGM_Recalb_Rqst || FXOS_MVM_INT_Flag || OTH_INT_Flag))
        {
            UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
            nrf_pwr_mgmt_run();
        }
    }   // main while loop
}
/** @} */