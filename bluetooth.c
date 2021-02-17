/**
  ******************************************************************************
  * Author             : Siraj Muhammad <sirajmuhammad@ou.edu>
  * File Name          : bluetooth.c
  * Description        : Everything related to Bluetooth communication
  ******************************************************************************
  *
  * Copyright (c) 2018 - 2019 The University of Oklahoma - WECAD Center
  * All rights reserved.
  *
  ******************************************************************************
  */

#define NRF_LOG_MODULE_NAME ble   // This is the log prefix of this module

/******************************************************************************
      Headers
******************************************************************************/
#include "bluetooth.h"
#include "nrf_pwr_mgmt.h"
#include "app_error.h"
#include "nrfx_log.h"
#include "common.h"
#include "configs.h"
#include "cli.h"
#include "nrf_cli_ble_uart.h"

NRF_LOG_MODULE_REGISTER();      // For log module prefix become effective

/******************************************************************************
      Defines
******************************************************************************/
/* BLE observer for the GATT module */
nrf_ble_gatt_t m_gatt;
NRF_SDH_BLE_OBSERVER(m_gatt_obs, NRF_BLE_GATT_BLE_OBSERVER_PRIO, nrf_ble_gatt_on_ble_evt, &m_gatt);
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

/******************************************************************************
      Variables Declarations
******************************************************************************/
char* ble_rx_data;                                                                  /**< Data array to be passed to the user's application data handler. */
static int8_t rssi = 0;

/******************************************************************************
      Functions
******************************************************************************/

/**@brief Function to stringify PHY mode
 *
 * @param[in]   phys   Bluetooth PHY mode.
 */
char const * phy_str(uint8_t phys)
{
    static char const * str[] =
    {
        "1 Mbps",
        "2 Mbps",
        "Coded",
        "Unknown"
    };

    switch (phys)
    {
        case BLE_GAP_PHY_1MBPS:
            return str[0];

        case BLE_GAP_PHY_2MBPS:
        case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS:
        case BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED:
            return str[1];

        case BLE_GAP_PHY_CODED:
            return str[2];

        default:
            return str[3];
    }
}

/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRFX_LOG_INFO("BLE is connected.");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            // Update TX power
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, ivccs_config.ble_tx_power);
            APP_ERROR_CHECK(err_code);
            // Receive RSSI values
            sd_ble_gap_rssi_start(m_conn_handle, 1, 0);
            // Initialize CLI
            cli_init(m_conn_handle);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRFX_LOG_INFO("BLE is disconnected.");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            cli_uninit();
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRFX_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE:
        {
            ble_gap_evt_phy_update_t const * p_phy_evt = &p_ble_evt->evt.gap_evt.params.phy_update;

            if (p_phy_evt->status == BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION)
            {
                // Ignore LL collisions.
                NRFX_LOG_DEBUG("LL transaction collision during PHY update.");
                break;
            }

            ble_gap_phys_t phys = {0};
            phys.tx_phys = p_phy_evt->tx_phy;
            phys.rx_phys = p_phy_evt->rx_phy;
            NRFX_LOG_INFO("PHY update %s. PHY set to %s.",
                         (p_phy_evt->status == BLE_HCI_STATUS_CODE_SUCCESS) ?
                         "accepted" : "rejected",
                         phy_str(phys.tx_phys));

        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // TODO: Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_RSSI_CHANGED:
            rssi = p_ble_evt->evt.gap_evt.params.rssi_changed.rssi;
            break;

        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sec_mode);
    
    char dev_name[25];
    memset(dev_name, 0x00, 25);
    sprintf(dev_name, "%s %s", DEVICE_NAME, ivccs_config.sensor_name);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          dev_name,
                                          strlen(dev_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRFX_LOG_DEBUG("Data len is set to 0x%X (%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    
        NRFX_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                      p_gatt->att_mtu_desired_central,
                      p_gatt->att_mtu_desired_periph);
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_dis_init_t     dis_init;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;
    
    m_qwr.initialized = 0;
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize CLI's UART service
    err_code = nrf_cli_ble_uart_service_init();
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
    dis_init.dis_char_rd_sec = SEC_OPEN;
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids = m_adv_uuids;

    /* Coded PHY dictates a null scan response, that's because Coded PHY sends extended advertisment packets
       and extended adv. packets cannot be both connectable and scannable. The API places the device in
       connectable non-scannable mode. */
    if (ivccs_config.ble_phy == BLE_GAP_PHY_CODED)
    {
        init.config.ble_adv_extended_enabled = true;
    }

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    
    if (ivccs_config.ble_phy == BLE_GAP_PHY_CODED || ivccs_config.ble_phy == BLE_GAP_PHY_1MBPS)
    {
        init.config.ble_adv_primary_phy = ivccs_config.ble_phy;
        init.config.ble_adv_secondary_phy = ivccs_config.ble_phy;
    }
    else
    {
        init.config.ble_adv_primary_phy = BLE_GAP_PHY_AUTO;
        init.config.ble_adv_secondary_phy = BLE_GAP_PHY_AUTO;
    }

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRFX_LOG_INFO("BLE is advertising...");
            break;
        case BLE_ADV_EVT_IDLE:
            NRFX_LOG_INFO("BLE is in idle state. Switching off BLE...");
            ble_adv_stop = true;
            break;
        default:
            break;
    }
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    // TODO: Need to study these parameters and modify to fit our needs
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing everything related to Bluetooth to
 *        get it ready.
 */
void bluetooth_init()
{
    ret_code_t err_code;

    // Initialize BLE related functions and stack
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    NRFX_LOG_INFO("Bluetooth is initialized successfully. PHY: %s, TX power: %d dBm.", phy_str(ivccs_config.ble_phy), ivccs_config.ble_tx_power);
}

/**@brief Function to start advertising.
 */
void bluetooth_advertising_start(void)
{
    uint32_t err_code;
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, ivccs_config.ble_tx_power);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for sending data over BLE to the connected peer.
 */
ret_code_t bluetooth_send(char* data)
{
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) return NRFX_ERROR_INVALID_STATE;
    cli_print(data);
}

/**@brief Function for updating the PHY mode, switching to long range, 1 Mbps, or 2 Mbps.
 */
ret_code_t bluetooth_set_phy(uint8_t value)
{
    ret_code_t err_code;
    // Set the global setting
    ivccs_config.ble_phy = value;

    // Change for CONN role
    ble_gap_phys_t const phys = 
    {
        .rx_phys = ivccs_config.ble_phy,
        .tx_phys = ivccs_config.ble_phy,
    };
    err_code = sd_ble_gap_phy_update(m_conn_handle, &phys);
    APP_ERROR_CHECK(err_code);

    NRFX_LOG_INFO("PHY is set to %s", phy_str(ivccs_config.ble_phy));

    return err_code;
}

/**@brief Function for updating the TX power of the transmitter.
 */
ret_code_t bluetooth_set_tx_pwr(ble_tx_power_t power)
{
    ret_code_t err_code;

    ivccs_config.ble_tx_power = power;
    // Set TX power for adv role
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, ivccs_config.ble_tx_power);
    APP_ERROR_CHECK(err_code);
    // Set TX power for connected role
    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, ivccs_config.ble_tx_power);
        APP_ERROR_CHECK(err_code);
    }

    NRFX_LOG_INFO("TX power is set to %d dBm", ivccs_config.ble_tx_power);

    return err_code;
}

void bluetooth_off()
{
    cli_do_uninit = true;
    ble_adv_stop = true;
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
}

void bluetooth_restart()
{
    ble_adv_stop = true;
    ble_adv_start = true;
    cli_uninit();
}

int8_t bluetooth_get_rssi()
{
    return rssi;
}