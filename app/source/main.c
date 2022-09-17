/**
 * @file       main.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Quang Ha
 * @brief      BOS (BLE Blood Oxygen Service)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"

#include "bsp/bsp.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_nus.h"
#include "ble/ble_ecg.h"
#include "bsp.h"
#include "nrf52832_peripherals.h"
#include "bsp/bsp_afe.h"
#include "bsp/bsp_bm.h"

#if defined(UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined(UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

/* Private defines ---------------------------------------------------- */
#define APP_BLE_CONN_CFG_TAG            1                                          /**< A tag identifying the SoftDevice BLE configuration. */

#define SENSORS_MEAS_INTERVAL           APP_TIMER_TICKS(2000)                      /**< Sensors measurement interval (ticks). */
#define BATT_LEVEL_MEAS_INTERVAL        APP_TIMER_TICKS(20000)                     /**< Battery level measurement interval (ticks). */

#define DEVICE_NAME                     "ECG-Device"                               /**< Name of device. Will be included in the advertising data. */

#define MANUFACTURER_NAME               "Medtec"                                   /**< Manufacturer. Will be passed to Device Information Service. */

#define ECG_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(8, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/* Private macros ----------------------------------------------------- */
BLE_ECG_DEF(m_ecg);                                                                 /**< BLE ECG service instance. */
BLE_BAS_DEF(m_bas);                                                                 /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_sensors_timer_id);                                                  /**< Sensor measurement timer. */
APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */

/* Private variables -------------------------------------------------- */
static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
  {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
  {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

uint32_t app_time;

/* Private function prototypes ---------------------------------------- */
static void timers_init(void);
static void gap_params_init(void);
static void nrf_qwr_error_handler(uint32_t nrf_error);
static void services_init(void);
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt);
static void conn_params_init(void);
static void sleep_mode_enter(void);
static void on_adv_evt(ble_adv_evt_t ble_adv_evt);
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context);
static void ble_stack_init(void);
static void gatt_init(void);
static void advertising_init(void);
static void log_init(void);
static void power_management_init(void);
static void advertising_start(void);

static void battery_level_meas_timeout_handler(void * p_context);
static void sensors_meas_timeout_handler(void * p_context);

static void battery_level_update(void);

static void ecg_service_init(void);
static void bas_service_init(void);
static void dis_service_init(void);

static void application_timers_start(void);

/* Function definitions ----------------------------------------------- */
/**
 * @brief Application main function.
 */
int main(void)
{
  // Initialize.
  log_init();
  timers_init();
  power_management_init();
  ble_stack_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();

  bsp_hw_init(); // Bsp init

  bsp_afe_init();

  bsp_bm_init();

  battery_level_update();

  // Start execution.
  application_timers_start();
  advertising_start();

  int16_t signal_val[ADS_NUM_CHANNEL];
  int16_t ecg_channel_buf[200];
  int16_t index = 0;

  for (;;)
  {
    if (nrf_gpio_pin_read(IO_AFE_DRDY) == false)
    {
      bsp_afe_get_ecg(signal_val);

      ecg_channel_buf[index]     = signal_val[0];
      ecg_channel_buf[index + 1] = signal_val[1];

      index += 2;

      if (index >= 100)
      {
        index = 0;
        ble_ecg_update(&m_ecg, (uint8_t *)&ecg_channel_buf, 200, BLE_CONN_HANDLE_ALL, BLE_ECG_CHANNEL_CHAR);
      }
    }
  }
}

/**
 * @brief         Function for assert macro callback.
 *
 * @param[in]     line_num     Line number of the failing ASSERT call.
 * @param[in]     p_file_name  File name of the failing ASSERT call.
 *
 * @attention     None
 *
 * @return        None
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/* Private function definitions --------------------------------------- */
/**
 * @brief         Function for initializing the timer module.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void timers_init(void)
{
  ret_code_t err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  // Create timers.
  err_code = app_timer_create(&m_sensors_timer_id,
                              APP_TIMER_MODE_REPEATED,
                              sensors_meas_timeout_handler);
  APP_ERROR_CHECK(err_code);

  err_code = app_timer_create(&m_battery_timer_id,
                              APP_TIMER_MODE_REPEATED,
                              battery_level_meas_timeout_handler);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for the GAP initialization.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void gap_params_init(void)
{
  uint32_t                err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
                                        (const uint8_t *)DEVICE_NAME,
                                        strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for handling Queued Write Module errors.
 *
 * @param[in]     nrf_error   Error code containing information about what went wrong.
 *
 * @attention     None
 *
 * @return        None
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief         Function for ECG service init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void ecg_service_init(void)
{
  uint32_t           err_code;
  ble_ecg_init_t     ecg_init;

  // Initialize ECG
  memset(&ecg_init, 0, sizeof(ecg_init));

  ecg_init.evt_handler          = NULL;
  ecg_init.support_notification = true;
  ecg_init.p_report_ref         = NULL;

  ecg_init.bl_rd_sec        = SEC_OPEN;
  ecg_init.bl_cccd_wr_sec   = SEC_OPEN;
  ecg_init.bl_report_rd_sec = SEC_OPEN;

  err_code = ble_ecg_init(&m_ecg, &ecg_init);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for BAS service init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void bas_service_init(void)
{
  uint32_t           err_code;
  ble_bas_init_t     bas_init;

  // Initialize Battery Service.
  memset(&bas_init, 0, sizeof(bas_init));

  bas_init.evt_handler          = NULL;
  bas_init.support_notification = true;
  bas_init.p_report_ref         = NULL;
  bas_init.initial_batt_level   = 100;

  // Here the sec level for the Battery Service can be changed/increased.
  bas_init.bl_rd_sec        = SEC_OPEN;
  bas_init.bl_cccd_wr_sec   = SEC_OPEN;
  bas_init.bl_report_rd_sec = SEC_OPEN;

  err_code = ble_bas_init(&m_bas, &bas_init);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for DIS service init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void dis_service_init(void)
{
  uint32_t           err_code;
  ble_dis_init_t     dis_init;

  // Initialize Device Information Service.
  memset(&dis_init, 0, sizeof(dis_init));

  ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

  dis_init.dis_char_rd_sec = SEC_OPEN;

  err_code = ble_dis_init(&dis_init);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for initializing services that will be used by the application.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void services_init(void)
{
  uint32_t           err_code;
  nrf_ble_qwr_init_t qwr_init = {0};

  // Initialize Queued Write Module.
  qwr_init.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  APP_ERROR_CHECK(err_code);

  // Initialize Custom Service
  ecg_service_init();

  // Initialize Battery Service.
  bas_service_init();

  // Initialize Device Information Service.
  dis_service_init();
}

/**
 * @brief         Function for handling an event from the Connection Parameters Module.
 *
 * @param[in]     p_evt     Event received from the Connection Parameters Module.
 *
 * @attention     None
 *
 * @return        None
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
  uint32_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
  {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**
 * @brief         Function for handling errors from the Connection Parameters module.
 *
 * @param[in]     nrf_error  Error code containing information about what went wrong.
 *
 * @attention     None
 *
 * @return        None
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief         Function for initializing the Connection Parameters module.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void conn_params_init(void)
{
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

/**
 * @brief         Function for putting the chip into sleep mode.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void sleep_mode_enter(void)
{
  uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

  // Prepare wakeup buttons.
  err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for handling advertising events.
 *
 * @param[in]     ble_adv_evt  Advertising event.
 *
 * @attention     None
 *
 * @return        None
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  uint32_t err_code;

  switch (ble_adv_evt)
  {
  case BLE_ADV_EVT_FAST:
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
    break;
  case BLE_ADV_EVT_IDLE:
    sleep_mode_enter();
    break;
  default:
    break;
  }
}

/**
 * @brief         Function for handling BLE events.
 *
 * @param[in]     p_ble_evt   Bluetooth stack event.
 * @param[in]     p_context   Unused.
 *
 * @attention     None
 *
 * @return        None
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
  uint32_t err_code;

  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GAP_EVT_CONNECTED:
    bsp_gpio_write(IO_RGB_BLUE, 0);
    bsp_gpio_write(IO_RGB_GREEN, 1);

    NRF_LOG_INFO("Connected");
    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    bsp_gpio_write(IO_RGB_GREEN, 0);
    bsp_gpio_write(IO_RGB_BLUE, 1);

    NRF_LOG_INFO("Disconnected");
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
  {
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys =
        {
          .rx_phys = BLE_GAP_PHY_AUTO,
          .tx_phys = BLE_GAP_PHY_AUTO,
        };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  }
  break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    // Pairing not supported
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

  default:
    break;
  }
}

/**
 * @brief         Function for the SoftDevice initialization.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
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

/**
 * @brief         Function for handling events from the GATT library.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
  if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
  {
    NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                 p_evt->conn_handle,
                 p_evt->params.att_mtu_effective);
  }
}

/**
 * @brief         Function for initializing the GATT library.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void gatt_init(void)
{
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for handling events from the BSP module.
 *
 * @param[in]     event   Event generated by button press.
 *
 * @attention     None
 *
 * @return        None
 */
void bsp_event_handler(bsp_event_t event)
{
  uint32_t err_code;
  switch (event)
  {
  case BSP_EVENT_SLEEP:
    sleep_mode_enter();
    break;

  case BSP_EVENT_DISCONNECT:
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
      APP_ERROR_CHECK(err_code);
    }
    break;

  case BSP_EVENT_WHITELIST_OFF:
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
      err_code = ble_advertising_restart_without_whitelist(&m_advertising);
      if (err_code != NRF_ERROR_INVALID_STATE)
      {
        APP_ERROR_CHECK(err_code);
      }
    }
    break;

  default:
    break;
  }
}

/**
 * @brief         Function for initializing the Advertising functionality.
 *
 * @param[in]     event   Event generated by button press.
 *
 * @attention     None
 *
 * @return        None
 */
static void advertising_init(void)
{
  uint32_t               err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type              = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance     = false;
  init.advdata.flags                  = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

  init.config.ble_adv_fast_enabled  = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
  init.evt_handler                  = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**
 * @brief         Function for initializing the nrf log module.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void log_init(void)
{
  ret_code_t err_code = NRF_LOG_INIT(app_timer_cnt_get);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**
 * @brief         Function for initializing power management.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void power_management_init(void)
{
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for starting advertising.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void advertising_start(void)
{
  uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Function for handling the Battery measurement timer timeout.
 *
 * @param[in]     p_context   Pointer to context
 *
 * @attention     None
 *
 * @return        None
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
  battery_level_update();
}

/**
 * @brief         Function for handling the sensor measurement timer timeout.
 *
 * @param[in]     p_context   Pointer to context
 *
 * @attention     None
 *
 * @return        None
 */
static void sensors_meas_timeout_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
}

/**
 * @brief         Function for handling the battery level update
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void battery_level_update(void)
{
  uint8_t battery_level;

  bsp_bm_get_soc(&battery_level);

  NRF_LOG_INFO("Battery avg : %d percent", battery_level);

  ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
}

/**
 * @brief         Function for starting application timers.
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void application_timers_start(void)
{
  ret_code_t err_code;

  // Start application timers.
  err_code =  app_timer_start(m_battery_timer_id, BATT_LEVEL_MEAS_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);
}

/* End of fi le -------------------------------------------------------- */
