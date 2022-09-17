/**
 * @file       ble_ecg.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-01-07
 * @author     Quang Ha
 * @brief      ECG (BLE ECG Service)
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "sdk_common.h"
#include "ble.h"
#include "ble_ecg.h"
#include "ble_srv_common.h"
#include "nrf_log.h"

/* Private defines ---------------------------------------------------- */
#define BLE_UUID_ECG_CHANNEL_CHARACTERISTIC  0x1235

#define ECG_BASE_UUID                                                                                \
  {                                                                                                  \
    {                                                                                                \
      0x41, 0xEE, 0x68, 0x3A, 0x99, 0x0F, 0x0E, 0x72, 0x85, 0x49, 0x8D, 0xB3, 0x00, 0x00, 0x00, 0x00 \
    }                                                                                                \
  } /**< Used vendor specific UUID. */

/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
static const uint16_t BLE_UUID_CHAR[] = {
  BLE_UUID_ECG_CHANNEL_CHARACTERISTIC
};

/* Private function prototypes ---------------------------------------- */
static void m_ble_ecg_on_connect(ble_ecg_t *p_ecg, ble_evt_t const *p_ble_evt);
static void m_ble_ecg_on_write(ble_ecg_t *p_ecg, ble_evt_t const *p_ble_evt);

static ret_code_t m_ble_ecg_add_char(ble_ecg_t *p_ecg, const ble_ecg_init_t *p_ecg_init, ble_ecg_charaterictic_t charac);
static ret_code_t m_ble_ecg_send_notification(ble_gatts_hvx_params_t *const p_hvx_params, uint16_t conn_handle);

/* Function definitions ----------------------------------------------- */
uint32_t ble_ecg_init(ble_ecg_t *p_ecg, ble_ecg_init_t const *p_ecg_init)
{
  ret_code_t err_code;
  ble_uuid_t ble_uuid;
  ble_uuid128_t ecg_base_uuid = ECG_BASE_UUID;

  VERIFY_PARAM_NOT_NULL(p_ecg);
  VERIFY_PARAM_NOT_NULL(p_ecg_init);

  // Initialize the service structure.
  p_ecg->evt_handler               = p_ecg_init->evt_handler;
  p_ecg->is_notification_supported = p_ecg_init->support_notification;

  // Add a custom base UUID.
  err_code = sd_ble_uuid_vs_add(&ecg_base_uuid, &p_ecg->uuid_type);
  VERIFY_SUCCESS(err_code);

  ble_uuid.type = p_ecg->uuid_type;
  ble_uuid.uuid = BLE_UUID_ECG_SERVICE;

  // Add the service.
  err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_ecg->service_handle);
  VERIFY_SUCCESS(err_code);

  // Add the ECG Characteristics.
  err_code = m_ble_ecg_add_char(p_ecg, p_ecg_init, BLE_ECG_CHANNEL_CHAR);
  VERIFY_SUCCESS(err_code);

  return err_code;
}

ret_code_t ble_ecg_update(ble_ecg_t *p_ecg, uint8_t *ecg,
                          uint16_t len, uint16_t conn_handle,
                          ble_ecg_charaterictic_t charac)
{
  ret_code_t err_code;

  // Send value if connected and notifying.
  if (conn_handle != BLE_CONN_HANDLE_INVALID)
  {
    ble_gatts_hvx_params_t hvx_params;

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_ecg->acc_char_handles[charac].value_handle;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.p_len  = &len;
    hvx_params.p_data = ecg;

    if (conn_handle == BLE_CONN_HANDLE_ALL)
    {
      ble_conn_state_conn_handle_list_t conn_handles = ble_conn_state_conn_handles();

      // Try sending notifications to all valid connection handles.
      for (uint32_t i = 0; i < conn_handles.len; i++)
      {
        if (ble_conn_state_status(conn_handles.conn_handles[i]) == BLE_CONN_STATUS_CONNECTED)
          err_code = m_ble_ecg_send_notification(&hvx_params, conn_handles.conn_handles[i]);
      }
    }
    else
    {
      err_code = m_ble_ecg_send_notification(&hvx_params, conn_handle);
    }
  }
  else
  {
    err_code = NRF_ERROR_INVALID_STATE;
  }

  return err_code;
}

void ble_ecg_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
  if ((p_context == NULL) || (p_ble_evt == NULL))
    return;

  ble_ecg_t *p_ecg = (ble_ecg_t *)p_context;

  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GAP_EVT_CONNECTED:
    m_ble_ecg_on_connect(p_ecg, p_ble_evt);
    break;

  case BLE_GATTS_EVT_WRITE:
    m_ble_ecg_on_write(p_ecg, p_ble_evt);
    break;

  default:
    break;
  }
}

/* Private function definitions --------------------------------------- */
/**
 * @brief         Function for adding the ECG characteristic.
 *
 * @param[in]     p_ecg         ECG Service structure.
 * @param[in]     p_ecg_init    Information needed to initialize the service.
 * @param[in]     char_uuid     Charaterictic UUID
 *
 * @attention     None
 *
 * @return        None
 */
static ret_code_t m_ble_ecg_add_char(ble_ecg_t *p_ecg, const ble_ecg_init_t *p_ecg_init, ble_ecg_charaterictic_t charac)
{
  ble_add_char_params_t   add_char_params;

  memset(&add_char_params, 0, sizeof(add_char_params));
  add_char_params.uuid              = BLE_UUID_CHAR[charac];
  add_char_params.max_len           = 250;
  add_char_params.init_len          = sizeof(uint8_t);
  add_char_params.char_props.notify = p_ecg->is_notification_supported;
  add_char_params.char_props.read   = 1;
  add_char_params.is_var_len        = true;
  add_char_params.cccd_write_access = p_ecg_init->bl_cccd_wr_sec;
  add_char_params.read_access       = p_ecg_init->bl_rd_sec;

  return characteristic_add(p_ecg->service_handle, &add_char_params, &(p_ecg->acc_char_handles[charac]));
}

/**
 * @brief         Function for sending notifications with the ECG characteristic.
 *
 * @param[in]     p_hvx_params Pointer to structure with notification data.
 * @param[in]     conn_handle  Connection handle.
 *
 * @attention     None
 *
 * @return        NRF_SUCCESS on success, otherwise an error code.
 * 
 */
static ret_code_t m_ble_ecg_send_notification(ble_gatts_hvx_params_t *const p_hvx_params, uint16_t conn_handle)
{
  ret_code_t err_code = sd_ble_gatts_hvx(conn_handle, p_hvx_params);

  if (err_code == NRF_SUCCESS)
  {
    NRF_LOG_INFO("ECG notification has been sent using conn_handle: 0x%04X", conn_handle);
  }
  else
  {
    NRF_LOG_DEBUG("Error: 0x%08X while sending notification with conn_handle: 0x%04X", err_code, conn_handle);
  }

  return err_code;
}

/**
 * @brief         Function for handling the Connect event.
 *
 * @param[in]     p_ecg       ECG Service structure.
 * @param[in]     p_ble_evt   Pointer to the event received from BLE stack.
 *
 * @attention     None
 *
 * @return        None
 */
static void m_ble_ecg_on_connect(ble_ecg_t *p_ecg, ble_evt_t const *p_ble_evt)
{

}

/**
 * @brief         Function for handling the Write event.
 *
 * @param[in]     p_ecg       ECG Service structure.
 * @param[in]     p_ble_evt   Pointer to the event received from BLE stack.
 *
 * @attention     None
 *
 * @return        None
 */
static void m_ble_ecg_on_write(ble_ecg_t *p_ecg, ble_evt_t const *p_ble_evt)
{

}

/* End of file -------------------------------------------------------- */
