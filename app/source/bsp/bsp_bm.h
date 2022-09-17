/**
 * @file       bsp_bm.h
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-05-09
 * @author     Quang Ha
 * @brief      Board Support Package Battery Monitor
 * @note       None
 * @example    None
 */

/* Define to prevent recursive inclusion ------------------------------ */
#ifndef __BSP_BM_H
#define __BSP_BM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------- */
#include "ds2728.h"
#include "bsp.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
typedef struct
{
  float temp;
  float voltage;
  float current;
  uint8_t capacity;
}
bsp_bm_info_t;

/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
/**
 * @brief         BSP battery monitor init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return
 * - BS_OK
 * - BS_ERROR
 */
base_status_t bsp_bm_init(void);

base_status_t bsp_bm_get_info(bsp_bm_info_t *bm);

base_status_t bsp_bm_get_soc(uint8_t *soc);

/* -------------------------------------------------------------------------- */
#ifdef __cplusplus
} // extern "C"
#endif
#endif // __BSP_BM_H

/* End of file -------------------------------------------------------- */
