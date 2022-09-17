
/**
 * @file       bsp.c
 * @copyright  Copyright (C) 2020 Hydratech. All rights reserved.
 * @license    This project is released under the Hydratech License.
 * @version    1.0.0
 * @date       2021-01-23
 * @author     Quang Ha
 * @brief      Board Support Package (BSP)
 * 
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "bsp.h"
#include "it_callback.h"

/* Private defines ---------------------------------------------------- */
#define TWI_INSTANCE         0
#define SPI_INSTANCE         1

/* Private enumerate/structure ---------------------------------------- */
static nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE);
static nrf_drv_spi_t m_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);

/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
static volatile bool_t data_ready = BS_FALSE;

/* Private variables -------------------------------------------------- */
/* Private function prototypes ---------------------------------------- */
static void m_bsp_i2c_init(void);
static void m_bsp_gpio_init(void);
static void m_bsp_spi_init(void);

/* Function definitions ----------------------------------------------- */
void bsp_hw_init(void)
{
  m_bsp_i2c_init();
  m_bsp_gpio_init();
  m_bsp_spi_init();
}

int bsp_i2c_write(uint8_t slave_addr, uint8_t reg_addr, uint8_t *p_data, uint32_t len)
{
  uint8_t buffer[10];

  memcpy(buffer, &reg_addr, 1);
  memcpy(buffer + 1, p_data, len);

  return nrf_drv_twi_tx(&m_twi, slave_addr, buffer, len + 1, false);
}

int bsp_i2c_read(uint8_t slave_addr, uint8_t reg_addr, uint8_t *p_data, uint32_t len)
{
  nrf_drv_twi_tx(&m_twi, slave_addr, (uint8_t *)&reg_addr, 1, true);

  return nrf_drv_twi_rx(&m_twi, slave_addr, p_data, len);
}

int bsp_spi_transmit_receive(uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
  data_ready = BS_FALSE;

  nrf_drv_spi_transfer(&m_spi, tx_data, len, rx_data, len);
  while (!data_ready)
  {
  }
  
  return BS_OK;
}

void bsp_delay_ms(uint32_t ms)
{
  nrf_delay_ms(ms);
}

void bsp_gpio_write(uint8_t pin , uint8_t state)
{
  nrf_gpio_pin_write(pin, state);
}

/* Private function definitions ---------------------------------------- */
/**
 * @brief         I2C init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_i2c_init(void)
{
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config =
  {
    .scl                = IO_I2C0_SCL,
    .sda                = IO_I2C0_SDA,
    .frequency          = NRF_DRV_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
    .clear_bus_init     = false
  };

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);

  nrf_drv_twi_enable(&m_twi);
}

void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
  data_ready = BS_TRUE;
}

/**
 * @brief         SPI init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_spi_init(void)
{
  ret_code_t err_code;

  nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

  spi_config.mosi_pin  = IO_AFE_MOSI;
  spi_config.miso_pin  = IO_AFE_MISO;
  spi_config.sck_pin   = IO_AFE_SCLK;
  spi_config.mode      = NRF_DRV_SPI_MODE_0;
  spi_config.frequency = NRF_DRV_SPI_FREQ_8M;

  err_code = nrf_drv_spi_init(&m_spi, &spi_config, spi_event_handler, NULL);
  APP_ERROR_CHECK(err_code);
}

/**
 * @brief         Gpio init
 *
 * @param[in]     None
 *
 * @attention     None
 *
 * @return        None
 */
static void m_bsp_gpio_init(void)
{
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  // // AFE ready config pin
  // nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  // in_config.pull = NRF_GPIO_PIN_PULLUP;

  // err_code = nrf_drv_gpiote_in_init(IO_AFE_DRDY, &in_config, exint_afe_drdy_event_handler);
  // APP_ERROR_CHECK(err_code);
  // nrf_drv_gpiote_in_event_enable(IO_AFE_DRDY, true);

  nrf_gpio_cfg_input(IO_AFE_DRDY, NRF_GPIO_PIN_PULLUP);

  // Output in setting
  nrf_gpio_cfg_output(IO_AFE_CS);
  bsp_gpio_write(IO_AFE_CS, 1);

  nrf_gpio_cfg_output(IO_AFE_RST);
  bsp_gpio_write(IO_AFE_RST, 1);

  nrf_gpio_cfg_output(IO_RGB_BLUE);
  nrf_gpio_cfg_output(IO_RGB_RED);
  nrf_gpio_cfg_output(IO_RGB_GREEN);

  bsp_gpio_write(IO_RGB_BLUE, 1);
  bsp_gpio_write(IO_RGB_RED, 1);
  bsp_gpio_write(IO_RGB_GREEN, 0);
}

/* End of file -------------------------------------------------------- */
