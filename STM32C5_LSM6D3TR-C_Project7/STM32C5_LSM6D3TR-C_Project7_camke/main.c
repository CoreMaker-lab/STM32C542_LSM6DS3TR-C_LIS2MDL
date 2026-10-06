/**
  ******************************************************************************
  * file           : main.c
  * brief          : Main program body
  *                  Calls target system initialization then loop in main.
  ******************************************************************************
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#include "mx_usart1.h"
#include <stdio.h>
#include <string.h>

#include "lsm6ds3tr-c_reg.h"

int _write(int file, char *ptr, int len)
{
    hal_uart_handle_t *huart1 = mx_usart1_uart_gethandle();

    if (huart1 != NULL)
    {
        HAL_UART_Transmit(huart1, ptr, len, 1000);
    }

    return len;
}

#define BOOT_TIME 10U
#define BOARD_EXPECTED_ID 0x6AU
#define FREE_FALL_DURATION 12U

static stmdev_ctx_t dev_ctx;
static volatile uint8_t thread_wake = 0;
static int32_t platform_write(void *handle,uint8_t reg,const uint8_t *bufp,uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg,uint8_t *bufp,uint16_t len);
static void platform_delay(uint32_t ms);

/**
 * @brief  EXTI trigger callback
 */
void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti, hal_exti_trigger_t trigger) {
    (void)trigger;
    if (HAL_EXTI_GetInstance(hexti) == HAL_EXTI_GPIO_0)
    {
        thread_wake = 1;
    }
}


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private functions prototype -----------------------------------------------*/

/**
  * brief:  The application entry point.
  * retval: none but we specify int to comply with C99 standard
  */
int main(void)
{
  /** System Init: this code placed in targets folder initializes your system.
    * It calls the initialization (and sets the initial configuration) of the peripherals.
    * You can use STM32CubeMX to generate and call this code or not in this project.
    * It also contains the HAL initialization and the initial clock configuration.
    */
  if (mx_system_init() != SYSTEM_OK)
  {
    return (-1);
  }
  else
  {
    /*
      * You can start your application code here
      */

    printf("HELLO\n");
    HAL_GPIO_WritePin(CS1_PORT, CS1_PIN, HAL_GPIO_PIN_SET);
    HAL_GPIO_WritePin(SA0_PORT, SA0_PIN, HAL_GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS2_PORT, CS2_PIN, HAL_GPIO_PIN_SET);

    lsm6ds3tr_c_int1_route_t pin_int = {0};
    uint8_t whoamI;

    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = mx_i2c1_i2c_gethandle();

    /* Wait sensor boot time */
    platform_delay(BOOT_TIME);

    /* Check device ID */
    lsm6ds3tr_c_device_id_get(&dev_ctx, &whoamI);

    printf("LSM6DS3TR_C_ID=0x%x,id=0x%x\n", LSM6DS3TR_C_ID, whoamI);

    if (whoamI != BOARD_EXPECTED_ID)
        while (1);

    /* Reset Device */
    lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);
    platform_delay(BOOT_TIME);

    /* Enable Block Data Update */
    lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Low-G accelerometer: 416 Hz, +/-2 g. */
    lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_416Hz);
    lsm6ds3tr_c_xl_power_mode_set(&dev_ctx, LSM6DS3TR_C_XL_HIGH_PERFORMANCE);
    lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);

    /* INT1: open-drain + active low. */
    lsm6ds3tr_c_pin_mode_set(&dev_ctx, LSM6DS3TR_C_OPEN_DRAIN);
    lsm6ds3tr_c_pin_polarity_set(&dev_ctx, LSM6DS3TR_C_ACTIVE_LOW);

    /* Enable interrupts in latched mode. */
    lsm6ds3tr_c_int_notification_set(&dev_ctx, LSM6DS3TR_C_INT_LATCHED);

    /* Free-fall threshold = 312 mg. */
    lsm6ds3tr_c_ff_threshold_set(&dev_ctx, LSM6DS3TR_C_FF_TSH_312mg);

    /* Duration = 12 / 416 Hz = ~29 ms. */
    lsm6ds3tr_c_ff_dur_set(&dev_ctx, FREE_FALL_DURATION);

    /* Route Free-fall event to INT1. */
    pin_int.int1_ff = PROPERTY_ENABLE;
    lsm6ds3tr_c_pin_int1_route_set(&dev_ctx, pin_int);

    while (1)
    {
        if (thread_wake)
        {
            lsm6ds3tr_c_all_sources_t status = {0};

            thread_wake = 0;

            /* Read interrupt source and clear the latched event. */
            if (lsm6ds3tr_c_all_sources_get(&dev_ctx, &status) != 0)
            {
                thread_wake = 1;
                platform_delay(100);
                continue;
            }

            if (status.wake_up_src.ff_ia)
            {
                printf("Free-fall detected\r\n");
            }
        }
    }
  }
} /* end main */



static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  if (HAL_I2C_MASTER_MemWrite((hal_i2c_handle_t *)handle, LSM6DS3TR_C_I2C_ADD_L, reg,
                              HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  /* NOTE: the 0x80 read bit is only required on the SPI interface.
   * On I2C the register address must be sent as-is. */
  if (HAL_I2C_MASTER_MemRead((hal_i2c_handle_t *)handle, LSM6DS3TR_C_I2C_ADD_L, reg,
                             HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}
