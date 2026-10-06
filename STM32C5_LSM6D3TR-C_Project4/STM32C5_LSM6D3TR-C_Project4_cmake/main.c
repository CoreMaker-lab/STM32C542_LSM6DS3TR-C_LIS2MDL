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
/* Private functions prototype -----------------------------------------------*/

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

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME            10 //ms
/* Private variables ---------------------------------------------------------*/
static uint8_t whoamI;
/* Extern variables ----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void platform_delay(uint32_t ms);
static stmdev_ctx_t dev_ctx;
static volatile uint8_t thread_wake = 0;
/* STM32 HAL expects the 8-bit I2C address.
 * SA0 = 0 -> 0xD5, SA0 = 1 -> 0xD7 (see LSM6DS3TR_C_I2C_ADD_L/H). */
#define LSM6DS3TR_C_I2C_ADD  LSM6DS3TR_C_I2C_ADD_L

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
  if (HAL_I2C_MASTER_MemWrite((hal_i2c_handle_t *)handle, LSM6DS3TR_C_I2C_ADD, reg,
                              HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
  /* NOTE: the 0x80 read bit is only required on the SPI interface.
   * On I2C the register address must be sent as-is. */
  if (HAL_I2C_MASTER_MemRead((hal_i2c_handle_t *)handle, LSM6DS3TR_C_I2C_ADD, reg,
                             HAL_I2C_MEM_ADDR_8BIT, bufp, len, 1000) != HAL_OK)
    return -1;
  return 0;
}

static void platform_delay(uint32_t ms) {
  HAL_Delay(ms);
}

/* EXTI line (INT1 on PB0) trigger callback: wake up the main loop. */
void HAL_EXTI_TriggerCallback(hal_exti_handle_t *hexti, hal_exti_trigger_t trigger)
{
  (void)hexti;
  (void)trigger;
  thread_wake = 1;
}

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

      lsm6ds3tr_c_int1_route_t int1_route = {0};
      uint8_t step_data[2] = {0};
      uint8_t rst = 0;
      uint16_t step_count = 0;

      /* Initialize mems driver interface */
      dev_ctx.write_reg = platform_write;
      dev_ctx.read_reg = platform_read;
      dev_ctx.mdelay = platform_delay;
      dev_ctx.handle = mx_i2c1_i2c_gethandle();

      /* Wait sensor boot time */
      platform_delay(BOOT_TIME);

      /* Check device ID */
      lsm6ds3tr_c_device_id_get(&dev_ctx, &whoamI);

      printf("LSM6DS3TR_C_ID=0x%x,id=0x%x\n",
             LSM6DS3TR_C_ID, whoamI);

      if (whoamI != LSM6DS3TR_C_ID)
        while (1);

      /* Perform software reset */
      lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);
      do {
        lsm6ds3tr_c_reset_get(&dev_ctx, &rst);
      } while (rst);
      platform_delay(10);

      /* Enable Block Data Update */
      lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

      /*
       * Pedometer works internally at 26 Hz.
       * Accelerometer ODR must be >= 26 Hz.
       */
      lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_52Hz);
      lsm6ds3tr_c_xl_power_mode_set(&dev_ctx, LSM6DS3TR_C_XL_HIGH_PERFORMANCE);

      /* Set accelerometer full scale */
      lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);

      /* INT1/INT2: open-drain + active low */
      lsm6ds3tr_c_pin_mode_set(&dev_ctx, LSM6DS3TR_C_OPEN_DRAIN);
      lsm6ds3tr_c_pin_polarity_set(&dev_ctx, LSM6DS3TR_C_ACTIVE_LOW);

      /* Enable pedometer and step counter */
      lsm6ds3tr_c_pedo_sens_set(&dev_ctx, PROPERTY_ENABLE);

      /* Set pedometer full scale */
      lsm6ds3tr_c_pedo_full_scale_set(&dev_ctx, LSM6DS3TR_C_PEDO_AT_2g);

      /*
       * Set pedometer debounce.
       *
       * Default value = 10 steps.
       * Set to 3 steps for easier demonstration.
       */
      lsm6ds3tr_c_pedo_debounce_steps_set(&dev_ctx, 3);

      /* Reset step counter */
      lsm6ds3tr_c_pedo_step_reset_set(&dev_ctx, PROPERTY_ENABLE);
      platform_delay(10);
      lsm6ds3tr_c_pedo_step_reset_set(&dev_ctx, PROPERTY_DISABLE);

      /* Route Step Detector event to INT1 */
      int1_route.int1_step_detector = PROPERTY_ENABLE;
      lsm6ds3tr_c_pin_int1_route_set(&dev_ctx, int1_route);

      printf("Pedometer start...\r\n");

      while (1) {

          if (thread_wake)
          {
            lsm6ds3tr_c_all_sources_t status = {0};

            thread_wake = 0;

            /* Read interrupt source */
            lsm6ds3tr_c_all_sources_get(&dev_ctx,
                                        &status);

            /* Check Step Detector event */
            if (status.func_src1.step_detected)
            {
              /* Read Step Counter (16-bit little endian) */
              lsm6ds3tr_c_read_reg(&dev_ctx,
                                   LSM6DS3TR_C_STEP_COUNTER_L,
                                   step_data, 2);
              step_count = (uint16_t)(step_data[0] | (step_data[1] << 8));

              printf("Step detected, Steps = %d\r\n",
                     step_count);
            }
          }
      }
  }
} /* end main */

