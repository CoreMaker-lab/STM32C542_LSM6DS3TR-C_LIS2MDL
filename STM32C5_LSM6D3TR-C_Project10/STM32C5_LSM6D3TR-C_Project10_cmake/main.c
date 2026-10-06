/**
  ******************************************************************************
  * file           : main.c
  * brief          : Main program body
  *                  LSM6DS3TR-C FIFO watermark interrupt + 6-axis data read.
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

#include "mx_usart1.h"
#include "mx_i2c1.h"
#include "mx_gpio_default.h"
#include <stdio.h>
#include <string.h>

#include "lsm6ds3tr-c_reg.h"

int _write(int file, char *ptr, int len)
{
    hal_uart_handle_t *huart1 = mx_usart1_uart_gethandle();

    if (huart1 != NULL)
    {
        HAL_UART_Transmit(huart1, (uint8_t *)ptr, len, 1000);
    }

    return len;
}

#define BOOT_TIME 10U
#define BOARD_EXPECTED_ID LSM6DS3TR_C_ID
/* LSM6DS3TR-C FIFO pattern with XL and GY both at no decimation:
 * gyroscope XYZ (6 bytes) then accelerometer XYZ (6 bytes) = 12 bytes. */
#define FIFO_PATTERN_LEN 12U
/* Watermark is expressed in bytes and must be a multiple of the pattern. */
#define FIFO_WATERMARK   (10U * FIFO_PATTERN_LEN)

static stmdev_ctx_t dev_ctx;
static volatile uint8_t thread_wake = 0;
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
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
    printf("HELLO\r\n");

    HAL_GPIO_WritePin(CS1_PORT, CS1_PIN, HAL_GPIO_PIN_SET);
    HAL_GPIO_WritePin(SA0_PORT, SA0_PIN, HAL_GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS2_PORT, CS2_PIN, HAL_GPIO_PIN_SET);

    uint8_t whoamI = 0;
    uint8_t rst = 0;
    lsm6ds3tr_c_int1_route_t pin_int = {0};

    /* Initialize driver interface. */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = mx_i2c1_i2c_gethandle();

    platform_delay(BOOT_TIME);

    /* Check device ID. */
    lsm6ds3tr_c_device_id_get(&dev_ctx, &whoamI);

    printf("LSM6DS3TR_C_ID=0x%x,id=0x%x\r\n",
           (unsigned int)BOARD_EXPECTED_ID, (unsigned int)whoamI);

    if (whoamI != BOARD_EXPECTED_ID)
        while (1);

    /* Reset device and wait for completion. */
    lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
      lsm6ds3tr_c_reset_get(&dev_ctx, &rst);
    } while (rst);
    platform_delay(BOOT_TIME);

    /* Enable Block Data Update. */
    lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* INT1: open-drain + active low (matches falling-edge EXTI on the board). */
    lsm6ds3tr_c_pin_mode_set(&dev_ctx, LSM6DS3TR_C_OPEN_DRAIN);
    lsm6ds3tr_c_pin_polarity_set(&dev_ctx, LSM6DS3TR_C_ACTIVE_LOW);

    /* Set full scales before starting the sensors. */
    lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);
    lsm6ds3tr_c_gy_full_scale_set(&dev_ctx, LSM6DS3TR_C_2000dps);

    /* Mask data-ready during filter settling. */
    lsm6ds3tr_c_filter_settling_mask_set(&dev_ctx, PROPERTY_ENABLE);

    /* Bypass clears FIFO. Watermark is expressed in bytes. */
    lsm6ds3tr_c_fifo_mode_set(&dev_ctx, LSM6DS3TR_C_BYPASS_MODE);
    lsm6ds3tr_c_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

    /* Batch gyroscope and accelerometer with no decimation.
     * FIFO pattern = GYRO (6 bytes) + XL (6 bytes). */
    lsm6ds3tr_c_fifo_xl_batch_set(&dev_ctx, LSM6DS3TR_C_FIFO_XL_NO_DEC);
    lsm6ds3tr_c_fifo_gy_batch_set(&dev_ctx, LSM6DS3TR_C_FIFO_GY_NO_DEC);

    /* FIFO ODR must be greater than or equal to the sensor ODR. */
    lsm6ds3tr_c_fifo_data_rate_set(&dev_ctx, LSM6DS3TR_C_FIFO_26Hz);

    /* Route FIFO watermark to INT1. */
    pin_int.int1_fth = PROPERTY_ENABLE;
    lsm6ds3tr_c_pin_int1_route_set(&dev_ctx, pin_int);

    /* Continuous FIFO: oldest unread data is overwritten when full. */
    lsm6ds3tr_c_fifo_mode_set(&dev_ctx, LSM6DS3TR_C_STREAM_MODE);

    /* Start accelerometer and gyroscope at 26 Hz. */
    lsm6ds3tr_c_xl_power_mode_set(&dev_ctx, LSM6DS3TR_C_XL_HIGH_PERFORMANCE);
    lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_26Hz);
    lsm6ds3tr_c_gy_power_mode_set(&dev_ctx, LSM6DS3TR_C_GY_HIGH_PERFORMANCE);
    lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_26Hz);

    while (1)
    {
        if (thread_wake)
        {
            uint16_t level = 0;
            uint16_t num_pattern;
            float xl_sum[3] = {0};
            float gy_sum[3] = {0};
            uint16_t xl_count = 0;
            uint16_t gy_count = 0;
            uint8_t waterm = 0;

            thread_wake = 0;

            /* Number of unread bytes in FIFO. */
            if (lsm6ds3tr_c_fifo_data_level_get(&dev_ctx, &level) != 0)
            {
                thread_wake = 1;
                platform_delay(100);
                continue;
            }

            num_pattern = level / FIFO_PATTERN_LEN;

            /* Read the records already present in this FIFO snapshot. */
            while (num_pattern-- > 0)
            {
                uint8_t raw_gy[6] = {0};
                uint8_t raw_xl[6] = {0};
                int16_t gy[3];
                int16_t xl[3];

                if ((lsm6ds3tr_c_fifo_raw_data_get(&dev_ctx, raw_gy, 6) != 0) ||
                    (lsm6ds3tr_c_fifo_raw_data_get(&dev_ctx, raw_xl, 6) != 0))
                {
                    thread_wake = 1;
                    platform_delay(100);
                    break;
                }

                /* Decode little-endian axes without unaligned pointers. */
                gy[0] = (int16_t)((uint16_t)raw_gy[0] | ((uint16_t)raw_gy[1] << 8));
                gy[1] = (int16_t)((uint16_t)raw_gy[2] | ((uint16_t)raw_gy[3] << 8));
                gy[2] = (int16_t)((uint16_t)raw_gy[4] | ((uint16_t)raw_gy[5] << 8));
                xl[0] = (int16_t)((uint16_t)raw_xl[0] | ((uint16_t)raw_xl[1] << 8));
                xl[1] = (int16_t)((uint16_t)raw_xl[2] | ((uint16_t)raw_xl[3] << 8));
                xl[2] = (int16_t)((uint16_t)raw_xl[4] | ((uint16_t)raw_xl[5] << 8));

                gy_sum[0] += lsm6ds3tr_c_from_fs2000dps_to_mdps(gy[0]);
                gy_sum[1] += lsm6ds3tr_c_from_fs2000dps_to_mdps(gy[1]);
                gy_sum[2] += lsm6ds3tr_c_from_fs2000dps_to_mdps(gy[2]);
                gy_count++;

                xl_sum[0] += lsm6ds3tr_c_from_fs2g_to_mg(xl[0]);
                xl_sum[1] += lsm6ds3tr_c_from_fs2g_to_mg(xl[1]);
                xl_sum[2] += lsm6ds3tr_c_from_fs2g_to_mg(xl[2]);
                xl_count++;
            }

            /* Print only after reading the batch to limit UART overhead.
             * These are batch averages, not individual FIFO samples. */
            if (xl_count > 0)
            {
                printf("XL avg (%u) [mg]: %.2f, %.2f, %.2f\r\n",
                        (unsigned int)xl_count,
                        (double)(xl_sum[0] / xl_count),
                        (double)(xl_sum[1] / xl_count),
                        (double)(xl_sum[2] / xl_count));
            }

            if (gy_count > 0)
            {
                printf("GY avg (%u) [mdps]: %.2f, %.2f, %.2f\r\n\r\n",
                        (unsigned int)gy_count,
                        (double)(gy_sum[0] / gy_count),
                        (double)(gy_sum[1] / gy_count),
                        (double)(gy_sum[2] / gy_count));
            }

            /* New samples arrived during reads/printing. If still above
             * watermark, keep servicing: INT1 may remain active low and
             * no additional falling edge would be generated. */
            lsm6ds3tr_c_fifo_wtm_flag_get(&dev_ctx, &waterm);
            if (waterm)
            {
                thread_wake = 1;
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
