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
#define BOARD_EXPECTED_ID LSM6DS3TR_C_ID
#define ACTIVITY_THRESHOLD   4U
#define INACTIVITY_DURATION  2U
#define WAKE_UP_DURATION     0U

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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
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

    printf("HELLO\r\n");

    HAL_GPIO_WritePin(CS1_PORT, CS1_PIN, HAL_GPIO_PIN_SET);
    HAL_GPIO_WritePin(SA0_PORT, SA0_PIN, HAL_GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CS2_PORT, CS2_PIN, HAL_GPIO_PIN_SET);

    uint8_t whoamI = 0;
    uint8_t rst = 0;
    uint8_t sleep_state = 0;   /* 0 = activity, 1 = inactivity (boot: active) */
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

    /* Reset device. */
    lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do {
        lsm6ds3tr_c_reset_get(&dev_ctx, &rst);
    } while (rst != 0);
    platform_delay(BOOT_TIME);

    /* Enable Block Data Update. */
    lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    /* Avoid events during accelerometer filter settling. */
    lsm6ds3tr_c_filter_settling_mask_set(&dev_ctx, PROPERTY_ENABLE);

    /* Low-G accelerometer: 416 Hz, +/-2 g. */
    lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_416Hz);
    lsm6ds3tr_c_xl_power_mode_set(&dev_ctx, LSM6DS3TR_C_XL_HIGH_PERFORMANCE);
    lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);

    /* INT1: open-drain + active low. */
    lsm6ds3tr_c_pin_mode_set(&dev_ctx, LSM6DS3TR_C_OPEN_DRAIN);
    lsm6ds3tr_c_pin_polarity_set(&dev_ctx, LSM6DS3TR_C_ACTIVE_LOW);

    /* Enable basic interrupts in latched mode. */
    lsm6ds3tr_c_int_notification_set(&dev_ctx, LSM6DS3TR_C_INT_LATCHED);

    /* Select the filter for wake-up and activity/inactivity detection. */
    lsm6ds3tr_c_xl_hp_path_internal_set(&dev_ctx, LSM6DS3TR_C_USE_SLOPE);

    /* Configure the wake-up (activity) threshold, 1 LSB = FS_XL / 64.
     * The same threshold is used for the inactivity detection. */
    lsm6ds3tr_c_wkup_threshold_set(&dev_ctx, ACTIVITY_THRESHOLD);

    /* Configure the duration to go in inactivity, 1 LSB = 512 / ODR. */
    lsm6ds3tr_c_act_sleep_dur_set(&dev_ctx, INACTIVITY_DURATION);

    /* Configure the wake-up duration, 1 LSB = 1 / ODR. */
    lsm6ds3tr_c_wkup_dur_set(&dev_ctx, WAKE_UP_DURATION);

    /* Enable inactivity function: XL at 12.5 Hz, GY not affected. */
    lsm6ds3tr_c_act_mode_set(&dev_ctx, LSM6DS3TR_C_XL_12Hz5_GY_NOT_AFFECTED);

    /* Route wake-up (activity) and inactivity state changes to INT1.
     * pin_int1_route_set() also enables the basic interrupts. */
    pin_int.int1_wu = PROPERTY_ENABLE;
    pin_int.int1_inact_state = PROPERTY_ENABLE;
    lsm6ds3tr_c_pin_int1_route_set(&dev_ctx, pin_int);

    printf("Activity/Inactivity detection start...\r\n");

    while (1)
    {
        if (thread_wake)
        {
            lsm6ds3tr_c_wake_up_src_t status = {0};
            thread_wake = 0;
            /* Read the state and acknowledge the latched event. */
            if (lsm6ds3tr_c_read_reg(&dev_ctx, LSM6DS3TR_C_WAKE_UP_SRC, (uint8_t *)&status, 1) != 0)
            {
                thread_wake = 1;
                platform_delay(100);
                continue;
            }

            /* Derive the current state and only report it when it changes,
             * so the message is printed once per state transition and not
             * repeatedly while the board stays in the same state. */
            uint8_t new_state;

            if (status.wu_ia)
                new_state = 0;                 /* wake-up (activity) event */
            else if (status.sleep_state_ia)
                new_state = 1;                 /* inactivity state */
            else
                new_state = sleep_state;       /* no valid info, keep */

            if (new_state != sleep_state)
            {
                sleep_state = new_state;

                if (sleep_state)
                    printf("Inactivity detected\r\n");
                else
                    printf("Activity detected\r\n");
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
