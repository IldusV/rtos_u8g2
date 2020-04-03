/*
  u8x8cb.c

  STM32L031

*/
#include "stm32f0xx_hal.h"
//#include "stm32l031xx.h"
#include "delay.h"
#include "u8x8.h"

extern I2C_HandleTypeDef hi2c1;

#define SSD1306_I2C_ADDRESS 0x78
#define  STM32_HAL_I2C_HANDLER  hi2c1
#define  STM32_HAL_I2C_TIMEOUT  2000

uint8_t psoc_gpio_and_delay_cb(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  switch(msg)
  {
    case U8X8_MSG_GPIO_AND_DELAY_INIT:
      /* only support for software I2C*/

      break;
    case U8X8_MSG_DELAY_NANO:
      /* not required for SW I2C */
    {
    volatile uint32_t i;
    for (i = 1; i <= arg_int*10; i++);
    }
      break;

    case U8X8_MSG_DELAY_10MICRO:
      /* not used at the moment */
      break;

    case U8X8_MSG_DELAY_100NANO:
      /* not used at the moment */
      break;

    case U8X8_MSG_DELAY_MILLI:
      //HAL_Delay(arg_int);
      //osDelay(arg_int);
      break;
    case U8X8_MSG_DELAY_I2C:
      /* arg_int is 1 or 4: 100KHz (5us) or 400KHz (1.25us) */
      //delay_micro_seconds(arg_int<=2?5:1);
      break;

    case U8X8_MSG_GPIO_I2C_CLOCK:

      break;
/*
    case U8X8_MSG_GPIO_MENU_SELECT:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_SELECT_PORT, KEY_SELECT_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_NEXT:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_NEXT_PORT, KEY_NEXT_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_PREV:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_PREV_PORT, KEY_PREV_PIN));
      break;
    case U8X8_MSG_GPIO_MENU_HOME:
      u8x8_SetGPIOResult(u8x8, Chip_GPIO_GetPinState(LPC_GPIO, KEY_HOME_PORT, KEY_HOME_PIN));
      break;
*/
    default:
      u8x8_SetGPIOResult(u8x8, 1);
      break;
  }
  return 1;
}

uint8_t u8x8_byte_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  if(msg == U8X8_MSG_BYTE_SEND)
  HAL_I2C_Mem_Write(&STM32_HAL_I2C_HANDLER, SSD1306_I2C_ADDRESS, (arg_int)?0x40:0, 1, (uint8_t *)arg_ptr, arg_int, STM32_HAL_I2C_TIMEOUT);
}

uint8_t u8x8_byte_stm32hal_hw_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  static uint8_t buffer[32];    /* u8g2/u8x8 will never send more than 32 bytes between START_TRANSFER and END_TRANSFER */
  static uint8_t buf_idx;
  uint8_t *data;
  uint8_t control = 0;

  switch(msg)
  {
    case U8X8_MSG_BYTE_SEND:
    {
        data = (uint8_t *)arg_ptr;
        while( arg_int > 0 )
        {
      buffer[buf_idx++] = *data;
      data++;
      arg_int--;
        }
    }
      break;
    case U8X8_MSG_BYTE_INIT:
      break;
    case U8X8_MSG_BYTE_SET_DC:
      break;
    case U8X8_MSG_BYTE_START_TRANSFER:
    {
      buf_idx = 0;
    }
    break;
    case U8X8_MSG_BYTE_END_TRANSFER:
    {
      HAL_I2C_Master_Transmit(&STM32_HAL_I2C_HANDLER, SSD1306_I2C_ADDRESS, &buffer[0], buf_idx, STM32_HAL_I2C_TIMEOUT);
    }
      break;
    default:
      return 0;
  }
  return 1;
}