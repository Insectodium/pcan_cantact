#include "stm32f0xx_hal.h"
#include "pcan_timestamp.h"
#include "pcan_led.h"
#include "pcan_protocol.h"
#include "pcan_usb.h"

#if BOARD == ollie
#define OUTPUT_EN_5V_Pin 			GPIO_PIN_13
#define OUTPUT_EN_5V_GPIO_Port		GPIOC
#define OUTPUT_EN_3V3_Pin 			GPIO_PIN_14
#define OUTPUT_EN_3V3_GPIO_Port 	GPIOC
#define OUTPUT_EN_1V8_Pin 			GPIO_PIN_15
#define OUTPUT_EN_1V8_GPIO_Port 	GPIOC
#define SW1_Pin 					GPIO_PIN_2
#define SW1_GPIO_Port 				GPIOA
#define LED_5V_Pin 					GPIO_PIN_5
#define LED_5V_GPIO_Port 			GPIOA
#define LED_3V3_Pin 				GPIO_PIN_6
#define LED_3V3_GPIO_Port 			GPIOA
#define LED_1V8_Pin 				GPIO_PIN_7
#define LED_1V8_GPIO_Port 			GPIOA
#define VS_5V_Pin 					GPIO_PIN_6
#define VS_5V_GPIO_Port 			GPIOB
#define VS_3V3_Pin 					GPIO_PIN_5
#define VS_3V3_GPIO_Port 			GPIOB
#define VS_1V8_Pin 					GPIO_PIN_4
#define VS_1V8_GPIO_Port 			GPIOB
#endif

void HAL_MspInit( void )
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

#ifdef EXTERNAL_CLOCK
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;

  /* HSE = 16MHZ */
#if EXTERNAL_CLOCK == 16
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  /* HSE = 8MHZ */
#elif EXTERNAL_CLOCK == 8
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
#else
#error invalid HSE_VALUE
#endif
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;

  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig( &RCC_ClkInitStruct, FLASH_LATENCY_1 );

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;

  HAL_RCCEx_PeriphCLKConfig( &PeriphClkInit );
}
#else
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}
#endif

void SysTick_Handler( void )
{
  HAL_IncTick();
}

int main( void )
{
  HAL_Init();

  SystemClock_Config();


#if BOARD == ollie
GPIO_InitTypeDef GPIO_InitStruct;

__HAL_RCC_GPIOA_CLK_ENABLE();
__HAL_RCC_GPIOB_CLK_ENABLE();
__HAL_RCC_GPIOC_CLK_ENABLE();

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOC, OUTPUT_EN_5V_Pin|OUTPUT_EN_3V3_Pin|OUTPUT_EN_1V8_Pin, GPIO_PIN_RESET);

/*Configure GPIO pin Output Level */
HAL_GPIO_WritePin(GPIOA, LED_5V_Pin|LED_3V3_Pin|LED_1V8_Pin, GPIO_PIN_RESET);

/*Configure GPIO pins : OUTPUT_EN_5V_Pin OUTPUT_EN_3V3_Pin OUTPUT_EN_1V8_Pin */
GPIO_InitStruct.Pin = OUTPUT_EN_5V_Pin|OUTPUT_EN_3V3_Pin|OUTPUT_EN_1V8_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
//
//	/*Configure GPIO pin : SW1_Pin */
GPIO_InitStruct.Pin = SW1_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

/*Configure GPIO pins : LED_5V_Pin LED_3V3_Pin LED_1V8_Pin */
GPIO_InitStruct.Pin = LED_5V_Pin|LED_3V3_Pin|LED_1V8_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
GPIO_InitStruct.Pull = GPIO_NOPULL;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/*Configure GPIO pins : VS_5V_Pin VS_3V3_Pin VS_1V8_Pin */
GPIO_InitStruct.Pin = VS_5V_Pin|VS_3V3_Pin|VS_1V8_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_PULLUP;
HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
if(HAL_GPIO_ReadPin(VS_5V_GPIO_Port, VS_5V_Pin) == GPIO_PIN_RESET)
{
	HAL_GPIO_WritePin(LED_5V_GPIO_Port, LED_5V_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_EN_5V_GPIO_Port, OUTPUT_EN_5V_Pin, GPIO_PIN_SET);
}
else if(HAL_GPIO_ReadPin(VS_3V3_GPIO_Port, VS_3V3_Pin) == GPIO_PIN_RESET)
{
	HAL_GPIO_WritePin(LED_3V3_GPIO_Port, LED_3V3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_EN_3V3_GPIO_Port, OUTPUT_EN_3V3_Pin, GPIO_PIN_SET);
}
else if(HAL_GPIO_ReadPin(VS_1V8_GPIO_Port, VS_1V8_Pin) == GPIO_PIN_RESET)
{
	HAL_GPIO_WritePin(LED_1V8_GPIO_Port, LED_1V8_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(OUTPUT_EN_1V8_GPIO_Port, OUTPUT_EN_1V8_Pin, GPIO_PIN_SET);
}

#endif

  pcan_usb_init();
#if BOARD != ollie
  pcan_led_init();
#endif
  pcan_timestamp_init();
  pcan_protocol_init();
#if BOARD != ollie
  pcan_led_set_mode( LED_CH0_RX, LED_MODE_BLINK_SLOW, 0 );
  pcan_led_set_mode( LED_CH0_TX, LED_MODE_BLINK_SLOW, 0 );
#endif
  for(;;)
  {
    pcan_usb_poll();
#if BOARD != ollie
    pcan_led_poll();
#endif
    pcan_protocol_poll();
  }
}
