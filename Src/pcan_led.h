#pragma once
#include <stdint.h>
#if BOARD != ollie
enum e_pcan_led
{
  LED_CH0_TX,
  LED_CH0_RX,
  LED_STAT,

  LED_TOTAL,
};

enum e_pcan_led_mode
{
  LED_MODE_NONE,
  LED_MODE_ON,
  LED_MODE_OFF,
  LED_MODE_BLINK_FAST,
  LED_MODE_BLINK_SLOW,
};

void pcan_led_init( void );
void pcan_led_set_mode( int led, int mode, uint16_t arg );
void pcan_led_poll( void );
#endif
