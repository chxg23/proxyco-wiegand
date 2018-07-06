/***
 *
 * Wiegand Library
 *
 * For sending Wiegand messages
 *
 */
#include <assert.h>
#include <hal/hal_gpio.h>
#include <bsp/bsp.h>
#include "console/console.h"
#include "wiegand/wiegand_log.h"

int g_d0_pin;
int g_d1_pin;


/* Initialize wiegand output, using pins d0_pin and d1_pin. The  */
void
wiegand_init(void)
{
  int rc = 0;

  g_d0_pin = MYNEWT_VAL(WIEGAND_D0_PIN);
  g_d1_pin = MYNEWT_VAL(WIEGAND_D1_PIN);

  rc = hal_gpio_init_out(g_d0_pin, HAL_GPIO_PULL_NONE);
  assert(rc == 0);

  rc = hal_gpio_init_out(g_d1_pin, HAL_GPIO_PULL_NONE);
  assert(rc == 0);

  console_printf("wiegand_init: D0: %d, D1: %d. Ready\n",
      g_d0_pin,
      g_d1_pin);
}

void
wiegand_write(uint32_t wiegand_bits, uint8_t *wiegand_data, uint8_t len)
{
  int pin_to_pulse;

  for (int i = 0; i < wiegand_bits; i++)
  {
    if ( (wiegand_data[i / 8] & (1 << (i % 8) )) != 0 )
    {
      pin_to_pulse = g_d1_pin;
    }
    else
    {
      pin_to_pulse = g_d0_pin;
    }

    hal_gpio_write(pin_to_pulse, 1);
    os_cputime_delay_usecs(MYNEWT_VAL(WIEGAND_PULSE_LEN));
    hal_gpio_write(pin_to_pulse, 0);
    os_cputime_delay_usecs(MYNEWT_VAL(WIEGAND_PERIOD_LEN));
  }
}
