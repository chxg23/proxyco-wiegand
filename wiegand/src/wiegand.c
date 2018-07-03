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

/* Pkg init function */
void
wiegand_init(void)
{
  int rc = 0;

  rc = hal_gpio_init_out(MYNEWT_VAL(WIEGAND_D0_PIN), HAL_GPIO_MODE_OUT);
  assert(rc == 0);

  rc = hal_gpio_init_out(MYNEWT_VAL(WIEGAND_D1_PIN), HAL_GPIO_MODE_OUT);
  assert(rc == 0);

  console_printf("wiegand_init: D0: %d, D1: %d. Ready\n",
      MYNEWT_VAL(WIEGAND_D0_PIN),
      MYNEWT_VAL(WIEGAND_D1_PIN));
}

void
wiegand_toggle(void)
{
  hal_gpio_toggle(MYNEWT_VAL(WIEGAND_D0_PIN));
  hal_gpio_toggle(MYNEWT_VAL(WIEGAND_D1_PIN));
}
