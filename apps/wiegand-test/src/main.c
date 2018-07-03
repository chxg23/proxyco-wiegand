#include <assert.h>
#include <string.h>

#include "sysinit/sysinit.h"
#include "os/os.h"
#include "bsp/bsp.h"
#include "console/console.h"
#include "hal/hal_gpio.h"
#include "wiegand/wiegand.h"

#ifdef ARCH_sim
#include "mcu/mcu_sim.h"
#endif

static volatile int g_task1_loops;

/* For LED toggling */
int g_led_pin;
int g_out_d0_pin;
int g_out_d1_pin;

/**
 * main
 *
 * The main task for the project. This function initializes packages,
 * and then blinks the BSP LED in a loop.
 *
 * @return int NOTE: this function should never return!
 */
int
main(int argc, char **argv)
{
  int state;
#ifdef ARCH_sim
  mcu_sim_parse_args(argc, argv);
#endif

  sysinit();

  g_led_pin = 18;
  hal_gpio_init_out(g_led_pin, 1);

  console_printf("blinky; pin=%d state=", g_led_pin);

  wiegand_init();

  while (1) {
    ++g_task1_loops;

    /* Wait one second */
    os_time_delay(OS_TICKS_PER_SEC);

    /* Toggle the LED */
    state = hal_gpio_toggle(g_led_pin);

    wiegand_toggle();

    console_write(state ? "0" : "1", 1);
  }
  assert(0);

  return 0;
}
