/***
 *
 * Wiegand Library
 *
 * For sending Wiegand messages
 *
 */
#include <assert.h>
#include "hal/hal_gpio.h"
#include "hal/hal_timer.h"
#include <bsp/bsp.h>
#include <ringbuf/ringbuf.h>
#include "console/console.h"
#include "wiegand/wiegand.h"
#include "wiegand/wiegand_log.h"

/* Debug LED */
int g_led1_pin;

/* GPIO Pins */
int g_d0_pin;
int g_d1_pin;

#define TASK1_TIMER_NUM     (1)
#define TASK1_TIMER_FREQ    (100000)

struct hal_timer g_task1_timer;
uint32_t g_task1_loops;

#define WIEGAND_MSG_BUF_COUNT (10)
#define WIEGAND_MSG_BUF_SIZE (sizeof(struct wiegand_msg) * WIEGAND_MSG_BUF_COUNT)

static uint8_t wiegand_out_buf[WIEGAND_MSG_BUF_SIZE] = {0};
static struct ringbuf wiegand_out_rb;

void
wiegand_write_cb(void *arg)
{
  static uint32_t count = 0;

  count++;

  if (count > 1000)
  {
    /* Toggle the LED */
    hal_gpio_toggle(g_led1_pin);
    count = 0;
  }

  hal_timer_start(&g_task1_timer, 10);
}

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

  hal_gpio_write(g_d0_pin, 1);
  hal_gpio_write(g_d1_pin, 1);

  console_printf("wiegand_init: D0: %d, D1: %d. Ready\n",
      g_d0_pin,
      g_d1_pin);

  g_led1_pin = LED_BLINK_PIN;
  hal_gpio_init_out(g_led1_pin, 1);

  rc = rb_init(&wiegand_out_rb, wiegand_out_buf, sizeof(wiegand_out_buf), WIEGAND_MSG_BUF_COUNT);
  assert(rc == 0);

#ifdef NRF52
  /* Configure hal_timer to handle sending out pulses. */
  rc = hal_timer_config(TASK1_TIMER_NUM, TASK1_TIMER_FREQ);
  assert(rc == 0);

  console_printf("wiegand_init: Freq: %d Rez: %lu\n",
    TASK1_TIMER_FREQ,
    hal_timer_get_resolution(TASK1_TIMER_NUM)
  );

  hal_timer_set_cb(TASK1_TIMER_NUM, &g_task1_timer, wiegand_write_cb, NULL);
  assert(rc == 0);

//   rc = hal_timer_start(&g_task1_timer, TASK1_TIMER_FREQ);
//   assert(rc == 0);
#endif
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

    hal_gpio_write(pin_to_pulse, 0);
    os_cputime_delay_usecs(MYNEWT_VAL(WIEGAND_PULSE_LEN));
    hal_gpio_write(pin_to_pulse, 1);
    os_cputime_delay_usecs(MYNEWT_VAL(WIEGAND_PERIOD_LEN));
  }
}
