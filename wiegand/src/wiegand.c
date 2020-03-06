/***
 *
 * Wiegand Library
 *
 * For sending Wiegand messages
 *
 */
#include <assert.h>
#include <hal/hal_gpio.h>
#include <hal/hal_timer.h>
#include <bsp/bsp.h>
#include <string.h>
#include <console/console.h>
#include "ringbuf/ringbuf.h"
#include "wiegand.h"
#include "wiegand_log.h"

/* Active-high or Active-low logic is configurable. */
#if MYNEWT_VAL(WIEGAND_ACTIVE_LOW)
#define ACTIVE    (0)
#define INACTIVE  (1)
#else
#define ACTIVE    (1)
#define INACTIVE  (0)
#endif

#if (MYNEWT_VAL(WIEGAND_CTRL_ENABLE_PIN) != WIEGAND_NO_GPIO)
#define WIEGAND_CRTL_ENABLE               (1)
#define WIEGAND_CTRL_ENABLE_OFF           (0)
#define WIEGAND_CTRL_ENABLE_ON            (1)
/* Number of ticks of the Wiegand IO timer before timing out the WIEGAND_CTRL_ENABLE_PIN. Set to double the
 * maximum number of bits in a Wiegand message to make sure that the longest possible message has time to
 * go out. */
#define WIEGAND_CTRL_ENABLE_TIMEOUT_TICKS (WIEGAND_MSG_MAX_LEN * 2)
#endif

/* Buffer of outgoing Wiegand Messages */
#define MSG_BUF_NUM (10)
#define MSG_BUF_LEN (MSG_BUF_NUM * sizeof(wiegand_msg_t))

/* Timer configuration. Note that each platform will require custom timer configuration! */

/* Timer frequency value. */
#define NRF_TIMER_FREQ    (400000)
/* Timer adjust offset, empirically determined. */
#define NRF_TIMER_ADJ     (4)

static uint8_t msg_buf[MSG_BUF_LEN] = {0};
static struct ringbuf msg_buf_rb;

static struct hal_timer g_wiegand_timer;

/* These values are all computed based on the platform-specific timer and the relevant syscfg values. */
static uint32_t g_timer_resolution = 0;       /* Actual timer resolution achieved. */
static uint32_t g_pulse_ticks = 0;            /* Timer ticks per pulse. */
static uint32_t g_period_ticks = 0;           /* Timer ticks per period between pulses. */
static uint32_t g_msg_wait_ticks = 0;         /* Minimum timer ticks between sending messages. */
static uint32_t g_msg_queue_wait_ticks = 0;   /* Wait time in ticks between checking message queue. */

void
wiegand_timer_cb(void *arg)
{
  static int bit_index = 0;
  static int pulse_pin;
  static int in_pulse = 0;
  static wiegand_msg_t msg = {0};
  struct ringbuf_iter it;
  uint8_t *entry = NULL;
#ifdef WIEGAND_CRTL_ENABLE
  static int wiegand_ctrl_enable_timeout = WIEGAND_CTRL_ENABLE_TIMEOUT_TICKS;

  /* This is a safety mechanism to catch us if get stuck somewhere, to make sure that we turn off
   * Wiegand control. We decrement it on every tick, if it runs out, we turn off Wiegand control. */

  if (wiegand_ctrl_enable_timeout > 0) {
    wiegand_ctrl_enable_timeout--;
  }

  if (wiegand_ctrl_enable_timeout <= 0) {
    hal_gpio_write(MYNEWT_VAL(WIEGAND_CTRL_ENABLE_PIN), WIEGAND_CTRL_ENABLE_OFF);
  }
#endif

  if (msg.bit_len == 0) {
    rb_iter_start(&msg_buf_rb, &it);

    entry = rb_iter_next(&msg_buf_rb, &it);
    /* No current message, but there may be one waiting. */
    if (entry != NULL) {
      /* Message ready to send */
      memcpy(&msg, entry, sizeof(msg));

     if (entry == msg_buf_rb.head && entry == msg_buf_rb.tail) {
       /* TODO: This doesn't seem right, but it doesn't seem like it's possible to empty a single item
        * out of the ringbuf with rb_flush_to. */
       rb_flush(&msg_buf_rb);
     } else {
      /* Step to next item, then flush to it. */
      entry = rb_iter_next(&msg_buf_rb, &it);
      if (entry == NULL) {
        WIEGAND_LOG(CRITICAL, "wiegand: rb_iter_next fell off end?!\n");
        return;
      } else {
        rb_flush_to(&msg_buf_rb, entry);
      }
     }

#ifdef WIEGAND_CRTL_ENABLE
      /* Turn on Wiegand control before trying to send! */
      hal_gpio_write(MYNEWT_VAL(WIEGAND_CTRL_ENABLE_PIN), WIEGAND_CTRL_ENABLE_ON);
      wiegand_ctrl_enable_timeout = WIEGAND_CTRL_ENABLE_TIMEOUT_TICKS;
#endif

      /* Will start sending below... */
    } else {
      /* No messages waiting. Sleep the timer for a while. */
      hal_timer_start(&g_wiegand_timer, g_msg_queue_wait_ticks);
#ifdef WIEGAND_CRTL_ENABLE
      /* Turn off Wiegand control between messages. */
      hal_gpio_write(MYNEWT_VAL(WIEGAND_CTRL_ENABLE_PIN), WIEGAND_CTRL_ENABLE_OFF);
#endif
      return;
    }
  }

  /* We're in the process of sending a Wiegand message. */
  if (in_pulse) {
    /* Pulse complete, pin goes back to active. */
    hal_gpio_write(pulse_pin, ACTIVE);

    in_pulse = 0;
    bit_index++;

    if (bit_index >= msg.bit_len) {
      /* Message complete, this was the last pulse. */
      bit_index = 0;
      msg.bit_len = 0;

      /* Message complete, sleep the timer. TODO: Should this be a much longer, specific wait? */
      hal_timer_start(&g_wiegand_timer, g_msg_wait_ticks);
#ifdef WIEGAND_CRTL_ENABLE
      /* Turn off Wiegand control between messages. */
      hal_gpio_write(MYNEWT_VAL(WIEGAND_CTRL_ENABLE_PIN), WIEGAND_CTRL_ENABLE_OFF);
#endif
      return;
    }

    hal_timer_start(&g_wiegand_timer, g_period_ticks);
    return;
  }

  /* Calculate the appropriate pin to pulse. */
  if ((msg.data[bit_index / 8] & (1 << (7 - bit_index % 8))) != 0) {
    pulse_pin = MYNEWT_VAL(WIEGAND_D0_PIN);
  } else {
    pulse_pin = MYNEWT_VAL(WIEGAND_D1_PIN);
  }

  /* Start pulse by setting pin inactive. */
  hal_gpio_write(pulse_pin, INACTIVE);
  in_pulse = 1;
  hal_timer_start(&g_wiegand_timer, g_pulse_ticks);

}

/**
 * Write wiegand message. Send message of *wiegand_data (of len len) and 
 * wiegand_bits. Messages are sent by placing them in a ringbuffer, which is
 * cleared by an interrupt-driven timer in order to avoid blocking the appliction.
 * 
 * wiegand_bits   Number of bits in the Wiegand message.
 * wiegand_data   Bytes that containt the Wiegand message bits.
 * len            Number of bytes in wiegand_data.
 * 
 * It is necessary to provide all of these pieces because the bit count doesn't
 * have to be, and often isn't, a multiple of 8.
 * 
 * Returns WIEGAND_WRITE_SUCCESS on success.
 */
wiegand_write_result_t
wiegand_write(uint32_t wiegand_bits, uint8_t *wiegand_data, uint8_t len)
{
  wiegand_msg_t msg;

  assert(wiegand_data != NULL);

  memset(&msg, 0, sizeof(wiegand_msg_t));

  if (wiegand_bits > WIEGAND_MSG_MAX_LEN) {
    WIEGAND_LOG(ERROR, "wiegand_write: Too many bits, %lu > %d!\n", wiegand_bits, WIEGAND_MSG_MAX_LEN);
    return WIEGAND_WRITE_MSG_TOO_BIG;
  }

  if (wiegand_bits / 8 > len) {
    WIEGAND_LOG(ERROR, "wiegand_write: Not enough bytes! Bits %lu, Bytes %d!\n", wiegand_bits, len);
    return WIEGAND_WRITE_INSUFFICENT_BITS;
  }

  WIEGAND_LOG(INFO, "wiegand_write: %lu bits\n", wiegand_bits);

  msg.bit_len = wiegand_bits;
  msg.len = len;
  memcpy(msg.data, wiegand_data, len);

  /* TODO: Check for ringbuffer overflow. Best to implement in ringbuf library. */
  rb_append(&msg_buf_rb, &msg, sizeof(msg));

  return WIEGAND_WRITE_SUCCESS;
}

/**
 * Initialize Wiegand library. Configures GPIO pins, wiegand message ringbuffer,
 * and timer-based interrupt for non-blocking high-frequency I/O. See syscfg.yml
 * for pin and timing configuration.
 * 
 * Portions of this are device specific, and the nRF52 is currently the only
 * supported device.
 */
void
wiegand_init(void)
{
  int rc = 0;

  rc = hal_gpio_init_out(MYNEWT_VAL(WIEGAND_D0_PIN), ACTIVE);
  assert(rc == 0);

  rc = hal_gpio_init_out(MYNEWT_VAL(WIEGAND_D1_PIN), ACTIVE);
  assert(rc == 0);

  WIEGAND_LOG(INFO, "wiegand_init: D0: %d, D1: %d. Ready\n",
      MYNEWT_VAL(WIEGAND_D0_PIN),
      MYNEWT_VAL(WIEGAND_D1_PIN));

#ifdef WIEGAND_CRTL_ENABLE
    rc = hal_gpio_init_out(MYNEWT_VAL(WIEGAND_CTRL_ENABLE_PIN), WIEGAND_CTRL_ENABLE_OFF);
    assert(rc == 0);

    WIEGAND_LOG(INFO, "wiegand_init: MYNEWT_VAL(WIEGAND_CTRL_ENABLE_PIN)=%d\n", MYNEWT_VAL(WIEGAND_CTRL_ENABLE_PIN));
#endif

  rc = rb_init(&msg_buf_rb, msg_buf, sizeof(msg_buf), sizeof(wiegand_msg_t));
  assert(rc == 0);

#if MYNEWT_VAL(BSP_NRF52) || MYNEWT_VAL(BSP_NRF52840)
  rc = hal_timer_config(MYNEWT_VAL(WIEGAND_NRF_TIMER), NRF_TIMER_FREQ);
  assert(rc == 0);

  g_timer_resolution = hal_timer_get_resolution(MYNEWT_VAL(WIEGAND_NRF_TIMER));

  g_pulse_ticks = (MYNEWT_VAL(WIEGAND_PULSE_LEN) * 1000) / g_timer_resolution - NRF_TIMER_ADJ;
  g_period_ticks = (MYNEWT_VAL(WIEGAND_PERIOD_LEN) * 1000) / g_timer_resolution - NRF_TIMER_ADJ;
  g_msg_wait_ticks = (MYNEWT_VAL(WIEGAND_MSG_WAIT_LEN) * 1000) / g_timer_resolution - NRF_TIMER_ADJ;
  g_msg_queue_wait_ticks = (MYNEWT_VAL(WIEGAND_MSG_QUEUE_WAIT_LEN) * 1000) / g_timer_resolution - NRF_TIMER_ADJ;

  WIEGAND_LOG(INFO, "wiegand_init: nRF Timer - Freq: %d Rez: %lu, pulse_ticks: %lu, period_ticks: %lu\n",
      NRF_TIMER_FREQ,
      g_timer_resolution,
      g_pulse_ticks,
      g_period_ticks
  );

  hal_timer_set_cb(MYNEWT_VAL(WIEGAND_NRF_TIMER), &g_wiegand_timer, wiegand_timer_cb, NULL);
  assert(rc == 0);

  rc = hal_timer_start(&g_wiegand_timer, 1);
  assert(rc == 0);
#endif
}
