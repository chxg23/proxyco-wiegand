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

#if MYNEWT_VAL(BSP_NRF52) || MYNEWT_VAL(BSP_NRF52840)
#include <mcu/nrf52_hal.h>
#elif MYNEWT_VAL(BSP_NRF5340)
  #include <mcu/nrf5340_hal.h>
#else
  #error "Platform not supported"
#endif

#include "ringbuf/ringbuf.h"
#include "wiegand.h"
#include "wiegand_log.h"
#include "wiegand_timer.h"

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
#define NRF_TIMER_FREQ    (16000000)
/* Timer adjust offset, empirically determined. 
   TODO: determine why this is needed (CT-3389) */
#define NRF_TIMER_ADJ     (4)

/* Find minimum bytes to represent number of bits */
#define WIEGAND_BITS_TO_BYTES(bits)      (((bits) >> 3) + (((bits) & 0x7) != 0))

static uint8_t msg_buf[MSG_BUF_LEN] = {0};
static struct ringbuf msg_buf_rb;
static struct os_sem g_wiegand_sem;
static uint8_t g_wiegand_busy;

static inline void
wiegand_timer_cb(void *arg)
{
  static uint32_t sr;
  __HAL_DISABLE_INTERRUPTS(sr);
  static int bit_index = 0;
  static int pulse_pin;
  static wiegand_msg_t msg = {0};
  struct ringbuf_iter it;
  uint8_t *entry = NULL;
  os_error_t err; 
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
        __HAL_ENABLE_INTERRUPTS(sr);
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
      wiegand_timer_relative(MYNEWT_VAL(WIEGAND_MSG_QUEUE_WAIT_LEN));
#ifdef WIEGAND_CRTL_ENABLE
      /* Turn off Wiegand control between messages. */
      hal_gpio_write(MYNEWT_VAL(WIEGAND_CTRL_ENABLE_PIN), WIEGAND_CTRL_ENABLE_OFF);
#endif
      __HAL_ENABLE_INTERRUPTS(sr);
      return;
    }


  }

  /* Calculate the appropriate pin to pulse. */
  if ((msg.data[bit_index / 8] & (1 << (7 - bit_index % 8))) != 0) {
    pulse_pin = MYNEWT_VAL(WIEGAND_D0_PIN);
  } else {
    pulse_pin = MYNEWT_VAL(WIEGAND_D1_PIN);
  }
   
  /* Start pulse by setting pin inactive. */
  hal_gpio_write(pulse_pin, INACTIVE);
  /* Block for WIEGAND_PULSE_LEN uS (minus timer adjustment offset) */
  wiegand_timer_delay_usecs(MYNEWT_VAL(WIEGAND_PULSE_LEN) - NRF_TIMER_ADJ);
  /* Pulse complete, pin goes back to active. */
  hal_gpio_write(pulse_pin, ACTIVE);
#if MYNEWT_VAL(WIEGAND_PULSE_PERIOD_BLOCK)
  /* Block for WIEGAND_PERIOD_LEN uS (minus timer adjustment offset) */
  wiegand_timer_delay_usecs(MYNEWT_VAL(WIEGAND_PERIOD_LEN) - NRF_TIMER_ADJ);
#endif

  bit_index++;

  if (bit_index >= msg.bit_len) {
      wiegand_timer_stop();
      err = os_sem_release(&g_wiegand_sem);
      assert(err == OS_OK);
      g_wiegand_busy = 0;
      /* Message complete, this was the last pulse. */
      bit_index = 0;
      memset(&msg, 0, sizeof(msg));
#ifdef WIEGAND_CRTL_ENABLE
      /* Turn off Wiegand control between messages. */
      hal_gpio_write(MYNEWT_VAL(WIEGAND_CTRL_ENABLE_PIN), WIEGAND_CTRL_ENABLE_OFF);
#endif
      wiegand_timer_relative(MYNEWT_VAL(WIEGAND_MSG_WAIT_LEN));
     __HAL_ENABLE_INTERRUPTS(sr);
      return;
  }
  
  /* Reschedule the next bit transmission as soon as possible */
#if MYNEWT_VAL(WIEGAND_PULSE_PERIOD_BLOCK)
  wiegand_timer_relative(0);
#else
  wiegand_timer_relative(MYNEWT_VAL(WIEGAND_PULSE_LEN) - NRF_TIMER_ADJ);
#endif
  __HAL_ENABLE_INTERRUPTS(sr);
  
}

/**
 * Write wiegand message. Send message of wiegand_data (of length len and 
 * wiegand_bits). Messages are sent by placing them in a ringbuffer, which is
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
  uint8_t len_bytes;
  os_error_t err;
  int rc;

  assert(wiegand_data != NULL);

  if (g_wiegand_busy) {
    WIEGAND_LOG(ERROR, "wiegand_write: failed, busy\n");
    return WIEGAND_BUSY;
  }

  g_wiegand_busy = 1;

  memset(&msg, 0, sizeof(wiegand_msg_t));

  if (wiegand_bits > WIEGAND_MSG_MAX_LEN) {
    WIEGAND_LOG(ERROR, "wiegand_write: Too many bits, %lu > %d!\n", wiegand_bits, WIEGAND_MSG_MAX_LEN);
    return WIEGAND_WRITE_MSG_TOO_BIG;
  }

  if (wiegand_bits / 8 > len) {
    WIEGAND_LOG(ERROR, "wiegand_write: Not enough bytes! Bits %lu, Bytes %d!\n", wiegand_bits, len);
    return WIEGAND_WRITE_INSUFFICENT_BITS;
  }

  /* Get appropriate number of bytes to copy */
  len_bytes = WIEGAND_BITS_TO_BYTES(wiegand_bits);

  WIEGAND_LOG(INFO, "wiegand_write: %lu bits, %d bytes\n", wiegand_bits, len_bytes);

  msg.bit_len = wiegand_bits;
  msg.len = len_bytes;
  memcpy(msg.data, wiegand_data, len_bytes);

  rc = wiegand_timer_start();
  if (rc) {
    return WIEGAND_TIMER_FAILURE;
  }

  /* TODO: Check for ringbuffer overflow. Best to implement in ringbuf library. */
  rb_append(&msg_buf_rb, &msg, sizeof(msg));

  err = os_sem_pend(&g_wiegand_sem, MYNEWT_VAL(WIEGAND_SEM_TMO_TICKS));
  if (err == OS_TIMEOUT) {
    return WIEGAND_WRITE_FAILURE_TMO;
  }

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

  rc = wiegand_timer_init(NRF_TIMER_FREQ, wiegand_timer_cb);
  assert(rc == 0);

  rc = os_sem_init(&g_wiegand_sem, 0);
  assert(rc == 0);
}
