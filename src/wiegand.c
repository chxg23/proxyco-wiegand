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
#include <ringbuf/ringbuf.h>
#include <string.h>
#include <console/console.h>
#include "wiegand/wiegand.h"
#include "wiegand/wiegand_log.h"

#define WIEGAND_BUFFER (10)

typedef struct jerRingbufferType {
  wiegand_msg_t buffer[WIEGAND_BUFFER];
  volatile uint32_t head;		// Index of ringbuffer head.
  volatile uint32_t tail;		// Index of ringbuffer tail.
} jerRingbufferType, *jerRingbufferPtr;

// Initialize jerRingbuffer at ringbufferPtr
void jerRingbufferInit(jerRingbufferPtr ringbufferPtr);

// Try to enqueue hidMsgPtr into ringbufferPtr. True on success.
bool jerRingbufferEnqueue(jerRingbufferPtr ringbufferPtr, wiegand_msg_t *wiegand_msg);

// Try to dequeue hidMsgPtr from ringbufferPtr. True on success.
bool jerRingbufferDequeue(jerRingbufferPtr ringbufferPtr, wiegand_msg_t *wiegand_msg);

void jerRingbufferInit
(
    jerRingbufferPtr ringbufferPtr
)
{
  memset(ringbufferPtr, 0, sizeof(jerRingbufferType));

  ringbufferPtr->head = 0;
  ringbufferPtr->tail = 0;
}

bool jerRingbufferEnqueue
(
    jerRingbufferPtr ringbufferPtr,
    wiegand_msg_t *msg
)
{
  uint32_t nextHidMsgFIFOHead = (ringbufferPtr->head + 1) % WIEGAND_BUFFER;

  if (nextHidMsgFIFOHead != ringbufferPtr->tail) {
    memcpy(ringbufferPtr->buffer[ringbufferPtr->head].data, msg, sizeof(wiegand_msg_t));
    ringbufferPtr->head = nextHidMsgFIFOHead;
    return true;
  } else {
    return false;
  }
}

bool jerRingbufferDequeue
(
    jerRingbufferPtr ringbufferPtr,
    wiegand_msg_t *msg
)
{
  if (ringbufferPtr->head != ringbufferPtr->tail) {
    memcpy(msg, ringbufferPtr->buffer[ringbufferPtr->tail].data, sizeof(wiegand_msg_t));
    ringbufferPtr->tail = (ringbufferPtr->tail + 1) %
        WIEGAND_BUFFER; // Increment readPos, loop if needed.

    return true;
  } else {
    return false;
  }
}

/* Debug LED */
int g_led1_pin;

/* GPIO Pins */
int g_d0_pin;
int g_d1_pin;

struct hal_timer g_wiegand_timer;
uint32_t g_task1_loops;

uint32_t g_timer_resolution = 0;  /* Actual timer resolution achieved. */
uint32_t g_pulse_ticks = 0;       /* Timer ticks per pulse. */
uint32_t g_period_ticks = 0;      /* Timer ticks per period between pulses. */
uint32_t g_msg_wait_ticks = 0;    /* Minimum timer ticks between sending messages. */

static jerRingbufferType wiegand_buffer = {0};

void
wiegand_timer_cb(void *arg)
{
  static int bit_index = 0;
  static int pulse_pin;
  static int in_pulse = 0;
  static wiegand_msg_t msg = {0};

  if (msg.bit_len == 0) {
    /* No current message, but there may be one waiting. */
    if (jerRingbufferDequeue(&wiegand_buffer, &msg)) {
      /* Message ready to send, will start sending below... */
    } else {
      /* No messages waiting. Sleep the timer for a while. */
      hal_timer_start(&g_wiegand_timer, 2000);
      return;
    }
  }

  /* We're in the process of sending a Wiegand message. */

  if (in_pulse) {
    hal_gpio_write(pulse_pin, 1);
    in_pulse = 0;
    bit_index++;

    if (bit_index > msg.bit_len) {
      /* Message complete, this was the last pulse. */
      bit_index = 0;
      msg.bit_len = 0;

      /* Message complete, sleep the timer. TODO: Should this be a much longer, specific wait? */
      hal_timer_start(&g_wiegand_timer, 20000);
      return;
    }

    hal_timer_start(&g_wiegand_timer, g_period_ticks);
    return;
  }

  if ((msg.data[bit_index / 8] & (1 << (bit_index % 8))) != 0) {
    pulse_pin = g_d1_pin;
  } else {
    pulse_pin = g_d0_pin;
  }

  hal_gpio_write(pulse_pin, 0);
  in_pulse = 1;
  hal_timer_start(&g_wiegand_timer, g_pulse_ticks);

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

  jerRingbufferInit(&wiegand_buffer);

  /* Each platform will require timer configuration. */
#ifdef NRF52
  /* Timer frequency value. */
#define NRF_TIMER_FREQ    (400000)
  /* Timer adjust offset, empirically determined. */
#define NRF_TIMER_ADJ     (4)

  rc = hal_timer_config(MYNEWT_VAL(WIEGAND_NRF_TIMER), NRF_TIMER_FREQ);
  assert(rc == 0);

  g_timer_resolution = hal_timer_get_resolution(MYNEWT_VAL(WIEGAND_NRF_TIMER));

  g_pulse_ticks = (MYNEWT_VAL(WIEGAND_PULSE_LEN) * 1000) / g_timer_resolution - NRF_TIMER_ADJ;
  g_period_ticks = (MYNEWT_VAL(WIEGAND_PERIOD_LEN) * 1000) / g_timer_resolution - NRF_TIMER_ADJ;
  g_msg_wait_ticks = (MYNEWT_VAL(WIEGAND_MSG_WAIT_LEN) * 1000) / g_timer_resolution - NRF_TIMER_ADJ;

  console_printf("wiegand_init: nRF Timer - Freq: %d Rez: %lu, pulse_ticks: %lu, period_ticks: %lu\n",
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

void
wiegand_write(uint32_t wiegand_bits, uint8_t *wiegand_data, uint8_t len)
{
  assert(wiegand_data != NULL);

  if (wiegand_bits > WIEGAND_MSG_MAX_LEN) {
    console_printf("wiegand_write: Too many bits, %lu > %d!\n", wiegand_bits, WIEGAND_MSG_MAX_LEN);
    return;
  }

  if (wiegand_bits / 8 > len) {
    console_printf("wiegand_write: Not enough bytes! Bits %lu, Bytes %d!\n", wiegand_bits, len);
    return;
  }

  console_printf("wiegand_write: %lu bits\n", wiegand_bits);

  wiegand_msg_t msg = {0};

  msg.bit_len = wiegand_bits;
  msg.len = len;
  memcpy(msg.data, wiegand_data, len);

  jerRingbufferEnqueue(&wiegand_buffer, &msg);
}
