#ifndef WIEGAND_H_
#define WIEGAND_H_

#define WIEGAND_NO_GPIO 0xFF
#define WIEGAND_MSG_MAX_LEN 200
#define WIEGAND_MSG_MAX_BYTES (WIEGAND_MSG_MAX_LEN/8)

typedef enum {
  WIEGAND_WRITE_SUCCESS = 0,
  WIEGAND_WRITE_MSG_TOO_BIG,
  WIEGAND_WRITE_INSUFFICENT_BITS,
} wiegand_write_result_t;

typedef struct {
  uint8_t bit_len;
  uint8_t len;
  uint8_t data[WIEGAND_MSG_MAX_BYTES];
} wiegand_msg_t;

/* Stop Wiegand timer */
int wiegand_timer_stop(void);
/* Start Wiegand timer */
int wiegand_timer_start(void);

/**
 * Initialize Wiegand library. Configures GPIO pins, wiegand message ringbuffer,
 * and timer-based interrupt for non-blocking high-frequency I/O. See syscfg.yml
 * for pin and timing configuration.
 * 
 * Portions of this are device specific, and the nRF52 is currently the only
 * supported device.
 */
void wiegand_init(void);

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
 * @return WIEGAND_WRITE_SUCCESS on success.
 */
wiegand_write_result_t wiegand_write(uint32_t wiegand_bits, uint8_t *wiegand_data, uint8_t len);

#endif /* WIEGAND_H_ */
