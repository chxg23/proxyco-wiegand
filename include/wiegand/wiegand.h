#define WIEGAND_NO_GPIO 0xFF

void wiegand_init(void);
void wiegand_write(uint32_t wiegand_bits, uint8_t *wiegand_data, uint8_t len);

void wiegand_toggle(void);

#define WIEGAND_MSG_MAX_LEN 200
#define WIEGAND_MSG_MAX_BYTES (WIEGAND_MSG_MAX_LEN/8)

struct wiegand_msg {
  uint8_t bit_len;
  uint8_t bodys[WIEGAND_MSG_MAX_BYTES];
};