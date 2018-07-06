#define WIEGAND_NO_GPIO 0xFF

void wiegand_init(void);
void wiegand_write(uint32_t wiegand_bits, uint8_t *wiegand_data, uint8_t len);

void wiegand_toggle(void);
