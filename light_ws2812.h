#ifndef LIGHT_WS2812_H_
#define LIGHT_WS2812_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "ws2812_config.h"

#define CONCAT(a, b)       a ## b
#define CONCAT_EXP(a, b)   CONCAT(a, b)

#define ws2812_PORTREG  CONCAT_EXP(PORT, ws2812_port)
#define ws2812_DDRREG   CONCAT_EXP(DDR,  ws2812_port)

void ws2812_sendarray_mask(uint8_t *array, uint16_t length, uint8_t pinmask);

#endif /* LIGHT_WS2812_H_ */
