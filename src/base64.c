#include <stdint.h>
#include "base64.h"
#include "uart.h"

inline uint8_t base64_basis(uint8_t n) {
  if(n >= 0 && n <= 25) return 'A' + (n - 0);
  if(n >= 26 && n <= 51) return 'a' + (n - 26);
  if(n >= 52 && n <= 61) return '0' + (n - 52);
  if(n == 62) return '+';
  if(n == 63) return '/';
  return '?';
}

void base64_encode(uint8_t* data, uint16_t len) {
    uint16_t i;

    for (i = 0; i < len - 2; i += 3) {
      uart_writec(base64_basis((data[i] >> 2) & 0x3F));
      uart_writec(base64_basis(((data[i] & 0x3) << 4)     | ((int) (data[i + 1] & 0xF0) >> 4)));
      uart_writec(base64_basis(((data[i + 1] & 0xF) << 2) | ((int) (data[i + 2] & 0xC0) >> 6)));
      uart_writec(base64_basis(data[i + 2] & 0x3F));
    }
    if (i < len) {
      uart_writec(base64_basis((data[i] >> 2) & 0x3F));
      if (i == (len - 1)) {
        uart_writec(base64_basis(((data[i] & 0x3) << 4)));
        uart_writec('=');
      } else {
          uart_writec(base64_basis(((data[i] & 0x3) << 4) | ((int) (data[i + 1] & 0xF0) >> 4)));
          uart_writec(base64_basis(((data[i + 1] & 0xF) << 2)));
      }
      uart_writec('=');
    }
}
