// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include <inttypes.h>

// #define GPIOA_BASE_ADDRESS 0x40020000U
// #define MODER_OFFSET 0X0U
// #define GPIOA_MODER_ADDR (GPIOA_BASE_ADDRESS + MODER_OFFSET)
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2]
};

// #define GPIOA ((struct gpio *) 0x40020000)
#define GPIO(bank) ((struct gpio *)(0x40020000 + 0x400 * (bank)))

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint8_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  uint8_t pinNo = PINNO(pin);
  gpio->MODER &= ~(3U << (pinNo * 2));
  gpio->MODER |= (mode & 3) << (pinNo * 2);
}

int main(void) {
  uint16_t pin = PIN('A', 3);
  gpio_set_mode(pin, GPIO_MODE_OUTPUT);
  return 0;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *src = &_sbss; src < &_ebss; src++) *src = 0;
  for (long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 91 STM32-specific handlers
__attribute__((section(".vectors"))) void (*tab[16 + 91])(void) = {_estack,
                                                                   _reset};
