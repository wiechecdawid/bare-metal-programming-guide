#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNUM(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
#define FREQ 16000000

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2];
};

// STM 32 in order to save power has GPIOs disabled by default. GPIO needs to be clocked with RCC in order to be enabled
// define RCC unit
struct rcc {
    volatile uint32_t CR, PLLCFGR, CFGR, CIR, AHB1RSTR, AHB2RSTR, AHB3RSTR, RESERVED0, APB1RSTR, APB2RSTR, RESERVED1[2], AHB1ENR, AHB2ENR, AHB3ENR,
        RESERVED2, APB1ENR, APB2ENR, RESERVED3[2], AHB1LPENR, AHB2LPENR, AHB3LPENR, RESERVED4, APB1LPENR, APB2LPENR, RESERVED5[2], BDCR, CSR,
        RESERVED6[2], SSCGR, PLLI2SCFGR;
};

struct systick {
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};

struct uart {
  volatile uint32_t SR, DR, BRR, CR1, CR2, CR3, GTPR;
};

#define UART1 ((struct uart *) 0x40011000)
#define UART2 ((struct uart *) 0x40004400)
#define UART3 ((struct uart *) 0x40004800)
#define SYSTICK ((struct systick *) 0xe000e010)
#define RCC ((struct rcc *) 0x40023800)
#define GPIO(bank)((struct gpio *) (0x40020000 + 0x400 * (bank)))

enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint16_t pin, uint8_t mode);
static inline void gpio_write(uint16_t pin, bool val);
static inline void spin(volatile uint32_t count);
static inline void systick_init(uint32_t ticks);
bool timer_expired(uint32_t *expirationTime, uint32_t period, uint32_t currentTime);
void delay(unsigned ms);
static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }
static inline void gpio_set_af(uint16_t pin, uint8_t afNumber);
static inline void uart_init(struct uart *uart, unsigned long baud);
static inline int uart_read_ready(struct uart *uart);
static inline uint8_t uart_read_byte(struct uart *uart);
static inline void uart_write_byte(struct uart *uart, uint8_t byte);
static inline void uart_write_buf(struct uart *uart, char *buf, size_t len);

int main(void) {
    uint16_t ledPin = PIN('A', 5);
    gpio_set_mode(ledPin, GPIO_MODE_OUTPUT);
    systick_init(16000000 / 1000);
    uint32_t timer, period = 500;
    uart_init(UART2, 115200);
    
    for (;;) {
        if (timer_expired(&timer, period, s_ticks)) {
          static bool on;
          gpio_write(ledPin, on);
          on = !on;
          uart_write_buf(UART2, "hi\r\n", 4);
        }
    }

    return 0;
}

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
    struct gpio *gpio = GPIO(PINBANK(pin));
    int pinNum = PINNUM(pin);
    RCC->AHB1ENR |= BIT(PINBANK(pin));
    gpio->MODER &= ~(3U << (pinNum * 2));
    gpio->MODER |= (mode & 3) << (pinNum * 2);
}

static inline void gpio_write(uint16_t pin, bool val){
    struct gpio *gpio = GPIO(PINBANK(pin));
    gpio->BSRR = (1u << PINNUM(pin)) << (val ? 0 : 16);
}

static inline void spin(volatile uint32_t count) {
  while (count--) (void) 0;
}

static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SYSTICK->LOAD = ticks - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable systick
  RCC->APB2ENR |= BIT(14);                   // Enable SYSCFG
}

// Implementation of Arduino delay() function (blocking)
void delay(unsigned ms) {
  uint32_t until = s_ticks + ms; // time in future we need to stop
  while (s_ticks < until) (void) 0; // Loop until then
}

// non - blocking way of measuring time
bool timer_expired(uint32_t *expirationTime, uint32_t period, uint32_t currentTime){
  if (currentTime + period < *expirationTime) *expirationTime = 0; // Time wrapped? Reset timer
  if (*expirationTime == 0) *expirationTime = currentTime + period; // First poll? Set expiration

  if (*expirationTime > currentTime) return false;  // Not expired yet, return
  
  *expirationTime = (currentTime - *expirationTime) > period ? currentTime + period : *expirationTime + period; // Next expiration time
  return true; // Expired, return true
}

static inline void gpio_set_af(uint16_t pin, uint8_t afNumber) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  int pinnumber = PINNUM(pin);

  gpio->AFR[pinnumber >> 3] &= ~(15UL << ((pinnumber & 7) * 4));
  gpio->AFR[pinnumber >> 3] |= ((uint32_t) afNumber) << ((pinnumber & 7) * 4);
}

static inline void uart_write_byte(struct uart *uart, uint8_t byte) {
  uart->DR = byte;
  while ((uart->SR & BIT(7)) == 0) spin(1);
}

static inline void uart_init(struct uart *uart, unsigned long baud) {
  uint8_t af = 7;
  uint16_t rx = 0, tx = 0;
 
  if (uart == UART1) RCC->APB2ENR |= BIT(4);
  if (uart == UART2) RCC->APB1ENR |= BIT(17);
  if (uart == UART3) RCC->APB1ENR |= BIT(18);

  if (uart == UART1) tx = PIN('B', 9), rx = PIN('B', 10);
  if (uart == UART2) tx = PIN('A', 2), rx = PIN('A', 3);
  if (uart == UART3) tx = PIN('C', 10), rx = PIN('C', 11);

  gpio_set_mode(tx, GPIO_MODE_AF);
  gpio_set_af(tx, af);
  
  gpio_set_mode(rx, GPIO_MODE_AF);
  gpio_set_af(rx, af);
  uart->CR1 = 0;
  uart->BRR = FREQ/baud;
  uart->CR1 |= BIT(13) | BIT(2) | BIT(3);
}

static inline int uart_read_ready(struct uart *uart) {
  return uart->SR & BIT(5);  // If RXNE bit is set, data is ready
}

static inline uint8_t uart_read_byte(struct uart *uart) {
  return (uint8_t) (uart->DR & 255);
}

static inline void uart_write_buf(struct uart *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
    extern long _sbss, _ebss, _sdata, _edata, _sidata;

    // Copy .data to RAM, zero .bss section
    for(long *src = &_sbss; src < &_ebss; src++) *src = 0;
    for(long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;

    main();
    for(;;) (void) 0; // Infinite loop in case 'main' returns
}

extern void _estack(void); // Defined in link.ld

__attribute__((section(".vectors"))) void (*tab[16+96])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler
};