#include "mcu.h"

static inline void systick_init(uint32_t ticks);
static volatile uint32_t s_ticks;
void SysTick_Handler(void) { s_ticks++; }

int main(void) {
    uint16_t ledPin = PIN('A', 5);
    gpio_set_mode(ledPin, GPIO_MODE_OUTPUT);
    systick_init(FREQ / 1000);
    uint32_t timer = 0, period = 500;
    uart_init(UART2, 115200);
    
    for (;;) {
        if (timer_expired(&timer, period, s_ticks)) {
          static bool on;
          gpio_write(ledPin, on);
          on = !on;
          if (on) uart_write_buf(UART2, "hi from uart\r\n", 14);
          if(!on) printf("hi from printf\r\n");
        }
    }

    return 0;
}

static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SYSTICK->LOAD = ticks - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable systick
  RCC->APB2ENR |= BIT(14);                   // Enable SYSCFG
}