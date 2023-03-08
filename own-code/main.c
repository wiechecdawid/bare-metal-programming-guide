int main(void) {
    return 0; // Do nothing yet
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
    _estack, _reset
};

// build firmware:
// arm-none-eabi-gcc -T link.ld -nostdlib main.o -o firmware.elf