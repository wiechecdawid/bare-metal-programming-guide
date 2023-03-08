// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
    for(;;) (void) 0;
}

extern void _estack(void); // Defined in link.ld

__attribute__((section(".vectors"))) void (*tab[16+96])(void) = {
    _estack, _reset
};