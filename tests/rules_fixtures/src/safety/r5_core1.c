/* Rule fixture (never compiled): R5 must fire. */
void f(void (*fn)(void)) { multicore_launch_core1(fn); }
