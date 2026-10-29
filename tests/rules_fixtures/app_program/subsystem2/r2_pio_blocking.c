/* Rule fixture (never compiled): R2 must fire on lines 2 and 3 (blocking PIO FIFO waits). */
void f(void *p) { (void)pio_sm_get_blocking(p, 0); }
void g(void *p) { pio_sm_put_blocking(p, 0, 1); }
