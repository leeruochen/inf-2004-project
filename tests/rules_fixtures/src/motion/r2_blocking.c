/* Rule fixture (never compiled): R2 must fire on both lines (marker only works in src/app/main.c). */
void f(void) { sleep_ms(5); }
void g(void) { sleep_ms(5); /* RULE-ALLOW: pre-kernel */ }
