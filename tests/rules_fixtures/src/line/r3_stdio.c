/* Rule fixture (never compiled): line 2 R3a, line 3 allowed (snprintf), line 4 R3b. */
void f(void) { printf("x"); }
void g(char *b) { snprintf(b, 4, "x"); }
void h(void) { tm_printf("x"); }
