/* Rule fixture (never compiled): nothing here may be reported. */
void f(void) { sleep_ms(1); /* RULE-ALLOW: pre-kernel */ }
/* never call sleep_ms( here */
/* multi-line comment
   printf("not code");
*/
// puts("also a comment");
