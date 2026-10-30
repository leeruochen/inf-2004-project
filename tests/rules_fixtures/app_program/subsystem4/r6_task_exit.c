/* Rule fixture (never compiled): R6 must fire on lines 2 and 3, not on 4. */
void f(void) { tk_ext_tsk(); }
void g(void) { tk_exd_tsk (); }
void h(void) { my_tk_ext_tsk(); /* tk_ext_tsk() in a comment */ }
