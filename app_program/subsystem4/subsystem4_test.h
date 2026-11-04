/**
 * @file    subsystem4_test.h
 * @brief   On-target tests of subsystem 4 (built only with make TEST=1).
 *
 * subsystem4_test() is also declared in app.h, where usermain() calls it;
 * this header exists so the test module has its own interface file
 * (Barr-C Rule 4.2.a).
 */

#ifndef SUBSYSTEM4_TEST_H
#define SUBSYSTEM4_TEST_H

#if APP_TEST
void subsystem4_test (void);
#endif

#endif /* SUBSYSTEM4_TEST_H */

/*** end of file ***/
