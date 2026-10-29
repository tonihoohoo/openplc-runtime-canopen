// SPDX-License-Identifier: Apache-2.0
//
// check.h - a minimal test harness: CHECK records a failure and goes on; main() returns the
// failure count, so ctest reports the file.

#ifndef CANOPEN_TEST_CHECK_H
#define CANOPEN_TEST_CHECK_H

#include <cstdio>

namespace canopen_test
{
inline int &failures()
{
    static int n = 0;
    return n;
}
} // namespace canopen_test

#define CHECK(cond)                                                                                \
    do                                                                                             \
    {                                                                                              \
        if (!(cond))                                                                               \
        {                                                                                          \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond);          \
            ++canopen_test::failures();                                                            \
        }                                                                                          \
    } while (0)

#define TEST_RESULT()                                                                              \
    (canopen_test::failures() == 0                                                                 \
         ? (std::printf("all checks passed\n"), 0)                                                 \
         : (std::printf("%d check(s) failed\n", canopen_test::failures()), 1))

#endif // CANOPEN_TEST_CHECK_H
