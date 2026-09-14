#pragma once

#ifdef assert
#undef assert
#endif

void __handle_fail(const char* message, const char* file_name, int line);

#define assert_fail_msg(message) __handle_fail(message, __FILE__, __LINE__)
#define assert_fail() assert_fail_msg("")
#define assert_msg(condition, message) \
    if (!(condition))                     \
    assert_fail_msg(message)
#define assert(condition) assert_msg(condition, "")
#define not_implemented() assert_fail_msg("function not implemented")

