#pragma once

#include <stdlib.h>

void assert_failed(const char *file, int line, const char *expr);

#ifndef NDEBUG
#define ASSERT(x) \
  do { \
    if (!(x)) { \
      assert_failed(__FILE__, __LINE__, #x); \
    } \
  } while(0)
#else
#define ASSERT(x) ((void)0)
#endif

#define ASSERT_NOT_NULL(p) ASSERT((p) != NULL)

