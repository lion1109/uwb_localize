#include <stdlib.h>
#include <stdio.h>

void assert_failed(const char *file, int line, const char *expr) {
    printf("ASSERT FAILED: %s:%d: %s\n", file, line, expr);
    fflush(stdout);
    abort(); // or esp_restart();
}

