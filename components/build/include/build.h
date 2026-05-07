#include "sdkconfig.h"
#include "debug.h"
#include "assert_custom.h"

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#ifndef BUILD_TYPE_DEBUG
#ifndef CONFIG_APP_BUILD_TYPE_DEBUG
#define BUILD_TYPE_DEBUG CONFIG_APP_BUILD_TYPE_DEBUG
#pragma message(set default BUILD_TYPE_DEBUG = y)
#define BUILD_TYPE_DEBUG CONFIG_APP_BUILD_TYPE
#else
#define BUILD_TYPE_DEBUG CONFIG_APP_BUILD_TYPE_DEBUG
#endif
#endif


#pragma message("DEBUG_LEVEL = " STR(DEBUG_LEVEL))
#pragma message("BUILD_TYPE_DEBUG = " STR(BUILD_TYPE_DEBUG))


/*
#define TAG "MAIN"

void app_main(void)
{
    LOGI(TAG, "System start");

    int *ptr = NULL;
    ASSERT_NOT_NULL(ptr);

    DEBUG_ONLY(
        int counter = 0;
        counter++;
        LOGD(TAG, "Counter: %d", counter);
    )

    LOGE(TAG, "Example error");
}

-------- CMakeLists.txt (snippet) --------
# Debug build
# idf.py build -DCMAKE_BUILD_TYPE=Debug

# Release build
# idf.py build -DCMAKE_BUILD_TYPE=Release

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(DEBUG_LEVEL=DEBUG_DEBUG)
else()
    add_compile_definitions(DEBUG_LEVEL=DEBUG_NONE NDEBUG)
endif()
*/

