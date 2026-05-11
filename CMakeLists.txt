cmake_minimum_required(VERSION 3.16)

if(UNIT_TEST)
    list(APPEND EXTRA_COMPONENT_DIRS test)
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my_dw3000_rx)

set(EXTRA_COMPONENT_DIRS components/dw3000)

#if(UNIT_TEST)
#    set(TEST_COMPONENTS "test")
#endif()
