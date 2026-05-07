cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(my_dw3000_rx)

# Setze USE_ARDUINO global (0 oder 1)
set(USE_ARDUINO 0 CACHE STRING "Disable Arduino component")

# Preprocessor-Makro global verfügbar machen
# add_compile_definitions(USE_ARDUINO=${USE_ARDUINO})

#if(USE_ARDUINO)
#    # Arduino als Komponente hinzufügen
#    set(EXTRA_COMPONENT_DIRS components/arduino components/dw3000)
#else()
    set(EXTRA_COMPONENT_DIRS components/dw3000)
#endif()
