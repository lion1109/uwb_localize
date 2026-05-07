idf_component_register(
	#SRCS "anchor.cpp"
	SRCS "simple_trx_board.cpp"
	#SRCS "simple_rx_irq.cpp"
	#SRCS "simple_tx.cpp"
	INCLUDE_DIRS "."
        REQUIRES uwb_board
)

# message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
# message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")

# target_compile_definitions(${COMPONENT_LIB} PRIVATE USE_ARDUINO=${USE_ARDUINO})
# target_compile_definitions(${COMPONENT_LIB} PRIVATE USE_ARDUINO=1)
