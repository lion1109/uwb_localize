idf_component_register(
	#SRCS "anchor.cpp"
	SRCS "simple_trx_board.cpp"
	#SRCS "simple_rx_irq.cpp"
	#SRCS "simple_tx.cpp"
	INCLUDE_DIRS "."
        REQUIRES uwb_board
)

if(UNIT_TEST)
    add_compile_definitions(UNIT_TEST)
endif()
