idf_component_register(SRCS "test_simple_trx_board.cpp"
                       INCLUDE_DIRS "../main"
                       REQUIRES unity uwb_board)

if(UNIT_TEST)
    add_compile_definitions(UNIT_TEST)
endif()

