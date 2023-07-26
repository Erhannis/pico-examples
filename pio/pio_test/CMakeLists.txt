add_executable(pio_test)

# by default the header is generated into the build dir
pico_generate_pio_header(pio_test ${CMAKE_CURRENT_LIST_DIR}/read_epc.pio)
pico_generate_pio_header(pio_test ${CMAKE_CURRENT_LIST_DIR}/write_fdic.pio)
# however, alternatively you can choose to generate it somewhere else (in this case in the source tree for check in)
#pico_generate_pio_header(pio_test ${CMAKE_CURRENT_LIST_DIR}/pio_test.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR})

target_sources(pio_test PRIVATE pio_test.c)

target_link_libraries(pio_test PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(pio_test)

# add url via pico_set_program_url
example_auto_set_url(pio_test)