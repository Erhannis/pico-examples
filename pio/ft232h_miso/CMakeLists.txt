add_executable(ft232h_miso)

#pico_set_binary_type(ft232h_miso copy_to_ram)

pico_generate_pio_header(ft232h_miso ${CMAKE_CURRENT_LIST_DIR}/am_broadcast.pio)
pico_generate_pio_header(ft232h_miso ${CMAKE_CURRENT_LIST_DIR}/freq_gen.pio)
pico_generate_pio_header(ft232h_miso ${CMAKE_CURRENT_LIST_DIR}/count2ftdi.pio)
pico_generate_pio_header(ft232h_miso ${CMAKE_CURRENT_LIST_DIR}/write2ftdi.pio)

target_sources(ft232h_miso PRIVATE ft232h_miso.c)

target_link_libraries(ft232h_miso PRIVATE pico_stdlib hardware_pio pico_multicore hardware_flash pico_flash hardware_exception)
pico_add_extra_outputs(ft232h_miso)

example_auto_set_url(ft232h_miso)