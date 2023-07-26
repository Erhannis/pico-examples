add_executable(am_broadcast)

pico_generate_pio_header(am_broadcast ${CMAKE_CURRENT_LIST_DIR}/am_broadcast.pio)

target_sources(am_broadcast PRIVATE am_broadcast.c)

target_link_libraries(am_broadcast PRIVATE pico_stdlib hardware_pio)
pico_add_extra_outputs(am_broadcast)

# add url via pico_set_program_url
example_auto_set_url(am_broadcast)