# add_executable(beep_keypad beep_keypad.c)

# # Add pico_multicore which is required for multicore functionality
# target_link_libraries(beep_keypad pico_stdlib pico_multicore pico_bootsel_via_double_reset hardware_spi hardware_sync)

# # create map/bin/hex file etc.
# pico_add_extra_outputs(beep_keypad)

add_executable(beep_keypad beep_keypad.c)

# must match with pio filename and executable name from above
pico_generate_pio_header(beep_keypad ${CMAKE_CURRENT_LIST_DIR}/hsync.pio)
pico_generate_pio_header(beep_keypad ${CMAKE_CURRENT_LIST_DIR}/vsync.pio)
pico_generate_pio_header(beep_keypad ${CMAKE_CURRENT_LIST_DIR}/rgb.pio)

# must match with executable name and source file names
target_sources(beep_keypad PRIVATE beep_keypad.c vga_graphics.c)

# must match with executable name
target_link_libraries(beep_keypad PRIVATE pico_stdlib pico_multicore hardware_pio hardware_dma hardware_sync hardware_spi)

# must match with executable name
pico_add_extra_outputs(beep_keypad)