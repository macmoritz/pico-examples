add_executable(pio_ili9341_lcd)

pico_generate_pio_header(pio_ili9341_lcd ${CMAKE_CURRENT_LIST_DIR}/ili9341_lcd.pio)

target_sources(pio_ili9341_lcd PRIVATE ili9341_lcd.c)

target_link_libraries(pio_ili9341_lcd PRIVATE pico_stdlib hardware_pio hardware_interp)
pico_add_extra_outputs(pio_ili9341_lcd)

# add url via pico_set_program_url
example_auto_set_url(pio_ili9341_lcd)
