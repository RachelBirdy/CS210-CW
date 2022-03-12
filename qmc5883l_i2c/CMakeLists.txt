add_executable(qmc5883l_i2c qmc5883l_i2c.c )

pico_enable_stdio_usb(qmc5883l_i2c 1)
pico_enable_stdio_uart(qmc5883l_i2c 0)

# pull in common dependencies and additional i2c hardware support
target_link_libraries(qmc5883l_i2c pico_stdlib hardware_i2c )

# create map/bin/hex file etc.
pico_add_extra_outputs(qmc5883l_i2c)

# add url via pico_set_program_url
# example_auto_set_url(mcp9808_i2c)
