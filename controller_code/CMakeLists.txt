add_executable(controller_main controller_main.cpp)

# pull in common dependencies and additional i2c hardware support
target_link_libraries(controller_main pico_stdlib hardware_i2c hardware_spi pico_multicore)

pico_enable_stdio_usb(controller_main 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(controller_main)
