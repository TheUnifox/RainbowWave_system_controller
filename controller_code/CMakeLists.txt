add_executable(controller_main MAX30101.cpp RA8875.cpp spo2_algorithm.cpp controller_main.cpp)

# pull in common dependencies and additional i2c hardware support
target_link_libraries(controller_main pico_stdlib hardware_i2c hardware_spi pico_multicore)

pico_enable_stdio_usb(controller_main 1)
pico_enable_stdio_uart(controller_main 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(controller_main)
