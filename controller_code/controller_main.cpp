#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/multicore.h"
#include "MAX30101.h"
#include "RA8875.h"
#include "pico/time.h"
#include "spo2_algorithm.h"

#define map(a1,a2,b1,b2,s) (b1 + (s-a1)*(b2-b1)/(a2-a1))

const uint LEDPIN = 25;

const uint cs_pin = 17;
const uint sck_pin = 18;
const uint mosi_pin = 19;
const uint miso_pin = 16;

const uint8_t sda_pin = 14;
const uint8_t scl_pin = 15;
uint8_t address = 0;

raspiRA8875 display = raspiRA8875(cs_pin, sck_pin, mosi_pin, miso_pin);

static int core0_rx_val = 0, core1_rx_val = 0;

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void core0_sio_irq() {
    // Just record the latest entry
    while (multicore_fifo_rvalid())
        core0_rx_val = multicore_fifo_pop_blocking();

    multicore_fifo_clear_irq();
}

void core1_sio_irq() {
    // Just record the latest entry
    while (multicore_fifo_rvalid())
        core1_rx_val = multicore_fifo_pop_blocking();

    multicore_fifo_clear_irq();
}

void core1_entry(){
	multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_sio_irq);

    irq_set_enabled(SIO_IRQ_PROC1, true);

	display.textWrite("Hi from core 1!\n");
	printf("Hi from core 1!\n");

    // Send something to Core0, this should fire the interrupt.
    multicore_fifo_push_blocking(1);

	sleep_ms(10); //wait a bit for core0 to say all good
	display.graphicsMode(); //set graphics mode and clear the screen
	display.clearActiveWindow(true);
	printf("graphics mode set, and screen cleared\n");

	//initialize a bunch of ints and a bool
	int ir_val, spo2, ypos, thickness, midpoint, bottom, top, y = 0;
	bool fifo_data;
	
	//initialize an int to keep track of the colour
	//and a map to keep track of the colours for each pixel
	uint8_t colour = 0xF0;
	uint8_t colour_map[800][430];

	// initialize the map with black
	for (int x=0; x<800; x++) {
		for (y=50; y<430; y++){
			colour_map[x][y] = 0x00;
		}
	}
	printf("colour map initialized... entering loop\n");

	//enter main core1 loop
    while (true){

		//wait for something to be available, then get the values needed
        fifo_data = multicore_fifo_rvalid();
		while(!fifo_data)
			{fifo_data = multicore_fifo_rvalid();}
		ir_val = multicore_fifo_pop_blocking();
		spo2 = multicore_fifo_pop_blocking();
		printf("got data from core 0\n");

		//calculate where on the screen the next part of the line should be,
		//the thickness, top, and bottom of the line
		//map ypos to be 50-430, for a 50 pixel buffer on the top and bottom of the screen
		ypos = 480 - map(0, 255, 50, 430, ir_val);
		thickness = map(1, 100, 1, 10, spo2);
		printf("calculated ypos %d, and thickness %d\n", ypos, thickness);

		bottom = (((thickness % 2) == 1) ? (ypos-((thickness/2)+0.5)) : (ypos-(thickness/2)));
		top = bottom + thickness;
		printf("calculated bottom %d, and top %d\n", bottom, top);

		//make a new column in the map
		if(spo2 >0 && spo2 < 100){
			for (y=50; y<430; y++) {
				if (y>=bottom && y<=top){
					colour_map[0][y] = colour;
				}
			}
		}
		printf("new column done\n");

		//go through the map and tell the screen to draw a pixel for the line
		//but only when there is something to draw, as well as above and below by a couple pixels
		// to clear any drawn pixels from previous pass that should be black now
		for (int x=0; x<800; x++) {
			bool line_found = false, top_cleared = false;
			for (y=50; y<430; y++){
				if (colour_map[x][y] > 0x00){
					display.drawPixel(x, y, colour_map[x][y]);
					if (!line_found){
						line_found = true;
					}
					else{
						for (int i=4; i>0; i--) {
							if(!top_cleared){
								display.drawPixel(x, y-i, 0x00);
								top_cleared = true;
							}
						}
					}
				}
				else{
					if(line_found){
						for(int i=0; i<4; i++){
							display.drawPixel(x, y+i, 0x00);
							line_found = false;
						}
					}
				}
			}
		}
		printf("screen printed\n");

		//go through the colour map and shift the columns right one
		//then set the first column black
		for (int x=800; x>0; x--) {
			for (y=50; y<430; y++){
				colour_map[x][y] = colour_map[x-1][y];
			}
		}
		for (y=50; y<430; y++){
			colour_map[0][y] = 0x00;
		}
		printf("map shifted\n");

		//and finally increment the colour
		if (colour == 0xFF){
			colour = 0xF0;
		}
		else{
			colour += 1;
		}
		printf("colour incremented\n");
	}
}

int check_i2c(){
	printf("checking i2c for device\n");
	for (int addr = 0; addr < (1 << 7); ++addr) {
		int ret;
    	uint8_t rxdata;
    	if (reserved_addr(addr)){
        	continue;
		}
    	else{
        	ret = i2c_read_blocking(i2c0, addr, &rxdata, 1, false);
			if (ret < 0){
				printf("device found at: %d\n", addr);
				return addr;
			}
		}
	}
	printf("no device found...\n");
	return -999;
}

int main()
{
	stdio_init_all();

	gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
	
	address = check_i2c();
	while(address < 0){
		address = check_i2c();
	}
	MAX30101 hr_sensor = MAX30101(sda_pin, scl_pin, address);
	display.displayBegin(RA8875_800x480);
	display.textMode();

	//hr_sensor.Multimode_init(0x03, 0x00, 0x04, 0x00, 0x0C, 0x0D, 0x0E, 0x00, 0x01, 0x02, 0x03);
	hr_sensor.Multimode_init(0x03, 0x00, 0x04, 0x00, 0x0C, 0x0D, 0x0E, 0x00, 0x01, 0x02, 0x03);
	
	multicore_launch_core1(core1_entry);

	display.textWrite("Hi from core 0!\n");
	printf("Hi from core 0!\n");

	bool fifo_data = multicore_fifo_rvalid();
	while(!fifo_data)
		{fifo_data = multicore_fifo_rvalid();}

	uint32_t g = multicore_fifo_pop_blocking();

	if (g == 1){
		display.textWrite("Both cores good and communicating!\n");
		printf("both cores are good and communicating!\n");
	}

	uint8_t data[100][3];
	uint32_t sample;
	uint32_t ir_data[100];
	uint32_t red_data[100];
	uint32_t green_data[100];
	int32_t spo2;
	int8_t spo2_val;
	int32_t hr;
	int8_t hr_val;

	for (int i=0; i<100; i++) {
		for (int j=0; j<3; j++) {
			sleep_ms(10);
			int bytes_read = hr_sensor.reg_read(i2c0, address, hr_sensor.REG_FIFO_DATA, &data[i][j], 1);
			while(bytes_read < 1)
				{bytes_read = hr_sensor.reg_read(i2c0, address, hr_sensor.REG_FIFO_DATA, &data[i][j], 1);}
			switch(j){
				case 0:
					ir_data[i] = data[i][0];
					printf("ir value got: %u", ir_data[i]);
				case 1:
					red_data[i] = data[i][1];
					printf("red value got: %u", red_data[i]);
				case 2:
					green_data[i] = data[i][2];
					printf("green value got: %u\n", green_data[i]);
			}
		}
	}

	printf("entering loop...\n");
	while(true){
		//sample = ((uint32_t)(data[0] & 0x03) << 16) | (data[1] << 8) | data[2];

        ///< Right shift the data based on the LED_PW setting
        //sample = sample >> 3; // 0=shift 3, 1=shift 2, 2=shift 1, 3=no shift

		for (int i = 25; i < 100; i++)
    	{
      		ir_data[i - 25] = ir_data[i];
      		red_data[i - 25] = red_data[i];
			green_data[i - 25] = green_data[i];
    	}
		printf("cleared 25 slots");

    //take 25 sets of samples before calculating the heart rate.
    	for (int i = 75; i < 100; i++)
    	{
			for (int j=0; j<3; j++) {
				sleep_ms(10);
				int bytes_read = hr_sensor.reg_read(i2c0, address, hr_sensor.REG_FIFO_DATA, &data[i][j], 1);
				while(bytes_read < 1)
					{bytes_read = hr_sensor.reg_read(i2c0, address, hr_sensor.REG_FIFO_DATA, &data[i][j], 1);}
				switch(j){
					case 0:
						ir_data[i] = data[i][j];
					case 1:
						red_data[i] = data[i][j];
					case 2:
						green_data[i] = data[i][j];
				}
			}
		}
		printf("got new data\n");

        maxim_heart_rate_and_oxygen_saturation(ir_data, 100, red_data, &spo2, &spo2_val, &hr, &hr_val);
		printf("hr data: %d", spo2);

		if (spo2_val){
			multicore_fifo_push_blocking(ir_data[99]);
			multicore_fifo_push_blocking(spo2);
			printf("spo2 valid, data sent to core 1");
		}
		sleep_ms(10);
	}
}