/*
 * avr_controller.c
 *
 * Created: 5/5/2025 7:28:21 PM
 * Author : Endeavor360
 */ 

#include <avr/io.h>


#include "config.h"
#include "motors.h"

int main(void) {
	// — initialize everything —
	motors_init();

	// — quick test sequence —
	// spin both motors forward at 200 RPM for 2 s, then stop
	motors_enable_left (true);
	motors_enable_right(true);
	motors_set_speed_left (200);
	motors_set_speed_right(200);
	_delay_ms(2000);

	motors_stop_all();

	// — now enter idle loop —
	while (1) {
		// you can call motors_move_left(...) or .right(...)
		// or hook in your state machine & USB parser here
	}
}
