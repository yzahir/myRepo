#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <main.h>
#include "leds.h"
#include "spi_comm.h"
#include "sensors/proximity.h"
#include "selector.h"
#include "motors.h"
#include "chprintf.h"
#include "usbcfg.h"


int main(void)
{
	usb_start();
    halInit();
    chSysInit();
    mpu_init();
    motors_init();
    clear_leds();
    spi_comm_start();

    int sel = 0;
    int direction =1;
    /* Infinite loop. */
    while (1) {
    	left_motor_set_speed(direction *350);
    	right_motor_set_speed(direction*-350);

    	chThdSleepMilliseconds(500);
    	set_body_led(1);
    	chThdSleepMilliseconds(500);
    	set_body_led(0);
    	sel++;

        if (get_selector() != 0 && sel >= get_selector()) {
        	direction *= -1;
        	sel = 0;
        }

    }
}
#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;
void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
