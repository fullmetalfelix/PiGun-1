/*
* This file contains function that handle the GPIO pins 
*/

#include <bcm2835.h>
#include <stdint.h>
#include <stdio.h>

#include "pigun.h"
#include "pigun-hid.h"
#include "pigun-gpio.h"
#include "pigun-mmal.h"



int pigun_GPIO_inited;
int button_delay = 3;		// delay between consecutive button presses

// List of gpio code that will be used as the 8 buttons
int pigun_button_pin[9] = {
	PIN_TRG,PIN_RLD,PIN_MAG,					// handle buttons
	PIN_BT0,PIN_BTU,PIN_BTD,PIN_BTL,PIN_BTR,	// d-pad buttons
	PIN_CAL										// calibration is last
};

int pigun_solenoid_timer = 0;

uint16_t pigun_button_state = 0;					// stores value at the bit position corresponding to button id, 1 if the button was just pressed
uint16_t pigun_button_newpress = 0;					// stores value at the bit position corresponding to button id, 1 if the button was just pressed

int pigun_button_holder[9] = { 0,0,0,0,0,0,0,0,0 }; // frame counters for each button

// status=0: ready to be pressed
// status=1: button is currently pressed
// status<0: button has been released as is "recharging"
int pigun_button_status[9] = {0,0,0,0,0,0,0,0,0};


/// @brief Initialises te GPIO system.
/// @return 0 if no problem occurred.
int pigun_GPIO_init() {

	pigun_GPIO_inited = 0;

	// set the green LED for connection
	// GPIO system
	if (!bcm2835_init()) {
		printf("PIGUN ERROR: failed to init BCM2835!\n");
		return 1;
	}

	// setup LEDs - start as off
	bcm2835_gpio_fsel(PIN_OUT_AOK, BCM2835_GPIO_FSEL_OUTP); bcm2835_gpio_write(PIN_OUT_AOK, LOW);
	bcm2835_gpio_fsel(PIN_OUT_ERR, BCM2835_GPIO_FSEL_OUTP); bcm2835_gpio_write(PIN_OUT_ERR, LOW);
	bcm2835_gpio_fsel(PIN_OUT_CAL, BCM2835_GPIO_FSEL_OUTP); bcm2835_gpio_write(PIN_OUT_CAL, LOW);

	// setup the pins for input buttons
	for (int i = 0; i < 9; i++) {
		bcm2835_gpio_fsel(pigun_button_pin[i], BCM2835_GPIO_FSEL_INPT);		// set as input
		bcm2835_gpio_set_pud(pigun_button_pin[i], BCM2835_GPIO_PUD_UP);		// give it a pullup resistor

		bcm2835_gpio_fen(pigun_button_pin[i]);								// detect falling edge (should happen when button is pressed=grounded)
	}
	
	// setup solenoid - start ON because it is connected to the 555 trigger
	bcm2835_gpio_fsel(PIN_OUT_SOL, BCM2835_GPIO_FSEL_OUTP); bcm2835_gpio_write(PIN_OUT_SOL, HIGH);

	pigun_button_state = 0;
	pigun_button_newpress = 0;

	pigun_solenoid_timer = 0;

	printf("PIGUN: GPIO system initialised.\n");
	pigun_GPIO_inited = 1;
	return 0;
}

int pigun_GPIO_stop() {

	if (pigun_GPIO_inited == 0) return 0;

	// set solenoid trigger to HIGH because it is attached to the 555
	bcm2835_gpio_write(PIN_OUT_SOL, HIGH);

	// switch off all LEDS
	bcm2835_gpio_write(PIN_OUT_AOK, LOW);
	bcm2835_gpio_write(PIN_OUT_ERR, LOW);
	bcm2835_gpio_write(PIN_OUT_CAL, LOW);

	// deallocate all resources
	bcm2835_close();
	pigun_GPIO_inited = 0;

	return 0;
}

/// @brief Sets the state of an output pin.
/// @param LEDPIN pin ID.
/// @param state 0 or 1.
void pigun_GPIO_output_set(int LEDPIN, int state) {
	bcm2835_gpio_write(LEDPIN, state);
}

void pigun_service_off(){
	printf("PIGUN: service -> idle\n");
	pigun.state = STATE_IDLE;
	bcm2835_gpio_write(PIN_OUT_CAL, LOW);
}


void pigun_recoil_fire(){

	// turn on the GPIO that triggers the solenoid
	bcm2835_gpio_write(PIN_OUT_SOL, SOL_FIRE);
	
	// duration of the trigger pulse
	pigun.recoilPulseTimer = -1; // has to be at least 1 frame to produce a pulse!

}


void pigun_buttons_process() {

	/* BUTTON SYSTEM
	* 
	* button pins are kept HIGH by the pizero and grounded (LOW) when the user presses the physical switch
	* the bcm2835 lib detects falling edge events (FEE)
	* the FEE only registers as a button press if the button is in the released state
	* after the press is registered, the button is locked in pressed state for X frames to avoid jitter
	* once the x frames are passed, we check if the pin state is still LOW
	* when it is not low, the button becomes released
	* 
	*/

	// mark all buttons as not being in "new press" state
	pigun_button_newpress = 0;

	// check all the buttons to detect new presses and formulate the HID report byte
	for (int i = 0; i < 9; i++) {

		// if button was just pressed now AND we are allowed to register
		if (bcm2835_gpio_eds(pigun_button_pin[i])) { // this checks the status flag, which is true when a falling edge is detected

			// clear GPIO event flag
			// this is done every time the event was detected, regardless of whether the
			// event really was a valid press, otherwise the BCM2835 wont detect it again!
			bcm2835_gpio_set_eds(pigun_button_pin[i]);

			// if the button was being reloaded, nothing should happen
			//if(pigun_button_status[i] < 0) continue;

			// register the press event if the button was released & ready to be pressed again
			if (pigun_button_status[i] == 0) {

				pigun_button_newpress |= (uint16_t)(1 << i);	// mark a good press event internally?
				pigun_button_state |= (uint16_t)(1 << i);		// this is for the HID report
				pigun_button_status[i] = 1;						// this is for internal use

				// ignore the rest of the code since it is dealing with button releases
				continue;
			} 
			
			// if the button is not in 0 state, then it is not ready to process this button-press event
		}

		// code here => the press event was NOT registered for this button

		// if button status 1 (pressed) and now it is at HIGH level (not grounded -> released)
		if (bcm2835_gpio_lev(pigun_button_pin[i]) == HIGH && pigun_button_status[i] == 1) {
			// set the status to negative # of frames this takes to recharge
			pigun_button_status[i] = -button_delay;
			pigun_button_state &= ~(uint16_t)(1 << i);
			continue;
		}

		// if the state is negative, increase by one (recharge), regardless of the actual state of the pin
		if(pigun_button_status[i] < 0) pigun_button_status[i]++;

	}

	pigun.report.buttons = pigun_button_state;	// send the state to the HID report (only LSB)

	// recharge the solenoid trigger
	if(pigun.recoilCooldownTimer < 0) {
		
		pigun.recoilCooldownTimer++;
		
		// if recharge complete, switch the trigger pin
		//if(pigun.recoilCooldownTimer == 0)
			//bcm2835_gpio_write(PIN_OUT_SOL, SOL_HOLD);
	}


	// *** deal with some specific buttons *** *****************************


	if (pigun.state == STATE_IDLE) {
		
		
		// process the trigger
		switch(pigun.recoilMode){
			case RECOIL_AUTO:
				// fire if the button is down and cooldown timer is 0
				if((pigun_button_state & 1) && pigun.recoilCooldownTimer == 0){
					pigun_recoil_fire();
					pigun.recoilCooldownTimer = -6; // hard-coded cooldown
				}
				break;
			case RECOIL_SELF:
				// fire on newpress events - no need to check cooldown
				if(pigun_button_newpress & MASK_TRG)
					pigun_recoil_fire();
				break;
			default:
				break;
		}


		if (pigun_button_newpress & MASK_CAL) { // on CAL start service mode
			
			printf("PIGUN: going in service mode\n");
			bcm2835_gpio_write(PIN_OUT_CAL, HIGH); // turn on the yellow LED

			// save the frame
			FILE* fbin = fopen("CALframe.bin", "wb");
			fwrite(pigun.framedata, sizeof(unsigned char), PIGUN_NPX, fbin);
			fclose(fbin);

			// print more debug to screen
			printf("PIGUN PEAKS:\n");
			for(int i=0;i<4;i++){
				printf("\t[%i]: [%f, %f]", i,
					pigun.detector.peaks[i].x, pigun.detector.peaks[i].y);
			}

			pigun.state = STATE_SERVICE;
		}

	}
	else if(pigun.state == STATE_SERVICE){

		if (pigun_button_newpress & MASK_TRG) { // on TRG go to calibration mode
			printf("PIGUN: service -> calibration\n");
			pigun.state = STATE_CAL_TL;
		}
		if (pigun_button_newpress & MASK_CAL) { // on CAL go back to idle
			pigun_service_off();
		}
		if (pigun_button_newpress & MASK_BT0) { // on BT0 switch solenoid mode

			pigun.recoilMode++;
			if(pigun.recoilMode > RECOIL_OFF) pigun.recoilMode = RECOIL_SELF;

			if(pigun.recoilMode == RECOIL_SELF) { // give it a kick if it goes to self mode
				pigun_recoil_fire();
			}
			printf("PIGUN: recoil mode [%i]\n",pigun.recoilMode);

		}



		// TODO: add other functions



		if(pigun_button_state == (MASK_TRG|MASK_RLD|MASK_MAG)) { // when all handle buttons are down at the same time
			printf("PIGUN: system shutdown\n");
			pigun.state = STATE_SHUTDOWN;
			system("sudo shutdown -P now");
		}
	}
	else if(pigun.state == STATE_CAL_TL){
		if (pigun_button_newpress & 1) { // on TRG set the top-left and wait for next corner
			
			pigun.cal_topleft = pigun.aim_normalised;
			printf("PIGUN: calibration top-left {%f, %f}\n", pigun.cal_topleft.x, pigun.cal_topleft.y);

			// save the calibration data
			pigun_calibration_save();

			pigun.state = STATE_CAL_BR;
		}
		else if (pigun_button_newpress & MASK_CAL){
			pigun_service_off();
		}
	}
	else if(pigun.state == STATE_CAL_BR){
		if (pigun_button_newpress & 1) { // if TRIGGER was just pressed
			
			pigun.cal_lowright = pigun.aim_normalised;
			printf("PIGUN: calibration low-right {%f, %f}\n", pigun.cal_lowright.x, pigun.cal_lowright.y);

			// save the calibration data
			pigun_calibration_save();

			// back to idle mode
			pigun.state = STATE_IDLE;
			bcm2835_gpio_write(PIN_OUT_CAL, LOW);
		}
		else if (pigun_button_newpress & MASK_CAL){
			pigun_service_off();
		}
	}



	// after the desired time, SOL 555 trigger goes off
	if (pigun.recoilPulseTimer == 0) bcm2835_gpio_write(PIN_OUT_SOL, SOL_HOLD);
	else if (pigun.recoilPulseTimer < 0) pigun.recoilPulseTimer++;

	// *********************************************************************

	// autoshutdown everything with some button combo
	if(pigun_button_state == 5 && pigun.state == STATE_SERVICE){
		printf("shutting down\n");
		pigun.state = STATE_SHUTDOWN;
		system("sudo shutdown -P now");
	}


}

