/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>

#include "btstack.h"

#include "pigun.h"
#include "pigun-gpio.h" // this is mine!
#include "pigun-hid.h" // this is mine!

/// @brief HID descriptor for joystick with extra output report (data)
const uint8_t hid_descriptor_joystick_mode[] = {
	0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
	0x09, 0x04,        // Usage (Joystick)
	0xA1, 0x01,        // Collection (Application)
		0x09, 0x01,        //   Usage (Pointer)
		0xA1, 0x00,        //   Collection (Physical)
			0x85, PIGUN_REPORT_ID,		   // 	Report ID 3

			0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
			0x16, 0x01, 0x80,  //   Logical Minimum 0x8001 (-32767)  
			0x26, 0xFF, 0x7F,  //   Logical Maximum 0x7FFF (32767)
			0x09, 0x30,        //     Usage (X)
			0x09, 0x31,        //     Usage (Y)
			0x75, 0x10,        //     Report Size (16)
			0x95, 0x02,        //     Report Count (2)
			0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

			0x05, 0x09,        //   Usage Page (Button)
			0x19, 0x01,        //   Usage Minimum (0x01)
			0x29, 0x08,        //   Usage Maximum (0x08)
			0x15, 0x00,        //   Logical Minimum (0)
			0x25, 0x01,        //   Logical Maximum (1)
			0x75, 0x01,        //   Report Size (1)
			0x95, 0x08,        //   Report Count (8)
			0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)


			0x09, 0x03,		  	// usage ID vendor defined
			0x15, 0x00,			// Logical Minimum (0)
			0x26, 0xFF, 0x00,  	// Logical Maximum (1)
			0x75, 0x08,        	// Report Size (8)
			0x95, 0x01,        	// Report Count (1)
			0x91, 0x02,			// output (data,Var,Abs)

		0xC0,              //   End Collection   --- 27 bytes
	0xC0              // End Collection --- 44 bytes
};

pigun_blinker_t *pigun_blinkers;
static void blinker_autoconnect(void);
static void blinker_connectLED(void); // switches the OK LED

/// @brief Callback for custom blinkers.
/// @param ts 
void pigun_blinker_event(btstack_timer_source_t *ts) {

	pigun_blinker_t *blk = NULL;

	// find the global blinker with this timersource
	for(int i=0; i<10; i++){
		if(ts == &(pigun_blinkers[i].timer)){
			blk = &(pigun_blinkers[i]);
			break;
		}
	}

	if(blk == NULL) return;
	
	if(blk->cancelled){
		blk->active = 0;
		return;
	}

	//printf("PIGUN-BLINKER[%i]: %i/%i\n", blk, blk->counter, blk->nblinks);
	
	// perform the custom action
	blk->callback();

	if(blk->nblinks > 0) {
		blk->counter++;
		if(blk->counter == blk->nblinks) {
			blk->active = 0;
			return;
		}
	}

	// code here => we have to schedule another blink
	btstack_run_loop_set_timer(&(blk->timer), blk->timeout);
	btstack_run_loop_add_timer(&(blk->timer));
	

}

int pigun_blinker_create(uint8_t nblinks, uint16_t timeout, blinker_callback_t callback) {

	pigun_blinker_t *blk = NULL;
	int bID = -1;

	// find the first inactive blinker
	for(int i=0; i<10; i++){
		if(!pigun_blinkers[i].active){
			bID = i;
			blk = &(pigun_blinkers[i]);
			break;
		}
	}

	if(blk == NULL) return bID;

	blk->active = 1;
	blk->nblinks = nblinks;
	blk->counter = 0;
	blk->cancelled = 0;
	blk->timeout = timeout;

	blk->callback = callback;
	blk->timer.process = &(pigun_blinker_event);

	btstack_run_loop_set_timer(&(blk->timer), timeout);
	btstack_run_loop_add_timer(&(blk->timer));

	return bID;
}
void pigun_blinker_stop(int bID) {
	pigun_blinkers[bID].cancelled = 1;
}

int blinkID_greenLED = -1;


static uint8_t hid_service_buffer[2500];
static uint8_t device_id_sdp_service_buffer[100];
static const char hid_device_name[] = "HID PiGun-1";
static btstack_packet_callback_registration_t hci_event_callback_registration;
static uint16_t hid_cid;
static uint8_t hid_boot_device = 0;

static enum {
	APP_BOOTING,
	APP_NOT_CONNECTED,
	APP_CONNECTING,
	APP_CONNECTED
} app_state = APP_BOOTING;





int compare_servers(bd_addr_t a, bd_addr_t b) {
	for (int i = 0; i < 6; i++) {
		if (a[i] != b[i]) return 0;
	}
	return 1;
}

void pigun_server_load(){

	pigun.nServers = 0;
	
	FILE* fin = fopen("servers.bin", "rb");
	if (fin == NULL) {
		// if file not found, create a file with no servers
		fin = fopen("servers.bin", "wb");
		fwrite(&(pigun.nServers), sizeof(int), 1, fin);
		fclose(fin);
		// and open it
		fin = fopen("servers.bin", "rb");
	}
	fread(&(pigun.nServers), sizeof(int), 1, fin);
	
	printf("PIGUN-HID: previous hosts: %i\n", pigun.nServers);
	for (int i = 0; i < pigun.nServers; i++) {
		fread(pigun.servers[i], sizeof(bd_addr_t), 1, fin);
		printf("\thost[%i]: %s\n", i, bd_addr_to_str(pigun.servers[i]));
	}
	fclose(fin);
}



// HID Report sending
static void send_report() {

	// this is the report to send
	// I do now know that the first byte is there for?!
	//uint8_t hid_report[] = { 0xa1, 0, 0, 0, 0, 0 };
	uint8_t hid_report[] = { 0xa1, PIGUN_REPORT_ID, 0, 0, 0, 0, 0 }; // first byte is a1=device to host request type, second byte is report ID

	
	hid_report[2] = (pigun.report.x) & 0xff;
	hid_report[3] = (pigun.report.x >> 8) & 0xff;
	hid_report[4] = (pigun.report.y) & 0xff;
	hid_report[5] = (pigun.report.y >> 8) & 0xff;
	hid_report[6] = pigun.report.buttons;
	

	//printf("sending x=%i (%i %i) y=%i (%i %i) \n", pigun.report.x, hid_report[1], hid_report[2], pigun.report.y, hid_report[3], hid_report[4]);
	hid_device_send_interrupt_message(hid_cid, &hid_report[0], 7); // 6 = sizeof(hid_report)
}

// called when host sends an output report
void set_report(uint16_t hid_cid, hid_report_type_t report_type, int report_size, uint8_t *report){

	/*printf("Host HID output report:\n");
	printf("\tHID CID: %i\n", hid_cid);
	printf("\tReport Type: %i\n", report_type);
	printf("\tReport Size: %i\n", report_size);
	printf("\tReport Data: ");
	for (int i=0; i<report_size; i++) {
		printf("%#02X ",report[i]);
	}
	printf("\n");*/

	// any report would just trigger the solenoid if possible
	if(pigun.recoilMode == RECOIL_HID) pigun_recoil_fire();
}


/// @brief Called when host sends an HID data message.
/// @param hid_cid the HID device ID
/// @param report_type HID report type (should be DATA)
/// @param report_id report ID
/// @param report_size size in bytes (should be 1)
/// @param report the report bytes
///
/// The report is one byte. The high-half is the command, the low-half the parameter.
/// 
/// 0x[0][1]: fire the solenoid once
/// 0x[1][k]: set solenoid mode: k=0,1,2,3 (pigun_recoilmode_t)
void set_data(uint16_t hid_cid, hid_report_type_t report_type, uint16_t report_id, int report_size, uint8_t *report){
	
	printf("Host HID output DATA:\n");
	printf("\tHID CID: %i\n", hid_cid);
	printf("\tReport Type: %i\n", report_type);
	printf("\tReport Size: %i\n", report_size);
	printf("\tReport ID: %i\n", report_id);
	printf("\tReport Data: ");
	for (int i=0; i<report_size; i++) {
		printf("%x ",report[i]);
	}
	printf("\n");

	uint8_t cmd = report[0]>>4;
	uint8_t par = report[0] & 0x0F;
	if(cmd == 1){ // set recoil mode
		if(par >= 0 && par <= RECOIL_OFF){
			pigun.recoilMode = par;
			printf("PIGUN-HID: recoil mode is now %i\n", par);
		}
		else
			printf("PIGUN-HID: invalid recoil mode [%i]\n", par);
	}else if(cmd == 0){
		if(par == 1 && pigun.recoilMode == RECOIL_HID)
			pigun_recoil_fire();
	}
	else{
		printf("PIGUN-HID: invalid data %x\n",report[0]);
	}
}


static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t packet_size){
	UNUSED(channel);
	UNUSED(packet_size);
	uint8_t status;
	
	//printf("got packet type: %i \n", packet_type);
	//if(app_state == APP_CONNECTED) printf("device boot protocol %i\n", hid_device_in_boot_protocol_mode(hid_cid));

	if (packet_type != HCI_EVENT_PACKET) return;

	switch (hci_event_packet_get_type(packet)) {
	case BTSTACK_EVENT_STATE:
		if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;
		app_state = APP_NOT_CONNECTED;
		break;

	case HCI_EVENT_USER_CONFIRMATION_REQUEST:
		// ssp: inform about user confirmation request
		log_info("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", hci_event_user_confirmation_request_get_numeric_value(packet));
		log_info("SSP User Confirmation Auto accept\n");
		break;

	// *** HID META EVENTS ***
	case HCI_EVENT_HID_META:
		switch (hci_event_hid_meta_get_subevent_code(packet)) {

		case HID_SUBEVENT_CONNECTION_OPENED:
			status = hid_subevent_connection_opened_get_status(packet);
			if (status != ERROR_CODE_SUCCESS) {
				// outgoing connection failed
				printf("PIGUN-HID: connection failed, status 0x%x\n", status);
				app_state = APP_NOT_CONNECTED;
				hid_cid = 0;

				// re-register timer
				pigun_blinker_create(1, 1000, &blinker_autoconnect);
				return;
			}

			app_state = APP_CONNECTED;
			hid_cid = hid_subevent_connection_opened_get_hid_cid(packet);
			bd_addr_t host_addr;
			hid_subevent_connection_opened_get_bd_addr(packet, host_addr);

			
			// save the server address - if not already there
			// rewrite the past servers list, putting the current one on top
			// only write 3 of them
			uint8_t isThere = 0;
			for (int i = 0; i < pigun.nServers; i++) {
				if (compare_servers(host_addr, pigun.servers[i])) {
					isThere = 1;
					break;
				}
			}
			uint32_t ns = pigun.nServers;
			if (!isThere) ns++;
			if (ns > 3) ns = 3;
			int written = 1;
			bd_addr_t newlist[3];

			FILE* fout = fopen("servers.bin", "wb");
			fwrite(&ns, sizeof(int), 1, fout);
			fwrite(host_addr, sizeof(bd_addr_t), 1, fout);
			memcpy(newlist[0], host_addr, sizeof(bd_addr_t));

			for (int i = 0; i < pigun.nServers; i++) {
				if (!compare_servers(host_addr, pigun.servers[i])) {
					fwrite(pigun.servers[i], sizeof(bd_addr_t), 1, fout);
					memcpy(newlist[written], pigun.servers[i], sizeof(bd_addr_t));
					written++;
					if (written == 3) break;
				}
			}
			fclose(fout);
			memcpy(pigun.servers[0], newlist[0], sizeof(bd_addr_t)*3);
			pigun.nServers = ns;
			
			// once connected turn off the green LED to save power
			pigun_blinker_stop(blinkID_greenLED);
			pigun_GPIO_output_set(PIN_OUT_AOK, 0); // make sure it turns off

			printf("PIGUN-HID: connected to %s, pigunning now...\n", bd_addr_to_str(host_addr));
			hid_device_request_can_send_now_event(hid_cid); // request a sendnow
			break;
		case HID_SUBEVENT_CONNECTION_CLOSED:
			printf("PIGUN-HID: disconnected\n");
			app_state = APP_NOT_CONNECTED;
			hid_cid = 0;

			// start blinking of the green LED again
			blinkID_greenLED = pigun_blinker_create(0, 800, &blinker_connectLED);

			break;
		case HID_SUBEVENT_CAN_SEND_NOW:
			// when the stack raises can_send_now event, we send a report... because we can!
			send_report(); // uses the global variable with gun data

			// and then we request another can_send_now because we are greedy! need to send moooar!
			hid_device_request_can_send_now_event(hid_cid);
			break;

		default:
			break;
		}
		break;


	default: //any other type of event
		break;
	}
}



/* @section Main Application Setup
 *
 * @text Listing MainConfiguration shows main application code. 
 * To run a HID Device service you need to initialize the SDP, and to create and register HID Device record with it. 
 * At the end the Bluetooth stack is started.
 */

/* LISTING_START(MainConfiguration): Setup HID Device */

int btstack_main(int argc, const char * argv[]);
int btstack_main(int argc, const char * argv[]) {
	(void)argc;
	(void)argv;

	// allow to get found by inquiry
	gap_discoverable_control(1);
	// use Limited Discoverable Mode; Peripheral; Keyboard as CoD
	gap_set_class_of_device(0x2504);
	// set local name to be identified - zeroes will be replaced by actual BD ADDR
	gap_set_local_name("PiGun 1F"); // ("PiGun 00:00:00:00:00:00");
	// allow for role switch in general and sniff mode
	gap_set_default_link_policy_settings( LM_LINK_POLICY_ENABLE_ROLE_SWITCH | LM_LINK_POLICY_ENABLE_SNIFF_MODE );
	// allow for role switch on outgoing connections - this allow HID Host to become master when we re-connect to it
	gap_set_allow_role_switch(true);

	// L2CAP
	l2cap_init();

	// SDP Server
	sdp_init();
	memset(hid_service_buffer, 0, sizeof(hid_service_buffer));

	uint8_t hid_virtual_cable = 0;
	uint8_t hid_remote_wake = 0;
	uint8_t hid_reconnect_initiate = 1;
	uint8_t hid_normally_connectable = 1;

	hid_sdp_record_t hid_params = {
		// hid device subclass 0x2504 joystick, hid counntry code 33 US
		0x2504, 33, // should be the joystick code
		hid_virtual_cable, hid_remote_wake, 
		hid_reconnect_initiate, hid_normally_connectable,
		hid_boot_device, 
		0xFFFF, 0xFFFF, 3200,
		hid_descriptor_joystick_mode,
		sizeof(hid_descriptor_joystick_mode), 
		hid_device_name
	};
	
	hid_create_sdp_record(hid_service_buffer, 0x10001, &hid_params);

	printf("PIGUN-HID: HID service record size: %u\n", de_get_len(hid_service_buffer));
	sdp_register_service(hid_service_buffer);

	// See https://www.bluetooth.com/specifications/assigned-numbers/company-identifiers if you don't have a USB Vendor ID and need a Bluetooth Vendor ID
	// device info: BlueKitchen GmbH, product 1, version 1
   	device_id_create_sdp_record(device_id_sdp_service_buffer, 0x10003, 0x0001, 0x048F, 2, 1);
	printf("Device ID SDP service record size: %u\n", de_get_len((uint8_t*)device_id_sdp_service_buffer));
	sdp_register_service(device_id_sdp_service_buffer);

	// HID Device
	hid_device_init(hid_boot_device, sizeof(hid_descriptor_joystick_mode), hid_descriptor_joystick_mode);
	
	// register for HCI events
	hci_event_callback_registration.callback = &packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);

	// register for HID events
	hid_device_register_packet_handler(&packet_handler);

	// sign up for host output reports?
	hid_device_register_set_report_callback(&set_report);
	hid_device_register_report_data_callback(&set_data);

	// turn on!
	hci_power_control(HCI_POWER_ON);

	// allocate blinkers
	pigun_blinkers = (pigun_blinker_t*)calloc(10, sizeof(pigun_blinker_t));

	// read the previous server addresses from servers.bin
	pigun_server_load();
	
	// start blinking of the green LED
	blinkID_greenLED = pigun_blinker_create(0, 800, &blinker_connectLED);

	// set one-shot timer for autoreconnect of there are servers in the list
	if (pigun.nServers != 0)
		pigun_blinker_create(1, 5000, &blinker_autoconnect);

	return 0;
}



static void blinker_autoconnect(){
	
	// increment counter
	static int snum = 0;

	if (app_state == APP_NOT_CONNECTED && pigun.nServers != 0) {

		// try connecting to a server
		printf("PIGUN-HID: trying to connect to %s...\n", bd_addr_to_str(pigun.servers[snum]));
		hid_device_connect(pigun.servers[snum], &hid_cid);

		snum++; if (snum == pigun.nServers)snum = 0;
	}
}
static void blinker_connectLED() {

	static uint8_t s = 0;

	// change state and restart the blink timer if not connected
	if (app_state != APP_CONNECTED) {

		// switch state
		s = (s) ? 0 : 1;

		pigun_GPIO_output_set(PIN_OUT_AOK, s);
	}
}
