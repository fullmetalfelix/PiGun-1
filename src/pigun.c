 /* This code sets up the camera and start the acquision/aiming loop.
 
 GENERAL IDEA OF THIS CODE
  *
  * the camera acquires... as fast as possible i guess!
  * camera.video output is configured with half resolution of the camera acquisition, at 90 fps
  * camera.video output is NOT connected to anything, but buffered into a pool
  * camera.video -> buffer (video_buffer_callback)
  * camera
  * video_buffer_callback copies the Y channel into another buffer
  * we use the Y channel to detect the IR LEDs
  *
  * camera video output uses MMAL_ENCODING_I420
  *
  *
  * */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/time.h>

#include <bcm2835.h>

#include "bcm_host.h"
#include "interface/vcos/vcos.h"

#include "interface/mmal/mmal.h"
#include "interface/mmal/util/mmal_default_components.h"
#include "interface/mmal/util/mmal_connection.h"

#include "pigun.h"
#include "pigun-gpio.h"
#include "pigun-hid.h"

#include <math.h>
#include <stdint.h>
#include <signal.h>

#include <X11/Xlib.h>
#include <X11/Xutil.h>


// main object with all pigun info
pigun_object_t pigun;

// the HID report
pigun_report_t global_pigun_report;


// detected peaks - in order
Peak* pigun_peaks;

// pointer to the current buffer->data
unsigned char* pigun_framedata;

// normalised aiming point - before calibration applies
pigun_aimpoint_t pigun_aim_norm;

// calibration points in normalised frame of reference
pigun_aimpoint_t pigun_cal_topleft;
pigun_aimpoint_t pigun_cal_lowright;


// GLOBAL MMAL STUFF
MMAL_PORT_T* pigun_video_port;
MMAL_POOL_T* pigun_video_port_pool;

pthread_mutex_t pigun_mutex;


/// @brief Save the calibration data for future use.
void pigun_calibration_save(){
	
	FILE* fbin = fopen("cdata.bin", "wb");
	fwrite(&(pigun.cal_topleft),  sizeof(pigun_aimpoint_t), 1, fbin);
	fwrite(&(pigun.cal_lowright), sizeof(pigun_aimpoint_t), 1, fbin);
	fclose(fbin);
}


static void preview_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer) { mmal_buffer_header_release(buffer); }


/* Camera Callback Function
*
* This is called when the camera has a frame ready.
* buffer->data has the pixel values in the chosen encoding (I420).
*
*/
static void video_buffer_callback(MMAL_PORT_T* port, MMAL_BUFFER_HEADER_T* buffer) {

#ifdef PIGUN_TIMING

	// Measure elapsed time and print instantaneous FPS
	static int loop = 0;                // Global variable for loop number
	static struct timespec t1 = { 0, 0 }; // Global variable for previous loop time
	struct timespec t2;                 // Current loop time
	float dt;                           // Elapsed time between frames

	clock_gettime(CLOCK_MONOTONIC, &t2);
	if (t1.tv_sec != 0 && t1.tv_nsec != 0) {
		dt = t2.tv_sec + (t2.tv_nsec / 1000000000.0) - (t1.tv_sec + (t1.tv_nsec / 1000000000.0));
	}
	loop++;

	if (loop > 0 && loop % 50 == 0) {
		printf("loop = %d, Framerate = %f fps, buffer->length = %d \n", loop, 1.0 / dt, buffer->length);
	}
	t1 = t2;
#endif

	MMAL_POOL_T* pool = (MMAL_POOL_T*)port->userdata;

	// call the peak detector function
	pigun_framedata = buffer->data;

	if (pigun_detect(pigun_framedata)) { // if there was a detector error, error LED goes on
		pigun_GPIO_output_set(PIN_OUT_ERR, 1);
	}
	else { // otherwise goes off
		pigun_GPIO_output_set(PIN_OUT_ERR, 0);
	}


	// the peaks are supposed to be ordered by the detector function

	// TODO: maybe add a mutex/semaphore so that the main bluetooth thread
	// will wait until this is done with the x/y aim before reading the HID report

	// computes the aiming position from the peaks
	pigun_calculate_aim();



	// check the buttons ***************************************************

	pigun_buttons_process();

	// TODO: maybe add a mutex/semaphore so that the main bluetooth thread
	// will wait until this is done with the buttons before reading the HID report

	/* BUTTON SYSTEM
	
		button pins are kept HIGH by the pizero and grounded (LOW) when the user presses the physical switch
		the bcm2835 lib detects falling edge events (FEE)
		the FEE only registers as a button press if the button is in the released state
		after the press is registered, the button is locked in pressed state for X frames to avoid jitter
		once the x frames are passed, we check if the pin state is still LOW
		when it is not low, the button becomes released
	
	*/

	// *********************************************************************

	// we are done with this buffer, we can release it!
	mmal_buffer_header_release(buffer);

	// and send one back to the port (if still open)
	if (port->is_enabled) {

		MMAL_STATUS_T status = MMAL_SUCCESS;
		MMAL_BUFFER_HEADER_T* new_buffer;
		new_buffer = mmal_queue_get(pool->queue);

		if (new_buffer)
			status = mmal_port_send_buffer(port, new_buffer);

		if (!new_buffer || status != MMAL_SUCCESS)
			printf("PIGUN ERROR: Unable to return a buffer to the video port\n");
	}
}



// Initialises the camera with libMMAL
int pigun_mmal_init(void) {

	printf("PIGUN: initializing camera...\n");

	MMAL_COMPONENT_T* camera = NULL;
	MMAL_COMPONENT_T* preview = NULL;

	MMAL_ES_FORMAT_T* format = NULL;

	MMAL_PORT_T* camera_preview_port = NULL;
	MMAL_PORT_T* camera_video_port = NULL;
	MMAL_PORT_T* camera_still_port = NULL;

	MMAL_POOL_T* camera_video_port_pool;

	MMAL_CONNECTION_T* camera_preview_connection = NULL;

	MMAL_STATUS_T status;

	// I guess this starts the driver?
	bcm_host_init();
	printf("BCM Host initialized.\n");

	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
	if (status != MMAL_SUCCESS) {
		printf("PIGUN ERROR: create camera returned %x\n", status);
		return -1;
	}

	// connect ports
	camera_preview_port = camera->output[MMAL_CAMERA_PREVIEW_PORT];
	camera_video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];
	camera_still_port = camera->output[MMAL_CAMERA_CAPTURE_PORT];

	// configure the camera component **********************************
	{
		MMAL_PARAMETER_CAMERA_CONFIG_T cam_config = {
			{ MMAL_PARAMETER_CAMERA_CONFIG, sizeof(cam_config)},
			.max_stills_w = PIGUN_CAM_X,
			.max_stills_h = PIGUN_CAM_Y,
			.stills_yuv422 = 0,
			.one_shot_stills = 1,
			.max_preview_video_w = PIGUN_CAM_X,
			.max_preview_video_h = PIGUN_CAM_Y,
			.num_preview_video_frames = 3,
			.stills_capture_circular_buffer_height = 0,
			.fast_preview_resume = 0,
			.use_stc_timestamp = MMAL_PARAM_TIMESTAMP_MODE_RESET_STC
		};
		mmal_port_parameter_set(camera->control, &cam_config.hdr);
	}
	// *****************************************************************
	// Setup camera video port format **********************************
	format = camera_video_port->format;

	format->encoding = MMAL_ENCODING_I420;
	format->encoding_variant = MMAL_ENCODING_I420;

	// video port outputs reduced resolution
	format->es->video.width = PIGUN_RES_X;
	format->es->video.height = PIGUN_RES_Y;
	format->es->video.crop.x = 0;
	format->es->video.crop.y = 0;
	format->es->video.crop.width = PIGUN_RES_X;
	format->es->video.crop.height = PIGUN_RES_Y;
	format->es->video.frame_rate.num = PIGUN_FPS;
	format->es->video.frame_rate.den = 1;

	camera_video_port->buffer_size = camera_video_port->buffer_size_recommended;
	camera_video_port->buffer_num = 4;

	// apply the format
	status = mmal_port_format_commit(camera_video_port);
	if (status != MMAL_SUCCESS) {
		printf("PIGUN ERROR: unable to commit camera video port format (%u)\n", status);
		return -1;
	}

	printf("PIGUN: camera video buffer_size = %d\n", camera_video_port->buffer_size);
	printf("PIGUN: camera video buffer_num = %d\n", camera_video_port->buffer_num);
	// *****************************************************************
	// crate buffer pool from camera.video output port *****************
	camera_video_port_pool = (MMAL_POOL_T*)mmal_port_pool_create(
		camera_video_port,
		camera_video_port->buffer_num,
		camera_video_port->buffer_size
	);
	camera_video_port->userdata = (struct MMAL_PORT_USERDATA_T*)camera_video_port_pool;

	// the port is enabled with the given callback function
	// the callback is called when a complete frame is ready at the camera.video output port
	status = mmal_port_enable(camera_video_port, video_buffer_callback);
	if (status != MMAL_SUCCESS) {
		printf("PIGUN ERROR: unable to enable camera video port (%u)\n", status);
		return -1;
	}
	printf("PIGUN: frame buffer created.\n");
	// *****************************************************************

	// not sure if this is needed - seems to work without as well
	status = mmal_component_enable(camera);

	printf("PIGUN: setting up parameters\n");
	
	// Disable exposure mode
	pigun_camera_exposuremode(camera, 0);

	// Set gains
	//if (argc == 3) {
		//int analog_gain = atoi(argv[1]);
		//int digital_gain = atoi(argv[2]);
		//pigun_camera_gains(camera, analog_gain, digital_gain);
	//}
	//
	// Setup automatic white balance
	//pigun_camera_awb(camera, 0);
	//pigun_camera_awb_gains(camera, 1, 1);

	// Setup blur
	pigun_camera_blur(camera, 1);
	printf("PIGUN: parameters set\n");

	// send the buffers to the camera.video output port so it can start filling them frame data

	// Send all the buffers to the encoder output port
	int num = mmal_queue_length(camera_video_port_pool->queue);
	int q;
	for (q = 0; q < num; q++) {
		MMAL_BUFFER_HEADER_T* buffer = mmal_queue_get(camera_video_port_pool->queue);

		if (!buffer)
			printf("PIGUN ERROR: Unable to get a required buffer %d from pool queue\n", q);

		if (mmal_port_send_buffer(camera_video_port, buffer) != MMAL_SUCCESS)
			printf("PIGUN ERROR: Unable to send a buffer to encoder output port (%d)\n", q);

		// we are not really dealing with errors... they are not supposed to happen anyway!
	}

	status = mmal_port_parameter_set_boolean(camera_video_port, MMAL_PARAMETER_CAPTURE, 1);
	if (status != MMAL_SUCCESS) {
		printf("PIGUN ERROR: %s: Failed to start capture\n", __func__); // what is this func?
		return -1;
	}

	// save necessary stuff to global vars
	pigun_video_port = camera_video_port;
	pigun_video_port_pool = camera_video_port_pool;

	printf("PIGUN: camera init done\n");
	return 0;
}



//void * test_main(int argc, char** argv) {
void* pigun_cycle(void* nullargs) {

	pigun.state = STATE_IDLE;
	
	// reset calibration
	pigun.cal_topleft.x = pigun.cal_topleft.y = 0;
	pigun.cal_lowright.x = pigun.cal_lowright.y = 1;
	
	// load calibration data if available
	FILE* fbin = fopen("cdata.bin", "rb");
	if (fbin == NULL) printf("PIGUN: no calibration data found\n");
	else {
		fread(&(pigun.cal_topleft), sizeof(pigun_aimpoint_t), 1, fbin);
		fread(&(pigun.cal_lowright), sizeof(pigun_aimpoint_t), 1, fbin);
		fclose(fbin);
	}

	// allocate peaks
	pigun.peaks = (Peak*)calloc(10, sizeof(pigun_peak_t));

	// pins should be initialised using the function in the GPIO module
	// called by the main thread when the program starts
	// because the bluetooth (HID) part also uses the LEDs to inform about connection status
	
	// Initialize the camera system
	int error = pigun_mmal_init();
	if (error != 0) {
		pigun_GPIO_output_set(PIN_OUT_ERR, 1);
		return NULL;
	}
	printf("PIGUN: MMAL started.\n");


	
	// repeat forever and ever!
	// there could be a graceful shutdown?
	int cameraON;
	while (1) {
		
		cameraON = 1;
		switch (pthread_mutex_trylock(&pigun_mutex)) {
		case 0: /* if we got the lock, unlock and return 1 (true) */
			pthread_mutex_unlock(&pigun_mutex);
			cameraON = 1;
			break;
		case EBUSY: /* return 0 (false) if the mutex was locked */
			cameraON = 0;
			break;
		}
		
		// if this thread could get the mutex lock, then the main thread is signalling a stop!
		if (cameraON) break;
	}
	
	free(pigun.peaks);

	pthread_exit((void*)0);
}

