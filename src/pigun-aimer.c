#include "pigun.h"
#include "pigun-hid.h"
#include "pigun-mmal.h"

/*
* at this point the peaks are:
* 
* 0------1
* |      |
* |      |
* 2------3
* 
* but the inverse projection assumes the peaks are in a different order:
* 
* A------C
* |      |
* |      |
* B------D
* 
* (so we just fix this in the definitions of x1,x2,...)
* 
* the resulting matrix C is stored in CMatrix, linearised
* the x,y, screen point then corresponds to
* P = cmat . {x, y, 1} / (cmat . {x, y, 1})[z]
* in recrangle coordinates
* P = (0,0) => A
* P = (1,1) => D
* it puzzles me that there is NO dependence on the rectangle aspect ratio?!
* 
*/
void pigun_calculate_aim() {
	
	float aim_x, aim_y;
	float x1 = pigun.detector.peaks[0].x;
	float x2 = pigun.detector.peaks[2].x;
	float x3 = pigun.detector.peaks[1].x;
	float x4 = pigun.detector.peaks[3].x;
	float y1 = pigun.detector.peaks[0].y;
	float y2 = pigun.detector.peaks[2].y;
	float y3 = pigun.detector.peaks[1].y;
	float y4 = pigun.detector.peaks[3].y;

#ifdef PIGUN_DEBUG
	printf("peaks: %f-%f/%f  %f-%f/%f  %f-%f/%f  %f-%f/%f \n",
	x1,y1,pigun.detector.peaks[0].total,
	x2,y2,pigun.detector.peaks[2].total,
	x3,y3,pigun.detector.peaks[1].total,
	x4,y4,pigun.detector.peaks[3].total);
#endif

	// build the transformation matrix using the 4 points and apply it to the center of camera image
	// check out the mathematica notebook for some more insight
	float dy = (y1 - y2);
	float dxy = x1 * y2 - x2 * y1;
	float d1 = 1.0 / (x4 * dy + dxy - x1 * y4 + x2 * y4);
	aim_x = (dy * (PIGUN_RES_X / 2) + (x2 - x1) * (PIGUN_RES_Y / 2) + dxy) * d1;

	dy = (y1 - y3);
	dxy = x1 * y3 - x3 * y1;
	d1 = 1.0 / (x4 * dy + dxy - x1 * y4 + x3 * y4);
	aim_y = (dy * (PIGUN_RES_X / 2) + (x3 - x1) * (PIGUN_RES_Y / 2) + dxy) * d1;

	// save the normalised aim position before messing with it - meaning 0,0 and 1,1 are the TR,LL corner LEDs
	pigun.aim_normalised.x = aim_x;
	pigun.aim_normalised.y = aim_y;

	// apply calibration
	aim_x = (aim_x - pigun.cal_topleft.x) / (pigun.cal_lowright.x - pigun.cal_topleft.x);
	aim_y = (aim_y - pigun.cal_topleft.y) / (pigun.cal_lowright.y - pigun.cal_topleft.y);

	// clamp between 0 and 1
	aim_x = (aim_x < 0) ? 0 : aim_x;
	aim_x = (aim_x > 1) ? 1 : aim_x;
	aim_y = (aim_y < 0) ? 0 : aim_y;
	aim_y = (aim_y > 1) ? 1 : aim_y;

#ifdef PIGUN_DEBUG
	printf("aimer output: x=%f y=%f \n",aim_x, aim_y);
#endif

	// write in report
	pigun.report.x = (int16_t)((2 * aim_x - 1) * 32767);
	pigun.report.y = (int16_t)((2 * aim_y - 1) * 32767);

	//printf("HID report: x=%i y=%i bt=%d\n", global_pigun_report.x, global_pigun_report.y, global_pigun_report.buttons);
}



/*
// ORIGINAL CODE
void pigun_compute_4corners(Peak* ABCD, float aspectRatio, float* CMatrix) {

	float x1 = ABCD[0].col;
	float x2 = ABCD[1].col;
	float x3 = ABCD[2].col;
	float x4 = ABCD[3].col;
	float y1 = ABCD[0].row;
	float y2 = ABCD[1].row;
	float y3 = ABCD[2].row;
	float y4 = ABCD[3].row;

	// build the transformation matrix using the 4 points
	// check out the mathematica notebook for some more insight
	float d1 = 1.0 / (x4 * (y1 - y2) + x1 * (y2 - y4) + x2 * (y4 - y1));
	CMatrix[0] = (y1 - y2) * d1; // dy
	CMatrix[1] = (x2 - x1) * d1;
	CMatrix[2] = (x1 * y2 - x2 * y1) * d1; // dxy

	float d2 = 1.0 / (-x3 * y1 + x4 * y1 + x1 * y3 - x4 * y3 - x1 * y4 + x3 * y4);
	CMatrix[3] = (y1 - y3) * d2;
	CMatrix[4] = (x3 - x1) * d2;
	CMatrix[5] = (x1 * y3 - x3 * y1) * d2;

	float d3 = 1.0 / (x3 * y2 - x4 * y2 - x2 * y3 + x4 * y3 + x2 * y4 - x3 * y4);
	CMatrix[6] = CMatrix[0] + CMatrix[3] + (y2 - y3) * d3;
	CMatrix[7] = CMatrix[1] + CMatrix[4] + (x3 - x2) * d3;
	CMatrix[8] = CMatrix[2] + CMatrix[5] + (x2 * y3 - x3 * y2) * d3;

	// all that is left to do is cmat.{x,y,1} = aiming point
	// x,y is the aiming point in camera space, which always is in the middle (XRES/2, YRES/2)
}
*/


