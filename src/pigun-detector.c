#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <math.h>
#include "pigun-detector.h"
#include "pigun-mmal.h"
#include "pigun.h"

#define ERROR_THRESHOLD 5       // How many cycles of unfound peaks is considered an error
#define THRESHOLD       130     // Intensity threshold for bright spots
#define SPARSE_STEP     4       // Step size for sparse sampling (every nth pixel)
#define MAX_PEAKS       4       // Maximum number of peaks
#define MAX_PEAK_SIZE   1000    // Maximum number of pixels in a peak
#define MIN_PEAK_SIZE   20      // Minimum number of pixels in a peak
#define MAX_SEARCH_DISTANCE (int)(0.25 * PIGUN_RES_X) // Maximum search distance around old peaks in pixels

/**
 * @brief For initializing detector.
 */
void pigun_detector_init() {
    pigun.detector.checked = calloc(PIGUN_RES_X * PIGUN_RES_Y, sizeof(uint8_t));
    pigun.detector.error = 0;
    memset(pigun.detector.peaks, 0, sizeof(pigun_peak_t)*MAX_PEAKS);

    // Set initial peak locations. Note that the ordering needs to be set
    // correctly.
    pigun_reset_peaks();
}

/**
 * @brief Used to compare peaks for sorting them.
 * 
 * @param a First peak
 * @param b Second peak
 * @return int
 */
int peak_compare_x(const void* a, const void* b) {
    // return -1 if a is before b
    pigun_peak_t* A = (pigun_peak_t*)a;
    pigun_peak_t* B = (pigun_peak_t*)b;
	
    if (A->x < B->x) return -1;
    else return 1;
}

/**
 * @brief Used to order peaks.
 *
 * The ordering is based on the distance of the peaks to the screen corners:
 *  - Peak closest to top-left corner = A
 *  - Peak closest to bottom-left corner = B
 *  - Peak closest to top-right corner = C
 *  - Peak closest to bottom-right corner = D
 * 
 * Assuming we sort the peaks using the peak.total indexer (ascending), the
 * camera sees:
 * 
 *   2---3      3---2
 *   |   |  OR  |   |
 *   0---1      1---0
 * 
 * OR any similar pattern... in the end all we know is that: a. the first 2
 * peaks are the bottom LED bar b. the last 2 are the top LED bar but after
 * sorting the ordering of top and bottom spots depends on camera rotation. We
 * have to manually adjust them so that we get:
 * 
 *   0---1
 *   |   |
 *   2---3
 * 
 * The aimer will use these in the correct order to compute the inverse
 * projection!
 * 
 * Warning! sorting by .total is a hack that only works when the .row/.col are
 * positive.  if we were to use prediction to extrapolate positions of peaks
 * that went out of camera view, the coordinates could be negative.
 */
void pigun_order_peaks() {
    pigun_peak_t sortedpeaks[MAX_PEAKS];

    // sort by col (horizontal coordinate)
    qsort(pigun.detector.peaks, MAX_PEAKS, sizeof(pigun_peak_t), peak_compare_x);
    // the first 2 peaks have to be 0 and 2 (unless u hold the gun like a gansta in which case u deserve to miss)
    // the one with smallest .row is 0
    // flip them if it is not so
    if (pigun.detector.peaks[0].y > pigun.detector.peaks[1].y) {
        sortedpeaks[0] = pigun.detector.peaks[1];
        sortedpeaks[2] = pigun.detector.peaks[0];
	} else {
        sortedpeaks[0] = pigun.detector.peaks[0];
        sortedpeaks[2] = pigun.detector.peaks[1];
    }
    // the same goes for peaks 2 and 3
    if (pigun.detector.peaks[2].y > pigun.detector.peaks[3].y) {
        sortedpeaks[1] = pigun.detector.peaks[3];
        sortedpeaks[3] = pigun.detector.peaks[2];
    } else {
        sortedpeaks[1] = pigun.detector.peaks[2];
        sortedpeaks[3] = pigun.detector.peaks[3];
    }
    memcpy(pigun.detector.peaks, sortedpeaks, sizeof(pigun_peak_t)*MAX_PEAKS);
}

/**
 * @brief Used to reset peaks to a default state.
 */
void pigun_reset_peaks() {
    pigun.detector.peaks[0].x = (int)(0.25 * PIGUN_RES_X);
    pigun.detector.peaks[0].y = (int)(0.25 * PIGUN_RES_Y);
    pigun.detector.peaks[0].dx = 0;
    pigun.detector.peaks[0].dy = 0;
    pigun.detector.peaks[1].x = (int)(0.75 * PIGUN_RES_X);
    pigun.detector.peaks[1].y = (int)(0.25 * PIGUN_RES_Y);
    pigun.detector.peaks[1].dx = 0;
    pigun.detector.peaks[1].dy = 0;
    pigun.detector.peaks[2].x = (int)(0.25 * PIGUN_RES_X);
    pigun.detector.peaks[2].y = (int)(0.75 * PIGUN_RES_Y);
    pigun.detector.peaks[2].dx = 0;
    pigun.detector.peaks[2].dy = 0;
    pigun.detector.peaks[3].x = (int)(0.75 * PIGUN_RES_X);
    pigun.detector.peaks[3].y = (int)(0.75 * PIGUN_RES_Y);
    pigun.detector.peaks[3].dx = 0;
    pigun.detector.peaks[3].dy = 0;
}

// Return an estimate for the peak location
pixel get_peak_estimate(pigun_peak_t *old_peak) {
    int x_est = (int)(old_peak->x + old_peak->dx);
    int y_est = (int)(old_peak->y + old_peak->dy);

    // Clamp boundaries to frame size
    if (x_est < 0) x_est = 0;
    if (y_est < 0) y_est = 0;
    if (x_est >= PIGUN_RES_X) x_est = PIGUN_RES_X - 1;
    if (y_est >= PIGUN_RES_Y) y_est = PIGUN_RES_Y - 1;

    return (pixel){(int)x_est, (int)y_est};
}

// Function to perform connected component analysis and compute blob properties
int compute_blob_properties(uint8_t *frame, uint8_t *checked, int width, int height,
                            int start_x, int start_y, uint32_t *sum_intensity,
                            uint32_t *sum_x, uint32_t *sum_y) {
    // Initialize stack for iterative flood-fill
    pixel stack[MAX_PEAK_SIZE];
    int stack_index = 0;
    int peak_size = 0;

    // Push the starting pixel onto the stack
    int idx = start_y * width + start_x;
    stack[stack_index++] = (pixel){start_x, start_y};
    checked[idx] = 1;

    while (stack_index > 0) {
        // Pop a pixel from the stack
        pixel p = stack[--stack_index];
        int x = p.x;
        int y = p.y;
        int idx = y * width + x;
        uint8_t intensity = frame[idx];
        ++peak_size;

        // Accumulate sums
        *sum_intensity += intensity;
        *sum_x += x * intensity;
        *sum_y += y * intensity;

        // Check neighboring pixels (4-connectivity)
        if (x > 0) {
            int idx_left = idx - 1;
            if (!checked[idx_left] && frame[idx_left] >= THRESHOLD) {
                stack[stack_index++] = (pixel){x - 1, y};
                checked[idx_left] = 1;
            }
        }
        if (x < width - 1) {
            int idx_right = idx + 1;
            if (!checked[idx_right] && frame[idx_right] >= THRESHOLD) {
                stack[stack_index++] = (pixel){x + 1, y};
                checked[idx_right] = 1;
            }
        }
        if (y > 0) {
            int idx_up = idx - width;
            if (!checked[idx_up] && frame[idx_up] >= THRESHOLD) {
                stack[stack_index++] = (pixel){x, y - 1};
                checked[idx_up] = 1;
            }
        }
        if (y < height - 1) {
            int idx_down = idx + width;
            if (!checked[idx_down] && frame[idx_down] >= THRESHOLD) {
                stack[stack_index++] = (pixel){x, y + 1};
                checked[idx_down] = 1;
            }
        }

        // Prevent stack overflow
        if (stack_index >= MAX_PEAK_SIZE) {
            break;
        }
    }

    // Check that peak size is not too small
    if (peak_size < MIN_PEAK_SIZE) {
        // printf("too small peak: [%i]\n", stack_index);
        return -1;
    // Check that peak size is not too big
    } else if (peak_size >= MAX_PEAK_SIZE) {
        // printf("too big peak: [%i]\n", stack_index);
        return -1;
    }
    return 0; // Success
}

int find_peak(int x0, int y0, int dx, int dy, uint8_t *frame, uint8_t *checked, pigun_peak_t *new_peak, pigun_peak_t *old_peak) {
    int x = x0 + dx;
    int y = y0 + dy;

    // Check if x and y are within bounds
    if (x >= 0 && x < PIGUN_RES_X && y >= 0 && y < PIGUN_RES_Y) {
        int idx = y * PIGUN_RES_X + x;
        if (!checked[idx] && frame[idx] >= THRESHOLD) {
            uint32_t sum_intensity = 0;
            uint32_t sum_x = 0;
            uint32_t sum_y = 0;
            int result = compute_blob_properties(frame, checked, PIGUN_RES_X, PIGUN_RES_Y,
                                                 x, y,
                                                 &sum_intensity, &sum_x, &sum_y);
            if (result == 0 && sum_intensity > 0) {
                float centroid_x = (float)sum_x / sum_intensity;
                float centroid_y = (float)sum_y / sum_intensity;

                // Update peak position and velocity
                float delta_x = centroid_x - old_peak->x;
                float delta_y = centroid_y - old_peak->y;

                new_peak->dx = delta_x;
                new_peak->dy = delta_y;
                new_peak->x = centroid_x;
                new_peak->y = centroid_y;
                return 1;
            }
        }
    }
    return 0;
}

void printFPS() {
    static int frames = 0;
    static double lastTime = 0.0;

    // Get the current time
    double currentTime = (double)clock() / CLOCKS_PER_SEC;
    frames++;

    // Calculate FPS every second
    if (currentTime - lastTime >= 1.0) {
        printf("FPS: %d\n", frames);
        
        // Reset the frame count and lastTime
        frames = 0;
        lastTime = currentTime;
    }
}

void pigun_detector_run(uint8_t *frame) {
    static int n_errors = ERROR_THRESHOLD;

    // Uncomment to print out FPS every second.
    // printFPS();

    // Array to store new peaks
    pigun_peak_t new_peaks[MAX_PEAKS];
    int peak_count = 0;

    // Reset the boolean array for marking pixels as checked.
    uint8_t* checked = pigun.detector.checked;
    memset(checked, 0, PIGUN_RES_X * PIGUN_RES_Y * sizeof(uint8_t));

    // Constants for sampling
    int max_delta = MAX_SEARCH_DISTANCE;
    int stride = SPARSE_STEP;

    // Search for a new peak from the vicinity of the old peak.
    for (int i = 0; i < MAX_PEAKS; i++) {
        // Predict new position using previous velocity
        pixel peak_estimate = get_peak_estimate(&pigun.detector.peaks[i]);
        int x0 = peak_estimate.x;
        int y0 = peak_estimate.y;
        int peak_found = 0;

        // Start sampling from delta = 0 (initial position), then expand outwards
        for (int delta = 0; delta <= max_delta && !peak_found; delta += stride) {
            // Top row
            int dy = -delta;
            for (int dx = -delta; dx <= delta && !peak_found; dx += stride) {
                peak_found = find_peak(x0, y0, dx, dy, frame, checked, &new_peaks[i], &pigun.detector.peaks[i]);
                if (peak_found) break;
            }
            if (peak_found) break;
            // Bottom row
            dy = delta;
            for (int dx = -delta; dx <= delta && !peak_found; dx += stride) {
                peak_found = find_peak(x0, y0, dx, dy, frame, checked, &new_peaks[i], &pigun.detector.peaks[i]);
                if (peak_found) break;
            }
            if (peak_found) break;
            // Left column
            int dx = -delta;
            for (int dy = -delta+1; dy < delta && !peak_found; dy += stride) {
                peak_found = find_peak(x0, y0, dx, dy, frame, checked, &new_peaks[i], &pigun.detector.peaks[i]);
                if (peak_found) break;
            }
            if (peak_found) break;
            // Right column
            dx = delta;
            for (int dy = -delta+1; dy < delta && !peak_found; dy += stride) {
                peak_found = find_peak(x0, y0, dx, dy, frame, checked, &new_peaks[i], &pigun.detector.peaks[i]);
                if (peak_found) break;
            }
        }
        // If peak is not found, we break the search for now.
        if (!peak_found) {
            break;
        } else {
            ++peak_count;
        }
    }

    // At this point we should have all the peak we wanted or maybe we are
    // short. If we are short, tell the callback we got an error. For temporary
    // failures, the old peak old peak positions are reported. When encountering
    // longer lasting errors, the peak search is reset in order to unstuck the
    // search.
    if (peak_count != MAX_PEAKS) {
        ++n_errors;
        if (n_errors >= ERROR_THRESHOLD) {
            pigun_reset_peaks();
            pigun.detector.error = 1;
        }
        return;
    }
    // Store new peaks as old peaks
    memcpy(pigun.detector.peaks, new_peaks, sizeof(pigun_peak_t)*MAX_PEAKS);

    pigun_order_peaks();
    pigun.detector.error = 0;
    n_errors = 0;
    return;
}

// Free up allocated memory
void pigun_detector_free(){
    free(pigun.detector.checked);
}