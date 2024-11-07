#include <inttypes.h>

#ifndef PIGUN_DETECTOR
#define PIGUN_DETECTOR

/// @brief Represents a peak in the image data
typedef struct {
    float x;             // Current x position (centroid)
    float y;             // Current y position (centroid)
    float dx;            // Velocity in x
    float dy;            // Velocity in y
} pigun_peak_t;

/// @brief Detector operational parameters.
typedef struct {
    uint8_t         error;       // 1 if there was an error after detecting
    uint8_t         *checked;    // one element for each px in the image
    pigun_peak_t    peaks[4];    // peaks detected

} pigun_detector_t;

// Stack structure for flood-fill algorithm
typedef struct {
    int x;
    int y;
} pixel;

void pigun_detector_init();
void pigun_reset_peaks();
void pigun_order_peaks();
void pigun_detector_run(unsigned char*);
void pigun_detector_free();

#endif