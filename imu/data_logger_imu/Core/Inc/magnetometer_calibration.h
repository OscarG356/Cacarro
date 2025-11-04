// magnetometer_calibration.h
#ifndef MAGNETOMETER_CALIBRATION_H
#define MAGNETOMETER_CALIBRATION_H

#include <float.h>

typedef struct {
    float min[3];
    float max[3];
    float offset[3];
    int sample_count;
} MagnetometerAutoCalib;

void Magnetometer_UpdateCalibration(MagnetometerAutoCalib* calib, const float raw[3]);
void Magnetometer_ComputeOffset(MagnetometerAutoCalib* calib);

#endif
