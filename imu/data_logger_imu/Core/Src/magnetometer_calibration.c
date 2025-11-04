// magnetometer_calibration.c
#include "magnetometer_calibration.h"

void Magnetometer_UpdateCalibration(MagnetometerAutoCalib* calib, const float raw[3]) {
    for (int i = 0; i < 3; i++) {
        if (raw[i] < calib->min[i]) calib->min[i] = raw[i];
        if (raw[i] > calib->max[i]) calib->max[i] = raw[i];
    }
    calib->sample_count++;
}

void Magnetometer_ComputeOffset(MagnetometerAutoCalib* calib) {
    for (int i = 0; i < 3; i++) {
        calib->offset[i] = (calib->max[i] + calib->min[i]) / 2.0f;
    }
}
