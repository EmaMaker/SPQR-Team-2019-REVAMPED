#pragma once

#include "behaviour_control/complementary_filter.h"
#include "behaviour_control/data_source.h"
#include <Arduino.h>

#define FILTER_DEFAULT_COEFF 0.85
#define FILTER_YANGLE_COEFF FILTER_DEFAULT_COEFF
#define FILTER_BANGLE_COEFF FILTER_DEFAULT_COEFF
#define FILTER_YANGLE_FIX_COEFF FILTER_DEFAULT_COEFF
#define FILTER_BANGLE_FIX_COEFF FILTER_DEFAULT_COEFF
#define FILTER_YDIST_COEFF FILTER_DEFAULT_COEFF
#define FILTER_BDIST_COEFF FILTER_DEFAULT_COEFF

class DataSourceCameraConic : public DataSource
{

public:
    DataSourceCameraConic(HardwareSerial *ser, int baud);
    void test() override;
    void readSensor() override;
    void computeCoordsAngles();
    // int getValueAtk(bool);
    // int getValueDef(bool);

    int count = 0, unkn_counter = 0;
    bool data_received = false, start = false, end = false, dash = false;
    char current_char = 'a', start_char = 'a', end_char = 'a'; // initialize to unused values

    int goalOrientation, old_goalOrientation, pAtk, pDef;
    String s1 = "", s2 = "";

    int yangle = 0, bangle = 0, yangle_fix = 0, bangle_fix = 0, ydist = 0, bdist = 0;
    ComplementaryFilter *filt_yangle, *filt_bangle, *filt_yangle_fix, *filt_bangle_fix, *filt_ydist, *filt_bdist;
};