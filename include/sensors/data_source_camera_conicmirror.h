#pragma once

#include "behaviour_control/data_source.h"
#include "behaviour_control/complementary_filter.h"

#define startp 105
#define endp 115
#define unkn 116
//Coords are mapped from 0 up to this value
#define MAP_MAX 100
#define HALF_MAP_MAX  50
//Imu To Camera Angle Mult
#define IMUTOC_AMULT 1

#define FILTER_DEFAULT_COEFF 0.25
#define FILTER_BY_COEFF FILTER_DEFAULT_COEFF
#define FILTER_BX_COEFF FILTER_DEFAULT_COEFF
#define FILTER_YY_COEFF FILTER_DEFAULT_COEFF
#define FILTER_YX_COEFF FILTER_DEFAULT_COEFF

/*Camera translation: because of mechanical imprecision, the center of the camera and the center of the cone mirror may not coincide
To overcome this, each coordinate needs to be shifted by some amount, defined on a per-robot basis that needs to be recalibrated each time.
These values need to be subtracted from the coords used in setMoveSetpoints*/

#define CAMERA_TRANSLATION_X 0
#define CAMERA_TRANSLATION_Y 0

class DataSourceCameraConic : public DataSource{

    public:
        DataSourceCameraConic(HardwareSerial* ser, int baud);
        void test() override;
        void readSensor() override;
        void computeCoordsAngles();
        // int getValueAtk(bool);
        // int getValueDef(bool);

        int yAngle, bAngle, yAngleFix, bAngleFix, yDist, bDist;

        int count = 0, unkn_counter;
        int xb, yb, xy, yy, true_xb, true_xy, true_yb, true_yy, calc_xb, calc_yb, calc_xy, calc_yy, true_xb_fixed,
            true_xy_fixed, true_yb_fixed, true_yy_fixed;
        bool data_received = false, start = false, end = false;

        int goalOrientation, old_goalOrientation, pAtk, pDef;
        int value;

        ComplementaryFilter *filter_yy, *filter_xy, *filter_yb, *filter_xb, *filter_yy_fix, *filter_xy_fix, *filter_yb_fix, *filter_xb_fix;
};