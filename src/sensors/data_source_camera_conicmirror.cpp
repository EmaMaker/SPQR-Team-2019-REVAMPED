#include "sensors/data_source_camera_conicmirror.h"
#include "behaviour_control/status_vector.h"
#include "vars.h"

// Comment out to disable complementary filters on angles
#define CAMERA_CONIC_FILTER_POINTS

DataSourceCameraConic::DataSourceCameraConic(HardwareSerial *ser_, int baud) : DataSource(ser_, baud)
{
    filt_yangle = new ComplementaryFilter(FILTER_YANGLE_COEFF);
    filt_bangle = new ComplementaryFilter(FILTER_BANGLE_COEFF);
    filt_ydist = new ComplementaryFilter(FILTER_YDIST_COEFF);
    filt_bdist = new ComplementaryFilter(FILTER_BDIST_COEFF);
    filt_yangle_fix = new ComplementaryFilter(FILTER_YANGLE_FIX_COEFF);
    filt_bangle_fix = new ComplementaryFilter(FILTER_BANGLE_FIX_COEFF);
}

void DataSourceCameraConic ::readSensor()
{
    while (ser->available())
    {
        current_char = ser->read();

        // update status vector
        CURRENT_INPUT_WRITE.cameraChar = current_char;

        // start
        if (current_char == 'B' || current_char == 'Y')
        {
            start_char = current_char;
            start = true;
            end = false;
            dash = false;
            s1 = "";
            s2 = "";
        }
        // end
        else if (current_char == 'b' || current_char == 'y')
        {
            end_char = current_char;

            if (start && dash && ((start_char == 'B' && end_char == 'b') || (start_char == 'Y' && end_char == 'y')))
            {

                if (start_char == 'B')
                {
                    bangle = s1.toInt();
                    bdist = s2.toInt();
                }
                else
                {
                    yangle = s1.toInt();
                    ydist = s2.toInt();
                }
                computeCoordsAngles();
            }

            end = true;
            start = false;
            dash = false;
            count = 0;
        } // dash
        else if (current_char == '-' && start)
        {
            dash = true;
        } // it's a number
        else if (isDigit(current_char) && start)
        {
            if (dash)
                s2 += current_char;
            else
                s1 += current_char;
        }
    }
}

// Angles are received in degrees
void DataSourceCameraConic ::computeCoordsAngles()
{

#ifdef CAMERA_CONIC_FILTER_POINTS
    yangle = filt_yangle->calculate(yangle);
    ydist = filt_ydist->calculate(ydist);
    bangle = filt_bangle->calculate(bangle);
    bdist = filt_bdist->calculate(bdist);
#endif

    // Fix angles using the IMU
    int tmp = CURRENT_DATA_READ.IMUAngle > 180 ? CURRENT_DATA_READ.IMUAngle - 360 : CURRENT_DATA_READ.IMUAngle;

    yangle_fix = (yangle + tmp + 360) % 360;
    bangle_fix = (bangle + tmp + 360) % 360;

    // TODO: Maybe add a complementary filter on fixed angles ?

    // Important: update status vector
    CURRENT_DATA_WRITE.yangle = yangle;
    CURRENT_DATA_WRITE.bangle = bangle;
    CURRENT_DATA_WRITE.yangle_fix = yangle_fix;
    CURRENT_DATA_WRITE.bangle_fix = bangle_fix;
    CURRENT_DATA_WRITE.ydist = ydist;
    CURRENT_DATA_WRITE.bdist = bdist;
    CURRENT_DATA_WRITE.bSeen = bangle != 999;
    CURRENT_DATA_WRITE.ySeen = yangle != 999;

    CURRENT_DATA_WRITE.atkGAngle = goalOrientation ? yangle : bangle;
    CURRENT_DATA_WRITE.atkGAngle_fix = goalOrientation ? yangle_fix : bangle_fix;
    CURRENT_DATA_WRITE.atkGDist = goalOrientation ? ydist : bdist;
    CURRENT_DATA_WRITE.atkSeen = goalOrientation ? CURRENT_DATA_WRITE.ySeen : CURRENT_DATA_WRITE.bSeen;

    CURRENT_DATA_WRITE.defGAngle = !goalOrientation ? yangle : bangle;
    CURRENT_DATA_WRITE.defGAngle_fix = !goalOrientation ? yangle_fix : bangle_fix;
    CURRENT_DATA_WRITE.defGDist = !goalOrientation ? ydist : bdist;
    CURRENT_DATA_WRITE.defSeen = !goalOrientation ? CURRENT_DATA_WRITE.ySeen : CURRENT_DATA_WRITE.bSeen;
}

void DataSourceCameraConic::test()
{
    DEBUG.println("Received char '" + String(CURRENT_INPUT_READ.cameraChar) + "'");
    DEBUG.println("BLUE GOAL:: Seen: " + String(CURRENT_DATA_READ.bSeen) + " | Angle: " + CURRENT_DATA_READ.bangle + " | Fixed Angle: " + CURRENT_DATA_READ.bangle_fix + " | Dist: " + CURRENT_DATA_READ.bdist);
    DEBUG.println("YELLOW GOAL:: Seen: " + String(CURRENT_DATA_READ.ySeen) + " | Angle: " + CURRENT_DATA_READ.yangle + " | Fixed Angle: " + CURRENT_DATA_READ.yangle_fix + " | Dist: " + CURRENT_DATA_READ.ydist);
}
