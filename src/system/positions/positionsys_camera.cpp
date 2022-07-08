#include "systems/position/positionsys_camera.h"
#include "behaviour_control/status_vector.h"
#include "math.h"
#include "sensors/sensors.h"
#include "vars.h"

int diff(int a, int b)
{
    int diffB = abs(min(a, b) - max(a, b));
    int diffB1 = 360 - diffB;
    int diff = min(diffB, diffB1);
    return diff;
}

PositionSysCamera::PositionSysCamera()
{
    Inputx = 0;
    Outputx = 0;
    Setpointx = 0;
    Inputy = 0;
    Outputy = 0;
    Setpointy = 0;

    vx = 0;
    vy = 0;
    axisx = 0;
    axisy = 0;
    givenMovement = false;

    X = new PID(&Inputx, &Outputx, &Setpointx, Kpx, Kix, Kdx, REVERSE);
    X->SetOutputLimits(-DIM_X_HALF, DIM_X_HALF);
    X->SetMode(AUTOMATIC);
    X->SetSampleTime(2);
    Y = new PID(&Inputy, &Outputy, &Setpointy, Kpy, Kiy, Kdy, REVERSE);
    Y->SetOutputLimits(-DIM_Y_HALF, DIM_Y_HALF);
    Y->SetMode(AUTOMATIC);
    Y->SetSampleTime(2);

    filterDir = new ComplementaryFilter(0.35);
    filterSpeed = new ComplementaryFilter(0.65);
}

void PositionSysCamera::update()
{
    int posx = 0, posy = 0;
    CURRENT_DATA_WRITE.camera_back_in_time = false;
    bool data_valid = true;

    if (CURRENT_DATA_READ.atkSeen || CURRENT_DATA_READ.defSeen)
    {
        method = 0;
        int distxd = -sin(radians(CURRENT_DATA_READ.defGAngle_fix)) * CURRENT_DATA_READ.defGDist;
        int distyd = abs(cos(radians(CURRENT_DATA_READ.defGAngle_fix))) * CURRENT_DATA_READ.defGDist;
        int distxa = -sin(radians(CURRENT_DATA_READ.atkGAngle_fix)) * CURRENT_DATA_READ.atkGDist;
        int distya = abs(cos(radians(CURRENT_DATA_READ.atkGAngle_fix))) * CURRENT_DATA_READ.atkGDist;

        // DEBUG.println("POSX1, POSY1 " + String(distxd) + "," + String(distyd));
        // DEBUG.println("POSX2, POSY2 " + String(distxa) + "," + String(distya));

        int posya = CAMERA_GOAL_ATK_Y - distya;
        int posyd = CAMERA_GOAL_DEF_Y + distyd;

        if (((distxd * distxa < 0) && abs(distxd - distxa) > 10) || (CURRENT_DATA_READ.atkSeen && CURRENT_DATA_READ.defSeen && abs(posya - posyd) > 25)) data_valid = false;

        if (data_valid)
        {
            if ((CURRENT_DATA_READ.atkSeen && !CURRENT_DATA_READ.defSeen) || (CURRENT_DATA_READ.atkSeen && CURRENT_DATA_READ.defSeen && CURRENT_DATA_READ.atkGDist < CURRENT_DATA_READ.defGDist))
            {
                posx = distxa;
                posy = posya;
            }
            else
            {
                posx = distxd;
                posy = posyd;
            }
        }
    }

    if (!data_valid || (!CURRENT_DATA_READ.atkSeen && !CURRENT_DATA_READ.defSeen))
    {
        method = 4;

        // Go back in time until we found a valid status, when we saw at least one goal
        for (int i = 1; i < dim; i++)
        {
            valid_data = getDataAtIndex_backwardsFromCurrent(i);
            if (valid_data.ySeen || valid_data.bSeen)
            {
                posx = valid_data.posx;
                posy = valid_data.posy;

                // Trick the status vector into thinking this was a valid status
                CURRENT_DATA_WRITE.ySeen = valid_data.ySeen;
                CURRENT_DATA_WRITE.bSeen = valid_data.bSeen;
                CURRENT_DATA_WRITE.camera_back_in_time = true;

                break;
            }
        }
    }

    CURRENT_DATA_WRITE.posx = posx;
    CURRENT_DATA_WRITE.posy = posy;

    Inputx = posx;
    Inputy = posy;
    // Prepare for receiving information about movement
    // Starting setpoint position as current position
    Setpointx = posx;
    Setpointy = posy;
    axisx = 0;
    axisy = 0;
    givenMovement = false;
}

// This means the last time this is called has the biggest priority, has for prepareDrive
void PositionSysCamera::setMoveSetpoints(int x, int y)
{
    // Setpointx = x + CAMERA_TRANSLATION_X;
    // Setpointy = y + CAMERA_TRANSLATION_Y;
    Setpointx = x;
    Setpointy = y;
    givenMovement = true;
    CameraPID();
}

void PositionSysCamera::addMoveOnAxis(int x, int y)
{
    axisx += x;
    axisy += y;
    givenMovement = true;
    CameraPID();
}

void PositionSysCamera::goCenter()
{
    setMoveSetpoints(CAMERA_CENTER_X, CAMERA_CENTER_Y);
}

void PositionSysCamera::centerGoal()
{
    setMoveSetpoints(CAMERA_GOAL_X, CAMERA_GOAL_Y);
}

/*Knowing the sum of the absolute values of the y position of the goals, it calculates the missing goal y knowing the other one
We know the sum of the absolute values is a fixed number.
By subtracting the absolute value of the goal y we know to the sum of the absolute values, we get the absolute value of the missing goal y
The sign of the goal y we found is simply the reverse of the one we got
*/

bool PositionSysCamera::isInTheVicinityOf(int x_, int y_)
{
    // Distance using pytagorean theorem
    return pow(CURRENT_DATA_READ.posx - x_, 2) + pow(CURRENT_DATA_READ.posy - y_, 2) <= VICINITY_DIST_TRESH * VICINITY_DIST_TRESH;
}

bool PositionSysCamera::isInRoughVicinityOf(int x_, int y_)
{
    // Distance using pytagorean theorem
    return pow(CURRENT_DATA_READ.posx - x_, 2) + pow(CURRENT_DATA_READ.posy - y_, 2) <= ROUGH_VICINITY_DIST_TRESH * ROUGH_VICINITY_DIST_TRESH;
}

bool PositionSysCamera::isAtDistanceFrom(int x_, int y_, int dist)
{
    // Distance using pytagorean theorem
    return pow(CURRENT_DATA_READ.posx - x_, 2) + pow(CURRENT_DATA_READ.posy - y_, 2) <= dist * dist;
}

void PositionSysCamera::CameraPID()
{
    if (givenMovement)
    {

        vx = 0;
        vy = 0;

        Setpointx += axisx;
        Setpointy += axisy;

        if (X->Compute() && Y->Compute())
        {

            // Compute an X and Y to give to the PID later
            // There's surely a better way to do this
            int dir = -90 - (atan2(Outputy, Outputx) * 180 / 3.14);
            dir = (dir + 360) % 360;

            float distance = hypot(Outputx, Outputy);
            float speed = distance > 2 ? 35 + map(distance, 0, MAX_DIST_EXPERIMENTAL, 0, MAX_VEL) : 0;

            // DEBUG.print("x: ");
            // DEBUG.print(Outputx);
            // DEBUG.print(" y:");
            // DEBUG.print(Outputy);
            // DEBUG.print(" Hypot:");
            // DEBUG.print(hypot(Outputx, Outputy));
            // DEBUG.print(" Speed:");
            // DEBUG.println(speed);

#ifdef DRIVE_VECTOR_SUM
            vx = ((speed * cosins[dir]));
            vy = ((-speed * sins[dir]));
            CURRENT_DATA_WRITE.addvx = vx;
            CURRENT_DATA_WRITE.addvy = vy;
#else
            int tmp = (CURRENT_DATA_WRITE.tilt + 360) % 360;
            dir = dir - tmp;
            if (dir < 0) dir += 360;
            drive->prepareDrive(dir, speed, CURRENT_DATA_WRITE.tilt);
#endif
        }
    }
}

void PositionSysCamera::test()
{
    DEBUG.println("Using method " + String(method) + " Position: (" + String(CURRENT_DATA_WRITE.posx) + ", " + String(CURRENT_DATA_WRITE.posy) + ")");
}
