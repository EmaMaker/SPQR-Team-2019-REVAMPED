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

    double anglea = (double)((90 - CURRENT_DATA_READ.atkGAngle_fix + 360) % 360);
    double angled = (double)((270 - CURRENT_DATA_READ.defGAngle_fix + 360) % 360);

    double anglea_rad = radians(anglea);
    double angled_rad = radians(angled);

    // DEBUG.println("Angles from goals " + String(anglea) + ", " + String(angled));

    // Calculate robot position based on just-read coordinates for camera. Using CURRENT_DATA_WRITE instead of CURRENT_DATA_READ othwerise we would be late by 1 loop cycle, but the calculations have to stay in sync
    // Coordinates are referred to a cartesian plane with the origin at the center of the field. Angles starting at the north of the robot
    if (CURRENT_DATA_WRITE.atkSeen && CURRENT_DATA_WRITE.defSeen)
    {
        // project two lines, from the center of the goals to the robot. The point of intersection of these two lines is the position of the robot
        // this doesn't work when the angles have tangents that approach infinity, so filtering for that case is needed

        if (CURRENT_DATA_READ.atkGAngle_fix >= 355 || CURRENT_DATA_READ.atkGAngle_fix <= 5 || (CURRENT_DATA_READ.defGAngle_fix >= 175 && CURRENT_DATA_READ.defGAngle_fix <= 185))
        {
            // fallback to a method without tangents
            // Extend two vector and find the point where they end, then take the average
            method = 1;

            int posx1 = (int)(cos(angled_rad) * CURRENT_DATA_READ.defGDist);
            int posy1 = (int)(CAMERA_GOAL_DEF_Y + sin(angled_rad) * CURRENT_DATA_READ.defGDist);
            int posx2 = (int)(cos(anglea_rad) * CURRENT_DATA_READ.atkGDist);
            int posy2 = (int)(CAMERA_GOAL_ATK_Y - sin(anglea_rad) * CURRENT_DATA_READ.atkGDist);

            // DEBUG.println("POSX1, POSY1 " + String(posx1) + "," + String(posy1));
            // DEBUG.println("POSX2, POSY2 " + String(posx2) + "," + String(posy2));

            posx = (int)((posx1 + posx2) * 0.5);
            posy = (int)((posy1 + posy2) * 0.5);
        }
        else
        {
            // resolved manually and checked with wolfram alpha
            // here is the solution https://www.wolframalpha.com/input?i=systems+of+equations+calculator&assumption=%7B%22F%22%2C+%22SolveSystemOf2EquationsCalculator%22%2C+%22equation1%22%7D+-%3E%22y-j+%3D+tan%28a%29%28x-i%29%22&assumption=%22FSelect%22+-%3E+%7B%7B%22SolveSystemOf2EquationsCalculator%22%7D%7D&assumption=%7B%22F%22%2C+%22SolveSystemOf2EquationsCalculator%22%2C+%22equation2%22%7D+-%3E%22y-v%3Dtan%28b%29%28x-u%29%22
            //(i,j), (u,v) are the coords of the two goals. Some stuff cancels out since we assume that the goals always have 0 as x coord
            method = 0;

            double tana = tan(anglea_rad);
            double tanb = tan(angled_rad);

            double tana_tanb_diff = tana - tanb;

            double posx_n = CAMERA_GOAL_DEF_Y - CAMERA_GOAL_ATK_Y;
            double posy_n = -CAMERA_GOAL_ATK_Y * tanb + CAMERA_GOAL_DEF_Y * tana;

            posx = (int)(posx_n / tana_tanb_diff);
            posy = (int)(posy_n / tana_tanb_diff);
        }
    }
    else if (!CURRENT_DATA_WRITE.atkSeen && CURRENT_DATA_WRITE.defSeen)
    {
        method = 2;

        // Extend a vector from a known point and reach the position of the robot
        posx = CAMERA_GOAL_X + cos(angled_rad) * CURRENT_DATA_READ.defGDist;
        posy = CAMERA_GOAL_DEF_Y + sin(angled_rad) * CURRENT_DATA_READ.defGDist;
    }
    else if (CURRENT_DATA_WRITE.atkSeen && !CURRENT_DATA_WRITE.defSeen)
    {
        method = 3;

        // Extend a vector from a known point and reach the position of the robot
        posx = CAMERA_GOAL_X + cos(anglea_rad) * CURRENT_DATA_READ.atkGDist;
        posy = CAMERA_GOAL_ATK_Y + sin(anglea_rad) * CURRENT_DATA_READ.atkGDist;
    }
    else
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
