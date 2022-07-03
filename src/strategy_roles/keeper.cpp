#include <Arduino.h>

#include "behaviour_control/status_vector.h"
#include "sensors/sensors.h"
#include "strategy_roles/games.h"
#include "strategy_roles/keeper.h"
#include "systems/position/positionsys_camera.h"
#include <Arduino.h>

int currentPosition = 0;

Keeper::Keeper() : Game()
{
    init();
}

Keeper::Keeper(LineSystem *ls_, PositionSystem *ps_) : Game(ls_, ps_)
{
    init();
}

void Keeper::init()
{
    shouldStrike = false;
}

void Keeper::realPlay()
{
    if (ball->ballSeen)
        oscillate();
    else
        ps->centerGoal();
}

void Keeper::oscillate()
{

    if (kick_status == 0)
    {
        // start kicking, go towards ball (timed event)
        drive->prepareDrive(kick_ball_angle, MAX_VEL_HALF, 0);
        if (millis() - t >= 300)
        {
            kick_status++;
            t = millis();
        }
    }
    else if (kick_status == 1)
    {
        if (millis() - t >= 350)
        {
            kick_status++;
        }
        else
        {
            drive->prepareDrive((kick_ball_angle + 180) % 360, MAX_VEL_HALF, 0);
        }
    }
    else
    {
        // default to follow ball
        keeper();

        // ball in roughly the same position
        if ((CURRENT_DATA_READ.ballAngleFix - getDataAtIndex_backwardsFromCurrent(1).ballAngleFix) / 100 <= 0.1 && CURRENT_DATA_READ.ballDistance <= 110 && (CURRENT_DATA_READ.ballDistance - getDataAtIndex_backwardsFromCurrent(1).ballDistance) / 100 <= 0.1)
        {
            if (!b)
            {
                // if not started start timer
                t = millis();
                b = true;
            }
        }
        else
            b = false; // if changed position stop timer

        // timer started and 3secs passed
        if (b && millis() - t >= 3000)
        {
            // start kick
            kick_status = 0;
            b = false;
            kick_ball_angle = CURRENT_DATA_READ.ballAngleFix;
            kick_start_x = CURRENT_DATA_READ.posx;
            kick_start_y = CURRENT_DATA_READ.posy;
            // restart the timer since kicking is a timed event
            t = millis();
        }
    }
}

void Keeper::keeper()
{
    shouldStrike = ball->isInFront() && CURRENT_DATA_READ.ballDistance <= KEEPER_ATTACK_DISTANCE;
    if (shouldStrike) return;

    int speed = MAX_VEL_HALF, dir = 0;
    speed = 0;

    if (CURRENT_DATA_READ.defSeen)
    {
        goal_angle = (CURRENT_DATA_READ.angleDefFix + 360) % 360;
        goal_distance = CURRENT_DATA_READ.distDef;
    }

    if (CURRENT_DATA_READ.ballAngleFix >= MOUTH_MAX_ANGLE && CURRENT_DATA_READ.ballAngleFix <= 180)
    {
        if (goal_angle <= 220)
        {
            dir = 90;
            speed = KEEPER_VEL;
            tilt = tiltfilter->calculate(0);
        }
        else
        {
            dir = 0;
            speed = 0;
            tilt = tiltfilter->calculate(tilt + 0.5);
        }
    }
    else if (CURRENT_DATA_READ.ballAngleFix <= MOUTH_MIN_ANGLE && CURRENT_DATA_READ.ballAngleFix > 180)
    {
        if (goal_angle >= 160)
        {
            dir = 270;
            speed = KEEPER_VEL;
            tilt = tiltfilter->calculate(0);
        }
        else
        {
            dir = 0;
            speed = 0;
            tilt = tiltfilter->calculate(tilt - 0.5);
        }
    }

    tilt = constrain(tilt, -25, 25);

    // if ((angle >= 220 || angle <= 160) && ball->isInFront())
    // {
    if ((CURRENT_INPUT_READ.lineByte & 16) == 16)
    { // 16 OUT 1 IN front
        dir -= 15 * sins[dir];
        if (speed == 0) speed = MAX_VEL_EIGTH;
    }
    else if ((CURRENT_INPUT_READ.lineByte & 64) == 64)
    { // 64 OUT 4 IN back
        dir += 30 * sins[dir];
        if (speed == 0) speed = MAX_VEL_EIGTH;
    }
    // }

    if (CURRENT_INPUT_READ.lineByte == 0)
    {
        speed = MAX_VEL_QUARTER;
        if (goal_distance <= 21 || goal_distance >= 24)
            if (goal_distance <= 23)
                dir = (goal_angle + 360 + 180) % 360;
            else
                dir = (goal_angle + 360) % 360;
    }

    int tmp = ((int)tilt + 360) % 360;
    dir = dir - tmp;
    if (dir < 0) dir += 360;
    drive->prepareDrive(dir, speed, tilt);
}

// shouldStrike = false;

// if (can_change_coords)
// {
//     if (CURRENT_DATA_READ.ballAngleFix >= 340 || CURRENT_DATA_READ.ballAngleFix <= 20)
//         currentPosition = currentPosition; // Unneeded, just here for clarity
//     else if (CURRENT_DATA_READ.ballAngleFix > 20 && CURRENT_DATA_READ.ballAngleFix < 90)
//     {
//         currentPosition++;
//     }
//     else if (CURRENT_DATA_READ.ballAngleFix > 270 && CURRENT_DATA_READ.ballAngleFix < 340)
//     {
//         currentPosition--;
//     }
//     else
//     {
//         shouldStrike = true;
//     }
//     can_change_coords = false;
// }

// currentPosition = constrain(currentPosition, KEEPER_POINT_LEFT_2, KEEPER_POINT_RIGHT_2);
// switch (currentPosition)
// {
// case KEEPER_POINT_LEFT_2:
//     x = KEEPER_POINT_LEFT_2_C;
//     y = KEEPER_Y_CORNER;
//     break;
// case KEEPER_POINT_LEFT:
//     x = KEEPER_POINT_LEFT_C;
//     y = CAMERA_GOAL_Y;
//     break;
// case KEEPER_POINT_CENTER:
//     x = KEEPER_POINT_CENTER_C;

//     y = CAMERA_GOAL_Y;
//     break;
// case KEEPER_POINT_RIGHT:
//     x = KEEPER_POINT_RIGHT_C;
//     y = CAMERA_GOAL_Y;
//     break;
// case KEEPER_POINT_RIGHT_2:
//     x = KEEPER_POINT_RIGHT_2_C;
//     y = KEEPER_Y_CORNER;
//     break;
// }

// if (((PositionSysCamera *)ps)->isInTheVicinityOf(x, y) || millis() - t >= 250)
// {
//     can_change_coords = true;
//     t = millis();
// }

// ps->setMoveSetpoints(x, y);