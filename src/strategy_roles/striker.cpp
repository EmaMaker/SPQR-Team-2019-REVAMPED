#include "strategy_roles/striker.h"
#include "behaviour_control/status_vector.h"
#include "math.h"
#include "sensors/data_source_ball.h"
#include "sensors/sensors.h"
#include "systems/position/positionsys_camera.h"
#include "vars.h"

Striker::Striker() : Game()
{
    init();
}

Striker::Striker(LineSystem *ls_, PositionSystem *ps_) : Game(ls_, ps_)
{
    init();
}

void Striker::init()
{
    ball_filter = new ComplementaryFilter(0.75f);
    tilt_filter = new ComplementaryFilter(0.9f);
}

void Striker::realPlay()
{
    if (CURRENT_DATA_READ.ballSeen)
        this->striker();
    else
    {
        ps->goCenter();
    }
}

float ctilt = 0;
unsigned long ttilt = 0;

void Striker::striker()
{

    if (CURRENT_DATA_READ.lineActive != 0) flag = false;

    int dir = 0, ball_deg = ball_filter->calculate(CURRENT_DATA_READ.ballAngle), plusang = 0; //, plusang = STRIKER_PLUSANG + 35 * (CURRENT_DATA_READ.ballAngle <= 90);
                                                                                              // if (plusang >= 80) plusang = 80;

    // if (CURRENT_DATA_READ.ballDistance >= 120)
    // {
    //     drive->prepareDrive(ball_deg > 180 ? ball_deg * 0.96 : ball_deg * 1.04, STRIKER_VEL, 0);
    // }
    // else
    // {
    // seguo palla

    if (ball_deg >= 345 || ball_deg <= 25)
    {
        dir = 0;
        flag = false;
    }
    else
    {
        double ball_x = sin(radians(ball_deg)) * CURRENT_DATA_READ.ballDistance;
        double ball_y = cos(radians(ball_deg)) * CURRENT_DATA_READ.ballDistance;
        double ball_y_new, ball_x_new;
        if (ball_deg <= 100 || ball_deg >= 260)
        {
            ball_y_new = ball_y - 100;
            ball_x_new = 0;
        }
        else
        {
            ball_y_new = ball_y - 100;
            if (ball_deg >= 180)
                ball_x_new = ball_x - 100;
            else
                ball_x_new = ball_x + 100;
        }

        int new_angle = 90 - (int)degrees(atan2(ball_y_new, ball_x));
        if (new_angle < 0) new_angle += 360;

        dir = new_angle;
    }

    // if (CURRENT_DATA_READ.atkSeen) atk_tilt = CURRENT_DATA_READ.angleAtkFix;
    dir = (dir + 360) % 360;
    drive->prepareDrive(dir, STRIKER_VEL, constrain(tilt_filter->calculate(CURRENT_DATA_READ.angleAtkFix), -25, 25));
    // DEBUG.println("Ball: " + String(ball_deg) + "/" + String(CURRENT_DATA_READ.ballDistance) + " Direction: " + String(dir) + "(" + String(ball_deg) + "+" + String(plusang) + ")");

    // if(ball->isInFront() && roller->roller_armed) roller->speed(ROLLER_DEFAULT_SPEED);
    // else roller->speed(roller->MIN);
    // }
}

int Striker::tilt()
{
    // return CURRENT_DATA_READ.ballAngle <= 110 || CURRENT_DATA_READ.ballAngle >= 270 ? CURRENT_DATA_READ.angleAtkFix : 0;
    // return CURRENT_DATA_READ.angleAtkFix;
    return 0;
}