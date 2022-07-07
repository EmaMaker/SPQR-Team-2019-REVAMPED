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

    int dir = 0, ball_deg = CURRENT_DATA_READ.ballAngle, plusang = STRIKER_PLUSANG + 8 * (CURRENT_DATA_READ.ballDistance <= 90);

    if (CURRENT_DATA_READ.ballDistance >= 125)
    {
        drive->prepareDrive(ball_deg > 180 ? ball_deg * 0.96 : ball_deg * 1.04, STRIKER_VEL, 0);
    }
    else
    {
        // seguo palla

        if (ball->isInFront())
        {
            dir = 0;
            flag = false;
        }
        else
        {
            if (!flag)
            {

                if (ball_deg <= 90)
                    dir = ball_deg + plusang;
                else if (ball_deg >= 270)
                    dir = ball_deg - plusang;
                else
                {
                    flag = true;
                    if (ball_deg <= 180)
                        plusang_flag = plusang;
                    else
                        plusang_flag = -plusang;
                }
            }
            else
                dir = ball_deg + plusang_flag;
        }

        dir = (dir + 360) % 360;
        drive->prepareDrive(dir, STRIKER_VEL, tilt());

        // if(ball->isInFront() && roller->roller_armed) roller->speed(ROLLER_DEFAULT_SPEED);
        // else roller->speed(roller->MIN);
    }
}

int Striker::tilt()
{
    return CURRENT_DATA_READ.ballAngle <= 90 || CURRENT_DATA_READ.ballAngle >= 270 ? CURRENT_DATA_READ.atkGAngle_fix : 0;
}