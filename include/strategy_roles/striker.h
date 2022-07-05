#pragma once

#include "behaviour_control/complementary_filter.h"
#include "motors_movement/drivecontroller.h"
#include "sensors/sensors.h"
#include "strategy_roles/game.h"

#define STRIKER_ATTACK_DISTANCE 110
#define STRIKER_TILT_STOP_DISTANCE 140
#define STRIKER_PLUSANG 30 // 40
#define STRIKER_PLUSANG_VISIONCONE 7
#define STRIKER_VEL MAX_VEL

class Striker : public Game
{

public:
    Striker();
    Striker(LineSystem *ls, PositionSystem *ps);

private:
    void realPlay() override;
    void init() override;
    void striker();
    int tilt();
    float ballTilt();

    int atk_speed, atk_direction = 0, atk_tilt, stato = 0, plusang_flag = 0;
    bool flag = false;

    ComplementaryFilter *ball_filter, *tilt_filter;
};
