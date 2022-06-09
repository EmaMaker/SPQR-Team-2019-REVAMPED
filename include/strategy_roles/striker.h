#pragma once

#include "sensors/data_source_camera_vshapedmirror.h"
#include "sensors/sensors.h"
#include "strategy_roles/game.h"

#define STRIKER_ATTACK_DISTANCE 110
#define STRIKER_TILT_STOP_DISTANCE 140
#define STRIKER_PLUSANG 70
#define STRIKER_PLUSANG_VISIONCONE 0

class Striker : public Game{

    public:
        Striker();
        Striker(LineSystem* ls, PositionSystem* ps);

    private:
        void realPlay() override;
        void init() override;
        void striker();
        int tilt();
        float ballTilt();

        int atk_speed, atk_direction;
        float atk_tilt, ball_tilt;

        int ball_angle_filter;
        ComplementaryFilter* ball_filter;

        bool gotta_tilt;
};
