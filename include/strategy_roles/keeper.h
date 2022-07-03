#pragma once

#include "behaviour_control/complementary_filter.h"
#include "strategy_roles/game.h"
#include "systems/position/positionsys_camera.h"
#include <Arduino.h>

#define KEEPER_3_POINTS
//#define KEEPER_5_POINTS

// #ifdef KEEPER_3_POINTS
#define KEEPER_POINT_LEFT_2 -2
#define KEEPER_POINT_LEFT -1
#define KEEPER_POINT_CENTER 0
#define KEEPER_POINT_RIGHT 1
#define KEEPER_POINT_RIGHT_2 2

#define KEEPER_POINT_LEFT_2_C CAMERA_GOAL_MIN_X
#define KEEPER_POINT_LEFT_C (CAMERA_GOAL_X + abs(CAMERA_GOAL_MIN_X) / -2)
#define KEEPER_POINT_CENTER_C CAMERA_GOAL_X
#define KEEPER_POINT_RIGHT_C ((CAMERA_GOAL_X + CAMERA_GOAL_MAX_X) / 2)
#define KEEPER_POINT_RIGHT_2_C CAMERA_GOAL_MAX_X

// #define KEEPER_Y_CORNER -45

// #define KEEPER_PANIC_Y -25

#define KEEPER_VEL MAX_VEL

// #endif
#define KEEPER_ATTACK_DISTANCE 140

class Keeper : public Game
{

public:
    Keeper();
    Keeper(LineSystem *, PositionSystem *);

public:
    bool shouldStrike = false;

private:
    void realPlay() override;
    void init() override;
    void keeper();
    void oscillate();
    void kick();

    int point_spacing, ball_x;
    int x = CAMERA_GOAL_X, y = CAMERA_GOAL_Y;
    bool can_change_coords = true;
    unsigned long t = millis();
    float tilt = 0;

    bool b = false;
    int kick_status = -1;
    int kick_ball_angle = 0;
    int kick_start_x = 0;
    int kick_start_y = 0;
    int goal_angle = 0, goal_distance = 0;

    ComplementaryFilter *tiltfilter = new ComplementaryFilter(0.8);
    ComplementaryFilter *ballfilter = new ComplementaryFilter(0.65);
};
