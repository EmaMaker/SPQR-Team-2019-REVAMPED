#pragma once

#include "PID_v2.h"
#include "systems/systems.h"
#include "behaviour_control/complementary_filter.h"
#include "behaviour_control/status_vector.h"

//Camera center: those setpoints correspond to what we consider the center of the field
#define CAMERA_CENTER_X 0
#define CAMERA_CENTER_Y -20

//Camera goal: those setpoints correspond to the position of the center of the goal on the field
#define CAMERA_GOAL_X 0
#define  CAMERA_GOAL_Y -12

#define CAMERA_GOAL_MIN_X -8
#define CAMERA_GOAL_MAX_X 8

#define CAMERA_CENTER_Y_ABS_SUM 60
//Actually it's ± MAX_VAL
#define MAX_X 50
#define MAX_Y (CAMERA_CENTER_Y_ABS_SUM/2)
#define MAX_DIST_EXPERIMENTAL 70
#define DIST_MULT 8

#define VICINITY_DIST_TRESH 2
#define ROUGH_VICINITY_DIST_TRESH 10

#define Kpx 2.5
#define Kix 0.1
#define Kdx 0
#define Kpy 3.5
#define Kiy 0.1
#define Kdy 0

class PositionSysCamera : public PositionSystem{

    public:
        PositionSysCamera();
        void goCenter() override;
        void centerGoal() override;
        void setMoveSetpoints(int x, int y);
        void addMoveOnAxis(int x, int y);
        void update() override;
        void test() override;
        void setCameraPID();
        void CameraPID();
        int calcOtherGoalY(int goalY);
        bool isInTheVicinityOf(int, int);
        bool isInRoughVicinityOf(int, int);
        bool isAtDistanceFrom(int, int, int);

        double Inputx, Outputx, Setpointx, Inputy, Outputy, Setpointy;
        int MAX_DIST, vx, vy, axisx, axisy;
        bool givenMovement;
        PID* X;
        PID* Y;
        ComplementaryFilter* filterDir;
        ComplementaryFilter* filterSpeed;

        data valid_data;

};
