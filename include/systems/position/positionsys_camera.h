#pragma once

#include "PID_v2.h"
#include "systems/systems.h"
#include "behaviour_control/complementary_filter.h"
#include "behaviour_control/status_vector.h"

//Camera center: those setpoints correspond to what we consider the center of the field
#define CAMERA_CENTER_X 0
#define CAMERA_CENTER_Y 0
//left and right limits of a goal
#define CAMERA_GOAL_MAX_X 8
#define CAMERA_GOAL_MIN_X (-8)

//dimensions of the field, for how we scale it
#define DIM_X 50
#define DIM_X_HALF 25
#define DIM_Y 80
#define DIM_Y_HALF 40

//where is the center of a goal blob as seen by openmv on the field. For atk goal it's positive, for def goal it's negative
#define CAMERA_GOAL_X 0
#define CAMERA_GOAL_Y DIM_Y_HALF
#define CAMERA_GOAL_ATK_Y CAMERA_GOAL_Y
#define CAMERA_GOAL_DEF_Y (-CAMERA_GOAL_Y)

//hipotenuse of dimensions of field
#define MAX_DIST_EXPERIMENTAL 94 


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
        bool isInTheVicinityOf(int, int);
        bool isInRoughVicinityOf(int, int);
        bool isAtDistanceFrom(int, int, int);

        double Inputx, Outputx, Setpointx, Inputy, Outputy, Setpointy;
        int MAX_DIST, vx, vy, axisx, axisy, method;
        bool givenMovement;
        PID* X;
        PID* Y;
        ComplementaryFilter* filterDir;
        ComplementaryFilter* filterSpeed;

        data valid_data;

};
