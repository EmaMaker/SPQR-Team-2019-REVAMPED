#pragma once

#include <Arduino.h>

#include "behaviour_control/complementary_filter.h"
#include "motors_movement/motor.h"

#include "PID_v2.h"


//BEST NUMBERS YET
//USE MOVING AVERAGE AND ANGLE WRAP
#define KP 4.2
#define KI 0.1
#define KD 0.3

#define UNLOCK_THRESH 800

//Max possible vel 310

// #define MAX_POSSIBLE_VEL 310
#define MAX_POSSIBLE_VEL 310
#define MAX_VEL MAX_POSSIBLE_VEL
#define MAX_VEL_EIGTH ((int)MAX_VEL*0.8)
#define MAX_VEL_HALF ((int)MAX_VEL*0.5)
#define MAX_VEL_3QUARTERS ((int)MAX_VEL*0.75)
#define MAX_VEL_QUARTER ((int)MAX_VEL*0.25)

//#define DRIVE_VECTOR_SUM 

class DriveController{

    public:

        DriveController(Motor* m1_, Motor* m2_, Motor* m3_, Motor* m4_);
        
        void drive(int dir=0, int speed=0, int tilt=0);
        void prepareDrive(int dir, int speed, int tilt=0);
        void drivePrepared();
        int directionAccountingForTilt(int, int);
        float updatePid();
        float torad(float f);
        void resetDrive();
        void stopAll();

        int vxp, vyp, vxn, vyn;
        bool canUnlock;
        unsigned long unlockTime;
        Motor* m1;
        Motor* m2;
        Motor* m3;
        Motor* m4;

    private:
        PID* pid;
        ComplementaryFilter* speedFilter;
        ComplementaryFilter* dirFilter;
        int pDir, pSpeed, pTilt, oldSpeed;
        float x, y, vx, vy, speed1, speed2, speed3, speed4, pidfactor, delta;

        double input, output, setpoint;
        
};