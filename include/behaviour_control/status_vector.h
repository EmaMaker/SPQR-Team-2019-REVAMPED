#pragma once
#include <Arduino.h>
#include "strategy_roles/game.h"
#include "systems/systems.h"

/**
 * STATUS VECTOR:
 * The status vector consists in two arrays of two different structs.
 * One (inputs) holds the raw input read by the various sensors on the robot
 * The other (datas) contains the useful data obtained by the eventual manipulation of the raw inputs
 * This is made so that it ha an history of the inputs and datas if needed.
 * This is an intermediator between all the classes representing the different components of the robot. It's preferable to not make the classes call one another
 * All the data held by the structs in the status vector will be described here.
 * 
 * REMEMBER: The value of a sensor in the status vector MUST be updated also if the sensor data didn't change
 * 
**/

#ifdef STATUS_VECTOR_CPP
#define sv_extr 
#else
#define sv_extr extern
#endif

#define dim 50
#define CURRENT_DATA_READ ( datas[((currentSVIndex-1+dim) % dim)]  )
#define CURRENT_DATA_WRITE ( datas[((currentSVIndex))]  )
#define CURRENT_INPUT_READ ( inputs[((currentSVIndex-1+dim) % dim)] )
#define CURRENT_INPUT_WRITE ( inputs[((currentSVIndex))] )

typedef struct input{
    int IMUAngle, USfr, USsx, USdx, USrr, BT;
    byte ballByte, cameraByte, lineByte, xb, yb, xy, yy;
    bool SW_DX, SW_SX;
}input;

typedef struct data{
    int IMUAngle = 0, IMUOffset = 0, ballAngle = 0, ballAngleFix, ballDistance, ballPresenceVal, 
        yAngle, bAngle, yAngleFix, bAngleFix, 
        yDist, bDist, 
        angleAtk, angleAtkFix, angleDef, angleDefFix, distAtk, distDef, yAtk, yAtkFix, xAtk, xAtkFix, yDef, yDefFix, xDef, xDefFix,
        cam_xb = 0, cam_yb = 0, cam_xy = 0, cam_yy = 0,
        speed, tilt, dir, axisBlock[4],
        USfr, USsx, USdx, USrr, 
        lineOutDir, matePos, role, cam_xb_fixed = 0,
        cam_xy_fixed = 0, cam_yb_fixed = 0, cam_yy_fixed = 0,
        posx, posy;
        float addvx, addvy;
    Game* game;
    LineSystem* lineSystem;
    PositionSystem* posSystem;
    byte lineSeen, lineActive;
    bool mate, 
        ATKgoal, DEFgoal, 
        atkSeen, defSeen, bSeen = true, ySeen = false,
        ballSeen, ballPresent;
}data;

sv_extr input inputs[dim];
sv_extr data datas[dim];
sv_extr int currentSVIndex;

void initStatusVector();
void updateStatusVector();
data getDataAtIndex(int index);
data getDataAtIndex_backwardsFromCurrent(int steps);