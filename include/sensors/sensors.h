#pragma once


#ifdef SENSORS_CPP
#define s_extr 
#else
#define s_extr extern
#endif

#include <Arduino.h>

#include "behaviour_control/ds_ctrl.h"
#include "motors_movement/motor.h"
#include "motors_movement/roller.h"
#include "motors_movement/drivecontroller.h"
#include "systems/systems.h"
#include "systems/lines/linesys_camera.h"
#include "systems/position/positionsys_zone.h"
#include "sensors/data_source_ball.h"
#include "sensors/data_source_bt.h"
#include "sensors/data_source_bno055.h"
#include "sensors/data_source_camera_conicmirror.h"
#include "sensors/data_source_camera_vshapedmirror.h"
#include "vars.h"

void initSensors();
void updateSensors();

s_extr vector<DataSource*> dUs;

s_extr DataSourceCtrl* usCtrl;

s_extr DataSourceBNO055* compass;
s_extr DataSourceBall* ball;
s_extr DataSourceCameraConic* camera;
s_extr DriveController* drive;
s_extr DataSourceBT* bt;

s_extr int role;

s_extr Roller* roller;