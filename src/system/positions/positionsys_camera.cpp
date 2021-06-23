#include "behaviour_control/status_vector.h"
#include "systems/position/positionsys_camera.h"
#include "sensors/sensors.h"
#include "vars.h"
#include "math.h"


PositionSysCamera::PositionSysCamera() {
    MAX_DIST = sqrt(MAX_X*MAX_X + MAX_Y*MAX_Y);
    Inputx = 0;
    Outputx = 0;
    Setpointx = 0;
    Inputy = 0;
    Outputy = 0;
    Setpointy = 0;

    vx = 0;
    vy = 0;
    axisx = 0;
    axisy = 0;
    givenMovement = false;
    
    X = new PID(&Inputx, &Outputx, &Setpointx, Kpx, Kix, Kdx, REVERSE);
    X->SetOutputLimits(-MAX_X, MAX_X);
    X->SetMode(AUTOMATIC);
    X->SetSampleTime(2);
    Y = new PID(&Inputy, &Outputy, &Setpointy, Kpy, Kiy, Kdy, REVERSE);
    Y->SetOutputLimits(-MAX_Y,MAX_Y);
    Y->SetMode(AUTOMATIC);
    Y->SetSampleTime(2);

    filterDir = new ComplementaryFilter(0.35);
    filterSpeed = new ComplementaryFilter(0.65);
}

void PositionSysCamera::update(){
    int posx = 0, posy = 0;

    //Calculate robot position based on just-read coordinates for camera. Using CURRENT_DATA_WRITE instead of CURRENT_DATA_READ othwerise we would be late by 1 loop cycle, but the calculations have to stay in sync
    //Coordinates are referred to a cartesian plane with the origin at the center of the field. Angles starting at the north of the robot
    if(CURRENT_DATA_WRITE.bSeen == true && CURRENT_DATA_WRITE.ySeen == true){
        posx = (CURRENT_DATA_WRITE.cam_xy + CURRENT_DATA_WRITE.cam_xb) / 2;
        posy = CURRENT_DATA_WRITE.cam_yb + CURRENT_DATA_WRITE.cam_yy;
    }else if (CURRENT_DATA_WRITE.bSeen == true && CURRENT_DATA_WRITE.ySeen == false){
        posx = CURRENT_DATA_WRITE.cam_xb;
        posy = CURRENT_DATA_WRITE.cam_yb + calcOtherGoalY(CURRENT_DATA_WRITE.cam_yb);
    }else if (CURRENT_DATA_WRITE.bSeen == false && CURRENT_DATA_WRITE.ySeen == true){
        posx = CURRENT_DATA_WRITE.cam_xy;
        posy = CURRENT_DATA_WRITE.cam_yy + calcOtherGoalY(CURRENT_DATA_WRITE.cam_yy);
    }

    //IMPORTANT STEP: or the direction of the plane will be flipped
    posx *= -1;
    posy *= -1;
    
    //Filtering error in calculation like this is a dirty hack, I know
    if(posx < -MAX_X || posx > MAX_X || posy <  -MAX_Y || posy > MAX_Y || (CURRENT_DATA_WRITE.bSeen == false && CURRENT_DATA_WRITE.ySeen == false) ) {
        // Go back in time until we found a valid status, when we saw at least one goal
        int i = 1;
        do{
            valid_data = getDataAtIndex_backwardsFromCurrent(i);
            i++;
        }while(!valid_data.ySeen && !valid_data.bSeen);

        if(valid_data.ySeen || valid_data.bSeen){
            posx = valid_data.posx;
            posy = valid_data.posy;

            // Trick the status vector into thinking this was a valid status
            CURRENT_DATA_WRITE.ySeen = valid_data.ySeen;
            CURRENT_DATA_WRITE.bSeen = valid_data.bSeen;

        }
    }

    CURRENT_DATA_WRITE.posx = posx;
    CURRENT_DATA_WRITE.posy = posy;
    Inputx = posx;
    Inputy = posy;

    //Prepare for receiving information about movement
    //Starting setpoint position as current position
    Setpointx = posx;
    Setpointy = posy;
    axisx = 0;
    axisy = 0;
    givenMovement = false;
}

//This means the last time this is called has the biggest priority, has for prepareDrive
void PositionSysCamera::setMoveSetpoints(int x, int y){
    // Setpointx = x + CAMERA_TRANSLATION_X;
    // Setpointy = y + CAMERA_TRANSLATION_Y;
    Setpointx = x;
    Setpointy = y;
    givenMovement = true;
    CameraPID();
}

void PositionSysCamera::addMoveOnAxis(int x, int y){
    axisx += x;
    axisy += y;
    givenMovement = true;
    CameraPID();
}

void PositionSysCamera::goCenter(){
    setMoveSetpoints(CAMERA_CENTER_X, CAMERA_CENTER_Y);
}

void PositionSysCamera::centerGoal(){
    setMoveSetpoints(CAMERA_GOAL_X, CAMERA_GOAL_Y);
}

/*Knowing the sum of the absolute values of the y position of the goals, it calculates the missing goal y knowing the other one
We know the sum of the absolute values is a fixed number.
By subtracting the absolute value of the goal y we know to the sum of the absolute values, we get the absolute value of the missing goal y
The sign of the goal y we found is simply the reverse of the one we got
*/
int PositionSysCamera::calcOtherGoalY(int goalY){
    int otherGoalY = CAMERA_CENTER_Y_ABS_SUM - abs(goalY);
    otherGoalY = goalY < 0 ? otherGoalY : -otherGoalY;
    return otherGoalY;
}

bool PositionSysCamera::isInTheVicinityOf(int x_, int y_){
    // Distance using pytagorean theorem
    return pow(CURRENT_DATA_READ.posx-x_, 2) + pow(CURRENT_DATA_READ.posy-y_, 2) <= VICINITY_DIST_TRESH*VICINITY_DIST_TRESH;
}

bool PositionSysCamera::isInRoughVicinityOf(int x_, int y_){
    // Distance using pytagorean theorem
    return pow(CURRENT_DATA_READ.posx-x_, 2) + pow(CURRENT_DATA_READ.posy-y_, 2) <= ROUGH_VICINITY_DIST_TRESH*ROUGH_VICINITY_DIST_TRESH;
}

void PositionSysCamera::CameraPID(){   
    if(givenMovement){

        vx = 0;
        vy = 0;

        Setpointx += axisx;
        Setpointy += axisy;

        X->Compute();
        Y->Compute();
        
        //Compute an X and Y to give to the PID later
        //There's surely a better way to do this
        int dir = -90-(atan2(Outputy,Outputx)*180/3.14);
        dir = (dir+360) % 360;

        // int dist = sqrt( ( pow(CURRENT_DATA_WRITE.posx-Setpointx,2) ) + pow(CURRENT_DATA_WRITE.posy-Setpointy, 2) );
        // int speed = map(dist*DIST_MULT, 0, MAX_DIST, 0, MAX_VEL);
        int speed = hypot(Outputx, Outputy) * DIST_MULT;
        // speed = speed > 10 ? speed : 0;
        dir = filterDir->calculate(dir);

        #ifdef DRIVE_VECTOR_SUM
        vx = ((speed * cosins[dir]));
        vy = ((-speed * sins[dir]));
        CURRENT_DATA_WRITE.addvx = vx;
        CURRENT_DATA_WRITE.addvy = vy;
        #else
        drive->prepareDrive(dir, speed, 0);
        #endif

    }
}

void PositionSysCamera::test(){
}
