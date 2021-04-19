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
    X->SetDerivativeLag(1);
    X->SetSampleTime(2);
    Y = new PID(&Inputy, &Outputy, &Setpointy, Kpy, Kiy, Kdy, REVERSE);
    Y->SetOutputLimits(-MAX_Y,MAX_Y);
    Y->SetMode(AUTOMATIC);
    Y->SetDerivativeLag(1);
    Y->SetSampleTime(2);

    filterDir = new ComplementaryFilter(0.65);
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
    }else{
        //TODO: no goal seen ?
    }
    //IMPORTANT STEP: or the direction of the plane will be flipped
    posx *= -1;
    posy *= -1;

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
    Setpointx = x + CAMERA_TRANSLATION_X;
    Setpointy = y + CAMERA_TRANSLATION_Y;
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

        int dist = sqrt( ( (CURRENT_DATA_WRITE.posx-Setpointx)*(CURRENT_DATA_WRITE.posx-Setpointx) ) + (CURRENT_DATA_WRITE.posy-Setpointy)*(CURRENT_DATA_WRITE.posy-Setpointy) );
        // int dist = sqrt(Outputx*Outputx + Outputy*Outputy);
        int speed = map(dist*DIST_MULT, 0, MAX_DIST, 0,  MAX_VEL);
        speed = filterSpeed->calculate(speed);
        speed = speed > 40 ? speed : 0;
        dir = filterDir->calculate(dir);
        // drive->prepareDrive(dir, speed, 0);


        //Disable below lines for now because they probably result in unexpected behaviour on lines. Re enabling them requires to comment out the drive->prepareDrive above
        //and check the notes in drivecontroller for the other stuff to comment and uncomment

        //TODO: add complementary filter on this speed if we keep using it
        vx = ((speed * cosins[dir]));
        vy = ((-speed * sins[dir]));

        CURRENT_DATA_WRITE.addvx = vx;
        CURRENT_DATA_WRITE.addvy = vy;
    }
}

void PositionSysCamera::test(){
}
