#include "motors_movement/drivecontroller.h"
#include "sensors/sensors.h"
#include "behaviour_control/status_vector.h"
#include "vars.h"

DriveController::DriveController(Motor* m1_, Motor* m2_, Motor* m3_, Motor* m4_){
    m1 = m1_;
    m2 = m2_;
    m3 = m3_;
    m4 = m4_;

    pDir = 0;
    pSpeed = 0;
    pTilt = 0;

    vx = 0;
    vy = 0;

    speed1 = 0;
    speed2 = 0;
    speed3 = 0;
    speed4 = 0;

    delta = 0;
    input = 0;
    output = 0;
    setpoint = 0;

    pid = new PID(&input, &output, &setpoint, KP, KI, KD,DIRECT);

    pid->SetSampleTime(2.5);
    pid->setAngleWrap(true);
    // pid->SetDerivativeLag(2);
    pid->SetOutputLimits(-255,255);
    pid->SetMode(AUTOMATIC);

    // Complementary filter for speed
    speedFilter = new ComplementaryFilter(0.3);

    canUnlock = true;
    unlockTime = 0;

    vxp = 0;
    vxn = 0;
    vyp = 0;
    vyn = 0;
}

void DriveController::prepareDrive(int dir, int speed, int tilt){
    pDir = dir;
    pSpeed = speed;
    pTilt = tilt;
}

void DriveController::drivePrepared(){
    drive(pDir, pSpeed, pTilt);
}

float DriveController::torad(float f){
    return (f * PI / 180.0);
}

void DriveController::drive(int dir, int speed, int tilt){
    speed = speedFilter->calculate(speed)*GLOBAL_SPD_MULT;
    
    //TODO: Changing CURRENT_DATA_READ to CURRENT_DATA_WRITE?
    // Disable vector sum because calculations are a bitty crappy imho. Will have to test if it's what makes the robot act strange with lines
    // Re enabling the below lines requires to comment out drive->prepareDrive and uncommenting the lines relative to vector sum inside positionsys_camera and comment out the other lines here
    #ifdef DRIVE_VECTOR_SUM
    vx = ((speed * cosins[dir])) + CURRENT_DATA_READ.addvx;
    vy = ((-speed * sins[dir])) + CURRENT_DATA_READ.addvy;
    #else
    vx = ((speed * cosins[dir]));
    vy = ((-speed * sins[dir]));
    #endif

    // if((((vy < 0 && vxn == 1) || (vy > 0 && vxp == 1) || (vx < 0 && vyp == 1) || (vx > 0 && vyn == 1)) && canUnlock) || (millis() > this->unlockTime+UNLOCK_THRESH)) {
    //     vxn = 0;
    //     vxp = 0;
    //     vyp = 0;
    //     vyn = 0;
    // }

    // if((vy > 0 && vxn == 1) || (vy < 0 && vxp == 1)) vy = 0;
    // if((vx > 0 && vyp == 1) || (vx < 0 && vyn == 1)) vx = 0; 

    speed1 = ((vx * sins[m1->angle] ) + (vy * cosins[m1->angle] ));
    speed2 = ((vx * sins[m2->angle]) + (vy * cosins[m2->angle]));
    speed3 = ((vx * sins[m3->angle]) + (vy * cosins[m3->angle]));
    speed4 = ((vx * sins[m4->angle]) + (vy * cosins[m4->angle]));

    // Calculate position error relative to the 0
    delta = CURRENT_DATA_READ.IMUAngle;
    if(delta > 180) delta = delta - 360;
    input = delta;

    if(tilt > 180) tilt = tilt - 360;
    setpoint = tilt;
    
    if(pid->Compute()){
        pidfactor = -output;
        speed1 += pidfactor;
        speed2 += pidfactor;
        speed3 += pidfactor;
        speed4 += pidfactor;

        // Find the maximum speed and scale all of them for the maximum to be 255
        float maxVel = 0;
        maxVel = max(abs(speed1), maxVel);
        maxVel = max(abs(speed2), maxVel);
        maxVel = max(abs(speed3), maxVel);
        maxVel = max(abs(speed4), maxVel);

        if(maxVel > 255){
            // Ratio to 255
            float ratio = maxVel/255;

            // //Scale all the velocities
            speed1 /= ratio;
            speed2 /= ratio;
            speed3 /= ratio;
            speed4 /= ratio;

            // DEBUG.print(speed1);
            // DEBUG.print(" | ");
            // DEBUG.print(speed2);
            // DEBUG.print(" | ");
            // DEBUG.print(speed3);
            // DEBUG.print(" | ");
            // DEBUG.print(speed4);
            // DEBUG.print(" | ");
            // DEBUG.println(maxVel);  
        }

        speed1 = constrain(speed1, -255, 255);
        speed2 = constrain(speed2, -255, 255);
        speed3 = constrain(speed3, -255, 255);
        speed4 = constrain(speed4, -255, 255);     

        m1->drive((int) speed1);
        m2->drive((int) speed2);
        m3->drive((int) speed3);
        m4->drive((int) speed4);

        oldSpeed = speed;
    }

    CURRENT_DATA_WRITE.dir = dir;
    CURRENT_DATA_WRITE.speed = speed;
    CURRENT_DATA_WRITE.tilt = tilt;
    CURRENT_DATA_WRITE.axisBlock[0] = vxp;
    CURRENT_DATA_WRITE.axisBlock[1] = vxn;
    CURRENT_DATA_WRITE.axisBlock[2] = vyp;
    CURRENT_DATA_WRITE.axisBlock[3] = vyn;
} 

void DriveController::resetDrive(){
    CURRENT_DATA_WRITE.addvx = 0;
    CURRENT_DATA_WRITE.addvy = 0;
    prepareDrive(0,0,0);
}

void DriveController::stopAll(){
    m1->stop();
    m2->stop();
    m3->stop();
    m4->stop();
}

int DriveController::directionAccountingForTilt(int wanted_direction, int tilt){
  return ((360-tilt+wanted_direction)+360)%360;
}