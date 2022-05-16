#include "behaviour_control/status_vector.h"
#include "sensors/data_source_ball.h"
#include "vars.h"

DataSourceBall :: DataSourceBall(HardwareSerial* ser_, int baud) : DataSource(ser_, baud) {
  ballSeen = false;
  distance = 0;
  angle = 0;
}

void DataSourceBall :: postProcess(){
  if((value & 0x01) == 1){
      distance = value;
      ballSeen = distance == 255 ? 0 : 1;
  }else{ 
    angle = value * 2;
    int imuAngle = CURRENT_DATA_READ.IMUAngle > 180 ? CURRENT_DATA_READ.IMUAngle - 360 : CURRENT_DATA_READ.IMUAngle;
    angleFix = (angle+imuAngle+360)%360;
  }
  CURRENT_INPUT_WRITE.ballByte = value;
  CURRENT_DATA_WRITE.ballAngle = angle;
  CURRENT_DATA_WRITE.ballAngleFix = angleFix;
  CURRENT_DATA_WRITE.ballDistance = distance;
  CURRENT_DATA_WRITE.ballSeen = ballSeen;
}

void DataSourceBall :: test(){
    this->update();
    // if(ballSeen){
    DEBUG.print(angle); 
    DEBUG.print(" | ");
    DEBUG.print(angleFix); 
    DEBUG.print(" | ");
    DEBUG.print(distance); 
    DEBUG.print(" | ");
    DEBUG.println(ballSeen); 
    // }else{
    // DEBUG.println("Not seeing ball");
    // }
}

bool DataSourceBall::isInFront(){
  return CURRENT_DATA_READ.ballSeen && (CURRENT_DATA_READ.ballAngle > MOUTH_MIN_ANGLE || CURRENT_DATA_READ.ballAngle < MOUTH_MAX_ANGLE );
}

bool DataSourceBall::isInMouth(){
  return isInFront() && CURRENT_DATA_READ.ballDistance<=MOUTH_DISTANCE;
}

bool DataSourceBall::isInMouthMaxDistance(){
  return isInFront() && CURRENT_DATA_READ.ballDistance <= MOUTH_MAX_DISTANCE;
}