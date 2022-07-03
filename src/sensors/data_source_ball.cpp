#include "sensors/data_source_ball.h"
#include "behaviour_control/status_vector.h"
#include "vars.h"

DataSourceBall ::DataSourceBall(HardwareSerial *ser_, int baud) : DataSource(ser_, baud)
{
}

byte buf[3] = {0, 0, 255};
void DataSourceBall ::readSensor()
{
    // while(ser->available() >= 3) DEBUG.print(ser->read());
    while (ser->available() >= 3)
        ser->readBytes(buf, 3);
}

void DataSourceBall ::postProcess()
{
    if ((buf[0] & 0b01000000) == 0b01000000 && (buf[1] & 0b10000000) == 0b10000000 && (buf[2] & 0b11000000) == 0b11000000)
    {
        angle = buf[0] & 0b00111111;
        angle |= (buf[1] & 0b00000111) << 6;
        distance = buf[2] & 0b00011111;
        distance |= (buf[1] & 0b00111000) << 2;

        ballSeen = distance != 255;

        int imuAngle = CURRENT_DATA_READ.IMUAngle > 180 ? CURRENT_DATA_READ.IMUAngle - 360 : CURRENT_DATA_READ.IMUAngle;
        angleFix = (angle + imuAngle + 360) % 360;

        CURRENT_DATA_WRITE.ballAngle = angle;
        CURRENT_DATA_WRITE.ballAngleFix = angleFix;
        CURRENT_DATA_WRITE.ballDistance = distance;
        CURRENT_DATA_WRITE.ballSeen = ballSeen;
    }
}

void DataSourceBall ::test()
{
    this->update();
    // if(ballSeen){
    DEBUG.print(angle);
    DEBUG.print(" | ");
    DEBUG.print(angleFix);
    DEBUG.print(" | ");
    DEBUG.print(distance);
    DEBUG.print(" | ");
    DEBUG.println(ballSeen);
}

bool DataSourceBall::isInFront()
{
    return CURRENT_DATA_READ.ballSeen && (CURRENT_DATA_READ.ballAngle > MOUTH_MIN_ANGLE || CURRENT_DATA_READ.ballAngle < MOUTH_MAX_ANGLE);
}

bool DataSourceBall::isInMouth()
{
    return isInFront() && CURRENT_DATA_READ.ballDistance <= MOUTH_DISTANCE;
}

bool DataSourceBall::isInMouthMaxDistance()
{
    return isInFront() && CURRENT_DATA_READ.ballDistance <= MOUTH_MAX_DISTANCE;
}