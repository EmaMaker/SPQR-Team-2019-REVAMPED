#pragma once
#include "behaviour_control/data_source.h"
#include <Arduino.h>

#define MOUTH_MIN_ANGLE 345
#define MOUTH_MAX_ANGLE 15
#define MOUTH_DISTANCE 100
#define MOUTH_MAX_DISTANCE 140

class DataSourceBall : public DataSource
{

public:
    DataSourceBall(HardwareSerial *ser, int baud);
    void readSensor() override;
    void postProcess() override;
    void test() override;
    bool isInMouth();
    bool isInMouthMaxDistance();
    bool isInFront();

    int angle = 0, distance = 0, angleFix = 0;
    bool ballSeen = false;
};