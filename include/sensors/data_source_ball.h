#pragma once
#include "behaviour_control/data_source.h"
#include <Arduino.h>

#define MOUTH_MIN_ANGLE 340
#define MOUTH_MAX_ANGLE 20
#define MOUTH_DISTANCE 100
#define MOUTH_MAX_DISTANCE 140

class DataSourceBall : public DataSource{

    public:
        DataSourceBall(HardwareSerial* ser, int baud);
        void postProcess() override;
        void test() override;
        bool isInMouth();
        bool isInMouthMaxDistance();
        bool isInFront();
        
        int angle, distance, angleFix;
        bool ballSeen;
}; 