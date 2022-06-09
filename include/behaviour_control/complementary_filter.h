#pragma once

#include "vars.h"

class ComplementaryFilter{
    public:
        ComplementaryFilter(float k);
        float calculate(float f);
    private:
        float K;
        float oldVal;
};