#pragma once

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

#include "Arduino.h"

class Stopwatch {
    public:
        Stopwatch();
        void Reset();
        unsigned long Milliseconds();
        float fSeconds();
        unsigned long iSeconds();
        float fMinutes();
        unsigned long iMinutes();
    private:
        unsigned long start_time;
};

#endif // STOPWATCH_H_