#pragma once

#ifndef TIMER_H_
#define TIMER_H_

#include <math.h>
#include "Stopwatch.h"

typedef enum {
    MILLISECONDS,
    SECONDS,
    MINUTES
} timer_resolution_t;

class Timer {
    public:
        Timer(float target, timer_resolution_t timer_resolution);
        void Start(void);
        void UpdateTarget(float new_target);
        float fRemaining(void);
        int iRemaining(void);
        bool IsComplete(void);
    private:
        float target_;
        timer_resolution_t timer_resolution_;
        Stopwatch stopwatch_;
};

#endif // TIMER_H_