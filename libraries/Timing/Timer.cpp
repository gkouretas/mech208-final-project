#include "Timer.h"

Timer::Timer(float target, timer_resolution_t timer_resolution) :
    target_(target),
    timer_resolution_(timer_resolution) { stopwatch_ = Stopwatch(); }

void Timer::Start() { stopwatch_.Reset(); }

void Timer::UpdateTarget(float new_target) { target_ = new_target; }

float Timer::fRemaining() {
    switch(timer_resolution_) {
        case(MILLISECONDS):
            return max(0.0, target_ - stopwatch_.Milliseconds());
        case(SECONDS):
            return max(0.0, target_ - stopwatch_.fSeconds());
        case(MINUTES):
            return max(0.0, target_ - stopwatch_.fMinutes());
        default:
            return 0.0;
    }
}

int Timer::iRemaining() {
    return (int)fRemaining();
}

bool Timer::IsComplete() {
    switch(timer_resolution_) {
        case(MILLISECONDS):
            return target_ - stopwatch_.Milliseconds() <= 0.0;
        case(SECONDS):
            return target_ - stopwatch_.fSeconds() <= 0.0;
        case(MINUTES):
            return target_ - stopwatch_.fMinutes() <= 0.0;
        default:
            return true;
    }
}