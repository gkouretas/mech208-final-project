#include "Stopwatch.h"

Stopwatch::Stopwatch() {
    start_time = millis(); 
}

void Stopwatch::Reset() {
    start_time = millis();
}

unsigned long Stopwatch::Milliseconds() {
    return millis() - start_time;
}

float Stopwatch::fSeconds() {
    return Milliseconds() / 1000.0;
}

unsigned long Stopwatch::iSeconds() {
    return Milliseconds() / 1000;
}

float Stopwatch::fMinutes() {
    return fSeconds() / 60.0;
}

unsigned long Stopwatch::iMinutes() {
    return iSeconds() / 60.0;
}