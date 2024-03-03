#pragma once

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

typedef struct {
    double kp;
    double ki;
    double kd;
} pid_gains_t;

class PID {
    public:
        PID(double kp, double ki, double kd);
        void PID::SetGains(double kp, double ki, double kd, bool reset = true);
        pid_gains_t GetGains() const;
        double GetDuration() const;
        double GetError() const;
        double GetOutput() const;
        double GetClampedOutput(double lower_limit, double upper_limit) const;
        double GetKpContribution() const;
        double GetKiContribution() const;
        double GetKdContribution() const;
        void Reset();
        void Step(double dt, double actual, double command);
    private:
        void ComputeSummedIntegral();
        void UpdateDuration(double dt);
        void UpdateError(double error);

        pid_gains_t gains_;
        double integral_sum_;
        double duration_;
        double output_;
        double dt_;
        double error_;
        double de_;
};

#endif // PID_H_