#pragma once

#ifndef PID_H_
#define PID_H_

#include <stdint.h>

typedef struct {
    double kp;
    double ki;
    double kd;
} pid_gains_t;

typedef struct {
    double kp;
    double ki;
    double kd;
    double ff;
} pid_contrib_t;

typedef enum {
    POSITIVE = 1,
    NEGATIVE = -1
} sign_t;

class PID {
    public:
        PID(double kp, double ki, double kd, sign_t sign = POSITIVE, double (*ff_callback)(double) = nullptr);
        void SetGains(double kp, double ki, double kd, bool reset = true);
        pid_gains_t GetGains() const;
        pid_contrib_t GetContributions() const;
        double GetDuration() const;
        double GetError() const;
        double GetOutput() const;
        double GetClampedOutput(double lower_limit, double upper_limit) const;        
        void Reset();
        void Step(double dt, double actual, double command);
    private:
        void ComputeSummedIntegral();
        void UpdateDuration(double dt);
        void UpdateError(double error);
        double ComputeKpContribution();
        double ComputeKiContribution();
        double ComputeKdContribution();
        double ComputeFeedForward(double actual);

        pid_gains_t gains_;
        sign_t sign_;
        pid_contrib_t contrib_;
        double (*ff_callback_)(double);
        double integral_sum_;
        double duration_;
        double ff_;
        double output_;
        double dt_;
        double error_;
        double de_;
};

#endif // PID_H_