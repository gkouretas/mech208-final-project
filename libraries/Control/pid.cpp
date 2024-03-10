#include "pid.h"

PID::PID(double kp, double ki, double kd, sign_t sign = POSITIVE, double (*ff_callback)(double) = nullptr) : 
gains_({ kp, ki, kd }), 
sign_(sign),
ff_callback_(ff_callback),
contrib_({ 0.0, 0.0, 0.0, 0.0 }), 
duration_(0.0), 
integral_sum_(0.0),
actual_(0.0), 
error_(0.0), 
output_(0.0) 
{}

void PID::Step(double dt, double actual, double command) { 
    this->actual_ = actual;
    this->UpdateDuration(dt);
    this->UpdateError(sign_ * (command - actual));
    this->output_ = this->ComputeKpContribution() + this->ComputeKiContribution() + this->ComputeKdContribution() + this->ComputeFeedForward(); 
}

void PID::SetGains(double kp, double ki, double kd, bool reset = true) {
    this->gains_ = {kp, ki, kd};
    if (reset) this->Reset();
}

pid_gains_t PID::GetGains() const { return this->gains_; }
pid_contrib_t PID::GetContributions() const { return this->contrib_; }

double PID::GetDuration() const { return this->duration_; }
double PID::GetError() const { return this->error_; }
double PID::GetOutput() const { return this->output_; }

double PID::GetClampedOutput(double lower_limit, double upper_limit) const { 
    if (this->output_ < lower_limit) return lower_limit;
    if (this->output_ > upper_limit) return upper_limit;
    return this->output_;
}

double PID::ComputeKpContribution() { 
    this->contrib_.kp = this->gains_.kp * this->error_; 
    return this->contrib_.kp;
}

double PID::ComputeKiContribution() { 
    this->ComputeSummedIntegral();
    this->contrib_.ki = gains_.ki * this->integral_sum_; 
    return this->contrib_.ki;
}

double PID::ComputeKdContribution() { 
    this->contrib_.kd = this->dt_ != 0.0 ? gains_.kd * (this->de_ / this->dt_) : 0.0; 
    return this->contrib_.kd;
}

double PID::ComputeFeedForward() {
    if (this->ff_callback_ == nullptr) {
        this->contrib_.ff = 0.0; // ff = 0 if no callback has been applied
    } else {
        this->contrib_.ff = ff_callback_(this->actual_); // otherwise, run callback
    }

    return this->contrib_.ff;
}

void PID::ComputeSummedIntegral() {
    this->integral_sum_ += this->error_ * this->dt_;
}

void PID::Reset() { 
    this->duration_ = 0;
    this->integral_sum_ = 0; 
}

void PID::UpdateDuration(double dt) {
    this->dt_ = dt;
    this->duration_ += dt;
}

void PID::UpdateError(double error) {
    this->de_ = error - this->error_;
    this->error_ = error;
}