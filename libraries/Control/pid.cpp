#include "pid.h"

PID::PID(double kp, double ki, double kd) : 
gains_({ kp, ki, kd }), duration_(0.0), integral_sum_(0.0), error_(0.0), output_(0.0) {}

void PID::Step(double dt, double actual, double command) { 
    this->UpdateDuration(dt);
    this->UpdateError(command - actual);
    this->output_ = this->GetKpContribution() + this->GetKiContribution() + this->GetKdContribution(); 
}

void PID::SetGains(double kp, double ki, double kd, bool reset = true) {
    this->gains_ = {kp, ki, kd};
    if (reset) this->Reset();
}

pid_gains_t PID::GetGains() const { return this->gains_; }
double PID::GetDuration() const { return this->duration_; }
double PID::GetError() const { return this->error_; }
double PID::GetOutput() const { return this->output_; }

double PID::GetClampedOutput(double lower_limit, double upper_limit) const { 
    if (this->output_ < lower_limit) return lower_limit;
    if (this->output_ > upper_limit) return upper_limit;
    return this->output_;
}

double PID::GetKpContribution() const { 
    return this->gains_.kp * this->error_; 
}

double PID::GetKiContribution() const { 
    this->ComputeSummedIntegral();
    return gains_.ki * this->integral_sum_; 
}

double PID::GetKdContribution() const { 
    return this->dt_ != 0.0 ? gains_.kd * (this->de_ / this->dt_) : 0.0; 
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