#include "pid.h"

Pid::Pid()
{

}

Pid::Pid(float kp, float ki, float kd, float mini, float maxi)
{
    this->kp.set(kp);
    this->ki.set(ki);
    this->kd.set(kd);
    this->min_integral.set(mini);
    this->max_integral.set(maxi);
}

float Pid::eval(float error)
{
    float y;
    integral = integral + error; // adding error to sum

    if(integral > max_integral.get()) integral = max_integral.get();
    if(integral < min_integral.get()) integral = min_integral.get();

    float rdiff = this->kd.get()*(error - pred_err);
    // Control calculation
    y = (this->kp.get()*error + this->ki.get()*integral + rdiff);
    pred_err = error; // current error has become "past error"

    return this->map(y);
}

float Pid::map(float y)
{
    float max_y = this->kp.get()*this->in_max.get() +
                  this->ki.get()*this->max_integral.get() +
                  this->kd.get()*in_max.get();
    float min_y = - max_y;

    float u = (y - min_y) * (max_y - min_y) / (max_y - min_y) + this->out_min.get();

    if(u > this->out_max.get()) u = this->out_max.get();
    if(u < this->out_min.get()) u = this->out_min.get();

    return u;
}
