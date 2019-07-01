#ifndef PID_H
#define PID_H

#include "property.h"


class Pid
{
public:
    Pid();
    Pid(float kp, float ki, float kd, float mini, float maxi);

/**
* @brief Proportional factor
*/
Property<float> kp = Property<float>(1);

/**
* @brief Integration factor
*/
Property<float> ki = Property<float>(0.001);

/**
 * @brief differentiation factor
 */
Property<float> kd = Property<float>(1);

/**
 * @brief Minimum integrator value
 */
Property<float> min_integral = Property<float>(-100);

/**
 * @brief Maximum integrator value
 */
Property<float> max_integral = Property<float>(100);

/**
 * @brief Maximum input value
 */
Property<float> in_max = Property<float>(50);

/**
 * @brief Maximum output value
 */
Property<float> out_max = Property<float>(255);

/**
 * @brief Minimum output value
 */
Property<float> out_min = Property<float>(50);

/**
 * @brief Control calculation
 * @param error
 * @return float
 */
float eval(float error);

private:
    float integral = 0;
    float pred_err = 0;

    /**
     * @brief map
     * @param y
     * @return
     */
    float map(float y);
};

#endif // PID_H
