// In header to allow for multiple PID controllers.

struct PID {
    float Kp;
    float Ki;
    float Kd;
    float tau;
    float integrator_lmt;
    float setpoint;
    float measured;
    float integrator;
    float differentiator;
    float prev_err;
    float prev_measured;
    absolute_time_t time_prev;
};