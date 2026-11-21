#ifndef PIDCONTROLLER_HPP
#define PIDCONTROLLER_HPP

class PIDController {
private:
    float kp, ki, kd, kf;  // <-- Added kf
    float min_output, max_output;
    float integral, prev_error;
    float target;

public:
    // Update constructor to accept kf
    PIDController(float p, float i, float d, float f, float min_out, float max_out) 
        : kp(p), ki(i), kd(d), kf(f), min_output(min_out), max_output(max_out), 
          integral(0.0f), prev_error(0.0f), target(0.0f) {}

    void set_target(float t) { target = t; }
    float get_target() const { return target; }

    float compute(float measured_value, float dt) {
        float error = target - measured_value;
        
        // Feed-Forward: Directly guess the required PWM based on target speed
        float ff_term = kf * target;

        // Proportional
        float p_term = kp * error;

        // Integral with Anti-Windup Clamping & Divide-by-Zero Protection
        integral += error * dt;
        float i_term = 0.0f;
        
        if (ki > 0.0001f) {  // Prevent division by zero
            i_term = ki * integral;
            if (i_term > max_output) {
                i_term = max_output;
                integral = max_output / ki;
            } else if (i_term < min_output) {
                i_term = min_output;
                integral = min_output / ki;
            }
        } else {
            integral = 0.0f; // Reset if Ki is 0
        }

        // Derivative
        float derivative = (error - prev_error) / dt;
        float d_term = kd * derivative;
        prev_error = error;
        
        // Sum and Clamp Total Output (Now includes ff_term!)
        float output = ff_term + p_term + i_term + d_term;
        
        if (output > max_output) return max_output;
        if (output < min_output) return min_output;
        
        return output;
    }

    void reset() {
        integral = 0.0f;
        prev_error = 0.0f;
    }
};

#endif // PIDCONTROLLER_HPP
