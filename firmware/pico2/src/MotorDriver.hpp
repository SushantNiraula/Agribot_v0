#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include "pico/stdlib.h"
#include "hardware/pwm.h"

class MotorDriver {
private:
    uint pin_rpwm, pin_lpwm, pin_ren, pin_len;
    uint slice_r, slice_l;
    uint chan_r, chan_l;

public:
    MotorDriver(uint rpwm, uint lpwm, uint ren, uint len) 
        : pin_rpwm(rpwm), pin_lpwm(lpwm), pin_ren(ren), pin_len(len) {}

    void init() {
        // Initialize Enable Pins and set them HIGH
        gpio_init(pin_ren); gpio_set_dir(pin_ren, GPIO_OUT); gpio_put(pin_ren, 1);
        gpio_init(pin_len); gpio_set_dir(pin_len, GPIO_OUT); gpio_put(pin_len, 1);

        // Set PWM pins to PWM function
        gpio_set_function(pin_rpwm, GPIO_FUNC_PWM);
        gpio_set_function(pin_lpwm, GPIO_FUNC_PWM);

        // Get PWM slices and channels
        slice_r = pwm_gpio_to_slice_num(pin_rpwm);
        slice_l = pwm_gpio_to_slice_num(pin_lpwm);
        chan_r = pwm_gpio_to_channel(pin_rpwm);
        chan_l = pwm_gpio_to_channel(pin_lpwm);

        // Set resolution to 0-255 (like Arduino)
        pwm_set_wrap(slice_r, 255);
        pwm_set_wrap(slice_l, 255);

        // Turn on PWM
        pwm_set_enabled(slice_r, true);
        pwm_set_enabled(slice_l, true);
    }

    // Input speed from -255 to 255
    void drive(int speed) {
        if (speed > 255) speed = 255;
        if (speed < -255) speed = -255;

        if (speed > 0) {
            pwm_set_chan_level(slice_r, chan_r, speed);
            pwm_set_chan_level(slice_l, chan_l, 0);
        } else if (speed < 0) {
            pwm_set_chan_level(slice_r, chan_r, 0);
            pwm_set_chan_level(slice_l, chan_l, -speed);
        } else {
            // Stop
            pwm_set_chan_level(slice_r, chan_r, 0);
            pwm_set_chan_level(slice_l, chan_l, 0);
        }
    }
};

#endif
