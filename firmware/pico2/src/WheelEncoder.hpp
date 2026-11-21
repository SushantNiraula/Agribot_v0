#ifndef WHEEL_ENCODER_HPP
#define WHEEL_ENCODER_HPP

#include "pico/stdlib.h"
#include "hardware/gpio.h"

class WheelEncoder {
private:
    uint pin_a;
    uint pin_b;
    volatile int32_t ticks;
    bool flip_direction;

public:
    WheelEncoder(uint gpio_a, uint gpio_b, bool flip = false) 
        : pin_a(gpio_a), pin_b(gpio_b), ticks(0), flip_direction(flip) {}

    void init() {
        gpio_init(pin_a);
        gpio_set_dir(pin_a, GPIO_IN);
        gpio_pull_up(pin_a);
        
        gpio_init(pin_b);
        gpio_set_dir(pin_b, GPIO_IN);
        gpio_pull_up(pin_b);
    }

    // Called automatically by the GPIO interrupt
    void handle_interrupt() {
        bool state_a = gpio_get(pin_a);
        bool state_b = gpio_get(pin_b);

        if (state_a != state_b) {
            flip_direction ? ticks-- : ticks++;
        } else {
            flip_direction ? ticks++ : ticks--;
        }
    }

    int32_t get_ticks() {
        return ticks;
    }
};

#endif
