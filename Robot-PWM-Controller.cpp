#include <stdio.h>
#include <stdlib.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <iostream>
// #include <chrono>
#include "Eigen/Dense"
#include <math.h>
#include "quik/geometry.hpp"
#include "quik/Robot.hpp"
#include "quik/IKSolver.hpp"

class PWMController {

    #define MAX_PWM 3000
    #define MIN_PWM 1000

    double normalized_value = 0.0;
    double pwm_value = 0.0;

    constexpr slice_num;
    constexpr channel;

    PWMController(constexpr pin) {

        // Initialize PWM hardware
        stdio_init_all();

        // Set up PWM on a specific GPIO pin
        gpio_set_function(pin, GPIO_FUNC_PWM);

        // initialize GPIO pins for PWM output
        gpio_init(pin);
        gpio_set_pulls(pin, false, true);

        // Find out which PWM slice is connected to GPIOs
        slice_num = pwm_gpio_to_slice_num(pin);
        channel = pwm_gpio_to_channel(pin);

         // 1MHz PWM clock from divider
        pwm_set_clkdiv_int_frac(slice_num, SYS_CLK_HZ / 1000000, 0);

        // Set period of 50 ms
        pwm_set_wrap(slice_num, 50000);
        // Center servo
        pwm_set_chan_level(slice_num, channel, 1500);

    }

    void startPWMControllers() {
        // Start the PWM
        pwm_set_enabled(slice_num, true);
    }

    int computeNormalizedValue(float value) { // in radians from 
        normalized_value = value/(2*M_PI);
        int pwm_value = (int) (normalized_value * (MAX_PWM - MIN_PWM) + MIN_PWM);
        return pwm_value;
    }

    void setPWMValue(float value) {
        int pwm_value = computeNormalizedValue(value);
        pwm_set_chan_level(slice_num, channel, pwm_value);
    }

}