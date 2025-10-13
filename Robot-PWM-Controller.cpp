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

    PWMController() {

        // Initialize PWM hardware
        stdio_init_all();

        // Set up PWM on a specific GPIO pin
        gpio_set_function(2, GPIO_FUNC_PWM);
        gpio_set_function(3, GPIO_FUNC_PWM);
        gpio_set_function(4, GPIO_FUNC_PWM);

        // initialize GPIO pins for PWM output
        gpio_init(2);
        gpio_set_pulls(2, false, true);
        gpio_init(3);
        gpio_set_pulls(3, false, true);
        gpio_init(4);
        gpio_set_pulls(4, false, true);

        // Find out which PWM slice is connected to GPIOs
        uint slice_num_2 = pwm_gpio_to_slice_num(2);
        uint32_t channel_2 = pwm_gpio_to_channel(2);
        uint slice_num_3 = pwm_gpio_to_slice_num(3);
        uint32_t channel_3 = pwm_gpio_to_channel(3);
        uint slice_num_4 = pwm_gpio_to_slice_num(4);
        uint32_t channel_4 = pwm_gpio_to_channel(4);

         // 1MHz PWM clock from divider
        pwm_set_clkdiv_int_frac(slice_num_2, SYS_CLK_HZ / 1000000, 0);
        pwm_set_clkdiv_int_frac(slice_num_3, SYS_CLK_HZ / 1000000, 0);
        pwm_set_clkdiv_int_frac(slice_num_4, SYS_CLK_HZ / 1000000, 0);

        // Set period of 50 ms
        pwm_set_wrap(slice_num_2, 50000);
        // Center servo
        pwm_set_chan_level(slice_num_2, channel_2, 1500);

        // Set period of 50 ms
        pwm_set_wrap(slice_num_3, 50000);
        // Center servo
        pwm_set_chan_level(slice_num_3, channel_3, 1500);

        // Set period of 50 ms
        pwm_set_wrap(slice_num_4, 50000);
        // Center servo
        pwm_set_chan_level(slice_num_4, channel_4, 1500);

    }

    void startPWMControllers() {
        // Start the PWM
        pwm_set_enabled(slice_num_2, true);
        pwm_set_enabled(slice_num_3, true);
        pwm_set_enabled(slice_num_4, true);
    }

    int computeNormalizedValue(float value) { // in radians from 
        normalized_value = value/(2*M_PI);
        int pwm_value = (int) (normalized_value * (MAX_PWM - MIN_PWM) + MIN_PWM);
        return pwm_value;
    }

    void setPWMValue(float value, uint slice_num, uint32_t channel) {
        int pwm_value = computeNormalizedValue(value);
        pwm_set_chan_level(slice_num, channel, pwm_value);
    }

}