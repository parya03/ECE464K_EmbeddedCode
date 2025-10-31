#include <stdio.h>
#include <stdlib.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <iostream>
// #include <chrono>
#include "Eigen/Dense"
#include "quik/geometry.hpp"
#include "quik/Robot.hpp"
#include "quik/IKSolver.hpp"
#include <cmath>

#define MAX_PWM 2500
#define MIN_PWM 500

#define US_PER_DEGREE (1000/90) // uS pulse per degree of movement
#define US_PER_RAD (1000/EIGEN_PI)

/**
 * Servo control class (PWM)
 * Servo gets centered with a 1500 uS pulse
 * Pulse ranges from 500-2500 uS corresponding to -180 - 180 degrees respectively
 */

class Servo {
    private:
        // Pulse width to set servo to zero (including offset)
        float zero_angle_offset_degrees = 0.0f;
        int zero_pw = 1500;

        float normalized_value = 0.0f;
        // float pwm_value = 0.0f;
        // Currently outputted PWM pulse width
        int curr_pwm_pw = 0;
        float rel_angle_deg = 0.0f; // Relative to zero offset

        int pin;
        int slice_num;
        int channel;
        bool inverted;
    
    public:
        explicit Servo(int pin) : Servo(pin, 0, 0) {}

        Servo(int pin, float angle_offset_degrees) : Servo(pin, angle_offset_degrees, 0) {}
        
        Servo(int pin, float angle_offset_degrees, bool invert) {
            this->pin = pin;
            
            inverted = invert;

            // initialize GPIO pins for PWM output
            gpio_init(pin);
            gpio_set_pulls(pin, false, true);
            gpio_set_dir(pin, GPIO_OUT);

            // Set up PWM on a specific GPIO pin
            gpio_set_function(pin, GPIO_FUNC_PWM);
            
            zero_angle_offset_degrees = angle_offset_degrees;
            zero_pw = 1500 + (angle_offset_degrees * US_PER_DEGREE);
            curr_pwm_pw = zero_pw;

            // Find out which PWM slice is connected to GPIOs
            slice_num = pwm_gpio_to_slice_num(pin);
            channel = pwm_gpio_to_channel(pin);

            // 1MHz PWM clock from divider
            pwm_set_clkdiv_int_frac(slice_num, SYS_CLK_HZ / 1000000, 0);

            // Set waveform period of 50 ms
            pwm_set_wrap(slice_num, 50000);

            // Center servo
            pwm_set_chan_level(slice_num, channel, curr_pwm_pw);
        }

        void print() {
            // char buf[100];

            // sprintf(buf, "Servo: Pin %d, PWM PW %d uS, angle %f deg, zero angle offset %f deg\n", pin, curr_pwm_pw, rel_angle_deg, zero_angle_offset_degrees);

            printf("Servo: Pin %d, PWM PW %d uS, rel angle %f deg, zero angle offset %f deg\n", pin, curr_pwm_pw, rel_angle_deg, zero_angle_offset_degrees);

            // return std::string(buf);
        }

        void startPWMControllers() {
            // Start the PWM
            pwm_set_enabled(slice_num, true);
        }

        void stopPWMControllers() {
            // Stop the PWM
            pwm_set_enabled(slice_num, false);
        }

        int computeNormalizedValue(float value) { // in radians from 
            normalized_value = value/(2*EIGEN_PI);
            int pwm_value = (int) (normalized_value * (MAX_PWM - MIN_PWM) + MIN_PWM);
            return pwm_value;
        }

        void setPWMValue(float value) {
            int pwm_value = computeNormalizedValue(value);
            pwm_set_chan_level(slice_num, channel, pwm_value);
        }

        void zero() {
            setAngleDegrees(0.0f);
        }

        void setAngleRad(float angle_rad) {
            float angle_deg = (angle_rad / EIGEN_PI) * 180.0f;

            setAngleDegrees(angle_deg);
        }

        // Total Angle = -180 - 180 taking offset into account
        void setAngleDegrees(float angle, bool isArm2=false) {
            rel_angle_deg = angle + zero_angle_offset_degrees;

            // printf("Angle requested: %f deg, after adding offset: %f deg\n", angle, rel_angle_deg);

            rel_angle_deg = fmodf(rel_angle_deg, 360.0f);

            // printf("Angle between -360 to 360: %f deg\n", rel_angle_deg);

            if(rel_angle_deg >= 270.0f && rel_angle_deg <= 360.0f) {
                rel_angle_deg -= 360.0f;
            }
            else if(rel_angle_deg <= -270.0f && rel_angle_deg >= -360.0f){
                rel_angle_deg += 360.0f;
            }
            else {
                // Not a valid angle -> cap it
                if(rel_angle_deg > 180 && rel_angle_deg < 270) {
                    rel_angle_deg = -90;
                }

                if(rel_angle_deg > 90 && rel_angle_deg <= 180) {
                    rel_angle_deg = 90;
                    if(isArm2) {
                        rel_angle_deg = 70;
                    }
                }

                if(rel_angle_deg >= -180 && rel_angle_deg < -90) {
                    rel_angle_deg = -90;
                }

                if(rel_angle_deg >= -270 && rel_angle_deg < -180) {
                    rel_angle_deg = 90;
                    if(isArm2) {
                        rel_angle_deg = 70;
                    }
                }
            }
            // if(rel_angle_deg < 0) {
            //     rel_angle_deg += 360.0f;
            // }
            // rel_angle_deg -= 180.0f;

            // printf("Angle in range -180 to 180: %f deg\n", rel_angle_deg);

            // if(rel_angle_deg < -90.0f) {
            //     rel_angle_deg = -90.0f; // range wrap between -90 to 90
            // }
            // if(rel_angle_deg > 90.0f) {
            //     rel_angle_deg = 90.0f; // range wrap between -90 to 90
            // }

            // printf("Angle in range -90 to 90: %f deg\n", rel_angle_deg);

            curr_pwm_pw = inverted ? 1500 + (rel_angle_deg * US_PER_DEGREE) : 1500 - (rel_angle_deg * US_PER_DEGREE);
            pwm_set_chan_level(slice_num, channel, curr_pwm_pw);
        }
};
