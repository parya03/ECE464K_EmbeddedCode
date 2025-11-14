#include <stdio.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <iostream>         
// #include <chrono>
#include "Eigen/Dense"
#include <math.h>
#include <sys/cdefs.h>
#include "projdefs.h"
#include "quik/geometry.hpp"
#include "quik/Robot.hpp"
#include "quik/IKSolver.hpp"
#include "Servo.hpp"
#include "FreeRTOS.h"
#include "robot.pb.h"
#include "stream_buffer.h"
#include "Communication.hpp"
#include <algorithm>
#include "Eigen/Dense"
// #include <queue>
#include <array>

#define STEP_SIZE 20
#define STEP_SIZE_MAX 20
#define STEP_SIZE_MIN 0
#define STEP_DELTA 5

using JointArray = std::array<double, 5>;
using JointArrayInt = std::array<int, 5>;

// Motor angles
// base arm1 arm2 pitch gripper_angle
JointArrayInt prev_pwm{1500, 1500, 1500, 1500, 1500};
// extern std::queue<JointArray> motor_angles_queue;
extern JointArray current_angles;

extern Servo base;
extern Servo arm1;
extern Servo arm2;
extern Servo wrist;
extern Servo gripper;

Servo* motors[5] = {&base, &arm1, &arm2, &wrist, &gripper};
int starting_pwm[3] = {1500, 1500, 1500};
int step_size[3] = {STEP_SIZE_MIN, STEP_SIZE_MIN, STEP_SIZE_MIN};
int step_increasing[3] = {true, true, true};

void MotorUpdate() {
    //printf("thread for motors\n");
    // JointArray curr_angles = motor_angles_queue.front();
    bool converged[5] = {false, false, false, false, false};

    // printf("=======================================\n");
    
    for(int i = 0; i < 3; i++) {
        Servo& motor = *motors[i];
        int pwm = motor.computePWMRad(current_angles[i]);
        // printf("Joint angle for motor %d: %f\n", i, current_angles[i]);
        int diff = prev_pwm[i] - pwm;

        if(diff == 0) {
            converged[i] = true;
            starting_pwm[i] = pwm;
            continue;
        }

        int direction = (diff > 0) ? 1 : -1;
        int remaining = std::abs(diff);
        int this_step = step_size[i];
        int difference = std::abs(pwm - starting_pwm[i]);

        if(difference >= 100) {
            // cap if step size is more
            if(this_step > remaining) {
                this_step = remaining;
            }

            // move one tick
            prev_pwm[i] += direction * this_step;
            motor.setPWM(prev_pwm[i]);

            // Check if we've reached the target for this motor
            if (prev_pwm[i] == pwm) {
                converged[i] = true;
                starting_pwm[i] = pwm;
            }

            // --- Update step size (accel then decel) ---
            if (step_increasing[i]) {
                step_size[i] += STEP_DELTA;
                if (step_size[i] >= STEP_SIZE_MAX) {
                    step_size[i] = STEP_SIZE_MAX;
                    step_increasing[i] = false;   // start decreasing on future calls
                }
            } else {
                step_size[i] -= STEP_DELTA;
                if (step_size[i] <= STEP_SIZE_MIN) {
                    step_size[i] = STEP_SIZE_MIN;
                    step_increasing[i] = true;    // start increasing again
                }
            }
        }
        else if (difference >= 10) {
                        
        }



    }
    // check if all motors converged to the desired angles
    bool allMotorsConverged = true;

    for(int i = 0; i < 3; i++) {
        if(converged[i] == false) {
            allMotorsConverged = false;
        }
        else {
            // printf("Angle converged for motor %d\n", i);
        }
    }
    
    // if all converged to desired angle, pop from queue
    if(allMotorsConverged){
        // motor_angles_queue.pop();
        printf("\nCONVERGENCE COMPLETED FOR ALL MOTORS!!\n\n");
    }
}
