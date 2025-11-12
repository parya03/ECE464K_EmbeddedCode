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

#define STEP_SIZE 10
#define STEP_SIZE_MAX 10
#define STEP_SIZE_MIN 8
#define LIMIT_TO_DECEL 25

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

void MotorUpdate() {
    //printf("thread for motors\n");
    // JointArray curr_angles = motor_angles_queue.front();
    bool converged[5] = {false, false, false, false, false};

    // printf("=======================================\n");
    
    for(int i = 0; i < 5; i++) {
        Servo& motor = *motors[i];
        int pwm = motor.computePWMRad(current_angles[i]);
        // printf("Joint angle for motor %d: %f\n", i, current_angles[i]);
        int diff = prev_pwm[i] - pwm;
        if(diff > 0) {
            if(diff > LIMIT_TO_DECEL) {
                motor.setPWM(prev_pwm[i] - STEP_SIZE_MAX);
                prev_pwm[i] = prev_pwm[i] - STEP_SIZE_MAX;
            }
            else {
                printf("Decelerating\n");
                if(diff > STEP_SIZE_MIN) {
                    motor.setPWM(prev_pwm[i] - STEP_SIZE_MIN);
                    prev_pwm[i] = prev_pwm[i] - STEP_SIZE_MIN;
                }
                else {
                    motor.setPWM(prev_pwm[i] - diff);
                    prev_pwm[i] = prev_pwm[i] - diff;
                }
                
            }
            
            printf("DECREMENT: Current pwm: %d, Target pwm: %d for motor %d\n", prev_pwm[i], pwm, i);
        }
        else if(diff < 0) {
            if(diff < -LIMIT_TO_DECEL) {
                motor.setPWM(prev_pwm[i] + STEP_SIZE_MAX);
                prev_pwm[i] = prev_pwm[i] + STEP_SIZE_MAX;
            }
            else {
                printf("Decelerating\n");
                if(diff < -STEP_SIZE_MIN) {
                    motor.setPWM(prev_pwm[i] + STEP_SIZE_MIN);
                    prev_pwm[i] = prev_pwm[i] + STEP_SIZE_MIN;
                }
                else {
                    motor.setPWM(prev_pwm[i] - diff);
                    prev_pwm[i] = prev_pwm[i] - diff;
                }
                
            }
            printf("INCREMENT: Current pwm: %d, Target pwm: %d for motor %d\n", prev_pwm[i], pwm, i);
        }
        else {
            // angle converged to given angle
            converged[i] = true;
        }

    }
    // check if all motors converged to the desired angles
    bool allMotorsConverged = true;

    for(int i = 0; i < 5; i++) {
        if(converged[i] == false) {
            allMotorsConverged = false;
        }
        else {
            printf("Angle converged for motor %d\n", i);
        }
    }
    
    // if all converged to desired angle, pop from queue
    if(allMotorsConverged){
        // motor_angles_queue.pop();
        printf("\nCONVERGENCE COMPLETED FOR ALL MOTORS!!\n\n");
    }
}
