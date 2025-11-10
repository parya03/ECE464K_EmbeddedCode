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
#include <queue>
#include <array>

#define STEP_SIZE 10

using JointArray = std::array<float, 5>;
using JointArrayInt = std::array<int, 5>;

// Motor angles
// base arm1 arm2 pitch gripper_angle
JointArrayInt prev_pwm{1500, 1500, 1500, 1500, 1500};
extern std::queue<JointArray> motor_angles_queue;
extern JointArray current_angles;

extern Servo base;
extern Servo arm1;
extern Servo arm2;
extern Servo wrist;
extern Servo gripper;

Servo motors[5] = {base, arm1, arm2, wrist, gripper};

void MotorUpdate() {
    printf("thread for motors\n");
    JointArray curr_angles = motor_angles_queue.front();
    bool converged[5] = {false, false, false, false, false};

    printf("=======================================\n");
    
    for(int i = 0; i < 5; i++) {
        Servo motor = motors[i];
        int pwm = motor.computePWM(current_angles[i]);
        printf("Joint angle for motor %d: %f\n", i, current_angles[i]);
        int diff = prev_pwm[i] - pwm;
        if(diff > 0) {
            if(diff > STEP_SIZE) {
                motor.setPWM(prev_pwm[i] - STEP_SIZE);
                prev_pwm[i] = prev_pwm[i] - STEP_SIZE;
            }
            else {
                motor.setPWM(prev_pwm[i] - diff);
                prev_pwm[i] = prev_pwm[i] - diff;
            }
            
            printf("DECREMENT: Prev pwm: %d, Current pwm: %d for motor %d\n", prev_pwm[i], pwm, i);
        }
        else if(diff < 0) {
            if(diff < -STEP_SIZE) {
                motor.setPWM(prev_pwm[i] + STEP_SIZE);
                prev_pwm[i] = prev_pwm[i] + STEP_SIZE;
            }
            else {
                motor.setPWM(prev_pwm[i] - diff);
                prev_pwm[i] = prev_pwm[i] - diff;
            }
            printf("INCREMENT: Prev pwm: %d, Current pwm: %d for motor %d\n", prev_pwm[i], pwm, i);
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
        motor_angles_queue.pop();
        printf("\nCONVERGENCE COMPLETED FOR ALL MOTORS!!\n\n");
    }
}