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


using JointArray = std::array<float, 5>;
using JointArrayInt = std::array<float, 5>;

// Motor angles
// base arm1 arm2 pitch gripper_angle
JointArrayInt prev_pwm{0, 0, 0, 0, 0};
extern std::queue<JointArray> motor_angles_queue;

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

    
    for(int i = 0; i < 5; i++) {
        Servo motor = motors[i];
        int pwm = motor.computePWM(curr_angles[i]);
        if(prev_pwm[i] > pwm) {
            motor.setPWM(prev_pwm[i] - 1);
            prev_pwm[i] = prev_pwm[i] - 1;
            printf("DECREMENET: Prev angle: %f, Current angle: %f for motor %d\n", prev_pwm[i], curr_angles[i], i);
        }
        else if(prev_pwm[i] < pwm) {
            motor.setPWM(prev_pwm[i] + 1);
            prev_pwm[i] = prev_pwm[i] + 1;
            printf("INCREMENT: Prev angle: %f, Current angle: %f for motor %d\n", prev_pwm[i], curr_angles[i], i);
        }
        else {
            // angle converged to given angle
            converged[i] = true;
            printf("Angle converged for motor %d\n", i);
        }

    }
    // check if all motors converged to the desired angles
    bool allMotorsConverged = true;

    for(int i = 0; i < 5; i++) {
        if(converged[i] == false) {
            allMotorsConverged = false;
        }
    }
    
    // if all converged to desired angle, pop from queue
    if(allMotorsConverged){
        motor_angles_queue.pop();
        printf("\nCONVERGENCE COMPLETE, POPPED NEW!\n\n");
    }
}