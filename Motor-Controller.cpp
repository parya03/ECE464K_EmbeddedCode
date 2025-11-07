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

extern float prev_angles[5];
extern float current_angles[5];
extern Servo base;
extern Servo arm1;
extern Servo arm2;
extern Servo wrist;
extern Servo gripper;

Servo motors[5] = {base, arm1, arm2, wrist, gripper};

int MotorControl_Task(void *pvParameters) {
    
    for(int i = 0; i < 5; i++) {
        float inc = 1.0f;
        float prev_angle = prev_angles[i];
        float current_angle = current_angles[i];
        if(prev_angle > current_angle) {
            inc = -1.0f;
        }
        Servo motor = motors[i];
        motor.setAngleRad(prev_angles[i] + inc);
    }

    prev_angles
   


    
}