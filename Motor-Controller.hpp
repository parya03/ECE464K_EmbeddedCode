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

#define DEBUG 0

// === externed variables ====
extern double current_angles[5];
extern Servo base;
extern Servo arm1;
extern Servo arm2;
extern Servo wrist;
extern Servo gripper;
//==============================

// == PWM smoothing variables ==
Servo* motors[3] = {&base, &arm1, &arm2};

// using an integration type smooting
struct CubicProfile {
    float start_pwm;
    float target_pwm;
    float t;
    float duration;
};

static CubicProfile profiles[3];
static float current_pwm_f[3] = {1500.0f, 1500.0f, 1500.0f};
static int current_pwm[3] = {1500, 1500, 1500};
static bool initialized = false;

static float global_time = 0.0f;
const float dt = 0.01f; // RTOS thread frequency (10 ms)
const float baseDur = 0.03f; // increasing (more slow and smooth), decreasing (faster and aggresive)
const float minDur      = 0.10f; // min time for smallest moves - increasing (makes even small moves take longer but fluid)
const float maxDeltaPWM = 400.0f; // for scaling - increasing (small scale but faster), decreasing (slower)
const float target_eps  = 5.0f; // increasing (ignores tiny changes, decreases resolution), decrease (could be jittery but tracks tiny changes)

// ==============================

void MotorUpdate() {
    
    global_time += dt;

    if(!initialized) {
        for (int i = 0; i < 3; ++i) {
            profiles[i].start_pwm  = current_pwm_f[i];
            profiles[i].target_pwm = current_pwm_f[i];
            profiles[i].t          = 0.0f;
            profiles[i].duration   = 0.001f;
        }
        initialized = true;
    }

    bool converged[3] = {false, false, false};
    
    for(int i = 0; i < 3; i++) {

        // receive latest target angle
        Servo& motor = *motors[i];
        int target_pwm = motor.computePWMRad(current_angles[i]);

        // check if target changed enough to start a new cubic segment
        float diff = target_pwm - profiles[i].target_pwm;
        if(fabsf(diff) > target_eps) {
            profiles[i].start_pwm = current_pwm_f[i];
            profiles[i].target_pwm = (float) target_pwm;
            profiles[i].t = 0.0f;

            // duration is scaling with move size
            float dist = fabsf(profiles[i].target_pwm - profiles[i].start_pwm);
            float scale = dist/maxDeltaPWM;

            // clamp the scale factors
            if(scale < 0.0f) scale = 0.0f;
            if(scale > 1.5f) scale = 1.5f;

            // amount of time to take to go from start to target pwm
            // dynamically calculted depending on how near or far target is from start position
            profiles[i].duration = minDur + baseDur * scale;
        }

        CubicProfile &p = profiles[i];
        p.t += dt;
        if(p.t > p.duration) p.t = p.duration;
        if(p.duration <= 0.0f || p.t >= p.duration) {
            current_pwm_f[i] = p.target_pwm;
            converged[i] = true;
        }
        else {
            float u = p.t / p.duration;
            float s = 3*u*u - 2*u*u*u; // cubic function
            current_pwm_f[i] = p.start_pwm + s * (p.target_pwm - p.start_pwm);
        }

        // send to servo
        current_pwm[i] = (int)current_pwm_f[i];
        motor.setPWM(current_pwm[i]);

        #if DEBUG
            if (i == 2) {   // motor 0 for now
                // t,cur,target,s
                printf("%.4f,%.1f,%.1f\n",global_time, current_pwm_f[i], p.target_pwm);     
            }
        #else
            printf("Motor %d -> start=%.1f target=%.1f cur=%.1f dur=%.3f t=%.3f\n",
                i, p.start_pwm, p.target_pwm, current_pwm_f[i], p.duration, p.t);
        #endif

    }
    // check if all motors converged to the desired angles
    bool allMotorsConverged = converged[0] && converged[1] && converged[2];
    
    // if all converged to desired angle, pop from queue
    if(allMotorsConverged){
        // motor_angles_queue.pop();
        #if DEBUG == 0
            printf("\nCONVERGENCE COMPLETED FOR ALL MOTORS!!\n\n");
        #endif
    }
}

