/**
 * Sets up FreeRTOS etc and starts the arm
 **/

#include <stdio.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include <iostream>
#include "Communication.hpp"
#include "Motor-Controller.cpp"

// #include <iostream>

#include "FreeRTOS.h"
#include "task.h"

void vApplicationMallocFailedHook(void) {
    __asm("nop");
}

#define MOTOR_CTRL_PERIOD_MS 100   // run every 100 ms

extern void RobotArm_Task(void *pvParameters);
extern void MotorUpdate();
TaskHandle_t RobotArm_Task_handle = NULL;
TaskHandle_t Decode_Task_handle = NULL;
TaskHandle_t PWM_Sweep_handle = NULL;

void MotorControl_Task(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(MOTOR_CTRL_PERIOD_MS);

    for (;;)
    {
        xLastWakeTime = xTaskGetTickCount();

        // printf("thread for motors\n");
        // Update motors
        MotorUpdate();

        // Block until the next cycle
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }

}


int main() {
    // TaskHandle_tArm_Control_Task;
    // static StackType_t Arm_Control_stack[ configMINIMAL_STACK_SIZE ];
    
    stdio_init_all();
    // while(1) {
    //     printf("Works yippee!\n");
    // }
    xTaskCreate(RobotArm_Task, "RobotArm-Task", 10240, NULL, 1, &RobotArm_Task_handle);
    vTaskCoreAffinitySet(RobotArm_Task_handle, 0x2); // Run this on second core exclusively
    xTaskCreate(Decode_Task, "Decode-Task", 10240, NULL, 1, &Decode_Task_handle);
    vTaskCoreAffinitySet(Decode_Task_handle, 0x1); // Run this on first core exclusively
    xTaskCreate(MotorControl_Task, "MotorSweep-Task", 10240, NULL, 1, &PWM_Sweep_handle);
    vTaskCoreAffinitySet(PWM_Sweep_handle, 0x1);
    
                                                     
    vTaskStartScheduler();

    vTaskDelete(NULL);
}
