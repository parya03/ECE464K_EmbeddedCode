/**
 * Sets up FreeRTOS etc and starts the arm
 **/

// #include <iostream>

#include "FreeRTOS.h"

void vApplicationMallocFailedHook(void) {
    __asm("nop");
}

// int main() {
//     // TaskHandle_tArm_Control_Task;
//     // static StackType_t Arm_Control_stack[ configMINIMAL_STACK_SIZE ];
// }
