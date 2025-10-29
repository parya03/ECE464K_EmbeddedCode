/**
 * Communication driver for getting data from PC
 **/

#include "Communication.hpp"
#include <hardware/sync.h>
#include <pico/error.h>
#include <pico/stdio.h>
// #include "projdefs.h"
// #include "robot.pb.h"

StreamBufferHandle_t communication_stream_buf = NULL;

static int decode_message(uint8_t in_buffer[INPUT_BUFFER_SIZE], size_t message_size_bytes, handdata_t *data_struct) {
    pb_istream_t istream = pb_istream_from_buffer(in_buffer, message_size_bytes);
    
    handtracking_HandData message = handtracking_HandData_init_zero; // Allocate stack space
    auto status = pb_decode(&istream, handtracking_HandData_fields, &message);
    if(!status) {
        printf("Error decoding protobuf message: %s\n", istream.errmsg);
        goto decode_return;
    }

    data_struct->timestamp = message.timestamp;
    data_struct->x = message.x;
    data_struct->y = message.y;
    data_struct->z = message.z;
    data_struct->openness = message.openness;
    data_struct->pitch = message.pitch;
    
    decode_return:
    return status;
}

// // UART interrupts used to read input data
// void uart_rx_interrupt() {
//     while (uart_is_readable(uart0)) {
//         uint8_t ch = uart_getc(uart0);
//
//         if(ch == '0') {
//             run_state = false;
//         }
//         if(ch == '1') {
//             run_state = true;
//         }
//         // Can we send it back?
//         if (uart_is_writable(uart0)) {
//             // Change it slightly first!
//             ch++;
//             uart_putc(uart0, ch);
//         }
//         // chars_rxed++;
//     }
// }

void Decode_Task(void *pvParameters) {
    // Input data buffer
    signed char input_data[INPUT_BUFFER_SIZE];
    int input_data_size = 0;

    // Set up UART interrupt, data buffer, etc                                t
    
    // Set up stream buffer
    // The stream buffer is an RTOS-controlled buffer to handle sending data between threads (this to IK thread)
    communication_stream_buf = xStreamBufferCreate(sizeof(handdata_t), sizeof(handdata_t));

    // Wait until UART available, sleep block if not 
    while(!stdio_usb_connected()) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    handdata_t handdata_temp = { 0 };

    while(1) {
        // Main loop
        
        // Grab input if it isn't EOF
        // TODO this is kinda jank so maybe fix it??
        input_data_size = 0;
        input_data[input_data_size] = stdio_getchar();
        if(input_data[input_data_size] == EOF) {
            vTaskDelay(pdMS_TO_TICKS(DATA_INPUT_TIME_MS));
            continue; // Still no input so start again
        }
        printf("Hit\n");
        while(input_data[input_data_size] != PICO_ERROR_TIMEOUT) {
            input_data_size++;
            if(input_data_size >= INPUT_BUFFER_SIZE) {
                printf("Serial data buffer overflowed!\n");
                // __asm("div r0, r0, #0"); // Cursed but functional way to force an exception
                while(1) {
                    disable_interrupts();
                    __asm("nop"); // Keep here so we don't have to keep unplugging the thing to force into BOOTSEL
                }
            }
            // printf("Char: %c, Data size: %d\n", input_data[input_data_size], input_data_size);
            input_data[input_data_size] = stdio_getchar_timeout_us(2000); // Wait a ms for new chars since serial baud rate may be slow
        }

        // input_data_size = stdio_get_until(input_data, INPUT_BUFFER_SIZE, 0);
        // if(input_data_size == PICO_ERROR_TIMEOUT) {
        //     // No data available yet, delay a bit and go back to top
        //     vTaskDelay(pdMS_TO_TICKS(DATA_INPUT_TIME_MS));
        //     continue;
        // }

        printf("USB input size: %d\n", input_data_size);

        auto status = decode_message(reinterpret_cast<uint8_t *>(input_data), input_data_size, &handdata_temp);
        if(!status) {
            printf("Error decoding message, status %d\n", status);
            continue;
        }

        printf("Handdata timestamp: %f\n", handdata_temp.timestamp);

        auto sb_status = xStreamBufferSend(communication_stream_buf, &handdata_temp, sizeof(handdata_t), pdMS_TO_TICKS(50));
        if(!sb_status) {
            printf("Error sending message thru stream buffer, status %d\n", sb_status);
        }
    }
}
