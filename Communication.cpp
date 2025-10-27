/**
 * Communication driver for getting data from PC
 **/

#include "pb.h"
#include "pb_decode.h" 
#include "robot.pb.h"
#include <stdio.h>

void decode_message(uint8_t in_buffer[128]) {
    pb_istream_t istream = pb_istream_from_buffer(in_buffer, 128);
    
    handtracking_HandData message = handtracking_HandData_init_zero; // Allocate stack space
    auto status = pb_decode(&istream, handtracking_HandData_fields, &message);

    if(status) {
        printf("Error decoding message!\n");
    }
}
