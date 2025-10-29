/**
 * Header for USB (UART) communication with the system
 **/

#ifndef COMMUNICATION_HPP
#define COMMUNICATION_HPP

#include "pb.h"
#include "pb_decode.h" 
#include "projdefs.h"
#include "robot.pb.h"
#include <pico/error.h>
#include <pico/stdio.h>
#include <stdio.h>
#include <cstdint>
#include "pico/stdio_usb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stream_buffer.h"

#define INPUT_BUFFER_SIZE 255 // Bytes 
#define HANDDATA_SB_SIZE (8 * sizeof(handdata_t)) // Stream buffer size must be minimum sizeof(handdata_t)
#define DATA_INPUT_TIME_MS 10 // Time per data input sample over UART

// Buffer used to send data to IK
extern StreamBufferHandle_t communication_stream_buf;

// Hand data struct definition directly from .proto file
typedef struct {
  double timestamp;
  float x;
  float y;
  float z;
  float openness;
  float pitch;
} handdata_t;

void Decode_Task(void *pvParameters);

#endif
