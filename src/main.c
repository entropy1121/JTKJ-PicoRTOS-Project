/*
 * Course Project: Morse Code Transmitter
 * ==============================================================================
 * AI TOOL USAGE DECLARATION
 * ==============================================================================
 * AI Tool: Google Gemini
 * Prompt: "Write a C FreeRTOS program for Pico W to read IMU sensor and send 
 * Morse code to USB. Use SW1 for space/enter and SW2 for dot/dash."
 * * Modifications by Student:
 * 1. I merged the initialization code into main for simplicity.
 * 2. Changed the variable names to be easier to understand.
 * 3. Implemented the specific protocol logic (dot/dash/space/end) inside the sensor task.
 * 4. Adjusted stack size to 4096 because printf was crashing the board.
 * 5. Used the specific TKJHAT SDK functions for the IMU and Buttons.
 * ==============================================================================
 */

#include <stdio.h>
#include <math.h> 
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>
#include "tkjhat/sdk.h"

#define STACK_SIZE 1024   //stack size
#define THRESHOLD 0.7f   //to check the device is flat or vertical

// Handles
QueueHandle_t myQueue;//connect sensor_task and print_task
SemaphoreHandle_t buttonSemaphore;//the synchronization mechanism that connect sersor_task and interrupt

volatile int button_pressed = 0; //record which button has been pressed, 1=S1 (space and \n), 2 = SW2 (dot and dash)

// Interrupt handler for buttons
static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;//to check is there a higher priority task
    
    if (gpio == SW2_PIN) {
        button_pressed = 2; 
        xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
    } 
    else if (gpio == SW1_PIN) {
        button_pressed = 1; 
        xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);//make sure device process the higher priority task first
}

// Task to read sensor and decide what to send
static void sensor_task(void *arg) {
    (void)arg;
    
    float ax, ay, az, gx, gy, gz, temp;
    char msg = 0;

    while (1) {
        // Wait here until button is pressed
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {//check if the buttons be pressed
            
            // Wait a bit for button bounce
            vTaskDelay(pdMS_TO_TICKS(50));

            // Check if button is really pressed
            uint pin = (button_pressed == 1) ? SW1_PIN : SW2_PIN;
            
            if (gpio_get(pin)) {
                // Read the sensor data
                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) == 0) {
                    
                    // printf("ax: %f, az: %f\n", ax, az); // debug
                    
                    bool flat = (fabsf(az) > THRESHOLD);//device is flat
                    bool vertical = (fabsf(ax) > THRESHOLD || fabsf(ay) > THRESHOLD);//device is vertical

                    if (button_pressed == 2) { //if S2 be pressed
                        if (flat) { 
                            msg = '.'; //message is dot
                            blink_led(1); 
                        } 
                        else if (vertical) {
                            msg = '-'; //message is dash
                            // LED feedback
                            set_led_status(true);
                            vTaskDelay(pdMS_TO_TICKS(200));
                            set_led_status(false);
                        }
                    } 
                    else if (button_pressed == 1) { //if S1 be pressed
                        if (flat) { //message is space
                            msg = ' '; 
                            blink_led(2);
                        } 
                        else if (vertical) {
                            msg = '\n'; //message is \n
                            blink_led(3);
                        }
                    }

                    if (msg != 0) {
                        xQueueSend(myQueue, &msg, 0); //send the message to print task
                        msg = 0; // initial message
                    }
                }
            }
            
            vTaskDelay(pdMS_TO_TICKS(200)); //avoid press button too fast
        }
    }
}

// Task to print to USB
static void print_task(void *arg) {
    (void)arg;
    char data;

    while (1) {
        if (xQueueReceive(myQueue, &data, portMAX_DELAY) == pdTRUE) { //check if there is data in 
            //message protocal
            if (data == '.') {
                printf(".");
            } 
            else if (data == '-') {
                printf("-");
            } 
            else if (data == ' ') {
                printf(" ");
            } 
            else if (data == '\n') {
                printf("  \n"); //end message:two space and \n
            }
            
            fflush(stdout); //clear the buffer
        }
    }
}

int main() {
    stdio_init_all();
    // sleep_ms(1000); // Wait for USB

    //Initialize the device
    init_hat_sdk();
    sleep_ms(300);
    init_led();
    init_sw1(); 
    init_sw2();

    //Initialize the device
    int ret = init_ICM42670();
    if (ret == 0) {
        ICM42670_start_with_default_values();
    } else {
        printf("IMU Failed!\n");
    }

    // Create Queue and Semaphore
    myQueue = xQueueCreate(20, sizeof(char));
    buttonSemaphore = xSemaphoreCreateBinary();

    // Setup GPIO interrupts
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(SW2_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // Create sersor_task and print_task
    xTaskCreate(sensor_task, "Sensor", STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(print_task, "Print", STACK_SIZE, NULL, 1, NULL);
    
    //Start freeRTOS
    vTaskStartScheduler();

    //while(1);
    return 0;
}