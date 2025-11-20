
/*
   use of AI:
   1.AI tool:gemini
     prompt:how to check device is flat or vertical
     how to modify: I test the device and changed the check point of flat or vertical position to 0.7f.
                    I add logic to connect dot and dash to the position of device in the loop.
                    I use ICM42670_read_sensor_data function to read the sensor data.
   2.AI tool:gemini
     prompt:How to use state machine to process button
     how to modify:I add logic of button 1 and 2 to change the state
                   I use xSemaphoreGiveFromISR to synchronize interrupt and tasks.
   3.AI tool:deepseek
     prompt:
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

// --- State Machine Definition (Based on Course Material) ---
// Introducing state as per lecture example
enum state { IDLE=1, TYPING, CONTROL };

// Global state variable, initialized to waiting state
//volatile 
enum state myState = IDLE;

// Handles
QueueHandle_t myQueue;//connect sensor_task and print_task
SemaphoreHandle_t buttonSemaphore;//the synchronization mechanism that connect sersor_task and interrupt

// Interrupt handler for buttons
static void btn_fxn(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;//to check is there a higher priority task
    
    // We change the state to a desired one
    // If-clause is used to check that the state transition is possible (Legality Check)
    if (myState == IDLE) {
        if (gpio == SW2_PIN) {
            // State transition IDLE -> TYPING (SW2 pressed)
            myState = TYPING;
            xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
        } 
        else if (gpio == SW1_PIN) {
            // State transition IDLE -> CONTROL (SW1 pressed)
            myState = CONTROL;
            xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriorityTaskWoken);
        }
    }
}

// Task to read sensor and decide what to send
static void sensor_task(void *arg) {
    (void)arg;
    
    //Initialize the device
    init_hat_sdk();
    sleep_ms(300);
    init_led();
    init_sw1(); 
    init_sw2();

    int ret = init_ICM42670();
    if (ret == 0) {
        ICM42670_start_with_default_values();
    } else {
        printf("IMU Failed!\n");
    }

    // Setup GPIO interrupts
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled_with_callback(SW2_PIN, GPIO_IRQ_EDGE_RISE, true, btn_fxn);

    float ax, ay, az, gx, gy, gz, temp;
    char msg = 0;

    while (1) {
        // Wait here until button is pressed
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {   //check if the buttons be pressed
            
            // Wait a bit for button bounce
            vTaskDelay(pdMS_TO_TICKS(50));

           
            
            
                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) == 0) {
                    
                    //printf("ax: %f, az: %f\n", ax, az); // debug
                    
                    bool flat = (fabsf(az) > THRESHOLD);//device is flat
                    bool vertical = (fabsf(ax) > THRESHOLD || fabsf(ay) > THRESHOLD);//device is vertical

                    // State Machine Logic
                    if (myState == TYPING) { // Functionality of TYPING state
                        if (flat) { 
                            msg = '.'; //message is dot
                            blink_led(1); 
                        } 
                        else if (vertical) {
                            msg = '-'; //message is dash
                            // LED feedback
                            blink_led(2);
                        }
                        // State transition TYPING -> IDLE
                        myState = IDLE;
                    } 
                    else if (myState == CONTROL) { // Functionality of CONTROL state
                        if (flat) { //message is space
                            msg = ' '; 
                            blink_led(3);
                        } 
                        else if (vertical) {
                            msg = '\n'; //message is \n
                            blink_led(4);
                        }
                        myState = IDLE;
                    }

                    if (msg != 0) {
                        xQueueSend(myQueue, &msg, 0); //send the message to print task
                        msg = 0; // initialize message
                    }
                }
            //}
            else {
                // If button was released too fast or bounce, return to IDLE
                myState = IDLE;
            }
            
            vTaskDelay(pdMS_TO_TICKS(200)); //avoid press button too fast
        }
    }
}

// Task to translate the message by protocal then print the message
static void print_task(void *arg) {
    (void)arg;
    char data;

    while (1) {
        if (xQueueReceive(myQueue, &data, portMAX_DELAY) == pdTRUE) { //check if there is data in queue
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
            
            
        }
    }
}

int main() {
    stdio_init_all();
    

    

    // Create Queue and Semaphore
    myQueue = xQueueCreate(20, sizeof(char));
    buttonSemaphore = xSemaphoreCreateBinary();

    // Create sersor_task and print_task
    xTaskCreate(sensor_task, "Sensor", STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(print_task, "Print", STACK_SIZE, NULL, 1, NULL);
    
    //Start freeRTOS
    vTaskStartScheduler();

    //while(1);
    return 0;
}
