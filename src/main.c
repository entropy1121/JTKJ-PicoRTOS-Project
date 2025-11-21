/*
   We want to divide 6 points like this:
   Ziteng: 3 points
   Xiaoya: 1.5 points
   Qihao:  1.5 points

   Group members:
        Qihao Zhao: write the interrupt, test the threshold to check the position of the device
        Xiaoya Gao: write print_task, joint the videos together, test the result
        Ziteng Zhao: write sensor_task, intergrate and debug the whole program, modify the sdk.c and Cmakelist

   use of AI:
   1.AI tool:gemini
     prompt:how to check device is flat or vertical
     how to modify: I test the device and changed the threshold of flat or vertical position to 0.7f.
                    I add logic to connect dot and dash to the position of device in the loop.
                    I use ICM42670_read_sensor_data function to read the sensor data.
   2.AI tool:gemini
     prompt:How to use state machine to process interrupt of button
     how to modify:I implemented myState
                   I added logic of button 1 and 2 to change the state
                   I used xSemaphoreGiveFromISR to send semaphore to sensor task
   3.AI tool:deepseek
     prompt:How to synchronize interrupt and FreeRTOS task
     how to modify:I include semphr.h
                   I create buttonSemaphore then use xSemaphoreTake receive semaphore from interrupt
   4.AI tool:gemini
     prompt:how to avoid unexpected message because of too fast action
     how to modify:I use vTaskDelay(pdMS_TO_TICKS(150)) in sensor_task
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
#define THRESHOLD 0.7    //to check the device is flat or vertical

enum state { IDLE=1, TYPING, CONTROL };//TYPING mode is for dot and dash, control mode is for space and new li
enum state myState = IDLE;             //initialize state as IDLE

QueueHandle_t myQueue;                 //connect sensor_task and print_task
SemaphoreHandle_t buttonSemaphore;     //the synchronization mechanism that connect sersor_task and interrupt

// Interrupt handler for buttons
static void btn_fxn(uint gpio, uint32_t events) {
    BaseType_t xHigherPriority = pdFALSE; //to check is there a higher priority task
    
    if (myState == IDLE) {
        if (gpio == SW2_PIN) {
            // when we press button2 the state change to TYPING
            myState = TYPING;
            xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriority);
        } 
        else if (gpio == SW1_PIN) {
            // when we press button1 the state change to CONTROL
            myState = CONTROL;
            xSemaphoreGiveFromISR(buttonSemaphore, &xHigherPriority);
        }
    }
    portYIELD_FROM_ISR(xHigherPriority);//change to higher priority task
}

// Task to read sensor and decide the content of message
static void sensor_task(void *arg) {
    (void)arg;
    
    //Initialize the device
    init_hat_sdk();
    sleep_ms(300);
    init_led();
    init_sw1(); 
    init_sw2();

    //IMU initialize
    int ret = init_ICM42670();
    if (ret == 0) {
        ICM42670_start_with_default_values();
    } else {
        printf("IMU innitialize fail");
    }

    // Setup GPIO interrupts
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_RISE, true, btn_fxn);
    gpio_set_irq_enabled_with_callback(SW2_PIN, GPIO_IRQ_EDGE_RISE, true, btn_fxn);

    float ax, ay, az, gx, gy, gz, temp;
    char msg = 0;

    while (1) {
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {   //check if the buttons be pressed

            vTaskDelay(pdMS_TO_TICKS(150));//avoid unexpected message because of too fast action

                //read the sensor data
                if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) == 0) {
                    
                    bool flat = (fabsf(az) > THRESHOLD);                              //device is flat
                    bool vertical = (fabsf(ax) > THRESHOLD || fabsf(ay) > THRESHOLD); //device is vertical

                    // State Machine Logic
                    // there is different led blink time for different message, so we have visible message from device
                    if (myState == TYPING) {      //Functionality of TYPING state
                        if (flat) {               //If the device is flat
                            msg = '.';            //message is dot
                            blink_led(1);         //led blink once
                        } 
                        else if (vertical) {      //If the device is vertical
                            msg = '-';            //message is dash
                            blink_led(2);         //led blink twice
                        }
                        myState = IDLE;
                    } 
                    else if (myState == CONTROL) {//Functionality of CONTROL state
                        if (flat) {               //If the device is flat
                            msg = ' ';            //message is space
                            blink_led(3);         //led blink three time
                        } 
                        else if (vertical) {      //If the device is vertical
                            msg = '\n';           //message is \n
                            blink_led(4);         //led blink four time
                        }
                        myState = IDLE;
                    }

                    if (msg != 0) {
                        xQueueSend(myQueue, &msg, 0); //send the message to print task
                        msg = 0;                      //initialize message
                    }
                }

            // If anything unexpeced happen,initialize myState to make sure the program can continue
            else {
                myState = IDLE;
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000)); 
        }
    }
}

// Task to translate the message by protocal then print the message to USB
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
                printf("\n"); 
            }
            
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

int main() {
    stdio_init_all();

    //Create Queue and Semaphore
    myQueue = xQueueCreate(20, sizeof(char));
    buttonSemaphore = xSemaphoreCreateBinary();
    
    //Create handles for tasks
    TaskHandle_t hSensorTask, hPrintTask = NULL;

    //Create sersor_task
    BaseType_t result = xTaskCreate(sensor_task, "sensor", STACK_SIZE, NULL, 2, &hSensorTask);
    
    //If error happen
    if(result != pdPASS) {
        printf("Sensor Task creation failed\n");
        return 0;
    }

    //Create print_task
    result = xTaskCreate(print_task, "print", STACK_SIZE, NULL, 1, &hPrintTask);
    
    //If error happen
    if(result != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }

    //Start freeRTOS
    vTaskStartScheduler();

    return 0;
}
