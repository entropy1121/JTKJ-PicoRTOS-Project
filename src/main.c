/*
 * Course Project: Morse Code Transmitter (Tier 1)
 * * ==============================================================================
 * AI TOOL USAGE DECLARATION (MANDATORY AS PER COURSE RULES)
 * ==============================================================================
 * AI Tool Used: Google Gemini (Model 1.5 Pro / 2.0 Flash)
 * * Prompt Used: 
 * "Help me write a FreeRTOS program for Raspberry Pi Pico W using C language. 
 * It needs two tasks: one to read an IMU sensor (ICM42670) triggered by buttons, 
 * and another to print Morse code to USB serial. The protocol requires specific 
 * spacing for dots, dashes, and message endings. Use the provided tkjhat/sdk.h."
 * * Modifications made by Student:
 * 1. Refactored the code structure to match the course's "task-based" architecture.
 * 2. Integrated specific TKJHAT SDK functions (ICM42670, GPIO) as per lab exercises.
 * 3. Implemented a button logic split: SW2 for typing, SW1 for spacing/enter.
 * 4. Added interrupt safety mechanisms (FromISR functions) as emphasized in lectures.
 * 5. Tuned stack sizes and delays to prevent system starvation.
 * ==============================================================================
 */

#include <stdio.h>
#include <math.h> 
#include <pico/stdlib.h>

// FreeRTOS Includes
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>

// Course SDK Includes
#include "tkjhat/sdk.h"

// --- Configuration Constants ---
#define STACK_SIZE      1024   // Adjusted to accommodate printf buffer
#define QUEUE_LEN       20     // Buffer size for outgoing characters
#define TILT_THRESHOLD  0.7f   // 0.7g threshold for orientation detection
// 定义控制命令的常量，这样代码更易读
#define CMD_DOT       '.'   // 点
#define CMD_DASH      '-'   // 划
#define CMD_SPACE     ' '   // 字母间隔
#define CMD_END_MSG   '\n'  // 消息结束 (你可以用 '\n'，也可以用 'E'，或者 0xFF)

// --- Global Variables (RTOS Objects) ---
QueueHandle_t msg_queue;
SemaphoreHandle_t btn_sem;
volatile int active_btn_id = 0; // 1 for SW1, 2 for SW2

// --- Interrupt Service Routine (ISR) ---
// Handles button presses efficiently as per "Interrupts" lecture
static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Identify which button triggered the interrupt
    if (gpio == SW2_PIN) {
        active_btn_id = 2; // Typing Action
        xSemaphoreGiveFromISR(btn_sem, &xHigherPriorityTaskWoken);
    } 
    else if (gpio == SW1_PIN) {
        active_btn_id = 1; // Control Action (Space/Enter)
        xSemaphoreGiveFromISR(btn_sem, &xHigherPriorityTaskWoken);
    }

    // Force context switch if a high priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// --- Helper Function: IMU Logic ---
// Reads sensor and returns current symbol based on position
// Returns 0 if error or indeterminate
char get_symbol_from_imu(int button_id) {
    float ax, ay, az, gx, gy, gz, temp;
    
    // Read raw data using SDK
    if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) != 0) {
        printf("Error: Sensor Read Failed\n");
        return 0;
    }

    // Determine orientation (Simple State Logic)
    // Z-axis dominant (> 0.7g) -> Flat
    bool is_flat = (fabsf(az) > TILT_THRESHOLD);
    // X or Y-axis dominant (> 0.7g) -> Tilted
    bool is_tilted = (fabsf(ax) > TILT_THRESHOLD || fabsf(ay) > TILT_THRESHOLD);

    char symbol = 0;

    // Logic Table:
    // SW2 (Type)    + Flat   = Dot (.)
    // SW2 (Type)    + Tilted = Dash (-)
    // SW1 (Control) + Flat   = Char Space ( )
    // SW1 (Control) + Tilted = End Message (E)
    
    if (button_id == 2) { // Typing
        if (is_flat) {
            symbol = '.';
            blink_led(1); // Feedback
        } else if (is_tilted) {
            symbol = '-';
            // Long flash for Dash
            set_led_status(true);
            vTaskDelay(pdMS_TO_TICKS(200)); // Blocking delay inside logic is okay here
            set_led_status(false);
        }
    } 
    else if (button_id == 1) { // Control
        if (is_flat) {
            symbol = CMD_SPACE; // 发送空格命令
            blink_led(2);
        } else if (is_tilted) {
            symbol = CMD_END_MSG; // 发送结束命令 (原本是 'E')
            blink_led(3);
        }
    }
    
    return symbol;
}

// --- Task 1: Sensor Task ---
// Priority: 2 (High) - Handles logic and hardware interaction
static void sensor_task(void *arg) {
    (void)arg;
    
    while (1) {
        // Block until interrupt gives semaphore
        if (xSemaphoreTake(btn_sem, portMAX_DELAY) == pdTRUE) {
            
            // Debounce: Wait for signal to settle
            vTaskDelay(pdMS_TO_TICKS(50));

            // Verify button is still pressed (simple debounce check)
            uint target_pin = (active_btn_id == 1) ? SW1_PIN : SW2_PIN;
            
            if (gpio_get(target_pin)) {
                char symbol = get_symbol_from_imu(active_btn_id);
                
                if (symbol != 0) {
                    xQueueSend(msg_queue, &symbol, 0);
                }
            }
            
            // Rate limit to prevent flooding
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

// --- Task 2: USB Print Task ---
// Priority: 1 (Low) - Consumes data from queue and prints via USB
static void print_task(void *arg) {
    (void)arg;
    char received_char;

    while (1) {
        // Block until queue has data
        if (xQueueReceive(msg_queue, &received_char, portMAX_DELAY) == pdTRUE) {
            
            // Protocol handling (as per Project Specifications)
            switch (received_char) {
        case CMD_DOT:       // 原本是 case '.':
            printf("."); 
            break;
        case CMD_DASH:      // 原本是 case '-':
            printf("-"); 
            break;
        case CMD_SPACE:     // 原本是 case ' ':
            printf(" "); 
            break;
        case CMD_END_MSG:   // 原本是 case 'E':
            // 这里才是真正发给电脑的协议内容
            // 只要收到 CMD_END_MSG，就打印协议规定的结尾符
            printf("  \n"); 
            break;
            }
            // Ensure data is sent immediately over USB
            fflush(stdout);
        }
    }
}

// --- Initialization Helper ---
void init_components(void) {
    // 1. Initialize Hat SDK (I2C, etc.)
    init_hat_sdk();
    sleep_ms(100); // Wait for hardware to settle

    // 2. Initialize Peripherals
    init_led();
    init_sw1(); 
    init_sw2();

    // 3. Initialize IMU with error handling
    int rc = init_ICM42670();
    if (rc == 0) {
        ICM42670_start_with_default_values();
    } else {
        printf("Warning: IMU Init Failed (Error %d)\n", rc);
    }
}

// --- Main Function ---
int main() {
    // Initialize stdio for USB communication
    stdio_init_all();
    
    // Optional: Wait for USB connection (helpful for debugging)
    // sleep_ms(2000); 
    
    printf("System Starting...\n");

    // Initialize hardware
    init_components();

    // Create RTOS objects
    msg_queue = xQueueCreate(QUEUE_LEN, sizeof(char));
    btn_sem = xSemaphoreCreateBinary();

    // Setup Interrupts (Rising Edge)
    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(SW2_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    // Create Tasks
    // Priorities: Sensor (2) > Print (1) to ensure input responsiveness
    TaskHandle_t hSensorTask = NULL;
    TaskHandle_t hPrintTask = NULL;

    xTaskCreate(sensor_task, "Sensor", STACK_SIZE, NULL, 2, &hSensorTask);
    xTaskCreate(print_task, "Print", STACK_SIZE, NULL, 1, &hPrintTask);

    // Start Scheduler
    printf("Starting Scheduler...\n");
    vTaskStartScheduler();

    // Should never reach here
    while(1);
    return 0;
}