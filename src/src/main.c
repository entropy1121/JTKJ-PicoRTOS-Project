/*
 * Fixed Protocol Logic main.c for JTKJ Project
 * Separates symbols (./-) from separators (space)
 */

#include <stdio.h>
#include <math.h> 
#include <pico/stdlib.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <semphr.h>

#include "tkjhat/sdk.h"

#define DEFAULT_STACK_SIZE 1024 // 防止堆栈溢出
#define QUEUE_LENGTH 20
#define TILT_THRESHOLD 0.7f 

QueueHandle_t xMorseQueue;
SemaphoreHandle_t xButtonSemaphore;
volatile int button_event_id = 0; 

/* 中断处理 */
static void gpio_callback(uint gpio, uint32_t events) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (gpio == SW2_PIN) {
        button_event_id = 2;
        xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
    } 
    else if (gpio == SW1_PIN) {
        button_event_id = 1;
        xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* 传感器任务 */
static void sensor_task(void *arg) {
    (void)arg;
    float ax, ay, az, gx, gy, gz, temp;
    char char_to_send = 0;

    while (1) {
        if (xSemaphoreTake(xButtonSemaphore, portMAX_DELAY) == pdTRUE) {
            vTaskDelay(pdMS_TO_TICKS(50)); // 消抖

            // 读取姿态
            if (ICM42670_read_sensor_data(&ax, &ay, &az, &gx, &gy, &gz, &temp) != 0) {
                printf("IMU Error\n");
                continue;
            }

            // 判断姿态
            // Z轴受重力(>0.7) -> 平放
            bool is_flat = (fabsf(az) > TILT_THRESHOLD);
            // X或Y轴受重力(>0.7) -> 侧立
            bool is_tilted = (fabsf(ax) > TILT_THRESHOLD || fabsf(ay) > TILT_THRESHOLD);

            // --- SW2: 只负责发送由点和划组成的“笔画” ---
            if (button_event_id == 2 && gpio_get(SW2_PIN)) {
                if (is_flat) {
                    char_to_send = '.'; // 平放按SW2 -> 点
                    blink_led(1);       
                } else if (is_tilted) {
                    char_to_send = '-'; // 侧立按SW2 -> 划
                    set_led_status(true); 
                    vTaskDelay(pdMS_TO_TICKS(200));
                    set_led_status(false);
                }
            } 
            // --- SW1: 负责控制“空格”和“回车” ---
            else if (button_event_id == 1 && gpio_get(SW1_PIN)) {
                if (is_flat) {
                    // 平放按SW1 -> 字母结束 (发送1个空格)
                    // Protocol: "Between every Morse-encoded character... there is one space"
                    char_to_send = ' '; 
                    blink_led(2);       
                } else if (is_tilted) {
                    // 侧立按SW1 -> 消息结束 (发送结束符)
                    // Protocol: "A message ends with two spaces and a line feed"
                    char_to_send = 'E'; 
                    blink_led(5);       
                }
            }

            // 发送数据到队列
            if (char_to_send != 0) {
                xQueueSend(xMorseQueue, &char_to_send, 0);
                char_to_send = 0; // 清空，防止重复发送
            }
            
            vTaskDelay(pdMS_TO_TICKS(200)); // 冷却时间
        }
    }
}

/* 打印任务 - 严格遵守协议 */
static void print_task(void *arg) {
    (void)arg;
    char received_char;

    while (1) {
        if (xQueueReceive(xMorseQueue, &received_char, portMAX_DELAY) == pdTRUE) {
            switch (received_char) {
                case '.':
                    printf("."); // 以前这里是 ". " (错了)，现在改成了 "." (对的)
                    break;
                case '-':
                    printf("-"); // 以前这里是 "- " (错了)，现在改成了 "-" (对的)
                    break;
                case ' ':
                    printf(" "); // 手动发送字母间隔
                    break;
                case 'E':
                    printf("  \n"); // 消息结束符
                    break;
            }
            fflush(stdout); // 确保立即输出
        }
    }
}

/* 主函数 */
int main() {
    stdio_init_all();
    init_hat_sdk();
    sleep_ms(100);
    
    // 初始化组件
    init_led();
    init_sw1(); 
    init_sw2();

    // IMU 初始化
    int rc = init_ICM42670();
    if (rc == 0) {
        ICM42670_start_with_default_values();
    } else {
        printf("IMU Init Failed\n");
    }

    // 任务与队列
    xMorseQueue = xQueueCreate(QUEUE_LENGTH, sizeof(char));
    xButtonSemaphore = xSemaphoreCreateBinary();

    gpio_set_irq_enabled_with_callback(SW1_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(SW2_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    xTaskCreate(sensor_task, "Sensor", DEFAULT_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(print_task, "Print", DEFAULT_STACK_SIZE, NULL, 1, NULL);

    vTaskStartScheduler();
    while(1);
    return 0;
}