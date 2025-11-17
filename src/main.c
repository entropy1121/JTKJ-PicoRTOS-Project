
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>
#include <pico/time.h> // 用于获取时间戳

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

// 引入双串口调试和 TinyUSB 库

#include "bsp/board.h"
#include "tusb.h"

// 引入摩尔斯电码转换头文件 (你需要创建 morse_data.h 和 morse_data.c)
#include "morse_data.h"


#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1 // CDC 接口 1 用于 CSV 数据输出


// 完善状态机定义：等待、读取、发送
enum state { 
    WAITING=1,      // 等待按钮触发
    READING_LIGHT,  // 正在读取传感器数据
    SENDING_DATA    // 正在处理并发送数据
};
enum state programState = WAITING;

// 全局变量：用于存储光照值 (使用 float 类型与 sdk.c 中的 get_light_level 匹配)
float ambientLight;

// 全局变量：用于跟踪 LED 状态 (假设 BOARD_LED_INDEX = 0)
bool led_is_on = false;
#define BOARD_LED_INDEX 0 

// 按钮中断处理函数
static void btn_fxn(uint gpio, uint32_t eventMask) {
    // 1. 切换 LED 状态 (使用 sdk.c 中找到的 set_led_level 函数)
    led_is_on = !led_is_on;
    set_led_level(BOARD_LED_INDEX, led_is_on); 

    // 2. 切换程序状态，从 WAITING 切换到 READING_LIGHT
    if (programState == WAITING) {
        programState = READING_LIGHT;
    }
}

// 传感器任务：负责数据采集和状态切换
static void sensor_task(void *arg){
    (void)arg;
    
    // 初始化光照传感器 (使用 sdk.c 中找到的 init_light_sensor 函数)
    //init_light_sensor(); 
   
    for(;;){
        
        // 适当延时，防止过度占用 CPU
        vTaskDelay(pdMS_TO_TICKS(100)); 

        if (programState == READING_LIGHT) {
            
            // 读取传感器数据并保存到全局变量 (使用 sdk.c 中找到的 get_light_level 函数)
            get_light_level(&ambientLight); 
            
            // 完成读取后，切换状态到发送
            programState = SENDING_DATA;
        }

        // 移除原始的调试打印和长延时
        // printf("sensorTask\n");
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// 打印任务：负责数据处理和串口输出
static void print_task(void *arg){
    (void)arg;
    char csv_buffer[128];
    char morse_output[128];
    
    while(1){
        
        // 适当延时
        vTaskDelay(pdMS_TO_TICKS(100)); 

        if (programState == SENDING_DATA) {
            
            // 1. 摩尔斯电码输出 (Debug 串口)
            // 将浮点数转换为整数部分进行摩尔斯编码 (假设 morse_encode 函数可用)
            uint32_t light_int = (uint32_t)ambientLight; 
            char *morse_code = morse_encode(light_int); 
            
            // 使用 usb_serial_print 输出摩尔斯电码
            snprintf(morse_output, sizeof(morse_output), "光照值: %.2f Lux -> 摩尔斯码: %s\r\n", ambientLight, morse_code);
            usb_serial_print(morse_output);

            // 2. CSV 数据输出 (CDC 1 接口)
            uint64_t timestamp = time_us_64();
            // 格式: timestamp, luminance
            snprintf(csv_buffer, sizeof(csv_buffer), "%llu, %.2f\r\n", timestamp, ambientLight);

            // 通过 CDC 1 接口发送 CSV 数据
            if (tud_cdc_n_connected(CDC_ITF_TX)) {
                tud_cdc_n_write(CDC_ITF_TX, csv_buffer, strlen(csv_buffer));
                tud_cdc_n_write_flush(CDC_ITF_TX);
            }

            // 完成发送后，切换状态回 WAITING
            programState = WAITING; 
        }

        // 移除原始的调试打印和长延时
        // printf("printTask\n");
        // vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


// TinyUSB 任务：处理 USB 通信堆栈
static void usbTask(void *arg) {
    (void)arg;
    while (1) {
        tud_task();              // 运行 TinyUSB 堆栈
    }
}

int main() {

    // 遵循练习要求，注释掉 stdio_init_all(); 
    // stdio_init_all();

    init_hat_sdk();
    sleep_ms(300); 

    // 初始化 LED 和按钮 (使用你模板中的函数名)
    // 注意：init_led/init_button2 不是 sdk.c 中的函数名，但保留它们以匹配模板结构
    // 假设它们能正确初始化引脚
    init_led();
    init_button2(); 
    
    // 注册中断处理函数
    gpio_set_irq_enabled_with_callback(BUTTON2, GPIO_IRQ_EDGE_RISE, true, btn_fxn);

    
    TaskHandle_t hSensorTask, hPrintTask, hUSB = NULL;

    // 创建 usbTask 任务以启用双串口通信
    xTaskCreate(usbTask, "usb", 2048, NULL, 3, &hUSB);
    #if (configNUMBER_OF_CORES > 1)
        vTaskCoreAffinitySet(hUSB, 1u << 0);
    #endif
    

    // 创建 sensor_task 任务
    BaseType_t result = xTaskCreate(sensor_task, // (en) Task function
                "sensor",                        // (en) Name of the task 
                DEFAULT_STACK_SIZE,              // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                            // (en) Arguments of the task 
                2,                               // (en) Priority of this task
                &hSensorTask);                   // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }
    
    // 创建 print_task 任务
    result = xTaskCreate(print_task,  // (en) Task function
                "print",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                &hPrintTask);         // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }

    // Start the scheduler (never returns)
    vTaskStartScheduler();
    
    // Never reach this line.
    return 0;
}