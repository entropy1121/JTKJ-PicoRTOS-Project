// 文件: src/morse_data.c
#include "morse_data.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// 静态缓冲区用于存储输出字符串
#define MORSE_BUFFER_SIZE 30 
static char morse_buffer[MORSE_BUFFER_SIZE];

char* morse_encode(uint32_t number) {
    // 确保缓冲区清零
    memset(morse_buffer, 0, MORSE_BUFFER_SIZE);
    
    // 将数字转换为字符串 (例如 123)
    char num_str[12];
    // 使用 snprintf 安全地将数字格式化为字符串
    snprintf(num_str, sizeof(num_str), "%lu", number);
    
    size_t len = strlen(num_str);
    
    // 遍历数字的每一位
    for (size_t i = 0; i < len; i++) {
        int digit = num_str[i] - '0'; // 将字符 '0'-'9' 转换为数字 0-9
        
        if (digit >= 0 && digit <= 9) {
            // 查找并拼接摩尔斯电码
            strcat(morse_buffer, MORSE_CODES[digit]);
        }
        
        // 添加一个空格作为数字间的分隔符 (除了最后一个数字)
        if (i < len - 1) {
            strcat(morse_buffer, " ");
        }
    }
    
    return morse_buffer; // 返回指向静态缓冲区的指针
}