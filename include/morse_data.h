// 文件: include/morse_data.h
#ifndef MORSE_DATA_H
#define MORSE_DATA_H

#include <stdint.h>

// 摩尔斯电码定义（数字 0-9）
// 格式: "." 为点, "-" 为划, " " 为数字间分隔
const char *MORSE_CODES[] = {
    "-----",    // 0
    ".----",    // 1
    "..---",    // 2
    "...--",    // 3
    "....-",    // 4
    ".....",    // 5
    "-....",    // 6
    "--...",    // 7
    "---..",    // 8
    "----."     // 9
};

/**
 * @brief 将一个 32 位整数转换为摩尔斯电码字符串。
 * @param number 待转换的整数值 (例如 ambientLight 的整数部分)。
 * @return 指向摩尔斯电码字符串的指针。
 */
char* morse_encode(uint32_t number);


#endif // MORSE_DATA_H