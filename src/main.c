
#include <stdio.h>
#include <string.h>
#include <math.h> // For roundf, fabs

#include <pico/stdlib.h>
#include <hardware/gpio.h> // For Button IRQ handling

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>

#include "tkjhat/sdk.h"

// Exercise 4. Include the libraries necessaries to use the usb-serial-debug, and tinyusb
// Tehtävä 4 . Lisää usb-serial-debugin ja tinyusbin käyttämiseen tarvittavat kirjastot.
#include "bsp/board.h"
#include "tusb_config.h"
#include "helper.h" 


#define DEFAULT_STACK_SIZE 2048
#define CDC_ITF_TX      1

// ===============================================
// Tehtävä 3: Tilakoneen esittely ja Globaalit muuttujat
// Exercise 3: Definition of the state machine and Global variables
// ===============================================
enum state { WAITING=1, READING_IMU, PROCESS_POSITION, SENDING_SYMBOL };
enum state programState = WAITING;

// Globaali synkronointimekanismi: FreeRTOS jono merkkien välittämiseksi tehtävien välillä
// Global synchronization mechanism: FreeRTOS queue for passing symbols between tasks
QueueHandle_t morse_symbol_queue;

// Globaalit muuttujat IMU-datan tallentamiseen
// Global variables for storing IMU data
float acc_x, acc_y, acc_z; 
const float THRESHOLD = 0.8f; // IMU detection threshold (approx. 0.8g)


// ===============================================
// Tehtävä 1: Painikkeen keskeytyskäsittelijä (SW1)
// Exercise 1: Button interrupt handler (SW1)
// ===============================================
static void btn_fxn(uint gpio, uint32_t eventMask) {
    // Tehtävä 1: Lähetä välilyönti painettaessa.
    // Exercise 1: Send a space when pressed. 
    if (gpio == TKJ_HAT_SW1_PIN && (eventMask & GPIO_IRQ_EDGE_FALL)) {
        char symbol = ' ';
        // Lähetä välilyönti jonoon, jotta print_task voi sen lähettää
        // Send the space to the queue so print_task can send it
        xQueueSendFromISR(morse_symbol_queue, &symbol, NULL);
        
        // Välkyttää LEDiä vahvistukseksi
        toggle_led(); 
    }
}


// ===============================================
// Tehtävä 2: Anturitehtävä (IMU lukeminen ja tilan tunnistus)
// Exercise 2: Sensor Task (IMU reading and state detection)
// ===============================================
static void sensor_task(void *arg){
    (void)arg;
    
    // Tehtävä 2: Alusta IMU anturi
    // Exercise 2: Initialize IMU sensor
    
    // init_hat_sdk() in main() already calls init_i2c_default()
    if (init_ICM42670() != 0) {
        printf("IMU init failed!\n");
        // Älä jatka jos init epäonnistui
        vTaskDelete(NULL); 
    }
    ICM42670_start_with_default_values();
    
    // Tila- ja lukulogiikka
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100ms update rate (10Hz)
    xLastWakeTime = xTaskGetTickCount();
    
    while(1){
        
        programState = READING_IMU;
        // Lukee IMU-tiedot. Huomaa: ICM42670_read_sensor_data palauttaa myös Gyro-tiedot.
        ICM42670_read_sensor_data(&acc_x, &acc_y, &acc_z, NULL, NULL, NULL, NULL);

        programState = PROCESS_POSITION;
        char detected_symbol = '\0';
        
        // Tunnistus: Piste (Dot): Laite tasaisesti pöydällä (Z-akseli alas)
        // Detection: Dot (.): Device flat on table (Z-axis down, approx -1g)
        if (acc_z < -THRESHOLD) {
            detected_symbol = '.';
            toggle_red_led(); // Välkyttää punaista LEDiä pisteelle
        }
        // Tunnistus: Viiva (Dash): Laite pystyssä, Y-akseli ylös
        // Detection: Dash (-): Device on its side (Y-axis up, approx +1g)
        else if (acc_y > THRESHOLD) {
            detected_symbol = '-';
            toggle_green_led(); // Välkyttää vihreää LEDiä viivalle
        }
        
        // Lähetä symboli (jos tunnistettu) print_taskille jonon kautta
        // Send the symbol (if detected) to print_task via the queue
        if (detected_symbol != '\0') {
            programState = SENDING_SYMBOL;
            // Lähetä jonoon. Älä estä: Jos jono on täynnä, vain ohita tämä symboli
            // Send to queue. Don't block: If the queue is full, just skip this symbol.
            xQueueSend(morse_symbol_queue, &detected_symbol, 0); 
            
            // Estä kaksoislähetys antamalla käyttäjälle aikaa liikuttaa laitetta
            // Prevent double-send by giving the user time to move the device.
            vTaskDelay(pdMS_TO_TICKS(500)); 
            
            programState = WAITING; // Palaa odottamaan
        } else {
            programState = WAITING;
        }

        // Odota seuraavaa sykliä
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


// ===============================================
// Tehtävä 2: Tulostustehtävä (Sarjaporttiviestintä)
// Exercise 2: Print Task (Serial Communication)
// ===============================================
static void print_task(void *arg){
    (void)arg;
    char symbol;
    
    while(1){
        // Odota, kunnes jono on vastaanottanut merkin sensor_taskilta
        // Wait until the queue has received a character from sensor_task
        if (xQueueReceive(morse_symbol_queue, &symbol, portMAX_DELAY) == pdPASS) {
            // Lähetä merkki ulkoiseen laitteeseen (työasemaan)
            // Send the character to the external device (workstation)
            
            // NOTE: usb_serial_print() handles the serial transmission
            usb_serial_print("%c", symbol);
            
            // Näytä konsolissa debuggausta varten (ei lähetetä sarjaportin kautta)
            printf("Sent Symbol: %c\n", symbol); 
        }
    }
}


// ===============================================
// Pääfunktio
// ===============================================
int main() {
    
    // Alusta TKJ-HAT
    init_hat_sdk(); 

    // Tehtävä 4: Estä pico_enable_stdio_usb:n käyttö ja alusta TinyUSB erikseen
    // Exercise 4: Prevent the use of pico_enable_stdio_usb and initialize TinyUSB separately
    // Huom: Varmista, että kommentoit pico_enable_stdio_usb CMakeLists.txt:ssä!
    board_init();
    tusb_init();

    // Tehtävä 1: Alusta LED ja painike (SW1)
    // Exercise 1: Initialize LED and Button (SW1)
    init_led();
    init_button1(); // SW1
    
    // Rekisteröi keskeytyskäsittelijä
    // Register the interrupt handler
    gpio_set_irq_enabled_with_callback(TKJ_HAT_SW1_PIN, 
                                      GPIO_IRQ_EDGE_FALL, 
                                      true, 
                                      &btn_fxn);


    // Luo synkronointijono (kapasiteetti 10 merkkiä)
    // Create the synchronization queue (capacity 10 characters)
    morse_symbol_queue = xQueueCreate(10, sizeof(char));
    if (morse_symbol_queue == NULL) {
        printf("Queue creation failed!\n");
        return 0;
    }


    // Luo tehtävät xTaskCreate:llä
    // Create the tasks with xTaskCreate
    BaseType_t result = xTaskCreate(sensor_task, // (en) Task function
                "sensor",                        // (en) Name of the task 
                DEFAULT_STACK_SIZE,              // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                            // (en) Arguments of the task 
                2,                               // (en) Priority of this task
                NULL);                           // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Sensor task creation failed\n");
        return 0;
    }
    result = xTaskCreate(print_task,  // (en) Task function
                "print",              // (en) Name of the task 
                DEFAULT_STACK_SIZE,   // (en) Size of the stack for this task (in words). Generally 1024 or 2048
                NULL,                 // (en) Arguments of the task 
                2,                    // (en) Priority of this task
                NULL);                // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("Print Task creation failed\n");
        return 0;
    }
    
    // TinyUSB Task (needed when not using stdio_usb)
    result = xTaskCreate(usb_task, // (en) Task function
                "usb",             // (en) Name of the task 
                2048,              // (en) Size of the stack for this task (in words).
                NULL,              // (en) Arguments of the task 
                3,                 // (en) Priority of this task (higher priority is good for USB)
                NULL);             // (en) A handle to control the execution of this task

    if(result != pdPASS) {
        printf("USB Task creation failed\n");
        return 0;
    }
    
    
    // Aloita FreeRTOS-ajastin (ei koskaan palaa)
    // Start the scheduler (never returns)...
    vTaskStartScheduler();
    
    // Tänne ei pitäisi päätyä
    return 0;
}