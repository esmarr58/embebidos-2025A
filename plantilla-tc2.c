#include <unistd.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_task_wdt.h"


#define numero_bit7 19
#define numero_bit6 20
#define numero_bit5 21
#define numero_bit4 35
#define numero_bit3 36
#define numero_bit2 37
#define numero_bit1 38
#define numero_bit0 39


void prendeLeds(uint8_t led_mask) {
    // Controlar cada LED con operaciones de desplazamiento de bits

    //Parte baja
    gpio_set_level(numero_bit0, (led_mask & 0b00000001) >> 0);  // LED 1 controla el bit 0
    gpio_set_level(numero_bit1, (led_mask & 0b00000010) >> 1);  // LED 2 controla el bit 1
    gpio_set_level(numero_bit2, (led_mask & 0b00000100) >> 2);  // LED 3 controla el bit 2
    gpio_set_level(numero_bit3, (led_mask & 0b00001000) >> 3);  // LED 4 controla el bit 3

    //Parte alta
    gpio_set_level(numero_bit4, (led_mask & 0b00010000) >> 4);  // LED 1 controla el bit 0
    gpio_set_level(numero_bit5, (led_mask & 0b00100000) >> 5);  // LED 2 controla el bit 1
    gpio_set_level(numero_bit6, (led_mask & 0b01000000) >> 6);  // LED 3 controla el bit 2
    gpio_set_level(numero_bit7, (led_mask & 0b10000000) >> 7);  // LED 4 controla el bit 3
}

void app_main(void) {
    esp_task_wdt_deinit();
    uint8_t numero1 = 255;     //signed int numero1 = -2;
    uint8_t numero2 = 1;     //unsigned int numero2 = 1;
    uint8_t resultado = numero1+numero2;

    /*Reseteo de los pines de salida*/

    gpio_reset_pin(numero_bit0);
    gpio_reset_pin(numero_bit1);
    gpio_reset_pin(numero_bit2);
    gpio_reset_pin(numero_bit3);
    gpio_reset_pin(numero_bit4);
    gpio_reset_pin(numero_bit5);
    gpio_reset_pin(numero_bit6);
    gpio_reset_pin(numero_bit7);

    gpio_set_direction(numero_bit0, GPIO_MODE_OUTPUT);
    gpio_set_direction(numero_bit1, GPIO_MODE_OUTPUT);
    gpio_set_direction(numero_bit2, GPIO_MODE_OUTPUT);
    gpio_set_direction(numero_bit3, GPIO_MODE_OUTPUT);
    gpio_set_direction(numero_bit4, GPIO_MODE_OUTPUT);
    gpio_set_direction(numero_bit5, GPIO_MODE_OUTPUT);
    gpio_set_direction(numero_bit6, GPIO_MODE_OUTPUT);
    gpio_set_direction(numero_bit7, GPIO_MODE_OUTPUT);


    

    //printf("Num1: %d + Num2:%d = %d \n", numero1, numero2, resultado);
    //printf("Resultado en binario: %08b\n", resultado);
    printf("Resultado en hexadecimal: 0x%02X\n", resultado);



    prendeLeds(resultado);


    while (true) {
        vTaskDelay(pdMS_TO_TICKS(500));
      
    }
}
