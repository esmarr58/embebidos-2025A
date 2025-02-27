#include <unistd.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_task_wdt.h"

#define A 19
#define B 20
#define C 21
#define D 47
#define E 48
#define F 35
#define G 36

#define cero    0b
#define uno     0b
#define dos     0b
#define tres    0b
#define cuatro  0b
#define cinco   0b
#define seis    0b
#define siete   0b
#define ocho    0b
#define nueve   0b


// Definimos los tiempos en milisegundos
int tiempoBajo = 19000; // 100 ms
int tiempoAlto = 1000; // 100 ms

uint8_t numeros[10] = {cero,uno,dos,tres,cuatro,cinco,seis,siete,ocho,nueve};

void decodificaSegmentos(uint8_t display){
     //Parte baja
    gpio_set_level(G, (display & 0b00000001) >> 0);  
    gpio_set_level(F, (display & 0b00000010) >> 1);  
    gpio_set_level(E, (display & 0b00000100) >> 2);  
    gpio_set_level(D, (display & 0b00001000) >> 3);  

    //Parte alta
    gpio_set_level(C, (display & 0b00010000) >> 4);  
    gpio_set_level(B, (display & 0b00100000) >> 5); 
    gpio_set_level(A, (display & 0b01000000) >> 6);  
}

void app_main(void) {
    esp_task_wdt_deinit();
    uint8_t contador = 0;
   

    gpio_reset_pin(A);
    gpio_reset_pin(B);
    gpio_reset_pin(C);
    gpio_reset_pin(D);
    gpio_reset_pin(E);
    gpio_reset_pin(F);
    gpio_reset_pin(G);

    gpio_set_direction(A, GPIO_MODE_OUTPUT);
    gpio_set_direction(B, GPIO_MODE_OUTPUT);
    gpio_set_direction(C, GPIO_MODE_OUTPUT);
    gpio_set_direction(D, GPIO_MODE_OUTPUT);
    gpio_set_direction(E, GPIO_MODE_OUTPUT);
    gpio_set_direction(F, GPIO_MODE_OUTPUT);
    gpio_set_direction(G, GPIO_MODE_OUTPUT);

    
    while (true) {
        decodificaSegmentos(numeros[contador]);
        printf("Cuenta: %d \n", contador);
        contador++;
        if(contador > 9) contador = 0;
        vTaskDelay(1000/portTICK_PERIOD_MS);
      
    }
}
