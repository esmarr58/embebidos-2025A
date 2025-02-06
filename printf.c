#include <unistd.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_task_wdt.h"

#define pinLED 12
#define ALTO 1
#define BAJO 0

// Definimos los tiempos en milisegundos
int tiempoBajo = 19000; // 100 ms
int tiempoAlto = 1000; // 100 ms

void app_main(void) {
    esp_task_wdt_deinit();
    int contador = 0;
    printf("Cuenta: %d \n", contador);

    gpio_reset_pin(pinLED);
    gpio_set_direction(pinLED, GPIO_MODE_OUTPUT);

    while (true) {
        gpio_set_level(pinLED, ALTO);
        usleep(tiempoBajo); // Convertir ms a microsegundos

        gpio_set_level(pinLED, BAJO);
        usleep(tiempoAlto); // Convertir ms a microsegundos
    }
}
