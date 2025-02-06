//Compilado en esp-idf v 6.10 en platformio con vsc el 5 de Febrero del 2025
#include <unistd.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_task_wdt.h"

#define BOTON1 GPIO_NUM_47
#define BOTON2 GPIO_NUM_21
#define LED GPIO_NUM_40

void app_main(void) {
    esp_task_wdt_deinit();

    // Restablecer pines a su configuración por defecto
    gpio_reset_pin(BOTON1);
    gpio_reset_pin(BOTON2);
    gpio_reset_pin(LED);

    // Configurar los botones como entrada con resistencia pull-up
    gpio_set_direction(BOTON1, GPIO_MODE_INPUT);
    gpio_set_direction(BOTON2, GPIO_MODE_INPUT);
    //gpio_set_pull_mode(BOTON1, GPIO_PULLUP_ONLY);
    //gpio_set_pull_mode(BOTON2, GPIO_PULLUP_ONLY);

    // Configurar el LED como salida
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    while (true) {
        // Leer el estado de los botones
        int estado1 = gpio_get_level(BOTON1);
        int estado2 = gpio_get_level(BOTON2);
        int respuesta = estado1 & estado2;
        // Aplicar la operación lógica AND y encender/apagar el LED
        gpio_set_level(LED, respuesta);
        printf("Entrada-1: %d, Entrada-2: %d -> Respuesta: %d\n", estado1, estado2, respuesta);
        usleep(1000000);
    }
}
