//Compilado en esp-idf v 6.10 en platformio con vsc el 5 de Febrero del 2025
#include "driver/gpio.h"

#define BOTON1 GPIO_NUM_4
#define BOTON2 GPIO_NUM_5
#define LED GPIO_NUM_2

void app_main(void) {
    // Restablecer pines a su configuración por defecto
    gpio_reset_pin(BOTON1);
    gpio_reset_pin(BOTON2);
    gpio_reset_pin(LED);

    // Configurar los botones como entrada con resistencia pull-up
    gpio_set_direction(BOTON1, GPIO_MODE_INPUT);
    gpio_set_direction(BOTON2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BOTON1, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BOTON2, GPIO_PULLUP_ONLY);

    // Configurar el LED como salida
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);

    while (true) {
        // Leer el estado de los botones
        int estado1 = gpio_get_level(BOTON1);
        int estado2 = gpio_get_level(BOTON2);

        // Aplicar la operación lógica AND y encender/apagar el LED
        gpio_set_level(LED, estado1 & estado2);
    }
}
