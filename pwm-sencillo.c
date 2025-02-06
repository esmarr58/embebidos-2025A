//espidf v6.10  - Compilado el 5 de Febrero 2025
#include <unistd.h>
#include "sdkconfig.h"
#include "driver/gpio.h"

#define pinLED 4
#define ALTO 1
#define BAJO 0

// Definimos los tiempos en milisegundos
int tiempoBajo = 100; // 100 ms
int tiempoAlto = 100; // 100 ms

void app_main(void) {
    gpio_reset_pin(pinLED);
    gpio_set_direction(pinLED, GPIO_MODE_OUTPUT);

    while (true) {
        gpio_set_level(pinLED, ALTO);
        usleep(tiempoAlto * 1000); // Convertir ms a microsegundos

        gpio_set_level(pinLED, BAJO);
        usleep(tiempoBajo * 1000); // Convertir ms a microsegundos
    }
}
