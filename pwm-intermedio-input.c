#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include <stdio.h>
#include "esp_task_wdt.h"

#define PIN_LED 40
#define PIN_INPUT 47  // Pin de entrada para modificar el ciclo de trabajo
#define ALTO 1
#define BAJO 0

void app_main(void) {
    esp_task_wdt_deinit();
    
    gpio_reset_pin(PIN_LED);
    gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);

    gpio_reset_pin(PIN_INPUT);
    gpio_set_direction(PIN_INPUT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_INPUT, GPIO_PULLDOWN_ONLY); // Evita fluctuaciones

    uint32_t frecuencia = 50;      // Frecuencia inicial en Hz
    uint8_t dutyCycle = 10;       // Ciclo de trabajo inicial en porcentaje
    uint32_t intervaloCambio_ms = 500; // Tiempo en ms antes de cambiar el PWM

    int64_t tiempoCambio = esp_timer_get_time(); // Marca de tiempo inicial
    int64_t tiempoReporte = esp_timer_get_time(); // Marca de tiempo para printf

    while (true) {
        uint32_t periodo_us = 1000000 / frecuencia;  // Periodo en microsegundos
        uint32_t tiempoAlto_us = (periodo_us * dutyCycle) / 100; // Tiempo en alto
        uint32_t tiempoBajo_us = periodo_us - tiempoAlto_us; // Tiempo en bajo

        int64_t tiempoInicio = esp_timer_get_time();

        // Mantener LED encendido durante tiempoAlto_us
        gpio_set_level(PIN_LED, ALTO);
        while ((esp_timer_get_time() - tiempoInicio) < tiempoAlto_us) {
            // Bucle ocupado para precisión
        }

        // Mantener LED apagado durante tiempoBajo_us
        tiempoInicio = esp_timer_get_time();
        gpio_set_level(PIN_LED, BAJO);
        while ((esp_timer_get_time() - tiempoInicio) < tiempoBajo_us) {
            // Bucle ocupado para precisión
        }

        // Verifica si ha pasado 1 segundo para imprimir el duty cycle
        if ((esp_timer_get_time() - tiempoReporte) > 1000000) {
            printf("Ciclo de trabajo: %d%%\n", dutyCycle);
            tiempoReporte = esp_timer_get_time(); // Reinicia el tiempo de reporte
        }

        // Verifica si ha pasado el intervalo para cambiar el PWM
        if ((esp_timer_get_time() - tiempoCambio) > (intervaloCambio_ms * 1000)) {
            tiempoCambio = esp_timer_get_time(); // Reinicia el tiempo de cambio

            // Incrementa el ciclo de trabajo con la entrada del pin 47
            if (gpio_get_level(PIN_INPUT)) {
                dutyCycle += 10;
                if (dutyCycle > 100) {
                    dutyCycle = 0; // Reinicia el ciclo de trabajo
                }
                printf("Botón presionado. Nuevo ciclo de trabajo: %d\n", dutyCycle);
            }
        }
    }
}
