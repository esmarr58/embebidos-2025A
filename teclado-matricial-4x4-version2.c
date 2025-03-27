#include "sdkconfig.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Pines de filas
#define ROW1 14
#define ROW2 13
#define ROW3 12
#define ROW4 11

// Pines de columnas
#define COL1 10
#define COL2 9
#define COL3 46
#define COL4 3

// Delay entre escaneos (antirebote incluido)
volatile int tiempoRetardo = 10;

uint8_t columnas[4] = {COL1, COL2, COL3, COL4};
uint8_t filas[4] = {ROW1, ROW2, ROW3, ROW4};

// Mapa del teclado 4x4
char mapaTeclado[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

static const char *TAG = "Teclado4x4:";

// Configuración inicial de pines
void configurarTeclado() {
    // Configurar columnas como salida (inicialmente en alto)
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(columnas[i]);
        gpio_set_direction(columnas[i], GPIO_MODE_OUTPUT);
        gpio_set_level(columnas[i], 1);
    }

    // Configurar filas como entrada con pull-up
    for (int j = 0; j < 4; j++) {
        gpio_reset_pin(filas[j]);
        gpio_set_direction(filas[j], GPIO_MODE_INPUT);
        gpio_set_pull_mode(filas[j], GPIO_PULLUP_ONLY);
    }
}

// Activa una columna a la vez (las demás en alto)
void activarColumna(uint8_t indice) {
    for (int i = 0; i < 4; i++) {
        gpio_set_level(columnas[i], (i == indice) ? 0 : 1);
    }
}

// Devuelve el índice de la fila activa (0-3), o -1 si ninguna
int8_t leerFilaActiva() {
    for (int i = 0; i < 4; i++) {
        if (gpio_get_level(filas[i]) == 0) {
            return i;
        }
    }
    return -1;
}

// Espera a que se libere la tecla (para antirebote)
void esperarLiberacion() {
    while (leerFilaActiva() != -1) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void app_main() {
    configurarTeclado();

    while (true) {
        for (int col = 0; col < 4; col++) {
            activarColumna(col);
            vTaskDelay(pdMS_TO_TICKS(tiempoRetardo));

            int8_t fila = leerFilaActiva();

            if (fila != -1) {
                char tecla = mapaTeclado[fila][col];
                ESP_LOGI(TAG, "Tecla presionada: %c", tecla);

                esperarLiberacion(); // Antirebote
            }
        }
    }
}
