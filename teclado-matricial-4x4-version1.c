#include "sdkconfig.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

// Definición de las filas y columnas del teclado
#define ROW1 13   // Pin para la fila 1
#define ROW2 11   // Pin para la fila 2
#define ROW3 10   // Pin para la fila 3
#define ROW4 9    // Pin para la fila 4

#define COL1 19   // Pin para la columna 1
#define COL2 20   // Pin para la columna 2
#define COL3 21   // Pin para la columna 3
#define COL4 47   // Pin para la columna 4

volatile int tiempoRetardo = 10;

uint8_t columnas[4] = {COL1, COL2, COL3, COL4};
uint8_t filas[4] = {ROW1,ROW2,ROW3,ROW4};

static const char *TAG = "Teclado4x4:";

void configurarTeclado(){
    //Configurar como salida a las columnas.
    for(int i = 0; i<4; i++){
        //i=0,1,2,3
        gpio_reset_pin(columnas[i]);
        gpio_set_direction(columnas[i],GPIO_MODE_OUTPUT);
        gpio_set_level(columnas[i],1);
    }

    //Configurar las entradas
    for(int j = 0; j<4; j++){
        gpio_reset_pin(filas[j]);
        gpio_set_direction(filas[j], GPIO_MODE_INPUT);
        gpio_set_pull_mode(filas[j],GPIO_PULLUP_ONLY);
    }


}
//Maquinas de estado FSM (Finite State Machine)
void rotaBit(uint8_t estadoActual){
    if(estadoActual == 1){
        gpio_set_level(COL1, 0);
        gpio_set_level(COL2, 1);
        gpio_set_level(COL3, 1);
        gpio_set_level(COL4, 1);
    }
    else if(estadoActual == 2){
        gpio_set_level(COL1, 1);
        gpio_set_level(COL2, 0);
        gpio_set_level(COL3, 1);
        gpio_set_level(COL4, 1);
    }
    else if(estadoActual == 3){
        gpio_set_level(COL1, 1);
        gpio_set_level(COL2, 1);
        gpio_set_level(COL3, 0);
        gpio_set_level(COL4, 1);
    }
    else if(estadoActual == 4){
        gpio_set_level(COL1, 1);
        gpio_set_level(COL2, 1);
        gpio_set_level(COL3, 1);
        gpio_set_level(COL4, 0);
    }

}

int8_t leerFilas(){
    int8_t resultado = -1;
    if(gpio_get_level(ROW1) == 0)        resultado = 1;    
    else if(gpio_get_level(ROW2) == 0)   resultado = 2;   
    else if(gpio_get_level(ROW3) == 0)   resultado = 3;    
    else if(gpio_get_level(ROW4) == 0)   resultado = 4;    
    else resultado = -1;

    return resultado;
}

void app_main() {
    configurarTeclado();
    uint8_t columnaSeleccionada = 1;

    while (true) {
        rotaBit(columnaSeleccionada);
        vTaskDelay(pdMS_TO_TICKS(tiempoRetardo));
        int8_t fila = leerFilas();
        //si fila es menor o igual a 0, no se presiono ninguna tecla
        if(fila > 0){
            //Se presiono una tecla, cual fue?
            char teclaPresionada = '\0';
            if(columnaSeleccionada == 1 && fila == 1) teclaPresionada = '1';
            else if(columnaSeleccionada == 1 && fila == 2) teclaPresionada = '4';
            else if(columnaSeleccionada == 1 && fila == 3) teclaPresionada = '7';
            else if(columnaSeleccionada == 1 && fila == 4) teclaPresionada = '*';
            else if(columnaSeleccionada == 2 && fila == 1) teclaPresionada = '2';
            else if(columnaSeleccionada == 2 && fila == 2) teclaPresionada = '5';
            else if(columnaSeleccionada == 2 && fila == 3) teclaPresionada = '8';
            else if(columnaSeleccionada == 2 && fila == 4) teclaPresionada = '0';
            else if(columnaSeleccionada == 3 && fila == 1) teclaPresionada = '3';
            else if(columnaSeleccionada == 3 && fila == 2) teclaPresionada = '6';
            else if(columnaSeleccionada == 3 && fila == 3) teclaPresionada = '9';
            else if(columnaSeleccionada == 3 && fila == 4) teclaPresionada = '#';
            else if(columnaSeleccionada == 4 && fila == 1) teclaPresionada = 'A';
            else if(columnaSeleccionada == 4 && fila == 2) teclaPresionada = 'B';
            else if(columnaSeleccionada == 4 && fila == 3) teclaPresionada = 'C';
            else if(columnaSeleccionada == 4 && fila == 4) teclaPresionada = 'D';
            else{ teclaPresionada = '\0';}

            if(teclaPresionada != '\0'){
                ESP_LOGI(TAG, "Tecla presionada: %c", teclaPresionada);
                teclaPresionada = '\0';
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }



        columnaSeleccionada++;
        if(columnaSeleccionada > 4) columnaSeleccionada = 1;
        
    }
}
