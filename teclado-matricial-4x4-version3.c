#include "sdkconfig.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include "esp_task_wdt.h"


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

volatile int tiempoRetardo = 10;

uint8_t columnas[4] = {COL1, COL2, COL3, COL4};
uint8_t filas[4] = {ROW1,ROW2,ROW3,ROW4};
volatile uint8_t columnaSeleccionada = -1;
volatile bool teclaPresionada = false;

// Mapa del teclado 4x4
char mapaTeclado[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

static const char *TAG = "Teclado4x4:";

volatile char tecla = '-';  // Declaración de la bandera para el primer pin

static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    columnaSeleccionada++;
    if(columnaSeleccionada > 3) columnaSeleccionada = -1;
    
     if(columnaSeleccionada == 0){
        gpio_set_level(COL1, 0);
        gpio_set_level(COL2, 1);
        gpio_set_level(COL3, 1);
        gpio_set_level(COL4, 1);
    }
    else if(columnaSeleccionada == 1){
        gpio_set_level(COL1, 1);
        gpio_set_level(COL2, 0);
        gpio_set_level(COL3, 1);
        gpio_set_level(COL4, 1);
    }
    else if(columnaSeleccionada == 2){
        gpio_set_level(COL1, 1);
        gpio_set_level(COL2, 1);
        gpio_set_level(COL3, 0);
        gpio_set_level(COL4, 1);
    }
    else if(columnaSeleccionada == 3){
        gpio_set_level(COL1, 1);
        gpio_set_level(COL2, 1);
        gpio_set_level(COL3, 1);
        gpio_set_level(COL4, 0);
    }
    return true;
}

static void IRAM_ATTR funcionInterrupcion1(void *arg) {
    tecla = mapaTeclado[0][columnaSeleccionada];
    teclaPresionada = true;
}

// Manejador de interrupción para el segundo pin
static void IRAM_ATTR funcionInterrupcion2(void *arg) {
    tecla = mapaTeclado[1][columnaSeleccionada];
    teclaPresionada = true;
}

// Manejador de interrupción para el tercer pin
static void IRAM_ATTR funcionInterrupcion3(void *arg) {
     tecla = mapaTeclado[2][columnaSeleccionada];
    teclaPresionada = true;
}
static void IRAM_ATTR funcionInterrupcion4(void *arg) {
     tecla = mapaTeclado[3][columnaSeleccionada];
    teclaPresionada = true;
}



void configurarTeclado(){
    //Configurar como salida a las columnas.
    for(int i = 0; i<4; i++){
        //i=0,1,2,3
        gpio_reset_pin(columnas[i]);
        gpio_set_direction(columnas[i],GPIO_MODE_OUTPUT);
        gpio_set_level(columnas[i],1);
    }

    gpio_reset_pin(ROW1);
    gpio_set_direction(ROW1, GPIO_MODE_INPUT);
    gpio_set_intr_type(ROW1, GPIO_INTR_NEGEDGE);  // Interrupción en flanco de bajada

    gpio_reset_pin(ROW2);
    gpio_set_direction(ROW2, GPIO_MODE_INPUT);
    gpio_set_intr_type(ROW2, GPIO_INTR_NEGEDGE);  // Interrupción en flanco de bajada

    gpio_reset_pin(ROW3);
    gpio_set_direction(ROW3, GPIO_MODE_INPUT);
    gpio_set_intr_type(ROW3, GPIO_INTR_NEGEDGE);  // Interrupción en flanco de bajada

    gpio_reset_pin(ROW4);
    gpio_set_direction(ROW4, GPIO_MODE_INPUT);
    gpio_set_intr_type(ROW4, GPIO_INTR_NEGEDGE);  // Interrupción en flanco de bajada

    // Instalación del servicio ISR
    gpio_install_isr_service(0);
    
    // Asignar manejadores de interrupciones a cada pin
    gpio_isr_handler_add(ROW1, funcionInterrupcion1, NULL);
    gpio_isr_handler_add(ROW2, funcionInterrupcion2, NULL);
    gpio_isr_handler_add(ROW3, funcionInterrupcion3, NULL);
    gpio_isr_handler_add(ROW4, funcionInterrupcion4, NULL);

}
//Maquinas de estado FSM (Finite State Machine)
void rotaBit(uint8_t estadoActual){
   

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

void configurarTimer(){
      // Configuración del timer
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,  // Fuente de reloj por defecto (APB o XTAL)
        .direction = GPTIMER_COUNT_UP,  // Contar hacia arriba
        .resolution_hz = 1000000,  // Configurar a 1 MHz para contar en microsegundos
    };
    gptimer_new_timer(&timer_config, &gptimer);  // Crear un nuevo timer con la configuración

    // Configurar la alarma del timer
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = 50000,  // 1 segundo en microsegundos
        .reload_count = 0,  // Reiniciar el contador a 0 después de alcanzar la alarma
        .flags.auto_reload_on_alarm = true,  // Recargar automáticamente la alarma
    };
    gptimer_set_alarm_action(gptimer, &alarm_config);

    // Registrar la función ISR para la alarma del timer
    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_timer_alarm,  // Llamar a la función on_timer_alarm al activarse la alarma
    };
    gptimer_register_event_callbacks(gptimer, &cbs, NULL);

    // Iniciar el timer
    gptimer_enable(gptimer);  // Habilitar el timer
    gptimer_start(gptimer);   // Iniciar el timer
}

void app_main() {
    configurarTeclado();
    configurarTimer();
    esp_task_wdt_deinit();


    while (true) {
       
       if(teclaPresionada){
                teclaPresionada = false;
                ESP_LOGI(TAG, "Tecla presionada: %c", tecla);
                tecla = '-';
                
            }
        vTaskDelay(pdMS_TO_TICKS(tiempoRetardo));        
        
        
    }
}
