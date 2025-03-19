#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <unistd.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include <stdio.h>
#include "esp_task_wdt.h"
#include "driver/gptimer.h"

#define pin_catodo_displayUnidades 19
#define pin_catodo_displayDecenas  45 
#define intervaloTimer_us (10000)
#define intervaloTimer2_us (1000000)

#define segmento_A 1
#define segmento_B 2
#define segmento_C 42
#define segmento_D 41
#define segmento_E 40
#define segmento_F 39
#define segmento_G 38

#define cero    0x7E
#define uno     0x30
#define dos     0x6D
#define tres    0x79
#define cuatro  0x33
#define cinco   0x5B
#define seis    0x5F
#define siete   0x70
#define ocho    0x7f
#define nueve   0x7b
#define todosApagados   0x00


uint8_t numerosCodifiados[11] = {cero, uno, dos, tres, cuatro, cinco, seis, siete, ocho, nueve, todosApagados};

 uint8_t unidades = 0;
 uint8_t decenas  = 0;
 uint8_t contador = 0;

volatile bool activacionUnidades = 0;

static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    activacionUnidades = !activacionUnidades;

    return true;
}

static bool IRAM_ATTR on_timer2_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    contador++;
    if(contador>99) contador=0;
    return true;
}

void decodifica_Numero(uint8_t numero, uint8_t *decenas_var, uint8_t *unidades_var) {
    if (numero > 99) {
        printf("Número fuera de rango. Debe estar entre 0 y 99.\n");
        return;
    }
    
    *decenas_var = numero / 10;  // Extrae las decenas
    *unidades_var = numero % 10;  // Extrae las unidades
}

void task_core_0(void *pvParameters) {
    while (1) {
        printf("Ejecutando en Core 0\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Espera de 1 segundo
    }
}

void decodificaSegmentos(uint8_t display){
    //Parte baja
   gpio_set_level(segmento_G, (display & 0b00000001) >> 0);  
   gpio_set_level(segmento_F, (display & 0b00000010) >> 1);  
   gpio_set_level(segmento_E, (display & 0b00000100) >> 2);  
   gpio_set_level(segmento_D, (display & 0b00001000) >> 3);  

   //Parte alta
   gpio_set_level(segmento_C, (display & 0b00010000) >> 4);  
   gpio_set_level(segmento_B, (display & 0b00100000) >> 5); 
   gpio_set_level(segmento_A, (display & 0b01000000) >> 6);  
}

void task_core_1(void *pvParameters) {   
    while (1) {
        decodifica_Numero(contador,&decenas, &unidades);
        if(activacionUnidades) { 
            gpio_set_level(pin_catodo_displayUnidades, 1);   
            gpio_set_level(pin_catodo_displayDecenas, 0);  
            //Apagar los segmentos
            decodificaSegmentos(numerosCodifiados[11]);        

            decodificaSegmentos(numerosCodifiados[unidades]);        
        }
        else         {  
            gpio_set_level(pin_catodo_displayUnidades, 0);   
            gpio_set_level(pin_catodo_displayDecenas, 1);   
            //Apagar los segmentos
            decodificaSegmentos(numerosCodifiados[11]);  

            decodificaSegmentos(numerosCodifiados[decenas]);         
        }
        //printf("Ejecutando en Core 1\n");
        //vTaskDelay(pdMS_TO_TICKS(1000)); // Espera de 1 segundo
    }
}




void app_main(void) {
    esp_task_wdt_deinit();

    gpio_reset_pin(pin_catodo_displayUnidades);
    gpio_reset_pin(pin_catodo_displayDecenas);
    gpio_reset_pin(segmento_A);
    gpio_reset_pin(segmento_B);
    gpio_reset_pin(segmento_C);
    gpio_reset_pin(segmento_D);
    gpio_reset_pin(segmento_E);
    gpio_reset_pin(segmento_F);
    gpio_reset_pin(segmento_G);

    gpio_set_direction(pin_catodo_displayUnidades, GPIO_MODE_OUTPUT);
    gpio_set_direction(pin_catodo_displayDecenas, GPIO_MODE_OUTPUT);
    gpio_set_direction(segmento_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(segmento_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(segmento_C, GPIO_MODE_OUTPUT);
    gpio_set_direction(segmento_D, GPIO_MODE_OUTPUT);
    gpio_set_direction(segmento_E, GPIO_MODE_OUTPUT);
    gpio_set_direction(segmento_F, GPIO_MODE_OUTPUT);
    gpio_set_direction(segmento_G, GPIO_MODE_OUTPUT);


    printf("Iniciando programa en ESP32-S3 con FreeRTOS\n");

     // Configuración del timer
     gptimer_handle_t gptimer = NULL;
     gptimer_config_t timer_config = {
         .clk_src = GPTIMER_CLK_SRC_DEFAULT,  // Fuente de reloj por defecto (APB o XTAL)
         .direction = GPTIMER_COUNT_UP,  // Contar hacia arriba
         .resolution_hz = 1000000,  // Configurar a 1 MHz para contar en microsegundos
     };
     ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));  // Crear un nuevo timer con la configuración
 
     // Configurar la alarma del timer
     gptimer_alarm_config_t alarm_config = {
         .alarm_count = intervaloTimer_us,  // 100 microsegundos
         .flags.auto_reload_on_alarm = true,  // Recargar automáticamente la alarma
     };
     ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
 
     // Registrar la función ISR para la alarma del timer
     gptimer_event_callbacks_t cbs = {
         .on_alarm = on_timer_alarm,  // Llamar a la función on_timer_alarm al activarse la alarma
     };
     ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));
 
     // Iniciar el timer
     ESP_ERROR_CHECK(gptimer_enable(gptimer));  // Habilitar el timer
     ESP_ERROR_CHECK(gptimer_start(gptimer));   // Iniciar el timer


      // Configuración del segundo timer
    gptimer_handle_t gptimer2 = NULL;
    gptimer_config_t timer2_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz para contar en microsegundos
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer2_config, &gptimer2));  // Crear el segundo timer

    // Configurar la alarma del segundo timer
    gptimer_alarm_config_t alarm2_config = {
        .alarm_count = intervaloTimer2_us,  // 1 segundo en microsegundos
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer2, &alarm2_config));

    // Registrar la función ISR para el segundo timer
    gptimer_event_callbacks_t cbs2 = {
        .on_alarm = on_timer2_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer2, &cbs2, NULL));

    // Iniciar el segundo timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer2));
    ESP_ERROR_CHECK(gptimer_start(gptimer2));
    
    xTaskCreatePinnedToCore(
        task_core_0,   // Función de la tarea
        "TaskCore0",   // Nombre de la tarea
        2048,          // Tamaño del stack en bytes
        NULL,          // Parámetro de entrada
        1,             // Prioridad de la tarea
        NULL,          // Handle de la tarea
        0              // Núcleo en el que se ejecutará
    );

    xTaskCreatePinnedToCore(
        task_core_1,   // Función de la tarea
        "TaskCore1",   // Nombre de la tarea
        2048,          // Tamaño del stack en bytes
        NULL,          // Parámetro de entrada
        1,             // Prioridad de la tarea
        NULL,          // Handle de la tarea
        1              // Núcleo en el que se ejecutará
    );
}
