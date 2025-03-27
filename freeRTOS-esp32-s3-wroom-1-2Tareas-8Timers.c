#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"

static const char *TAG = "DualCoreTimers";

// Callbacks para los timers (1 por tarea)
void timer_callback_1(TimerHandle_t xTimer) { ESP_DRAM_LOGI(TAG, "Timer 1 ejecutado (Core %d)", xPortGetCoreID());}
void timer_callback_2(TimerHandle_t xTimer) { ESP_DRAM_LOGI(TAG, "Timer 2 ejecutado (Core %d)", xPortGetCoreID()); }
void timer_callback_3(TimerHandle_t xTimer) { ESP_DRAM_LOGI(TAG, "Timer 3 ejecutado (Core %d)", xPortGetCoreID()); }
void timer_callback_4(TimerHandle_t xTimer) { ESP_DRAM_LOGI(TAG, "Timer 4 ejecutado (Core %d)", xPortGetCoreID()); }
void timer_callback_5(TimerHandle_t xTimer) { ESP_DRAM_LOGI(TAG, "Timer 5 ejecutado (Core %d)", xPortGetCoreID()); }
void timer_callback_6(TimerHandle_t xTimer) { ESP_DRAM_LOGI(TAG, "Timer 6 ejecutado (Core %d)", xPortGetCoreID()); }
void timer_callback_7(TimerHandle_t xTimer) { ESP_DRAM_LOGI(TAG, "Timer 7 ejecutado (Core %d)", xPortGetCoreID()); }
void timer_callback_8(TimerHandle_t xTimer) { ESP_DRAM_LOGI(TAG, "Timer 8 ejecutado (Core %d)", xPortGetCoreID()); }

// Tarea para Core 0
void task_core_0(void *pvParameters) {

     // Crear los 8 timers (periódicos, auto-reload)
    TimerHandle_t timers[4];
    timers[0] = xTimerCreate("Timer1", pdMS_TO_TICKS(10), pdTRUE, (void *)0, timer_callback_1);
    timers[1] = xTimerCreate("Timer2", pdMS_TO_TICKS(10), pdTRUE, (void *)1, timer_callback_2);
    timers[2] = xTimerCreate("Timer3", pdMS_TO_TICKS(2000), pdTRUE, (void *)2, timer_callback_3);
    timers[3] = xTimerCreate("Timer4", pdMS_TO_TICKS(2500), pdTRUE, (void *)3, timer_callback_4);


    // Iniciar todos los timers
    for (int i = 0; i < 4; i++) {
        if (timers[i] != NULL) {
            xTimerStart(timers[i], 0);
        }
    }

    while (1) {
        ESP_DRAM_LOGI(TAG, "Tarea del (Core %d)", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Tarea para Core 1
void task_core_1(void *pvParameters) {
     // Crear los 8 timers (periódicos, auto-reload)
    TimerHandle_t timers[4];
    timers[0] = xTimerCreate("Timer1", pdMS_TO_TICKS(10), pdTRUE, (void *)0, timer_callback_5);
    timers[1] = xTimerCreate("Timer2", pdMS_TO_TICKS(10), pdTRUE, (void *)1, timer_callback_6);
    timers[2] = xTimerCreate("Timer3", pdMS_TO_TICKS(2100), pdTRUE, (void *)2, timer_callback_7);
    timers[3] = xTimerCreate("Timer4", pdMS_TO_TICKS(2600), pdTRUE, (void *)3, timer_callback_8);


    // Iniciar todos los timers
    for (int i = 0; i < 4; i++) {
        if (timers[i] != NULL) {
            xTimerStart(timers[i], 0);
        }
    }

    while (1) {
        ESP_DRAM_LOGI(TAG, "Tarea del (Core %d)", xPortGetCoreID());
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void) {
    // Inicializar el log por serial
    esp_log_level_set(TAG, ESP_LOG_INFO);

   
    // Crear tareas en ambos núcleos
    xTaskCreatePinnedToCore(task_core_0, "TaskCore0", 8192, NULL, configMAX_PRIORITIES - 1, NULL, 0); // Core 0
    xTaskCreatePinnedToCore(task_core_1, "TaskCore1", 8192, NULL, configMAX_PRIORITIES - 1, NULL, 1); // Core 1

    ESP_LOGI(TAG, "Sistema iniciado: 2 núcleos + 8 timers");
}
