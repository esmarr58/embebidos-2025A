#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define CANAL_ADC ADC_CHANNEL_9         // ADC1 Canal 9 = GPIO10 en ESP32-S3
#define ATENUACION_ADC ADC_ATTEN_DB_12
#define RESOLUCION_ADC ADC_BITWIDTH_12
#define VOLTAJE_REFERENCIA 3300         // mV
#define INTERVALO_US 10000              // 10 ms
#define MUESTRAS_POR_SEGUNDO 100        // 100 muestras por segundo

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;

static TaskHandle_t tarea_temp_handle = NULL;  // Handle de la tarea

// Función para inicializar la calibración del ADC
bool inicializar_calibracion() {
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ATENUACION_ADC,
        .bitwidth = RESOLUCION_ADC,
    };
    return adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle) == ESP_OK;
}

// Función para inicializar el ADC
void init_adc() {
    adc_oneshot_unit_init_cfg_t config_adc = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&config_adc, &adc_handle));

    adc_oneshot_chan_cfg_t config_canal = {
        .atten = ATENUACION_ADC,
        .bitwidth = RESOLUCION_ADC,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, CANAL_ADC, &config_canal));

    if (!inicializar_calibracion()) {
        printf("Advertencia: No se pudo inicializar la calibración. Usando cálculo aproximado.\n");
    }
}

// Función para leer temperatura desde el ADC
float leer_temperatura() {
    int valor_bruto;
    float temperatura = 0.0;

    if (adc_oneshot_read(adc_handle, CANAL_ADC, &valor_bruto) == ESP_OK) {
        if (cali_handle != NULL) {
            int voltaje_calibrado;
            if (adc_cali_raw_to_voltage(cali_handle, valor_bruto, &voltaje_calibrado) == ESP_OK) {
                temperatura = voltaje_calibrado / 10.0;  // LM35: 10 mV/°C
            }
        } else {
            float voltaje = (valor_bruto * VOLTAJE_REFERENCIA) / 4095.0;
            temperatura = voltaje / 10.0;
        }
    }

    return temperatura;
}

// ISR del temporizador: notifica a la tarea
static bool IRAM_ATTR on_timer_alarm(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(tarea_temp_handle, &xHigherPriorityTaskWoken);
    return (xHigherPriorityTaskWoken == pdTRUE);
}

// Tarea que realiza la lectura y promediado
void tarea_temperatura(void *pvParameters) {
    float acumulador = 0.0;
    int muestras = 0;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Espera notificación de la ISR

        float temp = leer_temperatura();
        acumulador += temp;
        muestras++;

        if (muestras >= MUESTRAS_POR_SEGUNDO) {
            float promedio = acumulador / muestras;
            printf("Promedio de temperatura (1s): %.2f °C\n", promedio);
            acumulador = 0.0;
            muestras = 0;
        }
    }
}

void app_main() {
    init_adc();  // Inicializar ADC

    // Crear la tarea que leerá y promediará la temperatura
    xTaskCreate(tarea_temperatura, "TareaTemperatura", 2048, NULL, 5, &tarea_temp_handle);

    // Configurar el temporizador
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,  // 1 MHz = 1 us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = INTERVALO_US,  // 10 ms
        .reload_count = 0,
        .flags.auto_reload_on_alarm = true,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = on_timer_alarm,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}
