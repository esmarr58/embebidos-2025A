#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define CANAL_ADC ADC_CHANNEL_9         // Canal ADC para el LM35
#define ATENUACION_ADC ADC_ATTEN_DB_12  // Atenuación de 12 dB para rango de 0-3.3V
#define RESOLUCION_ADC ADC_BITWIDTH_12  // Resolución de 12 bits
#define VOLTAJE_REFERENCIA 3300         // Voltaje de referencia en mV (3.3V)

static adc_oneshot_unit_handle_t adc_handle = NULL;
static adc_cali_handle_t cali_handle = NULL;

// Función para inicializar y configurar la calibración del ADC
bool inicializar_calibracion() {
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ATENUACION_ADC,
        .bitwidth = RESOLUCION_ADC,
    };
    return adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle) == ESP_OK;
}

// Función para inicializar y configurar el ADC
void init_adc() {
    // Configuración del ADC en modo captura única
    adc_oneshot_unit_init_cfg_t config_adc = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&config_adc, &adc_handle));

    // Configuración del canal y la atenuación
    adc_oneshot_chan_cfg_t config_canal = {
        .atten = ATENUACION_ADC,
        .bitwidth = RESOLUCION_ADC,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, CANAL_ADC, &config_canal));

    // Inicializar la calibración
    if (!inicializar_calibracion()) {
        printf("Advertencia: No se pudo inicializar la calibración. Usando cálculo aproximado.\n");
    }
}

// Función para leer temperatura del LM35
float leer_temperatura() {
    int valor_bruto;
    float temperatura = 0.0;
    
    // Leer valor del ADC
    ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, CANAL_ADC, &valor_bruto));

    if (cali_handle != NULL) {
        // Usar calibración si está disponible
        int voltaje_calibrado;
        if (adc_cali_raw_to_voltage(cali_handle, valor_bruto, &voltaje_calibrado) == ESP_OK) {
            // LM35: 10mV por °C (0V=0°C, 100mV=10°C, etc.)
            temperatura = voltaje_calibrado / 10.0;
        }
    } else {
        // Cálculo aproximado si no hay calibración
        float voltaje = (valor_bruto * VOLTAJE_REFERENCIA) / 4095.0;
        temperatura = voltaje / 10.0;
    }
    
    return temperatura;
}

void app_main() {
    // Inicialización del ADC
    init_adc();

    while (1) {
        float temperatura = leer_temperatura();
        
        printf("Temperatura: %.2f °C\n", temperatura);

        // Esperar un segundo antes de la siguiente lectura
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Limpieza (aunque en app_main normalmente no se llega aquí)
    if (cali_handle != NULL) {
        adc_cali_delete_scheme_curve_fitting(cali_handle);
    }
    if (adc_handle != NULL) {
        adc_oneshot_del_unit(adc_handle);
    }
}
