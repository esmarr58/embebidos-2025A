/*
 * Contador 00–99 en display de 7 segmentos (2 dígitos, cátodo común)
 * Multiplexeo con transistores NPN (2N3904)
 * ESP32-S3 con ESP-IDF y PlatformIO
 *
 * NOTA:
 * - Cada segmento debe llevar resistencia limitadora (220–470 Ω).
 * - Emisor de cada 2N3904 a GND.
 * - Colector al cátodo común de cada dígito.
 * - Base a GPIO con resistencia (4.7k–10k).
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h" // Para imprimir en el monitor serial

// ============================
// Definición de pines (ajusta)
// ============================
#define SEG_A    7
#define SEG_B    46
#define SEG_C    16
#define SEG_D    15
#define SEG_E    17
#define SEG_F    8
#define SEG_G    18

#define BUTTON_PIN 11

#define DIG_DECENAS   4   // Transistor dígito izquierdo
#define DIG_UNIDADES  5   // Transistor dígito derecho

#define CANAL_ADC ADC_CHANNEL_9         
#define ATENUACION_ADC ADC_ATTEN_DB_12   // Atenuación de 12 dB para rango de 0 - 3.3V
#define VREF_POR_DEFECTO 1100            // Valor de referencia de voltaje (mV)

// Variables globales
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t cali_handle;
static const char* TAG = "BUTTON_COUNTER";

// Cola para manejar eventos del botón desde la ISR
static QueueHandle_t button_queue = NULL;

// Estructura para enviar eventos a través de la cola
typedef struct {
    uint32_t pin;
    uint32_t state;
} button_event_t;

// ============================
// Handler de interrupción del botón
// ============================
static void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    button_event_t event = {
        .pin = gpio_num,
        .state = gpio_get_level(gpio_num)
    };
    
    // Enviar evento a la cola (desde ISR)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(button_queue, &event, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// ============================
// Inicializar interrupción del botón
// ============================
static void init_button_interrupt(void) {
    // Crear cola para eventos del botón
    button_queue = xQueueCreate(10, sizeof(button_event_t));
    
    // Configurar GPIO del botón como entrada con pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // Interrupción en flanco descendente (botón presionado)
    };
    gpio_config(&io_conf);
    
    // Instalar servicio de interrupciones GPIO
    gpio_install_isr_service(0);
    
    // Agregar handler de interrupción para el botón
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, (void*) BUTTON_PIN);
}

// Función para inicializar y configurar la calibración del ADC
bool inicializar_calibracion() {
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ATENUACION_ADC,
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle) == ESP_OK;
}

// Función para inicializar y configurar el ADC
void init_adc() {
    // Configuración del ADC en modo captura única
    adc_oneshot_unit_init_cfg_t config_adc = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&config_adc, &adc_handle);

    // Configuración del canal y la atenuación
    adc_oneshot_chan_cfg_t config_canal = {
        .atten = ATENUACION_ADC,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, CANAL_ADC, &config_canal);

    // Inicializar la calibración
    
    if (!inicializar_calibracion()) {
        printf("Error al inicializar la calibración\n");
    }
    
}

// ============================
// Tabla de segmentos (cátodo común)
// ============================
// bit0=a, bit1=b, bit2=c, bit3=d, bit4=e, bit5=f, bit6=g
static const uint8_t mascara_digito[10] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F  // 9
};

static const gpio_num_t pines_segmento[7] = {
    SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G
};

// -----------------------------
// Inicialización estilo original
// -----------------------------
static void inicializar_pines(void)
{
    // Inicializar segmentos
    gpio_reset_pin(SEG_A); gpio_set_direction(SEG_A, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_B); gpio_set_direction(SEG_B, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_C); gpio_set_direction(SEG_C, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_D); gpio_set_direction(SEG_D, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_E); gpio_set_direction(SEG_E, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_F); gpio_set_direction(SEG_F, GPIO_MODE_OUTPUT);
    gpio_reset_pin(SEG_G); gpio_set_direction(SEG_G, GPIO_MODE_OUTPUT);

    // Inicializar transistores de dígitos
    gpio_reset_pin(DIG_DECENAS); gpio_set_direction(DIG_DECENAS, GPIO_MODE_OUTPUT);
    gpio_reset_pin(DIG_UNIDADES); gpio_set_direction(DIG_UNIDADES, GPIO_MODE_OUTPUT);

    // Estado inicial: todo apagado
    for (int i = 0; i < 7; i++) {
        gpio_set_level(pines_segmento[i], 0);
    }
    gpio_set_level(DIG_DECENAS, 0);
    gpio_set_level(DIG_UNIDADES, 0);
}

// -----------------------------
// Apagar ambos dígitos
// -----------------------------
static inline void deshabilitar_digitos(void)
{
    gpio_set_level(DIG_DECENAS, 0);
    gpio_set_level(DIG_UNIDADES, 0);
}

// -----------------------------
// Escribir segmentos
// -----------------------------
static inline void escribir_segmentos(uint8_t mascara)
{
    for (int b = 0; b < 7; b++) {
        int nivel = (mascara >> b) & 0x01;
        gpio_set_level(pines_segmento[b], nivel);
    }
}

// -----------------------------
// Mostrar un dígito (decena o unidad)
// -----------------------------
static inline void mostrar_un_digito(uint8_t valor, bool es_decena)
{
    if (valor > 9) valor = 0;

    deshabilitar_digitos(); // evitar ghosting
    escribir_segmentos(mascara_digito[valor]);

    if (es_decena) {
        gpio_set_level(DIG_DECENAS, 1);
    } else {
        gpio_set_level(DIG_UNIDADES, 1);
    }
}

// -----------------------------
// Separar decenas y unidades
// -----------------------------
static inline void separar_decenas_unidades(uint8_t n, uint8_t *dec, uint8_t *uni)
{
    if (n > 99) n = 99;
    *dec = n / 10;
    *uni = n % 10;
}

// -----------------------------
// Refrescar display durante tiempo_ms
// -----------------------------
static void refrescar_display(uint8_t dec, uint8_t uni, uint32_t tiempo_ms)
{
    // Cada ciclo: 1 ms decenas + 1 ms unidades = 2 ms
    uint32_t ciclos = tiempo_ms / 20;
    if (ciclos == 0) ciclos = 1;

    for (uint32_t i = 0; i < ciclos; i++) {
        mostrar_un_digito(dec, true);
        vTaskDelay(pdMS_TO_TICKS(10));   // 1 ms

        mostrar_un_digito(uni, false);
        vTaskDelay(pdMS_TO_TICKS(10));   // 1 ms
    }

    deshabilitar_digitos();
}

// ============================
// Programa principal
// ============================
void app_main(void)
{
    inicializar_pines();
    init_adc();
    init_button_interrupt();

    bool formato = true;
    int count = 0;
    button_event_t event;

    while (1) {
        // Leer ADC y procesar temperatura
        int valor_bruto;
        adc_oneshot_read(adc_handle, CANAL_ADC, &valor_bruto);
        int voltaje = 3300 * valor_bruto / 4096;
        float temperatura = voltaje / 10;
        float medicion = temperatura;

        // Procesar eventos del botón desde la cola (sin polling)
        if (xQueueReceive(button_queue, &event, 0) == pdTRUE) {
            // El botón fue presionado (flanco descendente)
            formato = !formato;
            if(formato){
                medicion = temperatura;
            } else {
                medicion = (temperatura * 9 / 5) + 32;
            }
            count++;
            ESP_LOGI(TAG, "Botón presionado (ISR), cuenta: %d", count);
        }

        // Mostrar en display
        uint8_t d, u;
        separar_decenas_unidades((int)medicion, &d, &u);
        refrescar_display(d, u, 1000);
        printf("Valor ADC: %d, Voltaje: %d mV, Temperatura: %.1f°%s\n", 
               valor_bruto, voltaje, medicion, formato ? "C" : "F");
    }
}
