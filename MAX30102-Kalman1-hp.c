#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ===================== Configuración OPTIMIZADA =====================
#define I2C_SCL_IO              9
#define I2C_SDA_IO              10
#define I2C_FREQ_HZ             400000
#define UART0_BAUD              2000000   // 3 Mbps para Serial Plotter
#define MAX30102_ADDR           0x57

// ===================== Configuración FFT =====================
#define FFT_SIZE                64
#define SAMPLE_RATE_HZ          100.0f
#define BPM_MIN                 40
#define BPM_MAX                 180

// ===================== Registros MAX30102 =====================
#define REG_INTR_STATUS_1       0x00
#define REG_INTR_STATUS_2       0x01
#define REG_INTR_ENABLE_1       0x02
#define REG_INTR_ENABLE_2       0x03
#define REG_FIFO_WR_PTR         0x04
#define REG_OVF_COUNTER         0x05
#define REG_FIFO_RD_PTR         0x06
#define REG_FIFO_DATA           0x07
#define REG_FIFO_CONFIG         0x08
#define REG_MODE_CONFIG         0x09
#define REG_SPO2_CONFIG         0x0A
#define REG_LED1_PA             0x0C
#define REG_LED2_PA             0x0D
#define REG_REV_ID              0xFE
#define REG_PART_ID             0xFF

// ===================== Bits/campos =====================
#define MODE_RESET_BIT          (1<<6)
#define MODE_HEART_RATE         0x02
#define INTR_A_FULL_EN          (1<<7)
#define INTR_PPG_RDY_EN         (1<<6)
#define FIFO_AVG_4              (2<<5)
#define FIFO_ROLLOVER_EN        (1<<4)
#define FIFO_A_FULL(n)          ((n)&0x0F)
#define SPO2_ADC_RANGE_4096     (1<<5)
#define SPO2_SR_100HZ           (1<<2)
#define SPO2_PW_411US           (3<<0)

// ===================== Detección de dedo =====================
#define FINGER_DETECTION_THRESHOLD 50000  // Ajusta según tu sensor
#define MIN_SIGNAL_STRENGTH 1000          // Mínima amplitud para considerar señal válida

// ===================== Handles =====================
static const char *TAG = "HR_KALMAN";
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t i2c_dev = NULL;

// ===================== Estructura Filtro de Kalman =====================
typedef struct {
    float q;        // Ruido del proceso
    float r;        // Ruido de la medición
    float x;        // Valor estimado
    float p;        // Error de estimación
    float k;        // Ganancia de Kalman
} kalman_filter_t;

// ===================== Estructuras FFT =====================
typedef struct { float real, imag; } complex_t;

static float fft_samples[FFT_SIZE];
static complex_t fft_buffer[FFT_SIZE];
static int sample_count = 0;
static uint32_t last_fft_time = 0;
static float current_bpm = 0.0f;

// ===================== Filtros =====================
static kalman_filter_t kalman_filter;
static float hp_prev_input = 0.0f, hp_prev_output = 0.0f;
static const float hp_a1 = -0.9682458366f;
static const float hp_b0 = 0.9841229183f;
static const float hp_b1 = -0.9841229183f;

// ===================== Inicialización Filtro Kalman =====================
static void kalman_init(kalman_filter_t *kf, float q, float r, float initial_value) {
    kf->q = q;
    kf->r = r;
    kf->x = initial_value;
    kf->p = 1.0f;  // Error inicial
    kf->k = 0.0f;
}

// ===================== Paso del Filtro Kalman =====================
static float kalman_update(kalman_filter_t *kf, float measurement) {
    // Predicción
    kf->p = kf->p + kf->q;
    
    // Actualización
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1 - kf->k) * kf->p;
    
    return kf->x;
}

// ===================== Filtro Paso Alto =====================
static inline float highpass_filter(float input) {
    float output = hp_b0 * input + hp_b1 * hp_prev_input - hp_a1 * hp_prev_output;
    hp_prev_input = input;
    hp_prev_output = output;
    return output;
}



static bool is_finger_present(float *samples, int num_samples) {
    // Calcular estadísticas básicas de la señal
    float sum = 0, min_val = samples[0], max_val = samples[0];
    
    for (int i = 0; i < num_samples; i++) {
        sum += samples[i];
        if (samples[i] < min_val) min_val = samples[i];
        if (samples[i] > max_val) max_val = samples[i];
    }
    
    float amplitude = max_val - min_val;
    float average = sum / num_samples;
    
    // Detectar por amplitud de señal
    if (amplitude < MIN_SIGNAL_STRENGTH) {
        return false;  // Señal muy plana = no hay dedo
    }
    
    // Detectar por nivel DC (RAW signal level)
    // Si el promedio está muy bajo, probablemente no hay dedo
    if (average < -FINGER_DETECTION_THRESHOLD || average > FINGER_DETECTION_THRESHOLD) {
        return false;
    }
    
    return true;
}

// ===================== FFT =====================
static void fft(complex_t *x, int n) {
    if (n <= 1) return;
    
    complex_t even[n/2], odd[n/2];
    for (int i = 0; i < n/2; i++) {
        even[i] = x[i*2];
        odd[i] = x[i*2 + 1];
    }
    
    fft(even, n/2);
    fft(odd, n/2);
    
    for (int k = 0; k < n/2; k++) {
        float angle = -2 * M_PI * k / n;
        complex_t t = {
            cosf(angle) * odd[k].real - sinf(angle) * odd[k].imag,
            cosf(angle) * odd[k].imag + sinf(angle) * odd[k].real
        };
        
        x[k].real = even[k].real + t.real;
        x[k].imag = even[k].imag + t.imag;
        x[k + n/2].real = even[k].real - t.real;
        x[k + n/2].imag = even[k].imag - t.imag;
    }
}

static float calculate_bpm_from_fft(void) {
    // Aplicar ventana y preparar FFT
    for (int i = 0; i < FFT_SIZE; i++) {
        float window = 0.5f * (1.0f - cosf(2 * M_PI * i / (FFT_SIZE - 1)));
        fft_buffer[i].real = fft_samples[i] * window;
        fft_buffer[i].imag = 0;
    }
    
    fft(fft_buffer, FFT_SIZE);
    
    // Buscar pico en rango cardíaco
    int min_bin = (int)(BPM_MIN / 60.0f * FFT_SIZE / SAMPLE_RATE_HZ);
    int max_bin = (int)(BPM_MAX / 60.0f * FFT_SIZE / SAMPLE_RATE_HZ);
    if (min_bin < 1) min_bin = 1;
    if (max_bin >= FFT_SIZE/2) max_bin = FFT_SIZE/2 - 1;
    
    float max_mag = 0;
    int peak_bin = min_bin;
    
    for (int i = min_bin; i <= max_bin; i++) {
        float mag = sqrtf(fft_buffer[i].real * fft_buffer[i].real + 
                         fft_buffer[i].imag * fft_buffer[i].imag);
        if (mag > max_mag) {
            max_mag = mag;
            peak_bin = i;
        }
    }
    
    float bpm = (float)peak_bin * SAMPLE_RATE_HZ * 60.0f / FFT_SIZE;
    return (bpm >= BPM_MIN && bpm <= BPM_MAX) ? bpm : 0.0f;
}

// ===================== UART0 ALTA VELOCIDAD =====================
static void uart0_setup(uint32_t baud) {
    const uart_port_t uart_num = UART_NUM_0;

    uart_config_t cfg = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(uart_num, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 2048, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(uart_num);
    setvbuf(stdout, NULL, _IOLBF, 0);
}

// ===================== I2C helpers =====================
static esp_err_t i2c_init_bus(void) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = { .enable_internal_pullup = 1 },
    };
    return i2c_new_master_bus(&bus_cfg, &i2c_bus);
}

static esp_err_t i2c_add_dev(uint8_t addr) {
    i2c_device_config_t dev_cfg = {
        .device_address = addr,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_dev);
}

static inline esp_err_t reg_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(i2c_dev, &reg, 1, data, len, 1000);
}

static inline esp_err_t reg_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(i2c_dev, buf, sizeof(buf), 1000);
}

// ===================== MAX30102 functions =====================
static esp_err_t max30102_reset(void) {
    ESP_RETURN_ON_ERROR(reg_write(REG_MODE_CONFIG, MODE_RESET_BIT), TAG, "RESET");
    for (int i = 0; i < 50; ++i) {
        uint8_t m=0;
        ESP_RETURN_ON_ERROR(reg_read(REG_MODE_CONFIG, &m, 1), TAG, "MODE read");
        if ((m & MODE_RESET_BIT) == 0) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t fifo_clear(void) {
    ESP_RETURN_ON_ERROR(reg_write(REG_FIFO_WR_PTR, 0x00), TAG, "WR_PTR");
    ESP_RETURN_ON_ERROR(reg_write(REG_OVF_COUNTER, 0x00), TAG, "OVF");
    ESP_RETURN_ON_ERROR(reg_write(REG_FIFO_RD_PTR, 0x00), TAG, "RD_PTR");
    return ESP_OK;
}

static uint8_t fifo_level(void) {
    uint8_t wr=0, rd=0;
    if (reg_read(REG_FIFO_WR_PTR, &wr, 1) != ESP_OK) return 0;
    if (reg_read(REG_FIFO_RD_PTR, &rd, 1) != ESP_OK) return 0;
    return (uint8_t)((32 + wr - rd) & 0x1F);
}

static esp_err_t max30102_init_hr(void) {
    ESP_RETURN_ON_ERROR(max30102_reset(), TAG, "reset");
    uint8_t fifo_cfg = FIFO_AVG_4 | FIFO_ROLLOVER_EN | FIFO_A_FULL(8);
    ESP_RETURN_ON_ERROR(reg_write(REG_FIFO_CONFIG, fifo_cfg), TAG, "FIFO_CONFIG");
    ESP_RETURN_ON_ERROR(reg_write(REG_MODE_CONFIG, MODE_HEART_RATE), TAG, "MODE_HR");
    
    uint8_t spo2_cfg = SPO2_ADC_RANGE_4096 | SPO2_SR_100HZ | SPO2_PW_411US;
    ESP_RETURN_ON_ERROR(reg_write(REG_SPO2_CONFIG, spo2_cfg), TAG, "SPO2_CONFIG");
    ESP_RETURN_ON_ERROR(reg_write(REG_LED1_PA, 0x24), TAG, "LED1 Red");
    ESP_RETURN_ON_ERROR(reg_write(REG_LED2_PA, 0x00), TAG, "LED2 Off");
    ESP_RETURN_ON_ERROR(reg_write(REG_INTR_ENABLE_1, INTR_PPG_RDY_EN | INTR_A_FULL_EN), TAG, "INTR1");
    ESP_RETURN_ON_ERROR(reg_write(REG_INTR_ENABLE_2, 0x00), TAG, "INTR2");
    
    // Limpiar interrupciones
    uint8_t d;
    (void)reg_read(REG_INTR_STATUS_1, &d, 1);
    (void)reg_read(REG_INTR_STATUS_2, &d, 1);
    
    ESP_RETURN_ON_ERROR(fifo_clear(), TAG, "FIFO clr");
    return ESP_OK;
}

static inline bool read_sample_one(uint32_t *val) {
    uint8_t b[3];
    if (reg_read(REG_FIFO_DATA, b, sizeof(b)) != ESP_OK) return false;
    uint32_t v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    if (val) *val = v & 0x3FFFF;
    return true;
}

// ===================== app_main CON KALMAN =====================
void app_main(void) {
    esp_log_level_set("*", ESP_LOG_WARN);
    uart0_setup(UART0_BAUD);

    if (i2c_init_bus() != ESP_OK) {
        printf("ERR:I2C_BUS\r\n");
        while (1) vTaskDelay(pdMS_TO_TICKS(500));
    }

    while (1) {
        if (i2c_master_probe(i2c_bus, MAX30102_ADDR, 200) == ESP_OK) {
            if (i2c_add_dev(MAX30102_ADDR) == ESP_OK) break;
        }
        printf("ERR:NO_DEV\r\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    uint8_t part=0, rev=0;
    (void)reg_read(REG_PART_ID, &part, 1);
    (void)reg_read(REG_REV_ID, &rev, 1);
    printf("ID PART:0x%02X REV:0x%02X\r\n", part, rev);

    if (max30102_init_hr() != ESP_OK) {
        printf("ERR:INIT\r\n");
    }

    // Inicializar filtro de Kalman
    // Parámetros ajustables: 
    // - q (ruido proceso): más alto = más sensible a cambios
    // - r (ruido medición): más alto = más suavizado
    kalman_init(&kalman_filter, 0.01f, 0.1f, 20000.0f); // Valores iniciales optimizados para PPG

    printf("Iniciando medición BPM con Filtro Kalman @ %d bauds\r\n", UART0_BAUD);
    printf("Kalman Q:%.3f R:%.3f\r\n", kalman_filter.q, kalman_filter.r);
    printf("FFT_SIZE:%d\r\n", FFT_SIZE);

    char line[64];
    uint32_t sample_counter = 0;
    
    while (1) {
        uint8_t s1 = 0;
        if (reg_read(REG_INTR_STATUS_1, &s1, 1) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (s1 & (INTR_PPG_RDY_EN | INTR_A_FULL_EN)) {
            uint8_t n = fifo_level();
            if (n == 0) n = 1;
            if (n > 8) n = 8;

            for (uint8_t k = 0; k < n; ++k) {
                uint32_t ppg_raw = 0;
                if (!read_sample_one(&ppg_raw)) break;
                
                // Pipeline de procesamiento de señal:
                // 1. Filtro paso alto (elimina drift)
                float filtered_hp = highpass_filter((float)ppg_raw);
                
                // 2. Filtro de Kalman (suavizado y reducción de ruido)
                float filtered_kalman = kalman_update(&kalman_filter, filtered_hp);
                
                // Almacenar para FFT (usamos la señal con Kalman)
                if (sample_count < FFT_SIZE) {
                    fft_samples[sample_count] = filtered_kalman;
                    sample_count++;
                }
                
                // Enviar todas las señales para comparación
                
                //printf("RAW:%d,HP:%d,KAL:%d\r\n", (int)ppg_raw, (int)(filtered_hp * 10.0f),  (int)(filtered_kalman * 10.0f));  
                //printf("HP:%d,KAL:%d\r\n", (int)(filtered_hp * 50.0f),  (int)(filtered_kalman * 50.0f));               
                // printf("%u\r\n", (unsigned)ppg_raw);
                //printf("HP:%d\r\n", (int)(filtered_hp * 10.0f));
    
                
                sample_counter++;
            }
            
            // Calcular BPM periódicamente
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (sample_count >= FFT_SIZE && (current_time - last_fft_time) > 1500) {
                
                if (is_finger_present(fft_samples, FFT_SIZE)) {
                    current_bpm = calculate_bpm_from_fft();
                    
                    if (current_bpm > 0) {
                        printf("BPM:%.1f,KG:%.3f\r\n", current_bpm, kalman_filter.k);
                    } else {
                        printf("BPM:---,KG:%.3f\r\n", kalman_filter.k);
                    }
                } else {
                    printf("NO_FINGER\r\n");  // Indicar que no hay dedo
                    current_bpm = 0;
                }
                
                sample_count = 0;
                last_fft_time = current_time;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(3));
    }
}
