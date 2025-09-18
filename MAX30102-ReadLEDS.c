#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ===================== Pines / Parámetros I2C =====================
#define I2C_MASTER_SCL_IO           9      // GPIO SCL (ESP32-S3)
#define I2C_MASTER_SDA_IO           10     // GPIO SDA (ESP32-S3)
#define I2C_MASTER_FREQ_HZ          100000 // 100 kHz

// ===================== Dirección I2C MAX30102 =====================
#define MAX30102_ADDRESS            0x57   // típica en breakouts

// ===================== Tags / Handles I2C =====================
static const char *TAG = "MAX30102";
static i2c_master_bus_handle_t  i2c_bus_handle   = NULL;
static i2c_master_dev_handle_t  i2c_device_handle = NULL;

// ===================== Registros MAX30102 =====================
#define MAX30102_REG_INTR_STATUS_1   0x00
#define MAX30102_REG_INTR_STATUS_2   0x01
#define MAX30102_REG_INTR_ENABLE_1   0x02
#define MAX30102_REG_INTR_ENABLE_2   0x03
#define MAX30102_REG_FIFO_WR_PTR     0x04
#define MAX30102_REG_OVF_COUNTER     0x05
#define MAX30102_REG_FIFO_RD_PTR     0x06
#define MAX30102_REG_FIFO_DATA       0x07
#define MAX30102_REG_FIFO_CONFIG     0x08
#define MAX30102_REG_MODE_CONFIG     0x09
#define MAX30102_REG_SPO2_CONFIG     0x0A
#define MAX30102_REG_LED1_PA         0x0C   // Red
#define MAX30102_REG_LED2_PA         0x0D   // IR
#define MAX30102_REG_TEMP_INTEGER    0x1F   // (opcional) temp int
#define MAX30102_REG_TEMP_FRACTION   0x20   // (opcional) temp frac
#define MAX30102_REG_REV_ID          0xFE
#define MAX30102_REG_PART_ID         0xFF

// ===================== Bits útiles =====================
// MODE_CONFIG
#define MODE_SHDN_BIT                (1<<7)
#define MODE_RESET_BIT               (1<<6)
#define MODE_HEART_RATE              0x02
#define MODE_SPO2                    0x03
#define MODE_MULTI_LED               0x07

// INTR_ENABLE_1
#define INTR_A_FULL_EN               (1<<7)
#define INTR_PPG_RDY_EN              (1<<6)

// FIFO_CONFIG
// [7:5] sample avg (0=1,1=2,2=4,3=8,4=16,5=32)
#define FIFO_AVG_4                   (2<<5)
#define FIFO_ROLLOVER_EN             (1<<4)
#define FIFO_A_FULL(n)               ((n)&0x0F)  // umbral (0..15)

// SPO2_CONFIG
// [6:5] ADC range     (0=2048nA,1=4096nA,2=8192nA,3=16384nA)
// [4:2] sample rate   (0=50,1=100,2=200,3=400,4=800,5=1000,6=1600,7=3200 Hz)
// [1:0] pulse width   (0=69us(15b),1=118us(16b),2=215us(17b),3=411us(18b))
#define SPO2_ADC_RANGE_4096          (1<<5)
#define SPO2_SR_100HZ                (1<<2)
#define SPO2_PW_411US                (3<<0)

// ===================== I2C: init y helpers =====================
static esp_err_t i2c_master_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = {
            .enable_internal_pullup = 1, // Recomendado: pull-ups externos 4.7k~10k
        },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &i2c_bus_handle),
                        TAG, "i2c_new_master_bus falló");

    i2c_device_config_t dev_config = {
        .device_address = MAX30102_ADDRESS,
        .scl_speed_hz   = I2C_MASTER_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &i2c_device_handle),
                        TAG, "i2c_master_bus_add_device falló");

    // (Opcional) Comprobar presencia
    ESP_RETURN_ON_ERROR(i2c_master_probe(i2c_bus_handle, MAX30102_ADDRESS, 1000),
                        TAG, "MAX30102 no responde en 0x%02X", MAX30102_ADDRESS);

    ESP_LOGI(TAG, "I2C listo (SDA=%d, SCL=%d, %d Hz)", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    return ESP_OK;
}

static esp_err_t max30102_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(i2c_device_handle, &reg, 1, data, len, 1000);
}

static esp_err_t max30102_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(i2c_device_handle, buf, sizeof(buf), 1000);
}

static void max30102_clear_interrupts(void) {
    uint8_t dump;
    (void)max30102_read_reg(MAX30102_REG_INTR_STATUS_1, &dump, 1);
    (void)max30102_read_reg(MAX30102_REG_INTR_STATUS_2, &dump, 1);
}

// ===================== IDs del dispositivo =====================
static esp_err_t max30102_read_ids(uint8_t *part_id, uint8_t *rev_id) {
    uint8_t tmp;
    ESP_RETURN_ON_ERROR(max30102_read_reg(MAX30102_REG_PART_ID, &tmp, 1), TAG, "PART_ID");
    if (part_id) *part_id = tmp;
    ESP_RETURN_ON_ERROR(max30102_read_reg(MAX30102_REG_REV_ID, &tmp, 1), TAG, "REV_ID");
    if (rev_id) *rev_id = tmp;
    return ESP_OK;
}

// ===================== Reset / FIFO =====================
static esp_err_t max30102_reset(void) {
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_MODE_CONFIG, MODE_RESET_BIT), TAG, "Write RESET");
    for (int i = 0; i < 50; ++i) { // ~500 ms
        uint8_t m = 0;
        ESP_RETURN_ON_ERROR(max30102_read_reg(MAX30102_REG_MODE_CONFIG, &m, 1), TAG, "Read MODE_CONFIG");
        if ((m & MODE_RESET_BIT) == 0) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t max30102_clear_fifo(void) {
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_FIFO_WR_PTR, 0x00), TAG, "WR_PTR");
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_OVF_COUNTER, 0x00), TAG, "OVF");
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_FIFO_RD_PTR, 0x00), TAG, "RD_PTR");
    return ESP_OK;
}

static uint8_t max30102_fifo_level(void) {
    uint8_t wr=0, rd=0;
    if (max30102_read_reg(MAX30102_REG_FIFO_WR_PTR, &wr, 1) != ESP_OK) return 0;
    if (max30102_read_reg(MAX30102_REG_FIFO_RD_PTR, &rd, 1) != ESP_OK) return 0;
    return (uint8_t)((32 + wr - rd) & 0x1F); // FIFO 32 posiciones
}

// ===================== Init SpO2 básico =====================
static esp_err_t max30102_init_spo2_basic(void) {
    ESP_RETURN_ON_ERROR(max30102_reset(), TAG, "Reset");

    // FIFO: promedio x4, rollover ON, A_FULL alto (0x0F)
    uint8_t fifo_cfg = FIFO_AVG_4 | FIFO_ROLLOVER_EN | FIFO_A_FULL(0x0F);
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_FIFO_CONFIG, fifo_cfg), TAG, "FIFO_CONFIG");

    // Modo SpO2
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_MODE_CONFIG, MODE_SPO2), TAG, "MODE_SPO2");

    // Rango 4096nA, sample rate 100 Hz, pulse width 411 us (18-bit)
    uint8_t spo2_cfg = SPO2_ADC_RANGE_4096 | SPO2_SR_100HZ | SPO2_PW_411US;
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_SPO2_CONFIG, spo2_cfg), TAG, "SPO2_CONFIG");

    // Corriente LEDs ~10 mA (ajusta según saturación/ruido)
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_LED1_PA, 0x24), TAG, "LED1 Red");
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_LED2_PA, 0x24), TAG, "LED2 IR");

    // Interrupciones: A_FULL y PPG_RDY
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_INTR_ENABLE_1, INTR_A_FULL_EN | INTR_PPG_RDY_EN), TAG, "INTR1");
    ESP_RETURN_ON_ERROR(max30102_write_reg(MAX30102_REG_INTR_ENABLE_2, 0x00), TAG, "INTR2");

    // Limpieza de flags y FIFO
    (void)max30102_clear_interrupts();
    ESP_RETURN_ON_ERROR(max30102_clear_fifo(), TAG, "Clear FIFO");

    ESP_LOGI(TAG, "MAX30102 en SpO2: 100 Hz, 18-bit, promedio=4, LED~10mA");
    return ESP_OK;
}

// ===================== Lectura de muestras (RED/IR) =====================
// Lee 6 bytes (RED 18-bit + IR 18-bit) del FIFO y devuelve true si ok.
static bool max30102_read_sample(uint32_t *red, uint32_t *ir) {
    uint8_t buf[6];
    if (max30102_read_reg(MAX30102_REG_FIFO_DATA, buf, sizeof(buf)) != ESP_OK) return false;

    uint32_t raw_red = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    uint32_t raw_ir  = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    raw_red &= 0x3FFFF; // 18 bits
    raw_ir  &= 0x3FFFF;

    if (red) *red = raw_red;
    if (ir)  *ir  = raw_ir;
    return true;
}

// ===================== app_main =====================
void app_main(void) {
    // 1) I2C
    ESP_ERROR_CHECK(i2c_master_init());

    // 2) IDs
    uint8_t part_id=0, rev_id=0;
    if (max30102_read_ids(&part_id, &rev_id) == ESP_OK) {
        ESP_LOGI(TAG, "PART_ID=0x%02X (esperado 0x15), REV_ID=0x%02X", part_id, rev_id);
        if (part_id != 0x15) {
            ESP_LOGW(TAG, "PART_ID inesperado. ¿Es MAX30102? Verifica módulo/conexión/alimentación.");
        }
    } else {
        ESP_LOGE(TAG, "No se pudo leer PART/REV ID");
    }

    // 3) Configuración básica SpO2
    ESP_ERROR_CHECK(max30102_init_spo2_basic());

    // 4) Bucle de adquisición (polling sencillo a ~100 Hz)
    while (1) {
        // Lee y limpia flags
        uint8_t st1=0, st2=0;
        (void)max30102_read_reg(MAX30102_REG_INTR_STATUS_1, &st1, 1);
        (void)max30102_read_reg(MAX30102_REG_INTR_STATUS_2, &st2, 1);

        // Si hay datos listos o FIFO casi lleno, consume lo disponible
        if (st1 & (INTR_PPG_RDY_EN | INTR_A_FULL_EN)) {
            uint8_t n = max30102_fifo_level();
            if (n == 0) n = 1; // intenta al menos una muestra
            for (uint8_t i = 0; i < n; ++i) {
                uint32_t red=0, ir=0;
                if (max30102_read_sample(&red, &ir)) {
                    ESP_LOGI(TAG, "RED=%6u  IR=%6u", (unsigned)red, (unsigned)ir);
                } else {
                    ESP_LOGW(TAG, "Lectura FIFO fallida");
                    break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms ~ 100 Hz de consulta
    }
}
