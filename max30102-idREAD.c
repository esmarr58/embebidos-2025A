#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//6.12.0 
// ==== Pines y parámetros I2C ====
#define I2C_MASTER_SCL_IO           9           // GPIO para SCL
#define I2C_MASTER_SDA_IO           10          // GPIO para SDA
#define MAX30102_ADDRESS            0x57        // Dirección I2C del MAX30102
#define I2C_MASTER_FREQ_HZ          100000      // Frecuencia I2C (100 kHz)

// ==== Tags y handles ====
static const char *TAG = "MAX30102";
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t i2c_device_handle = NULL;

// ==== Registros útiles del MAX30102 ====
#define MAX30102_REG_INTR_STATUS_1  0x00
#define MAX30102_REG_INTR_STATUS_2  0x01
#define MAX30102_REG_REV_ID         0xFE
#define MAX30102_REG_PART_ID        0xFF

// ===== Helpers I2C (driver i2c_master nuevo) =====
static esp_err_t i2c_master_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = {
            .enable_internal_pullup = 1,  // Idealmente usa pull-ups externos de 4.7k~10k
        },
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_config, &i2c_bus_handle),
                        TAG, "Error i2c_new_master_bus");

    i2c_device_config_t dev_config = {
        .device_address = MAX30102_ADDRESS,
        .scl_speed_hz  = I2C_MASTER_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &i2c_device_handle),
                        TAG, "Error i2c_master_bus_add_device");

    // (Opcional) Probar que el dispositivo responde en la dirección
    ESP_RETURN_ON_ERROR(i2c_master_probe(i2c_bus_handle, MAX30102_ADDRESS, 1000 /*ms*/),
                        TAG, "No responde MAX30102 (0x%02X)", MAX30102_ADDRESS);

    ESP_LOGI(TAG, "Bus I2C inicializado correctamente");
    return ESP_OK;
}

static esp_err_t max30102_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    // Escribe el registro y luego lee 'len' bytes
    return i2c_master_transmit_receive(i2c_device_handle, &reg, 1, data, len, 1000 /*ms*/);
}

static esp_err_t max30102_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(i2c_device_handle, buf, sizeof(buf), 1000 /*ms*/);
}

// ==== Función para leer PART_ID y REV_ID ====
static esp_err_t max30102_read_ids(uint8_t *part_id, uint8_t *rev_id) {
    esp_err_t ret;
    uint8_t tmp;

    ret = max30102_read_reg(MAX30102_REG_PART_ID, &tmp, 1);
    if (ret != ESP_OK) return ret;
    if (part_id) *part_id = tmp;

    ret = max30102_read_reg(MAX30102_REG_REV_ID, &tmp, 1);
    if (ret != ESP_OK) return ret;
    if (rev_id) *rev_id = tmp;

    return ESP_OK;
}

// (Opcional) Limpia flags de interrupción leyendo los registros de estado
static void max30102_clear_interrupts(void) {
    uint8_t dump;
    max30102_read_reg(MAX30102_REG_INTR_STATUS_1, &dump, 1);
    max30102_read_reg(MAX30102_REG_INTR_STATUS_2, &dump, 1);
}

// ==== app_main ====
void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());

    // (Opcional) Limpia cualquier flag de arranque
    max30102_clear_interrupts();

    uint8_t part_id = 0, rev_id = 0;
    esp_err_t ret = max30102_read_ids(&part_id, &rev_id);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "MAX30102 IDs -> PART_ID: 0x%02X, REV_ID: 0x%02X", part_id, rev_id);
        // En MAX30102 típico: PART_ID = 0x15
        if (part_id == 0x15) {
            ESP_LOGI(TAG, "Dispositivo reconocido: MAX30102 (PART_ID esperado 0x15)");
        } else {
            ESP_LOGW(TAG, "PART_ID inesperado (esperado 0x15). Verifica conexión/alimentación/placa.");
        }
    } else {
        ESP_LOGE(TAG, "Fallo al leer IDs del MAX30102: %s", esp_err_to_name(ret));
    }

    // Bucle simple: vuelve a leer los IDs cada segundo (solo a modo de demo)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        ret = max30102_read_ids(&part_id, &rev_id);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "PART_ID: 0x%02X, REV_ID: 0x%02X", part_id, rev_id);
        } else {
            ESP_LOGE(TAG, "Lectura I2C falló: %s", esp_err_to_name(ret));
        }
    }
}
