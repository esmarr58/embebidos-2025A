#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==== Pines y parámetros I2C ====
#define I2C_MASTER_SCL_IO           9
#define I2C_MASTER_SDA_IO           10
#define MAX30102_ADDRESS            0x57
#define I2C_MASTER_FREQ_HZ          100000

// ==== Tags y handles ====
static const char *TAG = "MAX30102";
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t i2c_device_handle = NULL;

// ==== Registros útiles ====
#define MAX30102_REG_MODE_CONFIG    0x09
#define MAX30102_REG_INTR_STATUS_1  0x00
#define MAX30102_REG_INTR_STATUS_2  0x01

// ===== Helpers I2C =====
static esp_err_t i2c_master_init(void) {
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = {
            .enable_internal_pullup = 1,
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

    ESP_RETURN_ON_ERROR(i2c_master_probe(i2c_bus_handle, MAX30102_ADDRESS, 1000),
                        TAG, "No responde MAX30102 (0x%02X)", MAX30102_ADDRESS);

    ESP_LOGI(TAG, "Bus I2C inicializado correctamente");
    return ESP_OK;
}

static esp_err_t max30102_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_transmit_receive(i2c_device_handle, &reg, 1, data, len, 1000);
}

static esp_err_t max30102_write_reg(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_master_transmit(i2c_device_handle, buf, sizeof(buf), 1000);
}

// ==== app_main ====
void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());

    // Ejemplo: escribir en MODE_CONFIG para resetear el MAX30102
    ESP_LOGI(TAG, "Escribiendo 0x40 en MODE_CONFIG (reset)...");
    ESP_ERROR_CHECK(max30102_write_reg(MAX30102_REG_MODE_CONFIG, 0x40));

    // Esperar un poco para que haga reset
    vTaskDelay(pdMS_TO_TICKS(100));

    // Leer de nuevo el mismo registro para verificar
    uint8_t mode_config = 0;
    ESP_ERROR_CHECK(max30102_read_reg(MAX30102_REG_MODE_CONFIG, &mode_config, 1));
    ESP_LOGI(TAG, "Lectura MODE_CONFIG -> 0x%02X", mode_config);

    // Bucle demo
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI(TAG, "Loop vivo, registro MODE_CONFIG sigue en 0x%02X", mode_config);
    }
}
