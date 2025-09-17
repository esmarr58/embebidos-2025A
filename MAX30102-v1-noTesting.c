#include <stdio.h>
#include <stdint.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Definiciones de pines y parámetros I2C
#define I2C_MASTER_SCL_IO           9          // GPIO para SCL
#define I2C_MASTER_SDA_IO           10         // GPIO para SDA
#define MAX30102_ADDRESS            0x57       // Dirección I2C del MAX30102
#define I2C_MASTER_FREQ_HZ          400000     // Frecuencia recomendada para MAX30102

// Registros del MAX30102
#define MAX30102_REG_INTR_STATUS_1  0x00
#define MAX30102_REG_INTR_STATUS_2  0x01
#define MAX30102_REG_INTR_ENABLE_1  0x02
#define MAX30102_REG_INTR_ENABLE_2  0x03
#define MAX30102_REG_FIFO_WR_PTR    0x04
#define MAX30102_REG_OVF_COUNTER    0x05
#define MAX30102_REG_FIFO_RD_PTR    0x06
#define MAX30102_REG_FIFO_DATA      0x07
#define MAX30102_REG_FIFO_CONFIG    0x08
#define MAX30102_REG_MODE_CONFIG    0x09
#define MAX30102_REG_SPO2_CONFIG    0x0A
#define MAX30102_REG_LED1_PA        0x0C
#define MAX30102_REG_LED2_PA        0x0D
#define MAX30102_REG_PILOT_PA       0x10
#define MAX30102_REG_MULTI_LED_CTRL1 0x11
#define MAX30102_REG_MULTI_LED_CTRL2 0x12
#define MAX30102_REG_TEMP_INTR      0x1F
#define MAX30102_REG_TEMP_FRAC      0x20
#define MAX30102_REG_TEMP_CONFIG    0x21
#define MAX30102_REG_PROX_INT_THRESH 0x30
#define MAX30102_REG_REV_ID         0xFE
#define MAX30102_REG_PART_ID        0xFF

// Tags para logs
static const char *TAG = "MAX30102";

// Variables para el manejo de I2C
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
static i2c_master_dev_handle_t i2c_device_handle = NULL;

// ** Inicialización del controlador I2C usando i2c-master **
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

    esp_err_t ret = i2c_new_master_bus(&bus_config, &i2c_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en i2c_new configuración: %s", esp_err_to_name(ret));
        return ret;
    }

    i2c_device_config_t dev_config = {
        .device_address = MAX30102_ADDRESS,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(i2c_bus_handle, &dev_config, &i2c_device_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error en i2c_master_bus_add_device: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Bus I2C inicializado correctamente");
    return ESP_OK;
}

// ** Escribir en registro del MAX30102 **
static esp_err_t max30102_write_register(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(i2c_device_handle, data, sizeof(data), 1000);
}

// ** Leer registro del MAX30102 **
static esp_err_t max30102_read_register(uint8_t reg, uint8_t *value) {
    return i2c_master_transmit_receive(i2c_device_handle, &reg, 1, value, 1, 1000);
}

// ** Leer múltiples registros del MAX30102 **
static esp_err_t max30102_read_registers(uint8_t reg, uint8_t *buffer, uint8_t length) {
    return i2c_master_transmit_receive(i2c_device_handle, &reg, 1, buffer, length, 1000);
}

// ** Inicializar el sensor MAX30102 **
static esp_err_t max30102_init(void) {
    esp_err_t ret;
    
    // Verificar PART_ID
    uint8_t part_id;
    ret = max30102_read_register(MAX30102_REG_PART_ID, &part_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al leer PART_ID");
        return ret;
    }
    
    ESP_LOGI(TAG, "PART_ID: 0x%02X", part_id);
    
    if (part_id != 0x15) {
        ESP_LOGE(TAG, "PART_ID incorrecto. Esperado: 0x15, Leído: 0x%02X", part_id);
        return ESP_FAIL;
    }

    // Reset del sensor
    ret = max30102_write_register(MAX30102_REG_MODE_CONFIG, 0x40);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al resetear el sensor");
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    // Configurar modo (SpO2 mode)
    ret = max30102_write_register(MAX30102_REG_MODE_CONFIG, 0x03);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar modo");
        return ret;
    }

    // Configurar SPO2 (sample rate = 100Hz, LED pulse width = 411μs)
    ret = max30102_write_register(MAX30102_REG_SPO2_CONFIG, 0x27);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar SPO2");
        return ret;
    }

    // Configurar corriente LED (LED1 = 7.6mA, LED2 = 7.6mA)
    ret = max30102_write_register(MAX30102_REG_LED1_PA, 0x24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar LED1");
        return ret;
    }
    
    ret = max30102_write_register(MAX30102_REG_LED2_PA, 0x24);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar LED2");
        return ret;
    }

    // Configurar FIFO (promedio de 4 muestras, rollover habilitado)
    ret = max30102_write_register(MAX30102_REG_FIFO_CONFIG, 0x4F);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al configurar FIFO");
        return ret;
    }

    // Habilitar interrupciones
    ret = max30102_write_register(MAX30102_REG_INTR_ENABLE_1, 0x40); // FIFO almost full
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error al habilitar interrupciones");
        return ret;
    }

    ESP_LOGI(TAG, "Sensor MAX30102 inicializado correctamente");
    return ESP_OK;
}

// ** Leer datos del FIFO **
static esp_err_t max30102_read_fifo(uint32_t *red, uint32_t *ir) {
    uint8_t fifo_data[6];
    esp_err_t ret = max30102_read_registers(MAX30102_REG_FIFO_DATA, fifo_data, 6);
    
    if (ret == ESP_OK) {
        // Los datos vienen en formato: [IR MSB, IR MID, IR LSB, RED MSB, RED MID, RED LSB]
        *ir = ((uint32_t)fifo_data[0] << 16) | ((uint32_t)fifo_data[1] << 8) | fifo_data[2];
        *red = ((uint32_t)fifo_data[3] << 16) | ((uint32_t)fifo_data[4] << 8) | fifo_data[5];
        
        // Máscara para 18 bits (los 6 bits superiores deben ser 0)
        *ir &= 0x3FFFF;
        *red &= 0x3FFFF;
    }
    
    return ret;
}

// ** Obtener punteros del FIFO **
static esp_err_t max30102_get_fifo_pointers(uint8_t *write_ptr, uint8_t *read_ptr, uint8_t *ovf_count) {
    esp_err_t ret;
    
    ret = max30102_read_register(MAX30102_REG_FIFO_WR_PTR, write_ptr);
    if (ret != ESP_OK) return ret;
    
    ret = max30102_read_register(MAX30102_REG_FIFO_RD_PTR, read_ptr);
    if (ret != ESP_OK) return ret;
    
    ret = max30102_read_register(MAX30102_REG_OVF_COUNTER, ovf_count);
    
    return ret;
}

// ** Limpiar FIFO **
static esp_err_t max30102_clear_fifo(void) {
    esp_err_t ret;
    
    // Resetear punteros del FIFO
    ret = max30102_write_register(MAX30102_REG_FIFO_WR_PTR, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = max30102_write_register(MAX30102_REG_FIFO_RD_PTR, 0x00);
    if (ret != ESP_OK) return ret;
    
    ret = max30102_write_register(MAX30102_REG_OVF_COUNTER, 0x00);
    
    return ret;
}

// ** Función principal **
void app_main(void) {
    // Inicializar el bus I2C
    ESP_ERROR_CHECK(i2c_master_init());

    // Inicializar el sensor MAX30102
    ESP_ERROR_CHECK(max30102_init());

    // Limpiar FIFO al inicio
    ESP_ERROR_CHECK(max30102_clear_fifo());

    uint32_t red_value, ir_value;
    uint8_t write_ptr, read_ptr, ovf_count;
    uint32_t sample_count = 0;

    while (1) {
        // Obtener estado del FIFO
        esp_err_t ret = max30102_get_fifo_pointers(&write_ptr, &read_ptr, &ovf_count);
        if (ret == ESP_OK) {
            int available_samples = (write_ptr - read_ptr) & 0x1F;
            
            if (available_samples > 0) {
                // Leer todas las muestras disponibles
                for (int i = 0; i < available_samples; i++) {
                    ret = max30102_read_fifo(&red_value, &ir_value);
                    if (ret == ESP_OK) {
                        sample_count++;
                        ESP_LOGI(TAG, "Muestra %lu: RED=%lu, IR=%lu", 
                                sample_count, red_value, ir_value);
                    } else {
                        ESP_LOGE(TAG, "Error al leer FIFO: %s", esp_err_to_name(ret));
                    }
                }
            } else {
                ESP_LOGD(TAG, "No hay muestras disponibles en FIFO");
            }
        } else {
            ESP_LOGE(TAG, "Error al leer punteros FIFO: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Espera 100ms antes de la siguiente lectura
    }
}
