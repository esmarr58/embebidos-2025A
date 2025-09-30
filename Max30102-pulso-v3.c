#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define I2C_SCL_IO              9
#define I2C_SDA_IO              10
#define I2C_FREQ_HZ             400000

#define UART0_BAUD              230400

#define MAX30102_ADDR           0x57

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

static const char *TAG = "HR_RAW_FFT";
static i2c_master_bus_handle_t  i2c_bus  = NULL;
static i2c_master_dev_handle_t  i2c_dev  = NULL;

typedef enum {
    MODE_RAW,
    MODE_FFT
} display_mode_t;

static volatile display_mode_t current_mode = MODE_RAW;

// UART0
static void uart0_setup(uint32_t baud)
{
    const uart_port_t uart_num = UART_NUM_0;
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 1024, 0, 0, NULL, 0));
    uart_config_t cfg = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(uart_num, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart_num,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    esp_vfs_dev_uart_use_driver(uart_num);  // ⚠️ Warning OK, sigue funcionando
    setvbuf(stdout, NULL, _IOLBF, 0);
}


// I2C
static esp_err_t i2c_init_bus(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .flags = { .enable_internal_pullup = 1 },
    };
    return i2c_new_master_bus(&bus_cfg, &i2c_bus);
}
static esp_err_t i2c_add_dev(uint8_t addr)
{
    i2c_device_config_t dev_cfg = {
        .device_address = addr,
        .scl_speed_hz   = I2C_FREQ_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_dev);
}
static inline esp_err_t reg_read(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(i2c_dev, &reg, 1, data, len, 1000);
}
static inline esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(i2c_dev, buf, sizeof(buf), 1000);
}
static inline void clear_interrupts(void)
{
    uint8_t d;
    (void)reg_read(REG_INTR_STATUS_1, &d, 1);
    (void)reg_read(REG_INTR_STATUS_2, &d, 1);
}

// MAX30102
static esp_err_t max30102_reset(void)
{
    ESP_RETURN_ON_ERROR(reg_write(REG_MODE_CONFIG, MODE_RESET_BIT), TAG, "RESET");
    for (int i = 0; i < 50; ++i) {
        uint8_t m=0;
        ESP_RETURN_ON_ERROR(reg_read(REG_MODE_CONFIG, &m, 1), TAG, "MODE read");
        if ((m & MODE_RESET_BIT) == 0) return ESP_OK;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_ERR_TIMEOUT;
}
static esp_err_t fifo_clear(void)
{
    ESP_RETURN_ON_ERROR(reg_write(REG_FIFO_WR_PTR, 0x00), TAG, "WR_PTR");
    ESP_RETURN_ON_ERROR(reg_write(REG_OVF_COUNTER, 0x00), TAG, "OVF");
    ESP_RETURN_ON_ERROR(reg_write(REG_FIFO_RD_PTR, 0x00), TAG, "RD_PTR");
    return ESP_OK;
}
static uint8_t fifo_level(void)
{
    uint8_t wr=0, rd=0;
    if (reg_read(REG_FIFO_WR_PTR, &wr, 1) != ESP_OK) return 0;
    if (reg_read(REG_FIFO_RD_PTR, &rd, 1) != ESP_OK) return 0;
    return (uint8_t)((32 + wr - rd) & 0x1F);
}
static esp_err_t max30102_init_hr(void)
{
    ESP_RETURN_ON_ERROR(max30102_reset(), TAG, "reset");
    uint8_t fifo_cfg = FIFO_AVG_4 | FIFO_ROLLOVER_EN | FIFO_A_FULL(8);
    ESP_RETURN_ON_ERROR(reg_write(REG_FIFO_CONFIG, fifo_cfg), TAG, "FIFO_CONFIG");
    ESP_RETURN_ON_ERROR(reg_write(REG_MODE_CONFIG, MODE_HEART_RATE), TAG, "MODE_HR");
    uint8_t spo2_cfg = SPO2_ADC_RANGE_4096 | SPO2_SR_100HZ | SPO2_PW_411US;
    ESP_RETURN_ON_ERROR(reg_write(REG_SPO2_CONFIG, spo2_cfg), TAG, "SPO2_CONFIG");
    //ESP_RETURN_ON_ERROR(reg_write(REG_LED1_PA, 0x24), TAG, "LED1 Red");
        ESP_RETURN_ON_ERROR(reg_write(REG_LED1_PA, 0x3F), TAG, "LED1 Red");

    ESP_RETURN_ON_ERROR(reg_write(REG_LED2_PA, 0x00), TAG, "LED2 Off");
    ESP_RETURN_ON_ERROR(reg_write(REG_INTR_ENABLE_1, INTR_PPG_RDY_EN | INTR_A_FULL_EN), TAG, "INTR1");
    ESP_RETURN_ON_ERROR(reg_write(REG_INTR_ENABLE_2, 0x00), TAG, "INTR2");
    clear_interrupts();
    ESP_RETURN_ON_ERROR(fifo_clear(), TAG, "FIFO clr");
    return ESP_OK;
}
static inline bool read_sample_one(uint32_t *val)
{
    uint8_t b[3];
    if (reg_read(REG_FIFO_DATA, b, sizeof(b)) != ESP_OK) return false;
    uint32_t v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    v &= 0x3FFFF;
    if (val) *val = v;
    return true;
}

// FFT básica por cruces
#define FFT_SIZE 512
static uint32_t buffer[FFT_SIZE];
static uint16_t buffer_index = 0;

float estimate_fft_peak_freq(uint32_t *samples, size_t n_samples, float sample_rate)
{
    uint32_t avg = 0;
    for (int i = 0; i < n_samples; ++i) avg += samples[i];
    avg /= n_samples;

    int crossings = 0;
    for (int i = 1; i < n_samples; ++i) {
        if ((samples[i-1] > avg && samples[i] < avg) ||
            (samples[i-1] < avg && samples[i] > avg)) {
            ++crossings;
        }
    }
    float seconds = (float)n_samples / sample_rate;
    return (float)crossings / (2.0f * seconds);
}

// Task para leer comandos del usuario
static void uart_command_task(void *arg)
{
    char c;
    while (1) {
        if (uart_read_bytes(UART_NUM_0, (uint8_t *)&c, 1, pdMS_TO_TICKS(10)) > 0) {
            if (c == 'R') current_mode = MODE_RAW;
            else if (c == 'F') current_mode = MODE_FFT;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// app_main
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    uart0_setup(UART0_BAUD);

    if (i2c_init_bus() != ESP_OK) {
        printf("ERR:I2C_BUS\r\n");
        while (1) { vTaskDelay(pdMS_TO_TICKS(500)); }
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
    (void)reg_read(REG_REV_ID,  &rev,  1);
    printf("ID PART:0x%02X REV:0x%02X\r\n", part, rev);

    if (max30102_init_hr() != ESP_OK) {
        printf("ERR:INIT\r\n");
    }

    xTaskCreate(uart_command_task, "uart_cmd", 2048, NULL, 1, NULL);

    while (1) {
        uint8_t s1 = 0;
        if (reg_read(REG_INTR_STATUS_1, &s1, 1) != ESP_OK) {
            printf("ERR:RD_ST\r\n");
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (s1 & (INTR_PPG_RDY_EN | INTR_A_FULL_EN)) {
            uint8_t n = fifo_level();
            if (n == 0) n = 1;
            if (n > 12) n = 12;

            for (uint8_t k = 0; k < n; ++k) {
                uint32_t ppg = 0;
                if (!read_sample_one(&ppg)) { printf("ERR:FIFO\r\n"); break; }

                if (current_mode == MODE_RAW) {
                    // ✅ Cambio de formato para evitar error de compilación
                    printf("PPG:%lu\r\n", (unsigned long)ppg);
                } else {
                    buffer[buffer_index++] = ppg;
                    if (buffer_index >= FFT_SIZE) {
                        float freq = estimate_fft_peak_freq(buffer, FFT_SIZE, 100.0f); // 100 Hz sample rate
                        printf("HR_FREQ:%.1f\r\n", freq);
                        buffer_index = 0;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
