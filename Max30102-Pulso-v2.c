#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"        // para esp_vfs_dev_uart_use_driver
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ===================== Configuración =====================
// I2C en ESP32-S3
#define I2C_SCL_IO              9
#define I2C_SDA_IO              10
#define I2C_FREQ_HZ             400000    // 400 kHz estable

// UART0 (USB nativo) -> Arduino Serial Plotter
#define UART0_BAUD              230400    // selecciona lo mismo en el Plotter

// MAX30102
#define MAX30102_ADDR           0x57

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
#define REG_LED1_PA             0x0C   // Red
#define REG_LED2_PA             0x0D   // IR
#define REG_REV_ID              0xFE
#define REG_PART_ID             0xFF

// ===================== Bits/campos =====================
// MODE_CONFIG
#define MODE_RESET_BIT          (1<<6)
#define MODE_HEART_RATE         0x02   // HR (solo LED1/Red)

// INTR_ENABLE_1
#define INTR_A_FULL_EN          (1<<7)
#define INTR_PPG_RDY_EN         (1<<6)

// FIFO_CONFIG
// [7:5] sample avg (0=1,1=2,2=4,3=8,4=16,5=32)
#define FIFO_AVG_4              (2<<5)     // un poco de suavizado
#define FIFO_ROLLOVER_EN        (1<<4)
#define FIFO_A_FULL(n)          ((n)&0x0F) // 0..15

// SPO2_CONFIG (aplica también en HR mode)
// [6:5] ADC range  (0=2048nA,1=4096nA,2=8192nA,3=16384nA)
// [4:2] sample rate(0=50,1=100,2=200,3=400,4=800,5=1000,6=1600,7=3200 Hz)
// [1:0] pulse width(0=69us(15b),1=118us(16b),2=215us(17b),3=411us(18b))
#define SPO2_ADC_RANGE_4096     (1<<5)
#define SPO2_SR_100HZ           (1<<2)     // 100 Hz (suficiente para HR)
#define SPO2_PW_411US           (3<<0)     // 18 bits

// ===================== Handles =====================
static const char *TAG = "HR_ONLY";
static i2c_master_bus_handle_t  i2c_bus  = NULL;
static i2c_master_dev_handle_t  i2c_dev  = NULL;

// ===================== UART0 helper =====================
static void uart0_setup(uint32_t baud)
{
    const uart_port_t uart_num = UART_NUM_0;

    // Instala el driver y redirige stdin/stdout/stderr a UART0 con el baud deseado
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

    // Disponible en IDF 5.5 (deprecada, pero funcional)
    esp_vfs_dev_uart_use_driver(uart_num);

    // Line-buffered; usaremos "\r\n" (CRLF) para el Plotter
    setvbuf(stdout, NULL, _IOLBF, 0);
}

// ===================== I2C helpers =====================
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

// ===================== MAX30102: HR mode =====================
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

    // FIFO: un poco de suavizado, rollover on, umbral medio
    uint8_t fifo_cfg = FIFO_AVG_4 | FIFO_ROLLOVER_EN | FIFO_A_FULL(8);
    ESP_RETURN_ON_ERROR(reg_write(REG_FIFO_CONFIG, fifo_cfg), TAG, "FIFO_CONFIG");

    // Modo Heart-Rate (solo LED1/Red)
    ESP_RETURN_ON_ERROR(reg_write(REG_MODE_CONFIG, MODE_HEART_RATE), TAG, "MODE_HR");

    // Config PW/ADC/SR (aplican en HR)
    uint8_t spo2_cfg = SPO2_ADC_RANGE_4096 | SPO2_SR_100HZ | SPO2_PW_411US;
    ESP_RETURN_ON_ERROR(reg_write(REG_SPO2_CONFIG, spo2_cfg), TAG, "SPO2_CONFIG");

    // LED1 (Red) activo, LED2 (IR) apagado
    ESP_RETURN_ON_ERROR(reg_write(REG_LED1_PA, 0x24), TAG, "LED1 Red"); // ~10 mA (ajusta a tu montaje)
    ESP_RETURN_ON_ERROR(reg_write(REG_LED2_PA, 0x00), TAG, "LED2 Off");

    // Interrupciones básicas
    ESP_RETURN_ON_ERROR(reg_write(REG_INTR_ENABLE_1, INTR_PPG_RDY_EN | INTR_A_FULL_EN), TAG, "INTR1");
    ESP_RETURN_ON_ERROR(reg_write(REG_INTR_ENABLE_2, 0x00), TAG, "INTR2");

    clear_interrupts();
    ESP_RETURN_ON_ERROR(fifo_clear(), TAG, "FIFO clr");
    return ESP_OK;
}

// Lee UNA muestra de 18 bits (HR mode = 3 bytes por muestra)
static inline bool read_sample_one(uint32_t *val)
{
    uint8_t b[3];
    if (reg_read(REG_FIFO_DATA, b, sizeof(b)) != ESP_OK) return false;

    uint32_t v = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
    v &= 0x3FFFF;

    if (val) *val = v;
    return true;
}

// ===================== app_main =====================
void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);

    // UART0 @ 230400 (USB nativo). El Plotter reinicia la S3 al abrir: normal.
    uart0_setup(UART0_BAUD);

    // I2C
    if (i2c_init_bus() != ESP_OK) {
        printf("ERR:I2C_BUS\r\n");
        while (1) { vTaskDelay(pdMS_TO_TICKS(500)); }
    }

    // Detectar MAX30102 en 0x57 (no aborta si no está)
    while (1) {
        if (i2c_master_probe(i2c_bus, MAX30102_ADDR, 200) == ESP_OK) {
            if (i2c_add_dev(MAX30102_ADDR) == ESP_OK) break;
        }
        printf("ERR:NO_DEV\r\n");         // no encontrado aún
        vTaskDelay(pdMS_TO_TICKS(1000));  // reintenta cada 1 s
    }

    // (Opcional) Leer IDs
    uint8_t part=0, rev=0;
    (void)reg_read(REG_PART_ID, &part, 1);
    (void)reg_read(REG_REV_ID,  &rev,  1);
    // Línea informativa (el Plotter puede ignorarla)
    printf("ID PART:0x%02X REV:0x%02X\r\n", part, rev);

    // Inicializar en Heart-Rate mode
    if (max30102_init_hr() != ESP_OK) {
        printf("ERR:INIT\r\n");
    }

    // Stream: una sola serie PPG (HR) en formato "PPG:<valor>\r\n"
    char line[24];
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
                int len = snprintf(line, sizeof(line), "PPG:%u", (unsigned)ppg);
                if (len > 0) printf("%s\r\n", line); // CRLF para Plotter
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // ~100 Hz cómodo
    }
}
