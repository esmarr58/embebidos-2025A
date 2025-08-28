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

// ============================
// Definición de pines (ajusta)
// ============================
#define SEG_A    2
#define SEG_B    3
#define SEG_C    4
#define SEG_D    5
#define SEG_E    6
#define SEG_F    7
#define SEG_G    8

#define DIG_DECENAS   10   // Transistor dígito izquierdo
#define DIG_UNIDADES  11   // Transistor dígito derecho

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
    uint32_t ciclos = tiempo_ms / 2;
    if (ciclos == 0) ciclos = 1;

    for (uint32_t i = 0; i < ciclos; i++) {
        mostrar_un_digito(dec, true);
        vTaskDelay(pdMS_TO_TICKS(1));   // 1 ms

        mostrar_un_digito(uni, false);
        vTaskDelay(pdMS_TO_TICKS(1));   // 1 ms
    }

    deshabilitar_digitos();
}

// ============================
// Programa principal
// ============================
void app_main(void)
{
    inicializar_pines();

    uint8_t numero = 0;
    uint8_t d, u;

    while (true) {
        separar_decenas_unidades(numero, &d, &u);

        // Mostrar número actual durante ~1 segundo
        refrescar_display(d, u, 1000);

        // Incrementar número
        numero = (numero + 1) % 100;
    }
}
