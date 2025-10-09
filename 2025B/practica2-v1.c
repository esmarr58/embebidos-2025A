/*
 * ESP32-S3 (ESP-IDF / PlatformIO)
 * 3 Displays 7 segmentos (cátodo común) con multiplexeo
 * 3 Botones en pull-down como selectores
 *
 * Función demo:
 * - Display izquierdo: número de experimento (1,2,3).
 * - Displays central/derecho: registro de 8 bits en HEX (00–FF).
 * - Botones:
 *     BTN_SEL_EXP: Cicla experimento (1→3).
 *     BTN_CFG_A  : Toggle flag A (bit1 del reg).
 *     BTN_CFG_B  : Toggle flag B (bit0 del reg).
 * - Heartbeat: bit7 del reg parpadea cada ~1s (dummy).
 *
 * Wiring resumen (cátodo común con NPN 2N3904 por dígito):
 * - Emisor a GND, colector al cátodo común del dígito.
 * - Base con R (4.7k–10k) al GPIO DIG_*.
 * - Cada segmento con R limitadora (220–470Ω) a su GPIO.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

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

#define DIG_ESTADO   9    // Dígito izquierdo (experimento)
#define DIG_DECHEX   10   // Dígito central (nibble alto del reg)
#define DIG_UNIHEX   11   // Dígito derecho (nibble bajo del reg)

// Botones (pull-down)
#define BTN_SEL_EXP  13   // Selector #1: cicla experimento 1..3
#define BTN_CFG_A    14   // Selector #2: toggle bandera A
#define BTN_CFG_B    15   // Selector #3: toggle bandera B

static const gpio_num_t PINES_SEG[7] = { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G };
static const gpio_num_t PINES_DIG[3] = { DIG_ESTADO, DIG_DECHEX, DIG_UNIHEX };

// ============================
// Tabla 7-seg (cátodo común)
// bit0=a,1=b,2=c,3=d,4=e,5=f,6=g
// ============================
static const uint8_t SEG_DEC[10] = {
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

static const uint8_t SEG_HEX[16] = {
    0x3F, // 0
    0x06, // 1
    0x5B, // 2
    0x4F, // 3
    0x66, // 4
    0x6D, // 5
    0x7D, // 6
    0x07, // 7
    0x7F, // 8
    0x6F, // 9
    0x77, // A
    0x7C, // b
    0x39, // C
    0x5E, // d
    0x79, // E
    0x71  // F
};

// =============== Utilidades HW ===============
static inline void segmentos_off(void) {
    for (int i = 0; i < 7; ++i) gpio_set_level(PINES_SEG[i], 0);
}
static inline void digitos_off(void) {
    for (int i = 0; i < 3; ++i) gpio_set_level(PINES_DIG[i], 0);
}
static inline void set_segmentos(uint8_t mask) {
    for (int b = 0; b < 7; ++b) {
        gpio_set_level(PINES_SEG[b], (mask >> b) & 0x1);
    }
}
static inline void habilitar_digito(gpio_num_t dig_pin) {
    // NPN a cátodo común: nivel alto habilita el dígito
    digitos_off();
    gpio_set_level(dig_pin, 1);
}

// =============== Inicialización ===============
static void init_gpio(void) {
    // Segmentos
    gpio_config_t io = {0};
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = 0;
    io.pull_down_en = 0;
    io.intr_type = GPIO_INTR_DISABLE;

    uint64_t mask_seg = 0;
    for (int i = 0; i < 7; ++i) mask_seg |= (1ULL << PINES_SEG[i]);
    io.pin_bit_mask = mask_seg;
    gpio_config(&io);

    // Dígitos
    uint64_t mask_dig = (1ULL << DIG_ESTADO) | (1ULL << DIG_DECHEX) | (1ULL << DIG_UNIHEX);
    io.pin_bit_mask = mask_dig;
    gpio_config(&io);

    segmentos_off();
    digitos_off();

    // Botones: entrada pull-down
    gpio_config_t ib = {0};
    ib.mode = GPIO_MODE_INPUT;
    ib.pull_up_en = 0;
    ib.pull_down_en = 1;           // pull-down interno
    ib.intr_type = GPIO_INTR_DISABLE;
    ib.pin_bit_mask = (1ULL << BTN_SEL_EXP) | (1ULL << BTN_CFG_A) | (1ULL << BTN_CFG_B);
    gpio_config(&ib);
}

// =============== Lógica de botones (debounce) ===============
typedef struct {
    gpio_num_t pin;
    int last_level;
    int64_t last_us;
} btn_t;

static const int64_t DEBOUNCE_US = 40000; // 40 ms

static bool btn_flanco_subida(btn_t* b) {
    int lvl = gpio_get_level(b->pin);
    int64_t now = esp_timer_get_time();

    bool flanco = false;
    if (lvl == 1 && b->last_level == 0) {
        if ((now - b->last_us) > DEBOUNCE_US) {
            flanco = true;
            b->last_us = now;
        }
    }
    b->last_level = lvl;
    return flanco;
}

// =============== Render de 3 dígitos ===============
static inline void render_estado(uint8_t experimento) {
    // Mostrar 1..3; si fuera 0, muestra guion medio (opcional)
    uint8_t mask = (experimento >= 1 && experimento <= 3) ? SEG_DEC[experimento] : 0x40; // 0x40 = g
    set_segmentos(mask);
    habilitar_digito(DIG_ESTADO);
}

static inline void render_hex(uint8_t reg8) {
    uint8_t hi = (reg8 >> 4) & 0x0F;
    uint8_t lo = reg8 & 0x0F;

    // Dígito central = nibble alto
    set_segmentos(SEG_HEX[hi]);
    habilitar_digito(DIG_DECHEX);

    // Pequeña pausa por multiplexeo
    vTaskDelay(pdMS_TO_TICKS(1));

    // Dígito derecho = nibble bajo
    set_segmentos(SEG_HEX[lo]);
    habilitar_digito(DIG_UNIHEX);
}

// =============== Main ===============
void app_main(void) {
    init_gpio();

    // Estado "dummy"
    uint8_t experimento = 1; // 1..3
    bool cfgA = false;       // bit1
    bool cfgB = false;       // bit0

    btn_t b_sel = { .pin = BTN_SEL_EXP, .last_level = 0, .last_us = 0 };
    btn_t b_a   = { .pin = BTN_CFG_A,   .last_level = 0, .last_us = 0 };
    btn_t b_b   = { .pin = BTN_CFG_B,   .last_level = 0, .last_us = 0 };

    int64_t last_hb = esp_timer_get_time();
    bool heartbeat = false;

    while (true) {
        // ---- Botones (pull-up al presionar: nivel=1) ----
        if (btn_flanco_subida(&b_sel)) {
            experimento = (experimento % 3) + 1; // 1→2→3→1
        }
        if (btn_flanco_subida(&b_a)) {
            cfgA = !cfgA;
        }
        if (btn_flanco_subida(&b_b)) {
            cfgB = !cfgB;
        }

        // ---- Heartbeat (bit7) cada ~1s ----
        int64_t now = esp_timer_get_time();
        if (now - last_hb > 1000000) { // 1,000,000 us = 1 s
            heartbeat = !heartbeat;
            last_hb = now;
        }

        // ---- Armar registro 8 bits (dummy) ----
        // [7]=HB, [6:4]=experimento(1..3), [3:2]=0, [1]=cfgA, [0]=cfgB
        uint8_t reg8 = (heartbeat ? 0x80 : 0x00) | ((experimento & 0x07) << 4) | ((cfgA ? 1 : 0) << 1) | (cfgB ? 1 : 0);

        // ---- Multiplexeo de 3 dígitos ----
        // 1) Estado (izq)
        render_estado(experimento);
        vTaskDelay(pdMS_TO_TICKS(1));

        // 2) HEX (centro y derecha)
        render_hex(reg8);
        vTaskDelay(pdMS_TO_TICKS(1));

        // Evitar ghosting
        digitos_off();
        segmentos_off();

        // Periodo de escaneo total ~2–3 ms → ~333–500 Hz por dígito (sin parpadeo)
    }
}
