/*
 * ESP32-S3 (ESP-IDF / PlatformIO)
 * 3 Displays 7 segmentos (cátodo común) con multiplexeo
 * 3 Botones en pull-down (ISR + esp_timer one-shot para debounce)
 *
 * Demo (dummy):
 * - Display izquierdo: experimento 1..3
 * - Displays central/derecho: reg8 en HEX (00–FF)
 * - reg8 = [7]=heartbeat, [6:4]=experimento(1..3), [1]=cfgA, [0]=cfgB
 *
 * Notas de HW (cátodo común con NPN para cada dígito):
 * - Emisor a GND, colector al cátodo común del dígito, base con R 4.7k–10k a GPIO.
 * - Segmentos con resistor 220–470Ω a cada GPIO.
 * - Botones pull-down: al presionar llevan el pin a VCC (configurar también pull_down interno).
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_log.h"

#define TAG "MUX_7SEG_ISR"

// ============================
// Pines (ajusta)
// ============================
#define SEG_A    2
#define SEG_B    3
#define SEG_C    4
#define SEG_D    5
#define SEG_E    6
#define SEG_F    7
#define SEG_G    8

#define DIG_ESTADO   9
#define DIG_DECHEX   10
#define DIG_UNIHEX   11

// Botones en pull-down
#define BTN_SEL_EXP  13  // Selector #1: experimento (1→2→3→1)
#define BTN_CFG_A    14  // Selector #2: toggle A (bit1)
#define BTN_CFG_B    15  // Selector #3: toggle B (bit0)

static const gpio_num_t PINES_SEG[7] = { SEG_A, SEG_B, SEG_C, SEG_D, SEG_E, SEG_F, SEG_G };
static const gpio_num_t PINES_DIG[3] = { DIG_ESTADO, DIG_DECHEX, DIG_UNIHEX };

// ============================
// Tablas 7-seg (cátodo común)
// ============================
static const uint8_t SEG_DEC[10] = {
    0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F
};
static const uint8_t SEG_HEX[16] = {
    0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F, // 0-9
    0x77,0x7C,0x39,0x5E,0x79,0x71                       // A-F
};

// ============================
// Estado global (acceso ligero)
// ============================
static volatile uint8_t s_experimento = 1; // 1..3
static volatile bool    s_cfgA = false;    // bit1
static volatile bool    s_cfgB = false;    // bit0
static volatile bool    s_heartbeat = false;

// ============================
// Utilidades HW
// ============================
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
    // NPN al cátodo común: alto = habilita dígito
    digitos_off();
    gpio_set_level(dig_pin, 1);
}

// ============================
/* Inicialización GPIO: segmentos, dígitos y botones */
static void init_gpio(void) {
    // Segmentos y dígitos
    gpio_config_t io = {0};
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = 0;
    io.pull_down_en = 0;
    io.intr_type = GPIO_INTR_DISABLE;

    uint64_t mask_seg = 0;
    for (int i = 0; i < 7; ++i) mask_seg |= (1ULL << PINES_SEG[i]);
    io.pin_bit_mask = mask_seg;
    gpio_config(&io);

    uint64_t mask_dig = (1ULL << DIG_ESTADO) | (1ULL << DIG_DECHEX) | (1ULL << DIG_UNIHEX);
    io.pin_bit_mask = mask_dig;
    gpio_config(&io);

    segmentos_off();
    digitos_off();

    // Botones input + pull-down + interrupción en flanco de subida
    gpio_config_t ib = {0};
    ib.mode = GPIO_MODE_INPUT;
    ib.pull_up_en = 0;
    ib.pull_down_en = 1; // activa pull-down interno
    ib.intr_type = GPIO_INTR_POSEDGE; // flanco de subida (pulso = 1)
    ib.pin_bit_mask = (1ULL << BTN_SEL_EXP) | (1ULL << BTN_CFG_A) | (1ULL << BTN_CFG_B);
    gpio_config(&ib);
}

// ============================
// Debounce con esp_timer
// ============================
typedef struct {
    gpio_num_t pin;
    esp_timer_handle_t tmr;
    int64_t lock_until_us; // para evitar reinicios demasiado rápidos
} btn_deb_t;

static btn_deb_t s_btn_sel, s_btn_a, s_btn_b;
static const int64_t DEBOUNCE_US = 40000; // 40 ms

// Forward decl. callbacks
static void IRAM_ATTR isr_btn_handler(void* arg);
static void tmr_btn_sel_cb(void* arg);
static void tmr_btn_a_cb(void* arg);
static void tmr_btn_b_cb(void* arg);

static void init_buttons_isr(void) {
    // Crear timers one-shot
    esp_timer_create_args_t a1 = { .callback = tmr_btn_sel_cb, .arg = NULL, .dispatch_method = ESP_TIMER_TASK, .name = "db_sel" };
    esp_timer_create_args_t a2 = { .callback = tmr_btn_a_cb,   .arg = NULL, .dispatch_method = ESP_TIMER_TASK, .name = "db_a"   };
    esp_timer_create_args_t a3 = { .callback = tmr_btn_b_cb,   .arg = NULL, .dispatch_method = ESP_TIMER_TASK, .name = "db_b"   };

    esp_timer_create(&a1, &s_btn_sel.tmr);
    esp_timer_create(&a2, &s_btn_a.tmr);
    esp_timer_create(&a3, &s_btn_b.tmr);

    s_btn_sel.pin = BTN_SEL_EXP; s_btn_sel.lock_until_us = 0;
    s_btn_a.pin   = BTN_CFG_A;   s_btn_a.lock_until_us = 0;
    s_btn_b.pin   = BTN_CFG_B;   s_btn_b.lock_until_us = 0;

    // Instalar servicio ISR y registrar handlers
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_SEL_EXP, isr_btn_handler, (void*)&s_btn_sel);
    gpio_isr_handler_add(BTN_CFG_A,   isr_btn_handler, (void*)&s_btn_a);
    gpio_isr_handler_add(BTN_CFG_B,   isr_btn_handler, (void*)&s_btn_b);
}

static void IRAM_ATTR isr_btn_handler(void* arg) {
    btn_deb_t* b = (btn_deb_t*)arg;
    int64_t now = esp_timer_get_time();

    // Pequeño "candado" para limitar interrupciones muy seguidas (ruido extremo)
    if (now < b->lock_until_us) return;

    // Deshabilita temporalmente la intr del pin y arranca timer de debounce
    gpio_intr_disable(b->pin);
    b->lock_until_us = now + DEBOUNCE_US;
    // One-shot a 40 ms
    esp_timer_start_once(b->tmr, DEBOUNCE_US);
}

// Tras 40 ms, confirmamos nivel alto y aplicamos acción, luego re-habilitamos intr
static void tmr_confirm_and_enable(btn_deb_t* b, void (*on_press)(void)) {
    // Confirmación: si sigue alto, contamos como "press"
    if (gpio_get_level(b->pin) == 1) {
        on_press();
    }
    // Rehabilitar interrupción
    gpio_intr_enable(b->pin);
}

// Acciones de usuario (actualizan estado global)
static void on_press_sel(void) {
    uint8_t e = s_experimento;
    e = (e % 3) + 1;
    s_experimento = e;
}
static void on_press_a(void) {
    s_cfgA = !s_cfgA;
}
static void on_press_b(void) {
    s_cfgB = !s_cfgB;
}

// Timers callbacks (task context)
static void tmr_btn_sel_cb(void* arg) { tmr_confirm_and_enable(&s_btn_sel, on_press_sel); }
static void tmr_btn_a_cb(void* arg)   { tmr_confirm_and_enable(&s_btn_a,   on_press_a);   }
static void tmr_btn_b_cb(void* arg)   { tmr_confirm_and_enable(&s_btn_b,   on_press_b);   }

// ============================
// Render de 3 dígitos
// ============================
static inline void render_estado(uint8_t experimento) {
    uint8_t mask = (experimento >= 1 && experimento <= 3) ? SEG_DEC[experimento] : 0x40; // guion medio (segmento g)
    set_segmentos(mask);
    habilitar_digito(DIG_ESTADO);
}

static inline void render_hex(uint8_t reg8) {
    uint8_t hi = (reg8 >> 4) & 0x0F;
    uint8_t lo = reg8 & 0x0F;

    set_segmentos(SEG_HEX[hi]);
    habilitar_digito(DIG_DECHEX);
    vTaskDelay(pdMS_TO_TICKS(1));

    set_segmentos(SEG_HEX[lo]);
    habilitar_digito(DIG_UNIHEX);
}

// ============================
// app_main
// ============================
void app_main(void) {
    init_gpio();
    init_buttons_isr();

    int64_t last_hb = esp_timer_get_time();

    while (true) {
        // Heartbeat cada ~1 s
        int64_t now = esp_timer_get_time();
        if (now - last_hb > 1000000) {
            s_heartbeat = !s_heartbeat;
            last_hb = now;
        }

        // Armar reg8 dummy (leer variables "volátiles" en local para empaquetar)
        uint8_t e   = s_experimento & 0x07;
        bool a      = s_cfgA;
        bool b      = s_cfgB;
        bool hb     = s_heartbeat;

        uint8_t reg8 = (hb ? 0x80 : 0x00) | (e << 4) | ((a ? 1 : 0) << 1) | (b ? 1 : 0);

        // Multiplexeo (~2–3 ms/scan)
        render_estado(e);
        vTaskDelay(pdMS_TO_TICKS(1));

        render_hex(reg8);
        vTaskDelay(pdMS_TO_TICKS(1));

        // Limpieza para evitar ghosting
        digitos_off();
        segmentos_off();
    }
}
