#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// ==================== PINES ====================
// Sensores siguelíneas
#define S1 18   // Izquierda extrema
#define S2 17   // Izquierda centro
#define S3 16   // Derecha centro
#define S4 15   // Derecha extrema

// Motores con L298N
#define IN1 11  // Motor A
#define IN2 12
#define IN3 13  // Motor B
#define IN4 14

// Tiempo entre lecturas
#define TIEMPO_LECTURA_MS 300

// ==================== MOVIMIENTOS ====================

void motores_parar(void) {
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 0);
}

void avanzar(void) {
    gpio_set_level(IN1, 1);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 1);
    gpio_set_level(IN4, 0);
}

void retroceder(void) {
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 1);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 1);
}

void girar_izquierda(void) {
    gpio_set_level(IN1, 0);
    gpio_set_level(IN2, 1);
    gpio_set_level(IN3, 1);
    gpio_set_level(IN4, 0);
}

void girar_derecha(void) {
    gpio_set_level(IN1, 1);
    gpio_set_level(IN2, 0);
    gpio_set_level(IN3, 0);
    gpio_set_level(IN4, 1);
}

// ==================== CONFIGURACIÓN DE GPIO ====================

void configurar_pines(void) {
    // Pines de motores como salida
    gpio_config_t motores = {
        .pin_bit_mask = (1ULL << IN1) |
                        (1ULL << IN2) |
                        (1ULL << IN3) |
                        (1ULL << IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&motores);

    // Pines de sensores como entrada
    gpio_config_t sensores = {
        .pin_bit_mask = (1ULL << S1) |
                        (1ULL << S2) |
                        (1ULL << S3) |
                        (1ULL << S4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&sensores);

    motores_parar();
}

// ==================== LECTURA DE SENSORES ====================

void leer_sensores(int *s1, int *s2, int *s3, int *s4) {
    *s1 = gpio_get_level(S1);
    *s2 = gpio_get_level(S2);
    *s3 = gpio_get_level(S3);
    *s4 = gpio_get_level(S4);
}

// ==================== ALGORITMO SIGUELÍNEAS ====================

void algoritmo_siguelineas(void) {
    int s1, s2, s3, s4;

    leer_sensores(&s1, &s2, &s3, &s4);

    printf("\nSensores: S1=%d  S2=%d  S3=%d  S4=%d\n", s1, s2, s3, s4);

    /*
        Suposición usada:

        0 = sensor detecta línea negra
        1 = sensor detecta fondo blanco

        S1 = izquierda extrema
        S2 = izquierda centro
        S3 = derecha centro
        S4 = derecha extrema
    */

    if (s2 == 0 && s3 == 0) {
        printf("Linea centrada -> AVANZAR\n");
        avanzar();
    }
    else if (s1 == 0 || s2 == 0) {
        printf("Linea hacia la izquierda -> GIRAR IZQUIERDA\n");
        girar_izquierda();
    }
    else if (s3 == 0 || s4 == 0) {
        printf("Linea hacia la derecha -> GIRAR DERECHA\n");
        girar_derecha();
    }
    else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1) {
        printf("Linea perdida -> RETROCEDER\n");
        retroceder();
    }
    else {
        printf("Estado no reconocido -> PARAR\n");
        motores_parar();
    }
}

// ==================== PROGRAMA PRINCIPAL ====================

void app_main(void) {
    printf("Iniciando robot siguelineas simple...\n");

    configurar_pines();

    printf("Pines configurados correctamente.\n");
    printf("Iniciando lectura de sensores...\n");

    while (1) {
        algoritmo_siguelineas();
        vTaskDelay(pdMS_TO_TICKS(TIEMPO_LECTURA_MS));
    }
}
