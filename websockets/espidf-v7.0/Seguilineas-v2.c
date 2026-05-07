#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_mac.h"
#include "esp_random.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"

#include "mdns.h"
#include "esp_http_server.h"
#include "cJSON.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "led_strip.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Sensor siguelíneas 4 canales
#define S1 18
#define S2 17
#define S3 16
#define S4 15

#define TAG "APP"
#define SIMULAR_DHT true

#ifndef PIN_RGB
#define PIN_RGB 48
#endif

#define PIN_LED 44

static const char *WIFI_SSID = "GWN571D045";
static const char *WIFI_PASS = "ESP32CUCEI$$";

#define WS_PORT 80
#define WS_URI  "/"

#define ADC_GPIO          4
#define ADC_UNIT          ADC_UNIT_1
#define ADC_CH            ADC_CHANNEL_3

#define DHT_MIN_MS_REAL 2000
#define DHT_MIN_MS_SIM   500

#define IN1 11
#define IN2 12
#define IN3 13
#define IN4 14

static volatile bool latidoRecibido = false;
static uint32_t CADA_MS = 2000;
static float tempC_cache = NAN;
static float hum_cache   = NAN;
static bool  dht_ok      = false;
static uint32_t ultimoDhtMs = 0;

static char nombreHost[32] = {0};

#define MAX_CLIENTES 8
static int ws_client_fds[MAX_CLIENTES];
static SemaphoreHandle_t ws_mutex;

static httpd_handle_t httpd = NULL;
static void leer_siguelineas(int *a1, int *a2, int *a3, int *a4);

// ==================== TIPOS ====================

typedef enum {
    Dir_Parado = 0,
    Dir_Avanzar,
    Dir_Retroceder,
    Dir_Izquierda,
    Dir_Derecha,
    Dir_Parar
} DirMov;

typedef enum {
    EST_LINEA_PARADO = 0,
    EST_LINEA_AVANZAR,
    EST_LINEA_RETROCEDER,
    EST_LINEA_IZQUIERDA,
    EST_LINEA_DERECHA
} EstadoLinea;

typedef enum {
    MODO_MANUAL = 0,
    MODO_SIGUELINEAS
} ModoRobot;

typedef struct {
    DirMov accion;
    uint32_t dur_ms;
    uint32_t t0_ms;
} CmdMotor;

// ==================== GLOBALES SIGUELINEAS ====================

static volatile bool siguelineas_auto = false;
static volatile ModoRobot modo_robot = MODO_MANUAL;

static uint32_t tiempo_estado_ms     = 1000;
static uint32_t tiempo_avanzar_ms    = 100;
static uint32_t tiempo_retroceder_ms = 100;
static uint32_t tiempo_izquierda_ms  = 50;
static uint32_t tiempo_derecha_ms    = 50;

static QueueHandle_t motorCmdQ = NULL;

static EstadoLinea decidir_estado_linea(int a1, int a2, int a3, int a4);
static const char* estado_linea_str(EstadoLinea estado);

// ==================== HELPERS ====================

static uint32_t millis32(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static const char* modo_robot_str(void) {
    return (modo_robot == MODO_SIGUELINEAS) ? "siguelineas" : "manual";
}

// ==================== WEBSOCKET HELPERS ====================

static void send_json_to_fd(int fd, cJSON *root) {
    if (!root) return;

    char *txt = cJSON_PrintUnformatted(root);
    if (!txt) return;

    httpd_ws_frame_t ws_pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t*)txt,
        .len = strlen(txt)
    };

    esp_err_t err = httpd_ws_send_frame_async(httpd, fd, &ws_pkt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ws send fd=%d err=%s", fd, esp_err_to_name(err));
    }

    free(txt);
}

static void broadcast_json(cJSON *root) {
    if (!root) return;

    if (xSemaphoreTake(ws_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < MAX_CLIENTES; ++i) {
            if (ws_client_fds[i] > 0) {
                send_json_to_fd(ws_client_fds[i], root);
            }
        }
        xSemaphoreGive(ws_mutex);
    }
}

static void ws_add_fd(int fd) {
    if (xSemaphoreTake(ws_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < MAX_CLIENTES; ++i) {
            if (ws_client_fds[i] == 0) {
                ws_client_fds[i] = fd;
                break;
            }
        }
        xSemaphoreGive(ws_mutex);
    }
}

static void ws_remove_fd(int fd) {
    if (xSemaphoreTake(ws_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < MAX_CLIENTES; ++i) {
            if (ws_client_fds[i] == fd) {
                ws_client_fds[i] = 0;
                break;
            }
        }
        xSemaphoreGive(ws_mutex);
    }
}

// ==================== LED STRIP ====================

static led_strip_handle_t led_strip;

static void rgb_set(uint8_t r, uint8_t g, uint8_t b) {
    if (!led_strip) return;

    led_strip_set_pixel(led_strip, 0, r, g, b);
    led_strip_refresh(led_strip);
}

// ==================== ADC ====================

static adc_oneshot_unit_handle_t adc_handle;

static int adc_read_raw(void) {
    int val = 0;

    if (adc_handle) {
        adc_oneshot_read(adc_handle, ADC_CH, &val);
    }

    return val;
}

// ==================== MOTORES ====================

static inline void motorA_stop(void)     { gpio_set_level(IN1, 0); gpio_set_level(IN2, 0); }
static inline void motorA_adelante(void) { gpio_set_level(IN1, 1); gpio_set_level(IN2, 0); }
static inline void motorA_atras(void)    { gpio_set_level(IN1, 0); gpio_set_level(IN2, 1); }

static inline void motorB_stop(void)     { gpio_set_level(IN3, 0); gpio_set_level(IN4, 0); }
static inline void motorB_adelante(void) { gpio_set_level(IN3, 1); gpio_set_level(IN4, 0); }
static inline void motorB_atras(void)    { gpio_set_level(IN3, 0); gpio_set_level(IN4, 1); }

static inline void motores_parar_total(void) { motorA_stop(); motorB_stop(); }
static inline void mover_avanzar(void)       { motorA_adelante(); motorB_adelante(); }
static inline void mover_retroceder(void)    { motorA_atras();    motorB_atras();    }
static inline void mover_izquierda(void)     { motorA_atras();    motorB_adelante(); }
static inline void mover_derecha(void)       { motorA_adelante(); motorB_atras();    }

// ==================== MOTOR TASK ====================

void MotorTask(void* arg) {
    bool enMovimiento = false;
    uint32_t t_fin = 0;

    for (;;) {
        CmdMotor nuevo;

        if (xQueueReceive(motorCmdQ, &nuevo, pdMS_TO_TICKS(250)) == pdTRUE) {
            enMovimiento = false;
            motores_parar_total();

            if (nuevo.accion != Dir_Parar && nuevo.accion != Dir_Parado && nuevo.dur_ms > 0) {
                switch (nuevo.accion) {
                    case Dir_Avanzar:
                        mover_avanzar();
                        break;

                    case Dir_Retroceder:
                        mover_retroceder();
                        break;

                    case Dir_Izquierda:
                        mover_izquierda();
                        break;

                    case Dir_Derecha:
                        mover_derecha();
                        break;

                    default:
                        motores_parar_total();
                        break;
                }

                nuevo.t0_ms = millis32();
                t_fin = nuevo.t0_ms + nuevo.dur_ms;
                enMovimiento = true;

                cJSON *ack = cJSON_CreateObject();
                if (ack) {
                    cJSON_AddStringToObject(ack, "tipo", "mover_inicio");
                    cJSON_AddStringToObject(ack, "modo", modo_robot_str());
                    broadcast_json(ack);
                    cJSON_Delete(ack);
                }
            }
        }

        if (enMovimiento) {
            uint32_t ahora = millis32();

            if ((int32_t)(ahora - t_fin) >= 0) {
                enMovimiento = false;
                motores_parar_total();

                cJSON *fin = cJSON_CreateObject();
                if (fin) {
                    cJSON_AddStringToObject(fin, "tipo", "mover_fin");
                    cJSON_AddStringToObject(fin, "modo", modo_robot_str());
                    broadcast_json(fin);
                    cJSON_Delete(fin);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== DHT ====================

static void actualizarLecturaDHT(bool forzar) {
    uint32_t ahora = millis32();
    uint32_t intervalo = SIMULAR_DHT ? DHT_MIN_MS_SIM : DHT_MIN_MS_REAL;

    if (!forzar && (ahora - ultimoDhtMs < intervalo)) return;

    if (SIMULAR_DHT) {
        int r1 = (int)(esp_random() % 601) - 300;
        int r2 = (int)(esp_random() % 2001) - 1000;

        tempC_cache = 25.0f + (r1 / 100.0f);
        hum_cache   = 45.0f + (r2 / 100.0f);
        dht_ok = true;
        ultimoDhtMs = ahora;
    } else {
        dht_ok = false;
    }
}

// ==================== MAC / HOSTNAME ====================

static void mac_sin_dospuntos(char *out, size_t outlen) {
    uint8_t mac[6];

    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    snprintf(out, outlen, "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void sufijo_mac6(char *out, size_t outlen) {
    char buf[13] = {0};

    mac_sin_dospuntos(buf, sizeof(buf));

    size_t L = strlen(buf);
    const char *s = (L >= 6) ? buf + (L - 6) : buf;

    snprintf(out, outlen, "%s", s);
}

// ==================== WIFI ====================

static void wifi_init(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = { 0 };

    strlcpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}

// ==================== MDNS ====================

static void mdns_start(void) {
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(nombreHost));
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32-S3 Server"));

    mdns_txt_item_t serviceTxtData[] = {
        {"proto", "json"}
    };

    ESP_ERROR_CHECK(mdns_service_add("ws", "_ws", "_tcp", WS_PORT, serviceTxtData, 1));
}

// ==================== WEBSOCKET HANDLER ====================

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        int fd = httpd_req_to_sockfd(req);
        ws_add_fd(fd);

        ESP_LOGI(TAG, "[WS] Cliente fd=%d conectado", fd);

        cJSON *msg = cJSON_CreateObject();

        cJSON_AddStringToObject(msg, "tipo", "saludo");
        cJSON_AddStringToObject(msg, "mensaje", "Servidor ESP32-S3 listo");
        cJSON_AddNumberToObject(msg, "puerto", WS_PORT);
        cJSON_AddStringToObject(msg, "nombre_host", nombreHost);
        cJSON_AddStringToObject(msg, "modo", modo_robot_str());

        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);

        char macfmt[18];
        snprintf(macfmt, sizeof(macfmt), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        cJSON_AddStringToObject(msg, "mac", macfmt);

        send_json_to_fd(fd, msg);
        cJSON_Delete(msg);

        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) return ret;

    if (ret == ESP_OK && ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        int fd = httpd_req_to_sockfd(req);
        ws_remove_fd(fd);
        ESP_LOGI(TAG, "[WS] Cliente fd=%d desconectado", fd);
        return ESP_OK;
    }

    if (ws_pkt.len) {
        ws_pkt.payload = (uint8_t *)malloc(ws_pkt.len + 1);
        if (!ws_pkt.payload) return ESP_ERR_NO_MEM;

        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);

        if (ret == ESP_OK) {
            ws_pkt.payload[ws_pkt.len] = 0;

            cJSON *root = cJSON_Parse((char*)ws_pkt.payload);

            if (root) {
                const char *tipo = cJSON_GetStringValue(cJSON_GetObjectItem(root, "tipo"));
                int fd = httpd_req_to_sockfd(req);

                if (tipo && strcmp(tipo, "latido") == 0) {
                    latidoRecibido = true;

                    cJSON *ack = cJSON_CreateObject();
                    cJSON_AddStringToObject(ack, "tipo", "latido_ok");
                    cJSON_AddNumberToObject(ack, "tiempo_ms", (double)millis32());
                    cJSON_AddStringToObject(ack, "modo", modo_robot_str());
                    send_json_to_fd(fd, ack);
                    cJSON_Delete(ack);

                } else if (tipo && strcmp(tipo, "leer_dht") == 0) {
                    actualizarLecturaDHT(true);

                    cJSON *resp = cJSON_CreateObject();
                    cJSON_AddStringToObject(resp, "tipo", "dht_lectura");
                    cJSON_AddBoolToObject(resp, "dht_ok", dht_ok);
                    cJSON_AddNumberToObject(resp, "temperatura", tempC_cache);
                    cJSON_AddNumberToObject(resp, "humedad", hum_cache);
                    send_json_to_fd(fd, resp);
                    cJSON_Delete(resp);

                } else if (tipo && strcmp(tipo, "led") == 0) {
                    int estado = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "estado"));

                    gpio_set_level(PIN_LED, estado ? 1 : 0);

                    cJSON *resp = cJSON_CreateObject();
                    cJSON_AddStringToObject(resp, "tipo", "led_ack");
                    cJSON_AddNumberToObject(resp, "estado", estado);
                    send_json_to_fd(fd, resp);
                    cJSON_Delete(resp);

                } else if (tipo && strcmp(tipo, "rgb") == 0) {
                    int r = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "r"));
                    int g = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "g"));
                    int b = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "b"));

                    rgb_set((uint8_t)r, (uint8_t)g, (uint8_t)b);

                    cJSON *ack = cJSON_CreateObject();
                    cJSON_AddStringToObject(ack, "tipo", "rgb_ack");
                    cJSON_AddNumberToObject(ack, "r", r);
                    cJSON_AddNumberToObject(ack, "g", g);
                    cJSON_AddNumberToObject(ack, "b", b);
                    send_json_to_fd(fd, ack);
                    cJSON_Delete(ack);

                } else if (tipo && strcmp(tipo, "mover") == 0) {
                    modo_robot = MODO_MANUAL;
                    siguelineas_auto = false;

                    const char *accionStr = cJSON_GetStringValue(cJSON_GetObjectItem(root, "accion"));
                    int ms = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "ms"));

                    if (ms <= 0) ms = 600;
                    if (ms > 8000) ms = 8000;

                    DirMov a = Dir_Avanzar;

                    if      (accionStr && strcmp(accionStr, "retroceder") == 0) a = Dir_Retroceder;
                    else if (accionStr && strcmp(accionStr, "izquierda")  == 0) a = Dir_Izquierda;
                    else if (accionStr && strcmp(accionStr, "derecha")    == 0) a = Dir_Derecha;
                    else if (accionStr && strcmp(accionStr, "parar")      == 0) a = Dir_Parar;

                    CmdMotor cmd = { a, (uint32_t)ms, 0 };
                    bool ok = (motorCmdQ && (xQueueSend(motorCmdQ, &cmd, 0) == pdTRUE));

                    cJSON *ack2 = cJSON_CreateObject();
                    cJSON_AddStringToObject(ack2, "tipo", ok ? "mover_ack" : "mover_err");
                    cJSON_AddStringToObject(ack2, "accion", accionStr ? accionStr : "avanzar");
                    cJSON_AddNumberToObject(ack2, "ms", ms);
                    cJSON_AddStringToObject(ack2, "modo", modo_robot_str());
                    send_json_to_fd(fd, ack2);
                    cJSON_Delete(ack2);

                } else if (tipo && strcmp(tipo, "leer_linea") == 0) {
                    int a1, a2, a3, a4;
                    leer_siguelineas(&a1, &a2, &a3, &a4);

                    EstadoLinea estado = decidir_estado_linea(a1, a2, a3, a4);

                    cJSON *resp = cJSON_CreateObject();
                    cJSON_AddStringToObject(resp, "tipo", "linea_lectura");
                    cJSON_AddNumberToObject(resp, "a1", a1);
                    cJSON_AddNumberToObject(resp, "a2", a2);
                    cJSON_AddNumberToObject(resp, "a3", a3);
                    cJSON_AddNumberToObject(resp, "a4", a4);
                    cJSON_AddStringToObject(resp, "estado", estado_linea_str(estado));
                    cJSON_AddBoolToObject(resp, "activo", siguelineas_auto);
                    cJSON_AddStringToObject(resp, "modo", modo_robot_str());
                    send_json_to_fd(fd, resp);
                    cJSON_Delete(resp);

                } else if (tipo && strcmp(tipo, "siguelineas_auto") == 0) {
                    bool activo = cJSON_IsTrue(cJSON_GetObjectItem(root, "activo"));

                    siguelineas_auto = activo;

                    if (activo) {
                        modo_robot = MODO_SIGUELINEAS;
                    } else {
                        modo_robot = MODO_MANUAL;

                        CmdMotor cmd = {
                            .accion = Dir_Parar,
                            .dur_ms = 0,
                            .t0_ms = 0
                        };

                        if (motorCmdQ) {
                            xQueueSend(motorCmdQ, &cmd, 0);
                        }
                    }

                    cJSON *resp = cJSON_CreateObject();
                    cJSON_AddStringToObject(resp, "tipo", "siguelineas_auto_ack");
                    cJSON_AddBoolToObject(resp, "activo", siguelineas_auto);
                    cJSON_AddStringToObject(resp, "modo", modo_robot_str());
                    send_json_to_fd(fd, resp);
                    cJSON_Delete(resp);

                } else if (tipo && strcmp(tipo, "siguelineas_config") == 0) {
                    int t_estado     = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "tiempo_estado"));
                    int t_avanzar    = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "avanzar"));
                    int t_retroceder = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "retroceder"));
                    int t_izquierda  = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "izquierda"));
                    int t_derecha    = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "derecha"));

                    if (t_estado > 20)      tiempo_estado_ms = t_estado;
                    if (t_avanzar > 0)      tiempo_avanzar_ms = t_avanzar;
                    if (t_retroceder > 0)   tiempo_retroceder_ms = t_retroceder;
                    if (t_izquierda > 0)    tiempo_izquierda_ms = t_izquierda;
                    if (t_derecha > 0)      tiempo_derecha_ms = t_derecha;

                    cJSON *resp = cJSON_CreateObject();
                    cJSON_AddStringToObject(resp, "tipo", "siguelineas_config_ack");
                    cJSON_AddNumberToObject(resp, "tiempo_estado", tiempo_estado_ms);
                    cJSON_AddNumberToObject(resp, "avanzar", tiempo_avanzar_ms);
                    cJSON_AddNumberToObject(resp, "retroceder", tiempo_retroceder_ms);
                    cJSON_AddNumberToObject(resp, "izquierda", tiempo_izquierda_ms);
                    cJSON_AddNumberToObject(resp, "derecha", tiempo_derecha_ms);
                    cJSON_AddStringToObject(resp, "modo", modo_robot_str());
                    send_json_to_fd(fd, resp);
                    cJSON_Delete(resp);
                }

                cJSON_Delete(root);
            }
        }

        free(ws_pkt.payload);
    }

    return ret;
}

// ==================== HTTPD START ====================

static httpd_handle_t start_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.server_port = WS_PORT;
    config.lru_purge_enable = true;

    ESP_ERROR_CHECK(httpd_start(&httpd, &config));

    httpd_uri_t ws = {
        .uri = WS_URI,
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(httpd, &ws));

    ESP_LOGI(TAG, "WebSocket en ws://%s.local:%d%s", nombreHost, WS_PORT, WS_URI);

    return httpd;
}

// ==================== TELEMETRIA ====================

static void telemetria_cb(void *arg) {
    if (!latidoRecibido) return;

    actualizarLecturaDHT(false);

    int adc = adc_read_raw();

    int a1, a2, a3, a4;
    leer_siguelineas(&a1, &a2, &a3, &a4);

    EstadoLinea estado = decidir_estado_linea(a1, a2, a3, a4);

    cJSON *msg = cJSON_CreateObject();

    cJSON_AddStringToObject(msg, "tipo", "telemetria");
    cJSON_AddNumberToObject(msg, "adc", adc);
    cJSON_AddNumberToObject(msg, "temperatura", tempC_cache);
    cJSON_AddNumberToObject(msg, "humedad", hum_cache);
    cJSON_AddBoolToObject(msg, "dht_ok", dht_ok);
    cJSON_AddNumberToObject(msg, "tiempo_ms", (double)millis32());

    cJSON_AddNumberToObject(msg, "linea_a1", a1);
    cJSON_AddNumberToObject(msg, "linea_a2", a2);
    cJSON_AddNumberToObject(msg, "linea_a3", a3);
    cJSON_AddNumberToObject(msg, "linea_a4", a4);
    cJSON_AddStringToObject(msg, "linea_estado", estado_linea_str(estado));

    cJSON_AddBoolToObject(msg, "siguelineas_auto", siguelineas_auto);
    cJSON_AddStringToObject(msg, "modo", modo_robot_str());

    broadcast_json(msg);
    cJSON_Delete(msg);
}

// ==================== SIGUELINEAS ====================

static void leer_siguelineas(int *a1, int *a2, int *a3, int *a4) {
    *a1 = gpio_get_level(S1);
    *a2 = gpio_get_level(S2);
    *a3 = gpio_get_level(S3);
    *a4 = gpio_get_level(S4);
}

static EstadoLinea decidir_estado_linea(int a1, int a2, int a3, int a4) {
    /*
        Suposición:
        0 = detecta línea negra
        1 = fondo blanco
    */

    if (a2 == 0 && a3 == 0) {
        return EST_LINEA_AVANZAR;
    }

    if (a1 == 0 || a2 == 0) {
        return EST_LINEA_IZQUIERDA;
    }

    if (a3 == 0 || a4 == 0) {
        return EST_LINEA_DERECHA;
    }

    if (a1 == 1 && a2 == 1 && a3 == 1 && a4 == 1) {
        return EST_LINEA_RETROCEDER;
    }

    return EST_LINEA_PARADO;
}

static const char* estado_linea_str(EstadoLinea estado) {
    switch (estado) {
        case EST_LINEA_AVANZAR:
            return "avanzar";

        case EST_LINEA_RETROCEDER:
            return "retroceder";

        case EST_LINEA_IZQUIERDA:
            return "izquierda";

        case EST_LINEA_DERECHA:
            return "derecha";

        default:
            return "parado";
    }
}

void LineaTask(void *arg) {
    EstadoLinea ultimo_estado = EST_LINEA_PARADO;

    for (;;) {
        if (modo_robot == MODO_SIGUELINEAS && siguelineas_auto) {
            int a1, a2, a3, a4;
            leer_siguelineas(&a1, &a2, &a3, &a4);

            EstadoLinea estado = decidir_estado_linea(a1, a2, a3, a4);

            if (estado != ultimo_estado) {
                ultimo_estado = estado;

                CmdMotor cmd = {
                    .accion = Dir_Parar,
                    .dur_ms = 0,
                    .t0_ms = 0
                };

                switch (estado) {
                    case EST_LINEA_AVANZAR:
                        cmd.accion = Dir_Avanzar;
                        cmd.dur_ms = tiempo_avanzar_ms;
                        break;

                    case EST_LINEA_RETROCEDER:
                        cmd.accion = Dir_Retroceder;
                        cmd.dur_ms = tiempo_retroceder_ms;
                        break;

                    case EST_LINEA_IZQUIERDA:
                        cmd.accion = Dir_Izquierda;
                        cmd.dur_ms = tiempo_izquierda_ms;
                        break;

                    case EST_LINEA_DERECHA:
                        cmd.accion = Dir_Derecha;
                        cmd.dur_ms = tiempo_derecha_ms;
                        break;

                    default:
                        cmd.accion = Dir_Parar;
                        cmd.dur_ms = 0;
                        break;
                }

                if (motorCmdQ) {
                    xQueueSend(motorCmdQ, &cmd, 0);
                }
            }

            cJSON *msg = cJSON_CreateObject();

            if (msg) {
                cJSON_AddStringToObject(msg, "tipo", "linea_estado");
                cJSON_AddNumberToObject(msg, "a1", a1);
                cJSON_AddNumberToObject(msg, "a2", a2);
                cJSON_AddNumberToObject(msg, "a3", a3);
                cJSON_AddNumberToObject(msg, "a4", a4);
                cJSON_AddStringToObject(msg, "estado", estado_linea_str(estado));
                cJSON_AddBoolToObject(msg, "activo", siguelineas_auto);
                cJSON_AddStringToObject(msg, "modo", modo_robot_str());

                broadcast_json(msg);
                cJSON_Delete(msg);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(tiempo_estado_ms));
    }
}

// ==================== APP MAIN ====================

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_LOGI(TAG, "Boot");

    char suf[8] = {0};
    sufijo_mac6(suf, sizeof(suf));
    snprintf(nombreHost, sizeof(nombreHost), "esp32-%s", suf);

    wifi_init();

    ESP_LOGI(TAG, "Conectando a WiFi...");

    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LED) |
                        (1ULL << IN1) |
                        (1ULL << IN2) |
                        (1ULL << IN3) |
                        (1ULL << IN4),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_config_t io_siguelineas = {
        .pin_bit_mask = (1ULL << S1) |
                        (1ULL << S2) |
                        (1ULL << S3) |
                        (1ULL << S4),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&io_siguelineas));

    gpio_set_level(PIN_LED, 0);
    motores_parar_total();

    led_strip_config_t strip_config = {
        .strip_gpio_num = PIN_RGB,
        .max_leds = 1,
        .led_model = LED_MODEL_WS2812,
        .flags.invert_out = false
    };

    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 10 * 1000 * 1000,
        .mem_block_symbols = 64,
        .flags.with_dma = false
    };

    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CH, &ch_cfg));

    mdns_start();

    ws_mutex = xSemaphoreCreateMutex();
    memset(ws_client_fds, 0, sizeof(ws_client_fds));

    motorCmdQ = xQueueCreate(5, sizeof(CmdMotor));

    xTaskCreatePinnedToCore(MotorTask, "MotorTask", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(LineaTask, "LineaTask", 4096, NULL, 1, NULL, 1);

    start_server();

    const esp_timer_create_args_t targs = {
        .callback = &telemetria_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "telemetria"
    };

    esp_timer_handle_t tmr;

    ESP_ERROR_CHECK(esp_timer_create(&targs, &tmr));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tmr, (uint64_t)CADA_MS * 1000ULL));

    ESP_LOGI(TAG, "Listo: %s.local", nombreHost);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
