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
#include "freertos/event_groups.h"

#include "esp_mac.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "mdns.h"
#include "esp_http_server.h"
#include "cJSON.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "led_strip.h"

#include "esp_adc/adc_oneshot.h"

// ==================== CONFIG GENERAL ====================
#define TAG "APP"

// SIMULAR DHT
#define SIMULAR_DHT true

#ifndef PIN_RGB
#define PIN_RGB 48
#endif

#define PIN_LED 44

// WiFi
static const char *WIFI_SSID = "GWN571D04";
static const char *WIFI_PASS = "ESP32CUCEI$$";

#define WIFI_MAX_RETRY        15
#define WIFI_CONNECT_TIMEOUT_MS 30000

// WebSocket en puerto 81 y ruta /ws
#define WS_PORT 81
#define WS_URI  "/ws"

// ADC: ESP32-S3 GPIO4 -> ADC1_CHANNEL_3
#define ADC_UNIT ADC_UNIT_1
#define ADC_CH   ADC_CHANNEL_3

// DHT simulado / real
#define DHT_MIN_MS_REAL 2000
#define DHT_MIN_MS_SIM   500

// Motores L298N
#define IN1 11
#define IN2 12
#define IN3 13
#define IN4 14

// ==================== ESTADO GLOBAL ====================
static volatile bool latidoRecibido = false;
static uint32_t CADA_MS = 2000;

static float tempC_cache = NAN;
static float hum_cache   = NAN;
static bool  dht_ok      = false;
static uint32_t ultimoDhtMs = 0;

static char nombreHost[32] = {0};

// ==================== WIFI EVENT GROUP ====================
static EventGroupHandle_t wifi_event_group = NULL;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int wifi_retry_num = 0;
static esp_netif_t *sta_netif = NULL;

// ==================== WEBSOCKET ====================
#define MAX_CLIENTES 8

static int ws_client_fds[MAX_CLIENTES] = {0};
static SemaphoreHandle_t ws_mutex = NULL;
static httpd_handle_t httpd = NULL;

// ==================== LED STRIP ====================
static led_strip_handle_t led_strip = NULL;

// ==================== ADC ====================
static adc_oneshot_unit_handle_t adc_handle = NULL;

// ==================== MOTORES ====================
typedef enum {
    Dir_Parado = 0,
    Dir_Avanzar,
    Dir_Retroceder,
    Dir_Izquierda,
    Dir_Derecha,
    Dir_Parar
} DirMov;

typedef struct {
    DirMov accion;
    uint32_t dur_ms;
    uint32_t t0_ms;
} CmdMotor;

static QueueHandle_t motorCmdQ = NULL;

// ==================== HELPERS TIEMPO ====================
static uint32_t millis32(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// ==================== WEBSOCKET HELPERS ====================
static void ws_remove_fd(int fd);

static void send_json_to_fd(int fd, cJSON *root) {
    if (!root || !httpd || fd < 0) return;

    char *txt = cJSON_PrintUnformatted(root);
    if (!txt) return;

    httpd_ws_frame_t ws_pkt = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)txt,
        .len = strlen(txt)
    };

    esp_err_t err = httpd_ws_send_frame_async(httpd, fd, &ws_pkt);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "[WS] Error enviando a fd=%d: %s. Se elimina cliente.",
                 fd, esp_err_to_name(err));
        ws_remove_fd(fd);
    }

    free(txt);
}

static void broadcast_json(cJSON *root) {
    if (!root || !ws_mutex) return;

    int fds[MAX_CLIENTES];
    int count = 0;

    /*
     * Copiamos los fd bajo mutex y enviamos fuera del mutex.
     * Esto evita deadlocks si send_json_to_fd() necesita eliminar un fd.
     */
    if (xSemaphoreTake(ws_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < MAX_CLIENTES; ++i) {
            if (ws_client_fds[i] > 0) {
                fds[count++] = ws_client_fds[i];
            }
        }
        xSemaphoreGive(ws_mutex);
    }

    for (int i = 0; i < count; ++i) {
        send_json_to_fd(fds[i], root);
    }
}

static void ws_add_fd(int fd) {
    if (!ws_mutex || fd < 0) return;

    if (xSemaphoreTake(ws_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        bool ya_existe = false;
        for (int i = 0; i < MAX_CLIENTES; ++i) {
            if (ws_client_fds[i] == fd) {
                ya_existe = true;
                break;
            }
        }

        if (!ya_existe) {
            bool agregado = false;
            for (int i = 0; i < MAX_CLIENTES; ++i) {
                if (ws_client_fds[i] == 0) {
                    ws_client_fds[i] = fd;
                    agregado = true;
                    break;
                }
            }

            if (!agregado) {
                ESP_LOGW(TAG, "[WS] Lista de clientes llena. fd=%d no agregado.", fd);
            }
        }

        xSemaphoreGive(ws_mutex);
    }
}

static void ws_remove_fd(int fd) {
    if (!ws_mutex || fd < 0) return;

    if (xSemaphoreTake(ws_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        for (int i = 0; i < MAX_CLIENTES; ++i) {
            if (ws_client_fds[i] == fd) {
                ws_client_fds[i] = 0;
                break;
            }
        }
        xSemaphoreGive(ws_mutex);
    }
}

// ==================== LED RGB ====================
static void rgb_set(uint8_t r, uint8_t g, uint8_t b) {
    if (!led_strip) return;

    ESP_ERROR_CHECK_WITHOUT_ABORT(led_strip_set_pixel(led_strip, 0, r, g, b));
    ESP_ERROR_CHECK_WITHOUT_ABORT(led_strip_refresh(led_strip));
}

// ==================== ADC ====================
static int adc_read_raw(void) {
    int val = 0;

    if (adc_handle) {
        esp_err_t err = adc_oneshot_read(adc_handle, ADC_CH, &val);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Error leyendo ADC: %s", esp_err_to_name(err));
            return 0;
        }
    }

    return val;
}

// ==================== MOTORES GPIO ====================
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

static void MotorTask(void *arg) {
    bool enMovimiento = false;
    uint32_t t_fin = 0;

    for (;;) {
        CmdMotor nuevo;

        if (motorCmdQ && xQueueReceive(motorCmdQ, &nuevo, pdMS_TO_TICKS(250)) == pdTRUE) {
            enMovimiento = false;
            motores_parar_total();

            if (nuevo.accion != Dir_Parar &&
                nuevo.accion != Dir_Parado &&
                nuevo.dur_ms > 0) {

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
                    cJSON_AddNumberToObject(ack, "ms", nuevo.dur_ms);
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
                    broadcast_json(fin);
                    cJSON_Delete(fin);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==================== DHT SIMULADO / STUB ====================
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
        /*
         * Lectura real no implementada todavía.
         * Si cambias SIMULAR_DHT a false, agrega aquí tu driver DHT.
         */
        dht_ok = false;
        ultimoDhtMs = ahora;
    }
}

// ==================== MAC / HOSTNAME HELPERS ====================
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
static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "[WiFi] STA_START. Conectando...");
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;

        ESP_LOGW(TAG, "[WiFi] Desconectado. reason=%d, retry=%d/%d",
                 disc ? disc->reason : -1,
                 wifi_retry_num,
                 WIFI_MAX_RETRY);

        if (wifi_retry_num < WIFI_MAX_RETRY) {
            wifi_retry_num++;
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        ESP_LOGI(TAG, "[WiFi] IP obtenida: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "[WiFi] Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        ESP_LOGI(TAG, "[WiFi] Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));

        wifi_retry_num = 0;
        xEventGroupClearBits(wifi_event_group, WIFI_FAIL_BIT);
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_and_wait_ip(void) {
    wifi_event_group = xEventGroupCreate();
    if (!wifi_event_group) {
        ESP_LOGE(TAG, "[WiFi] No se pudo crear EventGroup");
        return false;
    }

    ESP_ERROR_CHECK(esp_netif_init());

    esp_err_t err_loop = esp_event_loop_create_default();
    if (err_loop != ESP_OK && err_loop != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err_loop);
    }

    sta_netif = esp_netif_create_default_wifi_sta();
    if (!sta_netif) {
        ESP_LOGE(TAG, "[WiFi] No se pudo crear netif STA");
        return false;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        &wifi_event_handler,
        NULL,
        NULL
    ));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        &wifi_event_handler,
        NULL,
        NULL
    ));

    wifi_config_t wifi_config = {0};

    strlcpy((char *)wifi_config.sta.ssid,
            WIFI_SSID,
            sizeof(wifi_config.sta.ssid));

    strlcpy((char *)wifi_config.sta.password,
            WIFI_PASS,
            sizeof(wifi_config.sta.password));

    /*
     * Ajuste conservador para placas ESP32-S3 y routers variados:
     * - WPA2 como mínimo.
     * - PMF capaz pero no obligatorio.
     * - Se desactiva WiFi power save para evitar problemas con algunos AP.
     */
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "[WiFi] Esperando IP por DHCP...");

    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS)
    );

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "[WiFi] Conectado correctamente.");
        return true;
    }

    if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "[WiFi] Fallo después de %d reintentos.", WIFI_MAX_RETRY);
    } else {
        ESP_LOGE(TAG, "[WiFi] Timeout esperando IP.");
    }

    return false;
}

// ==================== mDNS ====================
static void mdns_start_service(void) {
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(nombreHost));
    ESP_ERROR_CHECK(mdns_instance_name_set("ESP32-S3 Server"));

    mdns_txt_item_t serviceTxtData[] = {
        {"proto", "json"}
    };

    ESP_ERROR_CHECK(mdns_service_add(
        "ws",
        "_ws",
        "_tcp",
        WS_PORT,
        serviceTxtData,
        1
    ));

    ESP_LOGI(TAG, "[mDNS] Host: %s.local", nombreHost);
}

// ==================== WEBSOCKET HANDLER ====================
static esp_err_t ws_handler(httpd_req_t *req) {
    int fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET) {
        ws_add_fd(fd);

        ESP_LOGI(TAG, "[WS] Cliente fd=%d conectado", fd);

        cJSON *msg = cJSON_CreateObject();
        if (msg) {
            cJSON_AddStringToObject(msg, "tipo", "saludo");
            cJSON_AddStringToObject(msg, "mensaje", "Servidor ESP32-S3 listo");
            cJSON_AddNumberToObject(msg, "puerto", WS_PORT);
            cJSON_AddStringToObject(msg, "nombre_host", nombreHost);

            uint8_t mac[6];
            esp_read_mac(mac, ESP_MAC_WIFI_STA);

            char macfmt[18];
            snprintf(macfmt,
                     sizeof(macfmt),
                     "%02X:%02X:%02X:%02X:%02X:%02X",
                     mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

            cJSON_AddStringToObject(msg, "mac", macfmt);
            send_json_to_fd(fd, msg);
            cJSON_Delete(msg);
        }

        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(ws_pkt));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;

    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[WS] Error leyendo longitud fd=%d: %s",
                 fd, esp_err_to_name(ret));
        ws_remove_fd(fd);
        return ret;
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ws_remove_fd(fd);
        ESP_LOGI(TAG, "[WS] Cliente fd=%d desconectado", fd);
        return ESP_OK;
    }

    if (ws_pkt.type != HTTPD_WS_TYPE_TEXT) {
        ESP_LOGW(TAG, "[WS] Frame no texto ignorado. fd=%d type=%d",
                 fd, ws_pkt.type);
        return ESP_OK;
    }

    if (ws_pkt.len == 0) {
        return ESP_OK;
    }

    ws_pkt.payload = (uint8_t *)calloc(1, ws_pkt.len + 1);
    if (!ws_pkt.payload) {
        return ESP_ERR_NO_MEM;
    }

    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "[WS] Error leyendo payload fd=%d: %s",
                 fd, esp_err_to_name(ret));
        free(ws_pkt.payload);
        ws_remove_fd(fd);
        return ret;
    }

    ws_pkt.payload[ws_pkt.len] = '\0';

    cJSON *root = cJSON_Parse((char *)ws_pkt.payload);
    if (!root) {
        ESP_LOGW(TAG, "[WS] JSON inválido recibido: %s", (char *)ws_pkt.payload);

        cJSON *err = cJSON_CreateObject();
        if (err) {
            cJSON_AddStringToObject(err, "tipo", "json_err");
            cJSON_AddStringToObject(err, "mensaje", "JSON invalido");
            send_json_to_fd(fd, err);
            cJSON_Delete(err);
        }

        free(ws_pkt.payload);
        return ESP_OK;
    }

    const char *tipo = cJSON_GetStringValue(cJSON_GetObjectItem(root, "tipo"));

    if (tipo && strcmp(tipo, "latido") == 0) {
        latidoRecibido = true;

        cJSON *ack = cJSON_CreateObject();
        if (ack) {
            cJSON_AddStringToObject(ack, "tipo", "latido_ok");
            cJSON_AddNumberToObject(ack, "tiempo_ms", (double)millis32());
            send_json_to_fd(fd, ack);
            cJSON_Delete(ack);
        }

    } else if (tipo && strcmp(tipo, "leer_dht") == 0) {
        actualizarLecturaDHT(true);

        cJSON *resp = cJSON_CreateObject();
        if (resp) {
            cJSON_AddStringToObject(resp, "tipo", "dht_lectura");
            cJSON_AddBoolToObject(resp, "dht_ok", dht_ok);
            cJSON_AddNumberToObject(resp, "temperatura", tempC_cache);
            cJSON_AddNumberToObject(resp, "humedad", hum_cache);
            send_json_to_fd(fd, resp);
            cJSON_Delete(resp);
        }

    } else if (tipo && strcmp(tipo, "led") == 0) {
        int estado = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "estado"));
        gpio_set_level(PIN_LED, estado ? 1 : 0);

        cJSON *resp = cJSON_CreateObject();
        if (resp) {
            cJSON_AddStringToObject(resp, "tipo", "led_ack");
            cJSON_AddNumberToObject(resp, "estado", estado ? 1 : 0);
            send_json_to_fd(fd, resp);
            cJSON_Delete(resp);
        }

    } else if (tipo && strcmp(tipo, "rgb") == 0) {
        int r = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "r"));
        int g = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "g"));
        int b = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "b"));

        r = MIN(MAX(r, 0), 255);
        g = MIN(MAX(g, 0), 255);
        b = MIN(MAX(b, 0), 255);

        rgb_set((uint8_t)r, (uint8_t)g, (uint8_t)b);

        cJSON *ack = cJSON_CreateObject();
        if (ack) {
            cJSON_AddStringToObject(ack, "tipo", "rgb_ack");
            cJSON_AddNumberToObject(ack, "r", r);
            cJSON_AddNumberToObject(ack, "g", g);
            cJSON_AddNumberToObject(ack, "b", b);
            send_json_to_fd(fd, ack);
            cJSON_Delete(ack);
        }

    } else if (tipo && strcmp(tipo, "mover") == 0) {
        const char *accionStr = cJSON_GetStringValue(cJSON_GetObjectItem(root, "accion"));
        int ms = cJSON_GetNumberValue(cJSON_GetObjectItem(root, "ms"));

        if (ms <= 0) ms = 600;
        if (ms > 8000) ms = 8000;

        DirMov a = Dir_Avanzar;

        if (accionStr && strcmp(accionStr, "retroceder") == 0) {
            a = Dir_Retroceder;
        } else if (accionStr && strcmp(accionStr, "izquierda") == 0) {
            a = Dir_Izquierda;
        } else if (accionStr && strcmp(accionStr, "derecha") == 0) {
            a = Dir_Derecha;
        } else if (accionStr && strcmp(accionStr, "parar") == 0) {
            a = Dir_Parar;
        }

        CmdMotor cmd = {
            .accion = a,
            .dur_ms = (uint32_t)ms,
            .t0_ms = 0
        };

        bool ok = false;
        if (motorCmdQ) {
            /*
             * Overwrite no se puede usar en colas de longitud >1.
             * Se usa xQueueSend con espera corta para no bloquear el servidor.
             */
            ok = (xQueueSend(motorCmdQ, &cmd, pdMS_TO_TICKS(20)) == pdTRUE);
        }

        cJSON *ack2 = cJSON_CreateObject();
        if (ack2) {
            cJSON_AddStringToObject(ack2, "tipo", ok ? "mover_ack" : "mover_err");
            cJSON_AddStringToObject(ack2, "accion", accionStr ? accionStr : "avanzar");
            cJSON_AddNumberToObject(ack2, "ms", ms);
            send_json_to_fd(fd, ack2);
            cJSON_Delete(ack2);
        }

    } else {
        cJSON *err = cJSON_CreateObject();
        if (err) {
            cJSON_AddStringToObject(err, "tipo", "cmd_err");
            cJSON_AddStringToObject(err, "mensaje", "Tipo de comando no reconocido");
            send_json_to_fd(fd, err);
            cJSON_Delete(err);
        }
    }

    cJSON_Delete(root);
    free(ws_pkt.payload);

    return ESP_OK;
}

// ==================== HTTPD START ====================
static httpd_handle_t start_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.server_port = WS_PORT;
    config.lru_purge_enable = true;
    config.max_open_sockets = MAX_CLIENTES + 3;

    ESP_ERROR_CHECK(httpd_start(&httpd, &config));

    httpd_uri_t ws = {
        .uri = WS_URI,
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };

    ESP_ERROR_CHECK(httpd_register_uri_handler(httpd, &ws));

    ESP_LOGI(TAG, "[HTTPD] WebSocket en ws://%s.local:%d%s",
             nombreHost, WS_PORT, WS_URI);

    return httpd;
}

// ==================== TELEMETRIA TIMER ====================
static void telemetria_cb(void *arg) {
    if (!latidoRecibido) return;

    actualizarLecturaDHT(false);

    int adc = adc_read_raw();

    cJSON *msg = cJSON_CreateObject();
    if (!msg) return;

    cJSON_AddStringToObject(msg, "tipo", "telemetria");
    cJSON_AddNumberToObject(msg, "adc", adc);
    cJSON_AddNumberToObject(msg, "temperatura", tempC_cache);
    cJSON_AddNumberToObject(msg, "humedad", hum_cache);
    cJSON_AddBoolToObject(msg, "dht_ok", dht_ok);
    cJSON_AddNumberToObject(msg, "tiempo_ms", (double)millis32());

    broadcast_json(msg);
    cJSON_Delete(msg);
}

// ==================== INIT HARDWARE ====================
static void gpio_init_app(void) {
    gpio_config_t io = {
        .pin_bit_mask =
            (1ULL << PIN_LED) |
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

    gpio_set_level(PIN_LED, 0);
    motores_parar_total();
}

static void rgb_init_app(void) {
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

    ESP_ERROR_CHECK(led_strip_new_rmt_device(
        &strip_config,
        &rmt_config,
        &led_strip
    ));

    ESP_ERROR_CHECK(led_strip_clear(led_strip));
}

static void adc_init_app(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT
    };

    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CH, &ch_cfg));
}

static void motor_task_init_app(void) {
    motorCmdQ = xQueueCreate(5, sizeof(CmdMotor));
    if (!motorCmdQ) {
        ESP_LOGE(TAG, "No se pudo crear cola de motores");
        return;
    }

    BaseType_t ok = xTaskCreatePinnedToCore(
        MotorTask,
        "MotorTask",
        4096,
        NULL,
        1,
        NULL,
        1
    );

    if (ok != pdPASS) {
        ESP_LOGE(TAG, "No se pudo crear MotorTask");
    }
}

static void telemetry_timer_init_app(void) {
    const esp_timer_create_args_t targs = {
        .callback = &telemetria_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "telemetria"
    };

    esp_timer_handle_t tmr = NULL;

    ESP_ERROR_CHECK(esp_timer_create(&targs, &tmr));
    ESP_ERROR_CHECK(esp_timer_start_periodic(
        tmr,
        (uint64_t)CADA_MS * 1000ULL
    ));
}

// ==================== APP MAIN ====================
void app_main(void) {
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "Boot ESP32-S3");

    char suf[8] = {0};
    sufijo_mac6(suf, sizeof(suf));
    snprintf(nombreHost, sizeof(nombreHost), "esp32-%s", suf);

    ESP_LOGI(TAG, "Hostname configurado: %s.local", nombreHost);

    ws_mutex = xSemaphoreCreateMutex();
    if (!ws_mutex) {
        ESP_LOGE(TAG, "No se pudo crear mutex WebSocket");
        return;
    }

    gpio_init_app();
    rgb_init_app();
    adc_init_app();
    motor_task_init_app();

    /*
     * Correccion importante:
     * antes el programa arrancaba mDNS, WebSocket y timers inmediatamente
     * despues de esp_wifi_connect().
     *
     * Ahora se espera hasta recibir IP_EVENT_STA_GOT_IP o timeout.
     */
    bool wifi_ok = wifi_init_and_wait_ip();

    if (wifi_ok) {
        mdns_start_service();
        start_server();
        telemetry_timer_init_app();

        ESP_LOGI(TAG, "Listo: ws://%s.local:%d%s",
                 nombreHost, WS_PORT, WS_URI);
    } else {
        /*
         * Si no hay IP, dejamos el programa vivo.
         * El manejador de eventos seguira intentando si hay nuevos disconnect/start,
         * pero no se arranca servidor sin IP.
         */
        ESP_LOGE(TAG, "WiFi no obtuvo IP. Revisa DHCP/router/senal/credenciales.");
        rgb_set(255, 0, 0);
    }

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
