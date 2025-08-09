#include <sdkconfig.h>

#ifdef CONFIG_CRSF_FIRMWARE_TX

#include "driver/gptimer_types.h"
#include "esp_err.h"
#include "esp_log_buffer.h"
#include "esp_log_level.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include <stdint.h>
#include <string.h>
static const char *TAG= "MAIN_TX";
static const char *TAG_CONTROLLER = "MAIN_TX_CONTROLLER";
static const char *TAG_MODULE = "MAIN_TX_MODULE";
static const char *TAG_UDP = "UDP_PARSER";

#include <driver/uart.h>
#include <driver/gptimer.h>
#include <hal/uart_types.h>

#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <soc/soc.h>
#include <soc/uart_periph.h>
#include <hal/uart_ll.h>
#include <rom/gpio.h>

#include "protocol_crsf.h"

#include <map>

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include <errno.h>
#include <string.h>

#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "protocol.pb.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#define WIFI_SSID       CONFIG_WIFI_SSID
#define WIFI_PASS       CONFIG_WIFI_PASSWORD

#define WIFI_MAXIMUM_RETRY 3      // Reduced from 5
#define WIFI_RECONNECT_DELAY_MS 30000  // 30 seconds between attempts
#define WIFI_MONITOR_INTERVAL_MS 60000

// WiFi event group
static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// UDP configuration
#define UDP_PORT 8888
#define UDP_BUFFER_SIZE 1024
#define TTP_HEADER "TTP\1"
#define TTP_HEADER_SIZE 4

#ifndef CRSF_FRAMETYPE_RC_CHANNELS_PACKED
#define CRSF_FRAMETYPE_RC_CHANNELS_PACKED 0x16
#endif

#ifndef GUID_H_MAX_M
#define GUID_H_MAX_M 500.0
#endif

// RC діапазон приймача: 988..2012 µs (рівно 1024)
#define US_MIN   988
#define US_MAX   2012
#define US_SPAN  (US_MAX - US_MIN)   // 1024
#define US_MID   1500
#define US_HALF  (US_SPAN/2)         // 512

static int s_retry_num = 0;
static uint64_t s_last_connect_attempt = 0;

#define UART_PORT_CONTROLLER UART_NUM_1
#define UART_PORT_MODULE UART_NUM_2

#define UART_PIN_CONTROLLER GPIO_NUM_13
#define UART_PIN_MODULE GPIO_NUM_14

#define CRSF_RATE_HZ 62
#define CRSF_BAUDRATE 115200
#define CRSF_INVERTED false

TaskHandle_t task_controller = NULL;
TaskHandle_t task_module = NULL;
QueueHandle_t queue_uart_controller = NULL;
QueueHandle_t queue_uart_module = NULL;

QueueHandle_t queue_crsf_controller = NULL;
QueueHandle_t queue_crsf_module = NULL;

static QueueHandle_t      udp_data_queue = nullptr;
static SemaphoreHandle_t  udp_buffer_mutex = nullptr;

uint64_t _last_crsf_period_us;

typedef struct {
    double timestamp_sec;
    bool precise_timestamp;
    char target_id[64];
    struct {
        float latitude_deg;
        float longitude_deg;
        float altitude_msl_m;
    } position;
    struct {
        float east_ms;
        float north_ms;
        float up_ms;
    } velocity;
    uint32_t state; // 0=Unstable, 1=Stable
} target_estimate_t;

typedef struct {
    target_estimate_t estimate;
    bool               is_target; // true = field 2, false = field 3
} udp_message_t;

// ---- Guidance дані ----
typedef struct {
    double az_rel_deg;      // 0..360
    double rel_height_m;    // −H..+H
    double src_timestamp_s; // сек (esp_timer або з estimate)
    bool   valid;
    bool   both_stable;
} guidance_metrics_t;

guidance_metrics_t g_guid;
SemaphoreHandle_t  g_guid_mtx = nullptr;

static inline void udp_buffer_add(const target_estimate_t *est,
                                  bool is_target)
{
    if (!udp_data_queue) return;

    udp_message_t msg{
        .estimate  = *est,
        .is_target = is_target,
    };

    xQueueSend(udp_data_queue, &msg, 0);
}

void IRAM_ATTR _half_duplex_set_rx(uart_port_t port, gpio_num_t pin) {
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_INPUT));
    //gpio_matrix_in(pin, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), true);
    esp_rom_gpio_connect_in_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), true);
    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY));
}

void IRAM_ATTR _half_duplex_set_tx(uart_port_t port, gpio_num_t pin) {
    ESP_ERROR_CHECK(gpio_set_pull_mode(pin, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(pin, 0));
    ESP_ERROR_CHECK(gpio_set_direction(pin, GPIO_MODE_OUTPUT));
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), false);
    esp_rom_gpio_connect_out_signal(pin, UART_PERIPH_SIGNAL(port, SOC_UART_TX_PIN_IDX), true, false);

    //constexpr uint8_t MATRIX_DETACH_IN_LOW = 0x30; // routes 0 to matrix slot
    //gpio_matrix_in(MATRIX_DETACH_IN_LOW, UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX), false); // Disconnect RX from all pads
    //gpio_matrix_out(pin, UART_PERIPH_SIGNAL(port, SOC_UART_TX_PIN_IDX), true, false);
}

void _setup_uart(uart_port_t port, gpio_num_t pin, QueueHandle_t *queue) {
    uart_config_t cfg = {
        .baud_rate = CRSF_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {
            .allow_pd = 0,
            .backup_before_sleep = 0,
        },
    };

    uint32_t uart_intr_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_OVF | UART_INTR_RXFIFO_TOUT;

    uart_intr_config_t intr_cfg {
        .intr_enable_mask = uart_intr_mask,
        .rx_timeout_thresh = 2,
        .txfifo_empty_intr_thresh = 0,
        .rxfifo_full_thresh = 64,
    };

    ESP_ERROR_CHECK(uart_param_config(port, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(port, pin, pin, -1, -1));
    ESP_ERROR_CHECK(uart_driver_install(port, 256, 256, 512, queue, 0));
    ESP_ERROR_CHECK(uart_intr_config(port, &intr_cfg));
    ESP_ERROR_CHECK(uart_enable_rx_intr(port));

    ESP_LOGI(TAG, "port=%u, tx_signal=%u, rx_signal=%u", port,
             UART_PERIPH_SIGNAL(port, SOC_UART_TX_PIN_IDX),
             UART_PERIPH_SIGNAL(port, SOC_UART_RX_PIN_IDX));

}

bool _uart_read_crsf_frame(const char *tag, uart_port_t port, QueueHandle_t queue_uart, 
                           crsf_frame_t *frame, TickType_t timeout) {
    bool result = false;
    uint8_t buff[256];

    uart_event_t event;
    size_t len_buffer = 0, len_read = 0;

    if (xQueueReceive(queue_uart, &event, timeout) == pdPASS) {
        ESP_ERROR_CHECK(uart_get_buffered_data_len(port, &len_buffer));

        if (event.type == UART_DATA || event.type == UART_BREAK) {
            len_read = uart_read_bytes(port, buff, len_buffer, portMAX_DELAY);

            crsf_parse_ctx_t ctx;
            memset(&ctx, 0, sizeof(crsf_parse_ctx_t));
            memset(frame, 0, sizeof(crsf_frame_t));
            ctx.frame = frame;

            for (int i = 0; i < len_read; i++) {
                crsf_frame_partial_parse(&ctx, buff[i]);

                if (ctx.status == CRSF_FRAME_STATUS_FRAME_READY) {
                    result = true;
                    break;
                } else if (ctx.status == CRSF_FRAME_STATUS_ERROR_LENGTH ||
                    ctx.status == CRSF_FRAME_STATUS_ERROR_CRC ||
                    ctx.status == CRSF_FRAME_STATUS_ERROR_SYNC) {
                    ESP_LOG_BUFFER_HEX(tag, reinterpret_cast<uint8_t *>(buff), len_read);
                    ESP_LOGE(tag, "crsf error=%u, event=%u, size=%u, flag=%u, len_buffer=%u", ctx.status, event.type, event.size, event.timeout_flag, len_buffer);
                    break;
                }
            }
        } else if (event.type == UART_FIFO_OVF) {
            ESP_LOGI(tag, "event=overflow, size=%u, flag=%u, len_buffer=%u", event.size, event.timeout_flag, len_buffer);
            uart_flush_input(port);
            xQueueReset(queue_uart);
        } else if (event.type == UART_BUFFER_FULL) {
            ESP_LOGI(tag, "event=full, size=%u, flag=%u, len_buffer=%u", event.size, event.timeout_flag, len_buffer);
            uart_flush_input(port);
            xQueueReset(queue_uart);
        } else {
            ESP_LOGI(tag, "event=0x%x, size=%u, flag=%u, len_buffer=%u", event.type, event.size, event.timeout_flag, len_buffer);
        }
    }

    uart_flush_input(port);
    return result;
}


static inline uint16_t us_to_crsf_raw(uint16_t us) {
    if (us < US_MIN) us = US_MIN; else if (us > US_MAX) us = US_MAX;
    return (uint16_t)(172 + ((uint32_t)(us - US_MIN) * 1639u) / 1024u);
}

static inline uint16_t crsf_raw_to_us(uint16_t raw) {
    if (raw < 172) raw = 172; else if (raw > 1811) raw = 1811;
    return (uint16_t)(US_MIN + ((uint32_t)(raw - 172) * 1024u) / 1639u);
}

static inline uint8_t* crsf_payload_ptr(crsf_frame_t *f) {
    return ((uint8_t*)&f->type) + 1;
}

static inline const uint8_t* crsf_payload_ptr_ro(const crsf_frame_t *f) {
    return ((const uint8_t*)&f->type) + 1;
}

static inline void crsf_unpack_rc_channels_us(const crsf_frame_t *f, uint16_t ch_us[16]) {
    const uint8_t *p = crsf_payload_ptr_ro(f); // 22 bytes
    for (int i = 0; i < 16; ++i) {
        uint32_t bitpos = i * 11;
        uint32_t byteix = bitpos >> 3;
        uint32_t bitoff = bitpos & 7;
        uint32_t b0 = p[byteix];
        uint32_t b1 = (byteix + 1 < 22) ? p[byteix + 1] : 0;
        uint32_t b2 = (byteix + 2 < 22) ? p[byteix + 2] : 0;
        uint32_t w  = b0 | (b1 << 8) | (b2 << 16);
        uint16_t raw = (w >> bitoff) & 0x7FFu;
        ch_us[i] = crsf_raw_to_us(raw);
    }
}

static inline void crsf_pack_rc_channels_us(crsf_frame_t *f, const uint16_t ch_us[16]) {
    uint8_t *p = crsf_payload_ptr(f);
    for (int i = 0; i < 22; ++i) p[i] = 0;
    for (int i = 0; i < 16; ++i) {
        uint16_t raw = us_to_crsf_raw(ch_us[i]);
        uint32_t bitpos = i * 11;
        uint32_t byteix = bitpos >> 3;
        uint32_t bitoff = bitpos & 7;
        uint32_t w = ((uint32_t)raw) << bitoff;
        p[byteix] |= (uint8_t)(w & 0xFF);
        if (byteix + 1 < 22) p[byteix + 1] |= (uint8_t)((w >> 8) & 0xFF);
        if (byteix + 2 < 22) p[byteix + 2] |= (uint8_t)((w >> 16) & 0xFF);
    }
}

static inline uint8_t crc8_d5(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
    }
    return crc;
}

void crsf_update_crc(crsf_frame_t *f) {
    static_assert(sizeof(crsf_frame_type_t) == 1, "crsf_frame_type_t must be 1 byte");

    uint8_t       *type_ptr = reinterpret_cast<uint8_t *>(&f->type);
    const uint8_t *type_cptr= reinterpret_cast<const uint8_t *>(&f->type);
    size_t len_no_crc = f->length - 1;
    uint8_t *crc_ptr  = type_ptr + len_no_crc;

    *crc_ptr = crc8_d5(type_cptr, len_no_crc);
}

static inline bool guidance_snapshot(guidance_metrics_t *out, double max_age_s) {
    bool ok = false;
    if (xSemaphoreTake(g_guid_mtx, pdMS_TO_TICKS(2)) == pdTRUE) {
        double now_s = (double)esp_timer_get_time() * 1e-6;
        ok = g_guid.valid && g_guid.both_stable && fabs(now_s - g_guid.src_timestamp_s) <= max_age_s;
        if (ok) *out = g_guid;
        xSemaphoreGive(g_guid_mtx);
    }
    return ok;
}

static inline uint16_t map_azimuth_deg_to_us(double deg) {
    if (deg < 0.0) deg = 0.0; else if (deg > 360.0) deg = 360.0;
    double us = US_MIN + (deg / 360.0) * US_SPAN;
    return (uint16_t)(us + 0.5);
}

static inline uint16_t map_height_m_to_us(double h) {
    if (h < -GUID_H_MAX_M) h = -GUID_H_MAX_M;
    if (h >  GUID_H_MAX_M) h =  GUID_H_MAX_M;
    double us = US_MID + (h / GUID_H_MAX_M) * US_HALF;
    return (uint16_t)(us + 0.5);
}

void _swap_crsf_frame(crsf_frame_t *frame) {
    static uint32_t _frame_type_counter[UINT8_MAX] = {0};
    _frame_type_counter[frame->type] = _frame_type_counter[frame->type] + 1;

    static uint64_t _last_stats_time = 0;
    if (_last_stats_time + 1000000 < esp_timer_get_time()) {
        _last_stats_time = esp_timer_get_time();

        char log[256];
        size_t log_cursor = 0;
        for (int i = 0; i < UINT8_MAX; i++) {
            if (_frame_type_counter[i] > 0) {
                log_cursor += snprintf(log + log_cursor, 256 - log_cursor, "0x%x=%lu, ", i, _frame_type_counter[i]);
                ESP_LOGI(TAG, "%s", log);
            }
        }
    }

    if (frame->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
        uint16_t ch[16];
        crsf_unpack_rc_channels_us(frame, ch);

        static bool s_guidance_mode = false;
        const int ON_TH = 1550, OFF_TH = 1450;
        if      (ch[4] > ON_TH)  s_guidance_mode = true;
        else if (ch[4] < OFF_TH) s_guidance_mode = false;

        guidance_metrics_t gm;
        bool have_guid = guidance_snapshot(&gm, /*max_age_s=*/0.5);

        if (s_guidance_mode && have_guid) {
            ch[0] = map_azimuth_deg_to_us(gm.az_rel_deg);
            ch[1] = US_MID;
            ch[2] = map_height_m_to_us(gm.rel_height_m);
            ch[3] = US_MID;

            crsf_pack_rc_channels_us(frame, ch);
            crsf_update_crc(frame);

            ESP_LOGI(TAG, "GUIDANCE: CH1=%u, CH3=%u", ch[0], ch[2]);
        }
    }
}

typedef struct {
    double az_true_deg;     // 0..360 від Півночі (ENU)
    double az_rel_deg;      // 0..360 відносно ground-track перехоплювача
    double elev_deg;        // кут місця
    double rel_height_m;    // Up-компонента (U) у ENU: таргет відносно перехоплювача
    double alt_diff_m;      // простий diff: target.alt - interceptor.alt
} rel_metrics_t;

static inline double _wrap360(double d){ while(d<0)d+=360; while(d>=360)d-=360; return d; }

static inline void _geodetic_to_ecef(double lat_deg, double lon_deg, double h_m,
                                     double *x, double *y, double *z)
{
    const double a=6378137.0;           // WGS-84
    const double f=1.0/298.257223563;
    const double e2=f*(2.0-f);
    const double lat=lat_deg*M_PI/180.0, lon=lon_deg*M_PI/180.0;
    const double sL=sin(lat), cL=cos(lat), sO=sin(lon), cO=cos(lon);
    const double N=a/sqrt(1.0 - e2*sL*sL);
    *x=(N+h_m)*cL*cO;
    *y=(N+h_m)*cL*sO;
    *z=(N*(1.0-e2)+h_m)*sL;
}

static inline void _ecef_to_enu(double dx,double dy,double dz,
                                double lat0_deg,double lon0_deg,
                                double *E,double *N,double *U)
{
    const double lat=lat0_deg*M_PI/180.0, lon=lon0_deg*M_PI/180.0;
    const double sL=sin(lat), cL=cos(lat), sO=sin(lon), cO=cos(lon);
    *E = -sO*dx +  cO*dy;
    *N = -sL*cO*dx - sL*sO*dy + cL*dz;
    *U =  cL*cO*dx +  cL*sO*dy + sL*dz;
}

// Курc руху (ground track) із ENU-швидкості; повертає true, якщо валідний
static inline bool _track_from_ENU(double Vn, double Ve, double *course_rad_out){
    const double Vh = hypot(Ve, Vn);
    if (Vh < 0.3) return false;                // поріг: стоїть/шум
    *course_rad_out = atan2(Ve, Vn);           // 0 = Пн, +CW (як азимут)
    return true;
}

static inline void compute_rel_metrics(const target_estimate_t *icp,
                                       const target_estimate_t *tgt,
                                       rel_metrics_t *out,
                                       double *last_valid_track_rad)
{
    // Вектор лінії візування (LOS)
    double xi,yi,zi, xt,yt,zt;
    _geodetic_to_ecef(icp->position.latitude_deg, icp->position.longitude_deg, icp->position.altitude_msl_m, &xi,&yi,&zi);
    _geodetic_to_ecef(tgt->position.latitude_deg, tgt->position.longitude_deg, tgt->position.altitude_msl_m, &xt,&yt,&zt);
    double E,N,U; _ecef_to_enu(xt-xi, yt-yi, zt-zi,
                               icp->position.latitude_deg, icp->position.longitude_deg,
                               &E,&N,&U);

    // Істинний азимут/кут місця
    double az_true = atan2(E, N); if (az_true < 0) az_true += 2.0*M_PI;
    double elev    = atan2(U, hypot(E,N));

    // Ground-track перехоплювача з його ENU-velocity
    double track_rad;
    bool have_track = _track_from_ENU(icp->velocity.north_ms, icp->velocity.east_ms, &track_rad);
    if (!have_track && last_valid_track_rad) track_rad = *last_valid_track_rad;
    if (have_track && last_valid_track_rad)  *last_valid_track_rad = track_rad;

    // Відносний азимут
    double az_rel = az_true - (have_track || last_valid_track_rad ? track_rad : 0.0);
    while (az_rel < 0) az_rel += 2.0*M_PI;
    while (az_rel >= 2.0*M_PI) az_rel -= 2.0*M_PI;

    out->az_true_deg   = _wrap360(az_true * 180.0/M_PI);
    out->az_rel_deg    = _wrap360(az_rel   * 180.0/M_PI);
    out->elev_deg      =            elev   * 180.0/M_PI;
    out->rel_height_m  = U;
    out->alt_diff_m    = (double)tgt->position.altitude_msl_m - (double)icp->position.altitude_msl_m;
}

typedef struct {
    char *buf;
    size_t cap;
} npb_string_sink_t;

static bool npb_read_string_cb(pb_istream_t *stream, const pb_field_t *field, void **arg) {
    npb_string_sink_t *s = (npb_string_sink_t*)(*arg);
    size_t n = stream->bytes_left < (s->cap - 1) ? stream->bytes_left : (s->cap - 1);
    if (!pb_read(stream, (pb_byte_t*)s->buf, n)) return false;
    s->buf[n] = '\0';
    while (stream->bytes_left) {
        uint8_t tmp[32];
        size_t chunk = stream->bytes_left < sizeof(tmp) ? stream->bytes_left : sizeof(tmp);
        if (!pb_read(stream, tmp, chunk)) return false;
    }
    return true;
}

static inline void map_estimate_np(const ai_skyfortress_guidance_TargetEstimate *in,
                                   target_estimate_t *out,
                                   const char *target_id)
{
    memset(out, 0, sizeof(*out));

    if (in->has_timestamp) {
        out->timestamp_sec = (double)in->timestamp.seconds + (double)in->timestamp.nanos * 1e-9;
        out->precise_timestamp = in->precise_timestamp;
    }
    if (target_id) {
        strncpy(out->target_id, target_id, sizeof(out->target_id) - 1);
    }
    if (in->has_position) {
        out->position.latitude_deg   = in->position.latitude_deg;
        out->position.longitude_deg  = in->position.longitude_deg;
        out->position.altitude_msl_m = in->position.altitude_msl_m;
    }
    if (in->has_velocity) {
        out->velocity.east_ms  = in->velocity.east_ms;
        out->velocity.north_ms = in->velocity.north_ms;
        out->velocity.up_ms    = in->velocity.up_ms;
    }
    out->state = (uint32_t)in->state;
}


static void process_udp_packet(uint8_t *data, size_t length) {
    if (length < TTP_HEADER_SIZE || memcmp(data, TTP_HEADER, TTP_HEADER_SIZE) != 0) {
        ESP_LOGW(TAG_UDP, "Invalid TTP header");
        return;
    }

    pb_istream_t stream = pb_istream_from_buffer(data + TTP_HEADER_SIZE,
                                                 length - TTP_HEADER_SIZE);

    while (stream.bytes_left > 0) {
        ai_skyfortress_guidance_ServerMessage msg = ai_skyfortress_guidance_ServerMessage_init_zero;

        char id_buf[64] = {0};
        npb_string_sink_t sink = { .buf = id_buf, .cap = sizeof(id_buf) };

        msg.message.target_estimate.target_id.funcs.decode = npb_read_string_cb;
        msg.message.target_estimate.target_id.arg = &sink;
        msg.message.interceptor_estimate.target_id.funcs.decode = npb_read_string_cb;
        msg.message.interceptor_estimate.target_id.arg = &sink;

        if (!pb_decode_delimited(&stream, ai_skyfortress_guidance_ServerMessage_fields, &msg)) {
            ESP_LOGE(TAG_UDP, "decode failed: %s", PB_GET_ERROR(&stream));
            break;
        }

        switch (msg.which_message) {
            case ai_skyfortress_guidance_ServerMessage_target_estimate_tag: {
                target_estimate_t est;
                map_estimate_np(&msg.message.target_estimate, &est, id_buf);
                udp_buffer_add(&est, /*is_target=*/true);
                break;
            }
            case ai_skyfortress_guidance_ServerMessage_interceptor_estimate_tag: {
                target_estimate_t est;
                map_estimate_np(&msg.message.interceptor_estimate, &est, id_buf);
                udp_buffer_add(&est, /*is_target=*/false);
                break;
            }
            case ai_skyfortress_guidance_ServerMessage_ping_tag:
                ESP_LOGI(TAG_UDP, "Ping");
                break;
            default:
                break;
        }
    }
}


static void send_client_status(int sock, struct sockaddr_in *client_addr) {
    uint8_t buf[128];
    memcpy(buf, TTP_HEADER, TTP_HEADER_SIZE);

    ai_skyfortress_guidance_ClientMessage cm = ai_skyfortress_guidance_ClientMessage_init_zero;
    cm.which_message = ai_skyfortress_guidance_ClientMessage_status_tag;
    cm.message.status.state = ai_skyfortress_guidance_ClientState_Ready;

    pb_ostream_t os = pb_ostream_from_buffer(buf + TTP_HEADER_SIZE, sizeof(buf) - TTP_HEADER_SIZE);
    if (!pb_encode_delimited(&os, ai_skyfortress_guidance_ClientMessage_fields, &cm)) {
        ESP_LOGE(TAG_UDP, "encode failed: %s", PB_GET_ERROR(&os));
        return;
    }

    size_t total = TTP_HEADER_SIZE + os.bytes_written;
    int err = sendto(sock, buf, total, 0, (struct sockaddr*)client_addr, sizeof(*client_addr));
    if (err < 0) ESP_LOGE(TAG_UDP, "sendto errno %d", errno);
}


static void _task_udp_processor(void *arg) {
    udp_message_t msg;
    uint32_t target_count = 0, interceptor_count = 0;

    static target_estimate_t s_last_target = {};
    static target_estimate_t s_last_icp    = {};
    static bool s_have_target = false, s_have_icp = false;

    static double s_last_track_rad = 0.0;

    while (1) {
        if (xQueueReceive(udp_data_queue, &msg, portMAX_DELAY) == pdTRUE) {
            const char* type = msg.is_target ? "Target" : "Interceptor";

            ESP_LOGI(TAG_UDP, "=== %s Estimate ===", type);
            ESP_LOGI(TAG_UDP, "Target ID: %s", msg.estimate.target_id);
            ESP_LOGI(TAG_UDP, "Timestamp: %.2f sec", msg.estimate.timestamp_sec);
            ESP_LOGI(TAG_UDP, "Precise: %s", msg.estimate.precise_timestamp ? "true" : "false");
            ESP_LOGI(TAG_UDP, "Position: lat=%.6f°, lon=%.6f°, alt=%.1fm",
                     msg.estimate.position.latitude_deg,
                     msg.estimate.position.longitude_deg,
                     msg.estimate.position.altitude_msl_m);
            ESP_LOGI(TAG_UDP, "Velocity: E=%.2fm/s, N=%.2fm/s, U=%.2fm/s",
                     msg.estimate.velocity.east_ms,
                     msg.estimate.velocity.north_ms,
                     msg.estimate.velocity.up_ms);
            ESP_LOGI(TAG_UDP, "State: %s", (msg.estimate.state == 1) ? "Stable" : "Unstable");

            if (msg.is_target) {
                target_count++;
                s_last_target = msg.estimate;
                s_have_target = true;
                ESP_LOGI(TAG_UDP, "Target count: %lu", target_count);
            } else {
                interceptor_count++;
                s_last_icp = msg.estimate;
                s_have_icp = true;
                ESP_LOGI(TAG_UDP, "Interceptor count: %lu", interceptor_count);
            }
            ESP_LOGI(TAG_UDP, "========================");

            if (s_have_icp && s_have_target) {

                rel_metrics_t m; 
                compute_rel_metrics(&s_last_icp, &s_last_target, &m, &s_last_track_rad);

                ESP_LOGI(TAG_UDP,
                         "REL: az_true=%.1f°, az_rel=%.1f° (vs ICP track), elev=%.1f°, "
                         "U=%.1fm, dAlt=%.1fm",
                         m.az_true_deg, m.az_rel_deg, m.elev_deg, m.rel_height_m, m.alt_diff_m);

                if (xSemaphoreTake(g_guid_mtx, pdMS_TO_TICKS(5)) == pdTRUE) {
                    g_guid.az_rel_deg = m.az_rel_deg;
                    g_guid.rel_height_m = m.rel_height_m;
                    g_guid.src_timestamp_s = (double)esp_timer_get_time() * 1e-6;
                    g_guid.valid = true;
                    g_guid.both_stable = (s_last_icp.state == 1 && s_last_target.state == 1);
                    xSemaphoreGive(g_guid_mtx);
                }
            }
        }
    }
}

static void _task_udp_server(void *arg) {
    struct sockaddr_in dest_addr = {0};
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);
    dest_addr.sin_addr.s_addr = INADDR_ANY;
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG_UDP, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_UDP, "Socket created");

    int opt = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG_UDP, "Socket unable to bind: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG_UDP, "Socket bound, listening on port %d", UDP_PORT);
    
    uint8_t rx_buffer[UDP_BUFFER_SIZE];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    
    while (1) {
        int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, 
                          (struct sockaddr *)&source_addr, &socklen);
        
        if (len < 0) {
            ESP_LOGE(TAG_UDP, "recvfrom failed: errno %d", errno);
            continue;
        }
        
        char addr_str[128];
        inet_ntoa_r(source_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG_UDP, "Received %d bytes from %s:%d", len, addr_str, ntohs(source_addr.sin_port));
        
        process_udp_packet(rx_buffer, len);
        
        send_client_status(sock, &source_addr);
    }
    
    close(sock);
    vTaskDelete(NULL);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data) {
    uint64_t now = esp_timer_get_time() / 1000;
    
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI("WIFI", "WiFi started, connecting...");
        esp_wifi_connect();
        s_last_connect_attempt = now;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* disconnected = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGW("WIFI", "Disconnected from WiFi (reason: %d)", disconnected->reason);
        
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            if (now - s_last_connect_attempt > WIFI_RECONNECT_DELAY_MS) {
                esp_wifi_connect();
                s_retry_num++;
                s_last_connect_attempt = now;
                ESP_LOGI("WIFI", "Retry to connect to the AP, attempt %d/%d", s_retry_num, WIFI_MAXIMUM_RETRY);
            } else {
                ESP_LOGI("WIFI", "Waiting before reconnect attempt (reducing phone stress)");
            }
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGE("WIFI", "Failed to connect to WiFi after %d attempts", WIFI_MAXIMUM_RETRY);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI("WIFI", "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void) {
    ESP_LOGI("WIFI", "Initializing WiFi for phone hotspot...");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    esp_netif_t* sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.ampdu_rx_enable = 0;
    cfg.ampdu_tx_enable = 0;
    cfg.amsdu_tx_enable = 0;
    cfg.nvs_enable = 1;
    
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.sta.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.sta.password, WIFI_PASS);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    
    wifi_config.sta.scan_method = WIFI_FAST_SCAN;
    wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifi_config.sta.threshold.rssi = -80;
    
    ESP_LOGI("WIFI", "Connecting to phone hotspot: %s", WIFI_SSID);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));
    
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(60000)); // 60 second timeout

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI("WIFI", "Connected to phone hotspot successfully!");
        
        esp_netif_ip_info_t ip_info;
        ESP_ERROR_CHECK(esp_netif_get_ip_info(sta_netif, &ip_info));
        ESP_LOGI("WIFI", "IP Address: " IPSTR, IP2STR(&ip_info.ip));
        ESP_LOGI("WIFI", "Gateway: " IPSTR, IP2STR(&ip_info.gw));
        ESP_LOGI("WIFI", "Netmask: " IPSTR, IP2STR(&ip_info.netmask));
        
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE("WIFI", "Failed to connect to phone hotspot");
    } else {
        ESP_LOGE("WIFI", "Connection timeout");
    }
}


static void _task_wifi(void *arg) {
    wifi_init_sta();
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(WIFI_MONITOR_INTERVAL_MS));
        
        wifi_ap_record_t ap_info;
        esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
        
        if (ret == ESP_OK) {
            if (ap_info.rssi < -70) {
                ESP_LOGW("WIFI", "Weak signal - RSSI: %d dBm", ap_info.rssi);
            }
        } else {
            ESP_LOGW("WIFI", "WiFi connection lost, will auto-reconnect");
        }
    }
}




static void _task_controller(void *arg) {
    _setup_uart(UART_PORT_CONTROLLER, UART_PIN_CONTROLLER, &queue_uart_controller);
    _half_duplex_set_rx(UART_PORT_CONTROLLER, UART_PIN_CONTROLLER);

    static uint32_t frames_rcvd = 0, frames_sent = 0;

    crsf_frame_t frame;

    while (true) {
        bool is_frame_ready = _uart_read_crsf_frame(TAG_CONTROLLER, UART_PORT_CONTROLLER, queue_uart_controller, &frame, portMAX_DELAY);

        if (is_frame_ready) {
            frames_rcvd += 1;

            _swap_crsf_frame(&frame);

            assert(xQueueSend(queue_crsf_controller, &frame, 0) == pdPASS);

            if (xQueueReceive(queue_crsf_module, &frame, 0) == pdPASS) {
                frames_sent += 1;

                _half_duplex_set_tx(UART_PORT_CONTROLLER, UART_PIN_CONTROLLER);
                assert(uart_write_bytes(UART_PORT_CONTROLLER, &frame, frame.length + 2) == frame.length + 2);
                ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT_CONTROLLER, portMAX_DELAY));
                _half_duplex_set_rx(UART_PORT_CONTROLLER, UART_PIN_CONTROLLER);
            }

            if (frames_rcvd % CRSF_RATE_HZ == 0) {
                //ESP_LOGI(TAG_CONTROLLER, "rcvd: %lu, sent: %lu, current frame sync=0x%x, type=0x%x, length=%u", frames_rcvd, frames_sent, frame.sync, frame.type, frame.length);
            }
        }
    }
}

static void _task_module(void *arg) {
    _setup_uart(UART_PORT_MODULE, UART_PIN_MODULE, &queue_uart_module);

    _half_duplex_set_rx(UART_PORT_MODULE, UART_PIN_MODULE);

    crsf_frame_t frame;

    uint32_t frames_sent = 0, frames_rcvd = 0;

    while (true) {
        if (xQueueReceive(queue_crsf_controller, &frame, portMAX_DELAY) == pdPASS) {
            frames_rcvd += 1;
            _half_duplex_set_tx(UART_PORT_MODULE, UART_PIN_MODULE);

            assert(uart_write_bytes(UART_PORT_MODULE, &frame, frame.length + 2) == frame.length + 2);
            ESP_ERROR_CHECK(uart_wait_tx_done(UART_PORT_MODULE, portMAX_DELAY));

            _half_duplex_set_rx(UART_PORT_MODULE, UART_PIN_MODULE);

            bool is_frame_ready = _uart_read_crsf_frame(TAG_MODULE, UART_PORT_MODULE, queue_uart_module, &frame, pdMS_TO_TICKS(1000/CRSF_RATE_HZ));

            if (is_frame_ready) {
                frames_sent += 1;
                assert(xQueueSend(queue_crsf_module, &frame, 0) == pdPASS);
            }
        }
    }
}

extern "C" void app_main(void) {
    ESP_LOGI("MAIN", "Starting application...");
    
    udp_buffer_mutex = xSemaphoreCreateMutex();
    udp_data_queue = xQueueCreate(20, sizeof(udp_message_t));

    g_guid_mtx = xSemaphoreCreateMutex();
    memset(&g_guid, 0, sizeof(g_guid));
     
    queue_crsf_controller = xQueueCreate(10, sizeof(crsf_frame_t));
    queue_crsf_module = xQueueCreate(10, sizeof(crsf_frame_t));

    assert(xTaskCreatePinnedToCore(_task_wifi, "wifi", 6144, NULL, 
           configMAX_PRIORITIES - 4, NULL, APP_CPU_NUM) == pdPASS);

    ESP_LOGI("MAIN", "Waiting for WiFi connection...");
    vTaskDelay(pdMS_TO_TICKS(8000));
    
    ESP_LOGI("MAIN", "Starting UDP tasks...");
    assert(xTaskCreatePinnedToCore(_task_udp_processor, "udp_proc", 4096, NULL, 
           configMAX_PRIORITIES - 3, NULL, APP_CPU_NUM) == pdPASS);
    
    assert(xTaskCreatePinnedToCore(_task_udp_server, "udp_server", 8192, NULL, 
           configMAX_PRIORITIES - 3, NULL, APP_CPU_NUM) == pdPASS);

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI("MAIN", "Starting CRSF tasks...");
    assert(xTaskCreatePinnedToCore(_task_module, "module", 4096, NULL, 
           configMAX_PRIORITIES - 2, &task_module, APP_CPU_NUM) == pdPASS);
    vTaskDelay(pdMS_TO_TICKS(100));
    assert(xTaskCreatePinnedToCore(_task_controller, "controller", 4096, NULL, 
           configMAX_PRIORITIES - 2, &task_controller, PRO_CPU_NUM) == pdPASS);
    
    ESP_LOGI("MAIN", "All tasks started successfully");
    ESP_LOGI("MAIN", "UDP server listening on port 8888");
    ESP_LOGI("MAIN", "Ready to receive target/interceptor data");
}

#endif