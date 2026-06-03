#ifndef SOCKET_SERVER_H
#define SOCKET_SERVER_H

#include <stddef.h>

/* ─── Sabitler ──────────────────────────────────────────────────────────────*/
#define COMMS_SOCKET_PATH   "/tmp/sonar.sock"
#define COMMS_BUFFER_BOYUT  4096
#define COMMS_MAX_CLIENTS   1

/* ─── Hata Kodları ──────────────────────────────────────────────────────────*/
#define COMMS_OK             0
#define COMMS_ERR_INIT      -1
#define COMMS_ERR_SOCKET    -2
#define COMMS_ERR_BIND      -3
#define COMMS_ERR_LISTEN    -4
#define COMMS_ERR_THREAD    -5
#define COMMS_ERR_NULL      -6
#define COMMS_ERR_BUFFER    -7
#define COMMS_ERR_NO_CLIENT -8

/* ─── Log Seviyeleri ────────────────────────────────────────────────────────*/
typedef enum {
    COMMS_LOG_DEBUG = 0,
    COMMS_LOG_INFO,
    COMMS_LOG_WARN,
    COMMS_LOG_ERROR
} CommsLogLevel;

/*
 * Komut handler tipi.
 *
 * cmd            : komut adı (ör. "steer", "freq")
 * params_json    : gelen tüm JSON satırı (ör. {"cmd":"steer","az":15.0,...})
 * response_buffer: handler yanıt JSON gövdesini buraya yazar
 *                  (ör. {"status":"ok"}) — comms bu gövdeyi tam yanıta sarar
 * buffer_size    : response_buffer kapasitesi (bayt)
 * Dönüş          : 0 başarı, negatif hata
 *
 * Örnek kullanım:
 *
 *   int my_handler(const char *cmd, const char *params,
 *                  char *resp, size_t sz) {
 *       if (strcmp(cmd, "steer") == 0) {
 *           snprintf(resp, sz, "{\"status\":\"ok\"}");
 *           return 0;
 *       }
 *       snprintf(resp, sz, "{\"status\":\"error\",\"message\":\"bilinmeyen komut\"}");
 *       return -1;
 *   }
 *
 *   int main(void) {
 *       comms_init(NULL);
 *       comms_register_handler(my_handler);
 *
 *       comms_emit_echo(1.45f, 8500.0f, 15.0f, 20.0f);
 *
 *       comms_cleanup();
 *       return 0;
 *   }
 */
typedef int (*comms_cmd_handler_t)(const char *cmd,
                                    const char *params_json,
                                    char       *response_buffer,
                                    size_t      buffer_size);

/* ─── Başlatma / Temizleme ──────────────────────────────────────────────────*/
int  comms_init(const char *socket_path);   /* NULL → COMMS_SOCKET_PATH     */
void comms_cleanup(void);

/* ─── Handler Kaydı ─────────────────────────────────────────────────────────*/
int  comms_register_handler(comms_cmd_handler_t handler);

/* ─── Event Yayınlama ───────────────────────────────────────────────────────*/
int  comms_emit_event(const char *event_name, const char *params_json);

/* ─── Kısayol Event Fonksiyonları ───────────────────────────────────────────*/
int  comms_emit_echo(float dist, float delay_us, float az, float el);
/* Aynı (az, el) konumundaki birden fazla echo'yu tek event olarak yayınlar.
 * dist[i]/delay_us[i] paralel diziler, n eleman sayısı. n<=0 ise no-op. */
int  comms_emit_echo_batch(const float *dist, const float *delay_us, int n,
                           float az, float el);
int  comms_emit_doppler(float freq, float velocity);
int  comms_emit_scan_progress(int step, int total, float az, float el);
int  comms_emit_temperature(float celsius, float sound_speed);
int  comms_emit_error(const char *module, const char *message);

/* ─── Bağlantı Durumu ───────────────────────────────────────────────────────*/
int  comms_is_client_connected(void);
int  comms_get_connected_count(void);

/* ─── Log ───────────────────────────────────────────────────────────────────*/
void comms_set_log_level(CommsLogLevel level);

#endif /* SOCKET_SERVER_H */
