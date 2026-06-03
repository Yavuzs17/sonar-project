/*
 * Derleme:
 *   gcc -o sonar main.c \
 *       pll/pll.c preamp/preamp.c tvg/tvg.c \
 *       phase_shift/phase_shift.c beamforming/beamforming.c \
 *       adc/adc.c sensor/sensor.c \
 *       comms/socket_server.c \
 *       -llgpio -lpthread -lm -lrt -lcjson -Wall -Wextra
 *
 * Çalıştırma:
 *   ./sonar                    Tam donanımla
 *   ./sonar --no-hardware      Mock modunda
 *   ./sonar --no-calib         Kalibrasyon yüklemeden
 *   ./sonar --help             Yardım
 */

#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "pll/pll.h"
#include "preamp/preamp.h"
#include "tvg/tvg.h"
#include "phase_shift/phase_shift.h"
#include "beamforming/beamforming.h"
#include "adc/adc.h"
#include "sensor/sensor.h"
#include "comms/socket_server.h"
#include <cjson/cJSON.h>

/* ─── Sabitler ───────────────────────────────────────────────────────────────*/
#define FREKANS_MIN_HZ            38000
#define FREKANS_MAX_HZ            42000
#define AZ_MIN_DEG               -30.0f
#define AZ_MAX_DEG                30.0f
#define EL_MIN_DEG               -30.0f
#define EL_MAX_DEG                30.0f
#define AZ_ADIM_DEG               5.0f
#define EL_ADIM_DEG               5.0f
#define TARAMA_TOPLAM_ADIM        169       /* 13 × 13                        */
#define DWELL_MS_VARSAYILAN       50
#define CALIB_DIZIN_VARSAYILAN    "./calib"
#define KOMUT_TAMPONU_BOY         256
#define CALIB_YOL_BOY             640

/* ─── Çıktı Modu ─────────────────────────────────────────────────────────────*/
typedef enum {
    OUTPUT_LOG  = 1,   /* yalnızca stderr log   */
    OUTPUT_JSON = 2,   /* yalnızca stdout JSON  */
    OUTPUT_BOTH = 3    /* ikisi birden          */
} OutputMode;

/* ─── Global Durum ───────────────────────────────────────────────────────────*/
static volatile sig_atomic_t shutdown_requested = 0;
static volatile bool         scan_aktif         = false;
static pthread_t             scan_thread;
static int                   current_freq_hz    = 40000;
static int                   dwell_default_ms   = DWELL_MS_VARSAYILAN;
static OutputMode            output_mode        = OUTPUT_BOTH;
static bool                  no_hardware        = false;
static bool                  no_calib           = false;
static char                  calib_dizin[512]   = CALIB_DIZIN_VARSAYILAN;
static float                 current_az         = 0.0f;
static float                 current_el         = 0.0f;

/* Başlatılan modülleri izle — cleanup sırasında yalnızca açılanlar kapatılır */
static bool init_adc        = false;
static bool init_sensor     = false;
static bool init_preamp     = false;
static bool init_tvg        = false;
static bool init_ps         = false;
static bool init_bf         = false;
static bool init_pll        = false;

/* ─── Sinyal Yakalayıcı ──────────────────────────────────────────────────────*/
static void sigint_handler(int sig) {
    (void)sig;
    shutdown_requested = 1;
}

/* ─── Çıktı Yardımcıları ─────────────────────────────────────────────────────*/

/* İnsan okunabilir log satırı — stderr */
static void log_msg(const char *seviye, const char *fmt, ...) {
    if (!(output_mode & OUTPUT_LOG)) return;
    fprintf(stderr, "[%s] ", seviye);
    va_list args;
    va_start(args, fmt);
    vfprintf(stderr, fmt, args);
    va_end(args);
    fputc('\n', stderr);
}

/* JSON event satırı — stdout */
static void emit_event(const char *payload) {
    if (!(output_mode & OUTPUT_JSON)) return;
    printf("%s\n", payload);
    fflush(stdout);
}

static void emit_echo(float dist, float delay_us, float az, float el) {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"event\":\"echo\",\"dist\":%.3f,\"delay_us\":%.1f,"
             "\"az\":%.1f,\"el\":%.1f}",
             dist, delay_us, az, el);
    emit_event(buf);
    comms_emit_echo(dist, delay_us, az, el);
    log_msg("INFO", "Echo: %.3f m, gecikme %.1f us, az=%.1f el=%.1f",
            dist, delay_us, az, el);
}

static void emit_doppler(float freq, float velocity) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"event\":\"doppler\",\"freq\":%.1f,\"velocity\":%.4f}",
             freq, velocity);
    emit_event(buf);
    comms_emit_doppler(freq, velocity);
    log_msg("INFO", "Doppler: frekans=%.1f Hz, hiz=%.4f m/s", freq, velocity);
}

static void emit_temperature(float celsius, float sound_speed) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"event\":\"temperature\",\"celsius\":%.2f,\"sound_speed\":%.2f}",
             celsius, sound_speed);
    emit_event(buf);
    comms_emit_temperature(celsius, sound_speed);
    log_msg("INFO", "Sicaklik: %.2f C, ses hizi: %.2f m/s", celsius, sound_speed);
}

static void emit_scan_progress(int step, int total, float az, float el) {
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"event\":\"scan_progress\",\"step\":%d,\"total\":%d,"
             "\"az\":%.1f,\"el\":%.1f}",
             step, total, az, el);
    emit_event(buf);
    comms_emit_scan_progress(step, total, az, el);
}

static void emit_error(const char *module, const char *msg) {
    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"event\":\"error\",\"module\":\"%s\",\"message\":\"%s\"}",
             module, msg);
    emit_event(buf);
    comms_emit_error(module, msg);
    log_msg("ERROR", "[%s] %s", module, msg);
}

/* ─── Burst Callback Wrapper ─────────────────────────────────────────────────
 * PLL tek callback desteklediğinden TVG ve Sensor aynı fonksiyondan çağrılır.
 * on_burst_start: PLL burst thread'inde INHIBIT deassert edilince tetiklenir.
 * Timestamp burada alınır → sensor echo zamanlaması maksimum doğrulukta.   */
static void on_burst_start(void) {
    tvg_on_burst_start();
    sensor_burst_baslat();
}

static void on_listen_start(void) {
    tvg_on_listen_start();
    sensor_burst_sonlandir();
}

/* ─── Kalibrasyon Yükleme ────────────────────────────────────────────────────
 * Dosya yoksa uyarı verir, varsayılan değerlerle devam edilir.             */
static void calib_yukle(void) {
    char path[CALIB_YOL_BOY];

    snprintf(path, sizeof(path), "%s/preamp_lut.csv", calib_dizin);
    if (preamp_lut_yukle_dosya(path) != 0)
        log_msg("WARN", "Preamp LUT yuklenemedi: %s", path);

    snprintf(path, sizeof(path), "%s/phase_shift_lut.csv", calib_dizin);
    if (ps_kalib_yukle_dosya(path) != 0)
        log_msg("WARN", "Phase shift LUT yuklenemedi: %s", path);

    snprintf(path, sizeof(path), "%s/tvg_lut.csv", calib_dizin);
    if (tvg_lut_yukle_dosya(path) != 0)
        log_msg("WARN", "TVG LUT yuklenemedi: %s", path);

    snprintf(path, sizeof(path), "%s/doppler_lut.csv", calib_dizin);
    if (sensor_doppler_lut_yukle_dosya(path) != 0)
        log_msg("WARN", "Doppler LUT yuklenemedi: %s", path);

    snprintf(path, sizeof(path), "%s/pll_lut.csv", calib_dizin);
    if (pll_lut_yukle_dosya(path) != 0)
        log_msg("WARN", "PLL LUT yuklenemedi: %s", path);
}

/* ─── Cleanup — Ters Sırayla ────────────────────────────────────────────────*/
static void cleanup_all(void) {
    comms_cleanup();
    log_msg("INFO", "Cleanup yapiliyor...");

    /* Tarama thread'ini durdur */
    if (scan_aktif) {
        scan_aktif = false;
        pthread_join(scan_thread, NULL);
    }

    if (init_pll)    { pll_cleanup();    init_pll    = false; }
    if (init_bf)     { bf_cleanup();     init_bf     = false; }
    if (init_ps)     { ps_cleanup();     init_ps     = false; }
    if (init_tvg)    { tvg_cleanup();    init_tvg    = false; }
    if (init_preamp) { preamp_cleanup(); init_preamp = false; }
    if (init_sensor) { sensor_cleanup(); init_sensor = false; }
    if (init_adc)    { adc_cleanup();    init_adc    = false; }

    log_msg("INFO", "Cleanup tamamlandi");
}

/* ─── Modül Başlatma ─────────────────────────────────────────────────────────*/

/* no_hardware aktifse init hatalarını görmezden gelir. */
static bool init_kontrol(int ret, const char *modul) {
    if (ret == 0) return true;
    if (no_hardware) {
        log_msg("WARN", "%s init hatasi=%d (mock mod, devam ediliyor)", modul, ret);
        return true;
    }
    emit_error(modul, "baslatilamadi");
    return false;
}

static bool baslat_moduller(void) {
    /* 1. ADC — Sensor için gerekli */
    if (!init_kontrol(adc_init(), "adc")) return false;
    init_adc = true;

    /* 2. Sensor — GPIO 16 echo zamanlama, ADC kullanır */
    if (!init_kontrol(sensor_init(), "sensor")) return false;
    init_sensor = true;

    /* 3. Preamp — I2C bus 1 */
    if (!init_kontrol(preamp_init(), "preamp")) return false;
    init_preamp = true;

    /* 4. TVG — I2C bus 1 */
    if (!init_kontrol(tvg_init(), "tvg")) return false;
    init_tvg = true;

    /* 5. Phase shift — I2C bus 1 */
    if (!init_kontrol(ps_init(), "phase_shift")) return false;
    init_ps = true;

    /* 6. Beamforming — phase_shift kullanır */
    if (!init_kontrol(bf_init(), "beamforming")) return false;
    init_bf = true;

    /* 7. PLL — I2C bus 3; callback'ler son olarak bağlanır */
    if (!init_kontrol(pll_init(), "pll")) return false;
    init_pll = true;

    pll_set_burst_start_callback(on_burst_start);
    pll_set_listen_start_callback(on_listen_start);

    return true;
}

/* ─── Echo batch helper ─────────────────────────────────────────────────────
 * Aynı (az, el) konumunda toplanmış birden fazla echo'yu tek pakette gönderir.
 * Hem stdout JSON (varsa) hem socket (comms_emit_echo_batch) yolu çağrılır. */
static void emit_echo_batch(const float *dists, const float *delays, int n,
                            float az, float el) {
    if (n <= 0) return;

    /* stdout JSON (legacy mode) */
    if (output_mode & OUTPUT_JSON) {
        char buf[1024];
        int off = snprintf(buf, sizeof(buf),
            "{\"event\":\"echo_batch\",\"az\":%.1f,\"el\":%.1f,\"echoes\":[",
            az, el);
        for (int i = 0; i < n && off < (int)sizeof(buf) - 64; i++) {
            off += snprintf(buf + off, sizeof(buf) - (size_t)off,
                "%s{\"dist\":%.3f,\"delay_us\":%.1f}",
                i ? "," : "", dists[i], delays[i]);
        }
        if (off < (int)sizeof(buf) - 2) {
            snprintf(buf + off, sizeof(buf) - (size_t)off, "]}");
            emit_event(buf);
        }
    }

    /* Socket clients */
    comms_emit_echo_batch(dists, delays, n, az, el);

    /* Tek satır özet log — n echo için 1 syscall */
    log_msg("INFO", "Echo batch: n=%d, az=%.1f el=%.1f", n, az, el);
}

/* ─── Simülasyon: Sahte Echo Üretici ────────────────────────────────────────
 * --no-hardware modunda gerçek sensor yerine matematiksel hedefler üretir.
 * Üç sabit hedef + rastgele gürültü echo'ları üretir; tek batch event olarak
 * yayınlar (önceden her echo ayrı paket gönderiyordu).                     */
static void simulate_fake_echoes(float az_deg, float el_deg) {
    static int tick = 0;
    tick++;

    float dists[8], delays[8];
    int n = 0;

    /* Hedef 1: Dalgalı duvar — tüm yönlerden görünür */
    {
        float wave   = 0.5f * sinf(az_deg * 0.05f + tick * 0.1f);
        float mesafe = 2.5f + wave;
        dists[n]  = mesafe;
        delays[n] = (mesafe * 2.0f / 343.0f) * 1e6f;
        n++;
    }

    /* Hedef 2: Sol-yan sabit hedef (az ≈ -20°, el ≈ 0°) */
    if (fabsf(az_deg - (-20.0f)) < 7.5f && fabsf(el_deg) < 7.5f) {
        float mesafe = 1.5f;
        dists[n]  = mesafe;
        delays[n] = (mesafe * 2.0f / 343.0f) * 1e6f;
        n++;
    }

    /* Hedef 3: Sağ-üst hareketli hedef (az > 10°, el > 10°) */
    if (az_deg > 10.0f && el_deg > 10.0f) {
        float oscillation = 0.3f * sinf(tick * 0.2f);
        float mesafe      = 3.5f + oscillation;
        dists[n]  = mesafe;
        delays[n] = (mesafe * 2.0f / 343.0f) * 1e6f;
        n++;
    }

    /* Rastgele gürültü echo'su — %10 ihtimal */
    if ((rand() % 10) == 0) {
        float mesafe = 1.0f + (rand() % 400) / 100.0f;   /* 1–5 m */
        dists[n]  = mesafe;
        delays[n] = (mesafe * 2.0f / 343.0f) * 1e6f;
        n++;
    }

    emit_echo_batch(dists, delays, n, az_deg, el_deg);
}

/* ─── Otomatik Tarama Thread'i ───────────────────────────────────────────────
 * Zigzag (bidirectional) tarama: her pass'in başında sweep_complete event'i
 * yayınlanır, azimuth yönü her pass'te tersine döner.
 * bf_get_scan_range her pass başında okunur → scan_range komutu anında etkin. */
static void *scan_thread_func(void *arg) {
    int dwell_ms = arg ? *(int *)arg : DWELL_MS_VARSAYILAN;
    if (arg) free(arg);

    log_msg("INFO", "Tarama basladi (dwell=%d ms, mod=%s)",
            dwell_ms, no_hardware ? "simulasyon" : "donanim");

    static int g_sweep_direction = 1;   /* 1: ileri (-az→+az), -1: geri (+az→-az) */
    static int g_sweep_count     = 0;
    static int cycle              = 0;

    while (scan_aktif) {
        /* Her pass başında güncel tarama aralığını al */
        double az_max_d, el_max_d, step_d;
        bf_get_scan_range(&az_max_d, &el_max_d, &step_d);
        float az_max = (float)az_max_d;
        float el_max = (float)el_max_d;
        float step   = (float)step_d;

        int az_cnt = (int)round(2.0 * az_max_d / step_d) + 1;
        int el_cnt = (int)round(2.0 * el_max_d / step_d) + 1;
        int toplam  = az_cnt * el_cnt;
        int adim    = 0;

        if ((cycle++ % 10) == 0)
            fprintf(stderr, "[scan] cycle %d: +-%.0f/+-%.0f, step %.0f deg (%d adim) dir=%s\n",
                    cycle, az_max_d, el_max_d, step_d, toplam,
                    g_sweep_direction > 0 ? "fwd" : "bwd");

        /* Pass başında sweep_complete event'ini tüm client'lara yayınla */
        g_sweep_count++;
        {
            char sc_fields[128];
            snprintf(sc_fields, sizeof(sc_fields),
                     "{\"direction\":\"%s\",\"count\":%d}",
                     g_sweep_direction > 0 ? "forward" : "backward",
                     g_sweep_count);
            comms_emit_event("sweep_complete", sc_fields);

            if (output_mode & OUTPUT_JSON) {
                printf("{\"event\":\"sweep_complete\","
                       "\"direction\":\"%s\",\"count\":%d}\n",
                       g_sweep_direction > 0 ? "forward" : "backward",
                       g_sweep_count);
                fflush(stdout);
            }
            log_msg("INFO", "Sweep #%d basladi (%s)",
                    g_sweep_count,
                    g_sweep_direction > 0 ? "forward" : "backward");
        }

        /* Azimuth başlangıç/bitiş/adım yönünü hesapla */
        float az_start = (g_sweep_direction > 0) ? -az_max : +az_max;
        float az_end   = (g_sweep_direction > 0) ? +az_max : -az_max;
        float az_step  = (g_sweep_direction > 0) ?  step   : -step;

        for (float az = az_start;
             (g_sweep_direction > 0) ? az <= az_end + 1e-6f
                                     : az >= az_end - 1e-6f;
             az += az_step) {

            if (!scan_aktif) break;

            for (float el = -el_max; el <= el_max + 1e-6f && scan_aktif; el += step) {

                /* Beamforming yönünü ayarla */
                bf_set_direction(az, el, (float)current_freq_hz);

                /* 60 pals burst + dinleme süresi bekle */
                pll_burst_start(60, 30);
                usleep((useconds_t)(dwell_ms * 1000));
                pll_burst_stop();

                /* Echo verisi: simülasyon veya gerçek donanım */
                if (no_hardware) {
                    simulate_fake_echoes(az, el);
                } else {
                    EchoOlay echo;
                    if (sensor_ilk_echo_oku(&echo) == SENSOR_OK)
                        emit_echo(echo.mesafe_m, echo.gecikme_us, az, el);
                }

                adim++;
                emit_scan_progress(adim, toplam, az, el);
            }
        }

        /* Sonraki pass için yönü ters çevir (zigzag) */
        g_sweep_direction = -g_sweep_direction;
    }

    log_msg("INFO", "Tarama thread sonlandi");
    return NULL;
}

/* ─── Durum Çıktısı ──────────────────────────────────────────────────────────*/
static void cmd_status(void) {
    float t = sensor_get_sicaklik_c();
    float c = sensor_get_ses_hizi();
    float f = sensor_get_doppler_frekans();
    float v = sensor_get_doppler_hiz(0);

    emit_temperature(t, c);
    emit_doppler(f, v);

    char buf[256];
    snprintf(buf, sizeof(buf),
             "{\"event\":\"status\",\"az\":%.1f,\"el\":%.1f,"
             "\"freq_hz\":%d,\"scan\":%s}",
             current_az, current_el, current_freq_hz,
             scan_aktif ? "true" : "false");
    emit_event(buf);
    log_msg("INFO", "Yon: az=%.1f el=%.1f | frekans=%d Hz | tarama=%s",
            current_az, current_el, current_freq_hz,
            scan_aktif ? "aktif" : "pasif");
}

/* ─── Yardım Mesajı ──────────────────────────────────────────────────────────*/
static void print_help(void) {
    fprintf(stderr,
        "Komutlar:\n"
        "  help                     Bu mesaji goster\n"
        "  status                   Sistem durumu (yön, frekans, sıcaklık)\n"
        "  quit / exit              Programdan cik\n"
        "\n"
        "  steer <az> <el>          Manuel yon (az/el: -30..+30 derece)\n"
        "  freq <hz>                PLL frekans (%d-%d Hz)\n"
        "  gain <kanal> <db>        Alici kazanci (kanal 0-6)\n"
        "  gain_all <db>            Tum alicilara ayni kazanc\n"
        "\n"
        "  burst_start              Manuel burst basla\n"
        "  burst_stop               Burst durdur\n"
        "\n"
        "  scan_start [dwell_ms]    Otomatik tarama (varsayilan: %d ms)\n"
        "  scan_stop                Taramayi durdur\n"
        "  scan_range <az> <el> <s> Tarama araligi (+-az +-el, s adim, derece)\n"
        "\n"
        "  read_temp                Sicaklik ve ses hizi\n"
        "  read_echo                Son echo bilgisi\n"
        "  read_doppler             Doppler frekans ve hiz\n"
        "\n"
        "  mode <log|json|both>     Cikti modunu degistir\n"
        "  log_level <0-3>          Log seviyesi (0=debug, 3=error)\n",
        FREKANS_MIN_HZ, FREKANS_MAX_HZ, dwell_default_ms);
}

/* ─── Socket Komut Handler ───────────────────────────────────────────────────
 * comms modülü tarafından çağrılır. JSON parse ederek aynı modül API'sini
 * kullanır ve JSON yanıt yazar. comms_cmd_handler_t imzasına uyar.        */
static int socket_komut_handler(const char *cmd, const char *params_json,
                                 char *response_buffer, size_t buffer_size) {
    cJSON *params = NULL;
    if (params_json && strlen(params_json) > 0)
        params = cJSON_Parse(params_json);

    int sonuc = 0;

    if (strcmp(cmd, "steer") == 0) {
        cJSON *az_obj = cJSON_GetObjectItem(params, "az");
        cJSON *el_obj = cJSON_GetObjectItem(params, "el");
        if (!az_obj || !el_obj) {
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"error\",\"message\":\"az ve el parametreleri gerekli\"}");
            sonuc = -1;
        } else {
            float az = (float)cJSON_GetNumberValue(az_obj);
            float el = (float)cJSON_GetNumberValue(el_obj);
            int ret = bf_set_direction(az, el, (float)current_freq_hz);
            if (ret == 0) {
                current_az = az;
                current_el = el;
                snprintf(response_buffer, buffer_size,
                         "{\"status\":\"ok\",\"az\":%.2f,\"el\":%.2f}", az, el);
            } else {
                snprintf(response_buffer, buffer_size,
                         "{\"status\":\"error\",\"message\":\"bf_set_direction hata: %d\"}",
                         ret);
                sonuc = -1;
            }
        }

    } else if (strcmp(cmd, "freq") == 0) {
        cJSON *hz_obj = cJSON_GetObjectItem(params, "hz");
        if (!hz_obj) {
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"error\",\"message\":\"hz parametresi gerekli\"}");
            sonuc = -1;
        } else {
            int hz = (int)cJSON_GetNumberValue(hz_obj);
            current_freq_hz = hz;
            pll_set_target_frequency(hz);
            sensor_set_burst_frekans((float)hz);
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"ok\",\"hz\":%d}", hz);
        }

    } else if (strcmp(cmd, "gain") == 0) {
        cJSON *kanal_obj = cJSON_GetObjectItem(params, "kanal");
        cJSON *db_obj    = cJSON_GetObjectItem(params, "db");
        if (!kanal_obj || !db_obj) {
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"error\",\"message\":\"kanal ve db parametreleri gerekli\"}");
            sonuc = -1;
        } else {
            int   kanal = (int)cJSON_GetNumberValue(kanal_obj);
            float db    = (float)cJSON_GetNumberValue(db_obj);
            int ret = preamp_set_gain_db(kanal, db);
            if (ret == 0) {
                snprintf(response_buffer, buffer_size,
                         "{\"status\":\"ok\",\"kanal\":%d,\"db\":%.2f}", kanal, db);
            } else {
                snprintf(response_buffer, buffer_size,
                         "{\"status\":\"error\",\"message\":\"preamp_set_gain_db hata: %d\"}",
                         ret);
                sonuc = -1;
            }
        }

    } else if (strcmp(cmd, "gain_all") == 0) {
        cJSON *db_obj = cJSON_GetObjectItem(params, "db");
        if (!db_obj) {
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"error\",\"message\":\"db parametresi gerekli\"}");
            sonuc = -1;
        } else {
            float db = (float)cJSON_GetNumberValue(db_obj);
            preamp_set_all_gain_db(db);
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"ok\",\"db\":%.2f}", db);
        }

    } else if (strcmp(cmd, "burst_start") == 0) {
        /* on_burst_start callback'i sensor ve TVG'yi bilgilendirir */
        pll_burst_start(60, 30);
        snprintf(response_buffer, buffer_size, "{\"status\":\"ok\"}");

    } else if (strcmp(cmd, "burst_stop") == 0) {
        pll_burst_stop();
        snprintf(response_buffer, buffer_size, "{\"status\":\"ok\"}");

    } else if (strcmp(cmd, "scan_start") == 0) {
        int dwell_ms = dwell_default_ms;
        cJSON *dwell_obj = cJSON_GetObjectItem(params, "dwell_ms");
        if (dwell_obj) dwell_ms = (int)cJSON_GetNumberValue(dwell_obj);
        if (dwell_ms <= 0) dwell_ms = dwell_default_ms;

        if (scan_aktif) {
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"error\",\"message\":\"tarama zaten aktif\"}");
            sonuc = -1;
        } else {
            int *arg = malloc(sizeof(int));
            if (!arg) {
                snprintf(response_buffer, buffer_size,
                         "{\"status\":\"error\",\"message\":\"bellek hatasi\"}");
                sonuc = -1;
            } else {
                *arg = dwell_ms;
                scan_aktif = true;
                if (pthread_create(&scan_thread, NULL, scan_thread_func, arg) != 0) {
                    scan_aktif = false;
                    free(arg);
                    snprintf(response_buffer, buffer_size,
                             "{\"status\":\"error\",\"message\":\"thread olusturulamadi\"}");
                    sonuc = -1;
                } else {
                    snprintf(response_buffer, buffer_size,
                             "{\"status\":\"ok\",\"dwell_ms\":%d}", dwell_ms);
                }
            }
        }

    } else if (strcmp(cmd, "scan_stop") == 0) {
        if (!scan_aktif) {
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"error\",\"message\":\"tarama zaten durmus\"}");
            sonuc = -1;
        } else {
            scan_aktif = false;
            pthread_join(scan_thread, NULL);
            snprintf(response_buffer, buffer_size, "{\"status\":\"ok\"}");
        }

    } else if (strcmp(cmd, "read_temp") == 0) {
        float t = sensor_get_sicaklik_c();
        float c = sensor_get_ses_hizi();
        snprintf(response_buffer, buffer_size,
                 "{\"status\":\"ok\",\"celsius\":%.2f,\"sound_speed\":%.2f}", t, c);

    } else if (strcmp(cmd, "read_echo") == 0) {
        EchoOlay echo;
        if (sensor_ilk_echo_oku(&echo) == SENSOR_OK) {
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"ok\",\"dist_m\":%.3f,\"delay_us\":%.1f}",
                     echo.mesafe_m, echo.gecikme_us);
        } else {
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"error\",\"message\":\"echo yakalanmadi\"}");
            sonuc = -1;
        }

    } else if (strcmp(cmd, "read_doppler") == 0) {
        float freq = sensor_get_doppler_frekans();
        float v    = sensor_get_doppler_hiz(0);
        snprintf(response_buffer, buffer_size,
                 "{\"status\":\"ok\",\"freq\":%.1f,\"velocity\":%.3f}", freq, v);

    } else if (strcmp(cmd, "status") == 0) {
        snprintf(response_buffer, buffer_size,
                 "{\"status\":\"ok\",\"freq\":%d,\"scan_active\":%s,"
                 "\"az\":%.1f,\"el\":%.1f,\"temp\":%.2f}",
                 current_freq_hz, scan_aktif ? "true" : "false",
                 current_az, current_el, sensor_get_sicaklik_c());

    } else if (strcmp(cmd, "scan_range") == 0) {
        /* JSON formu: {"cmd":"scan_range","az_max":20,"el_max":20,"step":5}
         * Text formu (plain_cmd fallback): "scan_range 20 20 5"             */
        double az = 0, el = 0, step = 0;
        int params_ok = 0;

        /* JSON yolu: params içinde az_max/el_max/step ara */
        if (params) {
            cJSON *az_obj   = cJSON_GetObjectItem(params, "az_max");
            cJSON *el_obj   = cJSON_GetObjectItem(params, "el_max");
            cJSON *step_obj = cJSON_GetObjectItem(params, "step");
            if (az_obj && el_obj && step_obj) {
                az   = cJSON_GetNumberValue(az_obj);
                el   = cJSON_GetNumberValue(el_obj);
                step = cJSON_GetNumberValue(step_obj);
                params_ok = 1;
            }
        }

        /* Text yolu: params_json'dan sscanf ile parse et */
        if (!params_ok && params_json) {
            if (sscanf(params_json, "scan_range %lf %lf %lf", &az, &el, &step) == 3 ||
                sscanf(params_json, "%lf %lf %lf", &az, &el, &step) == 3) {
                params_ok = 1;
            }
        }

        if (!params_ok) {
            snprintf(response_buffer, buffer_size,
                     "{\"status\":\"error\","
                     "\"message\":\"az_max, el_max, step gerekli\"}");
            sonuc = -1;
        } else {
            fprintf(stderr, "[main] scan_range: az=%.0f el=%.0f step=%.0f\n",
                    az, el, step);
            if (bf_set_scan_range(az, el, step) == 0) {
                /* Onay event'ini socket üzerinden gönder (emit_event stdout'a yazar) */
                char efields[64];
                snprintf(efields, sizeof(efields),
                         "{\"az\":%.1f,\"el\":%.1f,\"step\":%.1f}", az, el, step);
                comms_emit_event("scan_range_set", efields);
                snprintf(response_buffer, buffer_size,
                         "{\"status\":\"ok\",\"az\":%.1f,\"el\":%.1f,\"step\":%.1f}",
                         az, el, step);
            } else {
                snprintf(response_buffer, buffer_size,
                         "{\"status\":\"error\","
                         "\"message\":\"gecersiz aralik (step tam kat olmali, maks 90)\"}");
                sonuc = -1;
            }
        }

    } else if (strcmp(cmd, "freq_sweep_start") == 0) {
        int min_hz   = SWEEP_VARSAYILAN_MIN;
        int max_hz   = SWEEP_VARSAYILAN_MAX;
        int step_hz  = SWEEP_VARSAYILAN_ADIM;
        int dwell_ms = SWEEP_VARSAYILAN_BEKLEME;
        if (params) {
            cJSON *o;
            if ((o = cJSON_GetObjectItem(params, "min_hz")))   min_hz   = (int)cJSON_GetNumberValue(o);
            if ((o = cJSON_GetObjectItem(params, "max_hz")))   max_hz   = (int)cJSON_GetNumberValue(o);
            if ((o = cJSON_GetObjectItem(params, "step_hz")))  step_hz  = (int)cJSON_GetNumberValue(o);
            if ((o = cJSON_GetObjectItem(params, "dwell_ms"))) dwell_ms = (int)cJSON_GetNumberValue(o);
        }
        pll_sweep_start(min_hz, max_hz, step_hz, dwell_ms);
        snprintf(response_buffer, buffer_size,
                 "{\"status\":\"ok\",\"min_hz\":%d,\"max_hz\":%d,"
                 "\"step_hz\":%d,\"dwell_ms\":%d}",
                 min_hz, max_hz, step_hz, dwell_ms);

    } else if (strcmp(cmd, "freq_sweep_stop") == 0) {
        pll_sweep_stop();
        snprintf(response_buffer, buffer_size, "{\"status\":\"ok\"}");

    } else {
        snprintf(response_buffer, buffer_size,
                 "{\"status\":\"error\",\"message\":\"bilinmeyen komut\"}");
        sonuc = -1;
    }

    if (params) cJSON_Delete(params);
    return sonuc;
}

/* ─── Komut İşleyici ─────────────────────────────────────────────────────────
 * Tek satır komutu parse eder.
 * Döndürür: false → çıkış isteği, true → devam.                           */
static bool isle_komut(char *satir) {
    /* Baştaki boşlukları atla, sondaki satır sonu karakterlerini sil */
    while (*satir == ' ' || *satir == '\t') satir++;
    size_t len = strlen(satir);
    while (len > 0 && (satir[len-1] == '\n' || satir[len-1] == '\r' ||
                       satir[len-1] == ' '))
        satir[--len] = '\0';
    if (len == 0) return true;

    char cmd[64];
    if (sscanf(satir, "%63s", cmd) != 1) return true;
    const char *args = satir + strlen(cmd);   /* cmd'den sonrası */

    /* ── help ──────────────────────────────────────────────────────────────── */
    if (strcmp(cmd, "help") == 0) {
        print_help();

    /* ── status ────────────────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "status") == 0) {
        cmd_status();

    /* ── quit / exit ───────────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "quit") == 0 || strcmp(cmd, "exit") == 0) {
        return false;

    /* ── steer <az> <el> ───────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "steer") == 0) {
        float az, el;
        if (sscanf(args, "%f %f", &az, &el) != 2) {
            log_msg("WARN", "Kullanim: steer <az> <el>");
        } else if (az < AZ_MIN_DEG || az > AZ_MAX_DEG ||
                   el < EL_MIN_DEG || el > EL_MAX_DEG) {
            log_msg("WARN", "Aralik disi: az=[%.0f,%.0f] el=[%.0f,%.0f]",
                    AZ_MIN_DEG, AZ_MAX_DEG, EL_MIN_DEG, EL_MAX_DEG);
        } else {
            bf_set_direction(az, el, (float)current_freq_hz);
            current_az = az;
            current_el = el;
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "{\"event\":\"steer\",\"az\":%.1f,\"el\":%.1f}", az, el);
            emit_event(buf);
            log_msg("INFO", "Steering: az=%.1f, el=%.1f", az, el);
        }

    /* ── freq <hz> ─────────────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "freq") == 0) {
        int hz;
        if (sscanf(args, "%d", &hz) != 1) {
            log_msg("WARN", "Kullanim: freq <hz>");
        } else if (hz < FREKANS_MIN_HZ || hz > FREKANS_MAX_HZ) {
            log_msg("WARN", "Frekans aralik disi: %d Hz (%d-%d)",
                    hz, FREKANS_MIN_HZ, FREKANS_MAX_HZ);
            emit_error("pll", "frekans aralik disi");
        } else {
            current_freq_hz = hz;
            pll_set_target_frequency(hz);
            sensor_set_burst_frekans((float)hz);
            log_msg("INFO", "Frekans: %d Hz", hz);
        }

    /* ── gain <kanal> <db> ─────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "gain") == 0) {
        int   kanal;
        float db;
        if (sscanf(args, "%d %f", &kanal, &db) != 2) {
            log_msg("WARN", "Kullanim: gain <kanal> <db>");
        } else if (preamp_set_gain_db(kanal, db) != 0) {
            log_msg("WARN", "Kazanc ayarlanamadi: kanal=%d db=%.1f", kanal, db);
        } else {
            log_msg("INFO", "Kazanc: kanal=%d, %.1f dB", kanal, db);
        }

    /* ── gain_all <db> ─────────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "gain_all") == 0) {
        float db;
        if (sscanf(args, "%f", &db) != 1) {
            log_msg("WARN", "Kullanim: gain_all <db>");
        } else {
            preamp_set_all_gain_db(db);
            log_msg("INFO", "Tum kanallar kazanc: %.1f dB", db);
        }

    /* ── burst_start ───────────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "burst_start") == 0) {
        /* on_burst_start callback'i TVG ve Sensor'ı otomatik bilgilendirir */
        /* 60 pals burst + 30 ms dinleme — pll.h'taki BURST_VARSAYILAN_PALS ve
           BURST_VARSAYILAN_DINLEME_MS sabitleriyle uyumlu */
        pll_burst_start(60, 30);
        log_msg("INFO", "Manuel burst basladi");

    /* ── burst_stop ────────────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "burst_stop") == 0) {
        pll_burst_stop();
        log_msg("INFO", "Burst durduruldu");

    /* ── scan_start [dwell_ms] ─────────────────────────────────────────────── */
    } else if (strcmp(cmd, "scan_start") == 0) {
        if (scan_aktif) {
            log_msg("WARN", "Tarama zaten aktif (once scan_stop yazin)");
        } else {
            int dwell = dwell_default_ms;
            sscanf(args, "%d", &dwell);
            if (dwell <= 0) dwell = dwell_default_ms;

            int *arg = malloc(sizeof(int));
            if (!arg) {
                log_msg("ERROR", "Tarama thread bellek hatasi");
            } else {
                *arg = dwell;
                scan_aktif = true;
                if (pthread_create(&scan_thread, NULL, scan_thread_func, arg) != 0) {
                    log_msg("ERROR", "Tarama thread baslatılamadi");
                    scan_aktif = false;
                    free(arg);
                }
            }
        }

    /* ── scan_stop ─────────────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "scan_stop") == 0) {
        if (!scan_aktif) {
            log_msg("WARN", "Tarama zaten durmus");
        } else {
            scan_aktif = false;
            pthread_join(scan_thread, NULL);
            log_msg("INFO", "Tarama durduruldu");
        }

    /* ── scan_range <az_max> <el_max> <step> ──────────────────────────────── */
    } else if (strcmp(cmd, "scan_range") == 0) {
        double az, el, step;
        if (sscanf(args, "%lf %lf %lf", &az, &el, &step) != 3) {
            log_msg("WARN", "Kullanim: scan_range <az_max> <el_max> <step>");
        } else if (bf_set_scan_range(az, el, step) == 0) {
            char buf[128];
            snprintf(buf, sizeof(buf),
                     "{\"event\":\"scan_range_set\",\"az\":%.1f,"
                     "\"el\":%.1f,\"step\":%.1f}", az, el, step);
            emit_event(buf);
            log_msg("INFO", "Tarama araligi: az=+-%.0f el=+-%.0f step=%.0f",
                    az, el, step);
        } else {
            log_msg("WARN", "Gecersiz tarama araligi (step tam kat olmali, maks 90)");
        }

    /* ── read_temp ─────────────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "read_temp") == 0) {
        float t = sensor_get_sicaklik_c();
        float c = sensor_get_ses_hizi();
        emit_temperature(t, c);

    /* ── read_echo ─────────────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "read_echo") == 0) {
        EchoOlay echo;
        if (sensor_ilk_echo_oku(&echo) != SENSOR_OK)
            log_msg("WARN", "Gecerli echo yok");
        else
            emit_echo(echo.mesafe_m, echo.gecikme_us, current_az, current_el);

    /* ── read_doppler ──────────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "read_doppler") == 0) {
        float freq = sensor_get_doppler_frekans();
        float hiz  = sensor_get_doppler_hiz(0);
        emit_doppler(freq, hiz);

    /* ── mode <log|json|both> ──────────────────────────────────────────────── */
    } else if (strcmp(cmd, "mode") == 0) {
        char mod[16];
        if (sscanf(args, "%15s", mod) != 1) {
            log_msg("WARN", "Kullanim: mode <log|json|both>");
        } else if (strcmp(mod, "log") == 0) {
            output_mode = OUTPUT_LOG;
            fprintf(stderr, "[INFO] Mod: log\n");
        } else if (strcmp(mod, "json") == 0) {
            output_mode = OUTPUT_JSON;
        } else if (strcmp(mod, "both") == 0) {
            output_mode = OUTPUT_BOTH;
            fprintf(stderr, "[INFO] Mod: both\n");
        } else {
            log_msg("WARN", "Gecersiz mod: %s (log|json|both)", mod);
        }

    /* ── log_level <0-3> ───────────────────────────────────────────────────── */
    } else if (strcmp(cmd, "log_level") == 0) {
        int lvl;
        if (sscanf(args, "%d", &lvl) != 1 || lvl < 0 || lvl > 3) {
            log_msg("WARN", "Kullanim: log_level <0-3>");
        } else {
            adc_set_log_level((ADC_LogLevel)lvl);
            sensor_set_log_level((SensorLogLevel)lvl);
            pll_set_log_level(lvl);
            preamp_set_log_level(lvl);
            tvg_set_log_level(lvl);
            ps_set_log_level(lvl);
            bf_set_log_level(lvl);
            log_msg("INFO", "Log seviyesi: %d", lvl);
        }

    } else {
        log_msg("WARN", "Bilinmeyen komut: '%s' (help yazin)", cmd);
    }

    return true;
}

/* ─── Komut Satırı Argümanları ───────────────────────────────────────────────*/
static void parse_args(int argc, char *argv[]) {
    static const struct option uzun_secenekler[] = {
        {"no-hardware", no_argument,       NULL, 'n'},
        {"calib-dir",   required_argument, NULL, 'c'},
        {"no-calib",    no_argument,       NULL, 'C'},
        {"log-level",   required_argument, NULL, 'l'},
        {"help",        no_argument,       NULL, 'h'},
        {NULL, 0, NULL, 0}
    };

    int secim;
    while ((secim = getopt_long(argc, argv, "nc:Cl:h", uzun_secenekler, NULL)) != -1) {
        switch (secim) {
        case 'n':
            no_hardware = true;
            break;
        case 'c':
            strncpy(calib_dizin, optarg, sizeof(calib_dizin) - 1);
            calib_dizin[sizeof(calib_dizin) - 1] = '\0';
            break;
        case 'C':
            no_calib = true;
            break;
        case 'l': {
            int lvl = atoi(optarg);
            if (lvl < 0 || lvl > 3)
                fprintf(stderr, "[WARN] Gecersiz log seviyesi: %s\n", optarg);
            /* Modüller henüz başlatılmadı; log_level komutuyla sonradan da ayarlanabilir */
            break;
        }
        case 'h':
            fprintf(stderr,
                "Kullanim: %s [secenekler]\n"
                "  --no-hardware        Mock mod (donanim gerekmez)\n"
                "  --calib-dir <path>   Kalibrasyon dizini (varsayilan: %s)\n"
                "  --no-calib           Kalibrasyon dosyalarini yukleme\n"
                "  --log-level <0-3>    Log seviyesi (0=debug, 3=error)\n"
                "  --help               Bu mesaji goster\n",
                argv[0], CALIB_DIZIN_VARSAYILAN);
            exit(EXIT_SUCCESS);
        default:
            fprintf(stderr, "Bilinmeyen secenek. --help ile yardim alin.\n");
            exit(EXIT_FAILURE);
        }
    }
}

/* ─── Ana Program ────────────────────────────────────────────────────────────*/
int main(int argc, char *argv[]) {
    parse_args(argc, argv);

    srand((unsigned)time(NULL));
    signal(SIGINT, sigint_handler);

    log_msg("INFO", "Sonar sistemi baslatiliyor%s",
            no_hardware ? " (mock mod)" : "");

    if (!baslat_moduller()) {
        cleanup_all();
        return EXIT_FAILURE;
    }

    if (!no_calib)
        calib_yukle();

    /* Comms (Unix socket sunucusu) başlat */
    log_msg("INFO", "Comms socket baslatiliyor...");
    if (comms_init(NULL) != COMMS_OK) {
        log_msg("WARN", "Comms baslatılamadi, sadece stdin modunda devam ediliyor");
    } else {
        comms_register_handler(socket_komut_handler);
        log_msg("INFO", "Comms hazir: /tmp/sonar.sock");
    }

    log_msg("INFO", "Tum moduller hazir. Komut bekleniyor ('help' ile liste)");

    /* ── Ana komut döngüsü ──────────────────────────────────────────────────*/
    char satir[KOMUT_TAMPONU_BOY];

    while (!shutdown_requested) {
        /* JSON modunda prompt gösterme — pipe ile kullanımda gürültü yapar */
        if (output_mode & OUTPUT_LOG)
            fprintf(stderr, "> ");

        if (!fgets(satir, sizeof(satir), stdin)) {
            if (feof(stdin))
                log_msg("INFO", "stdin kapandi, cikiliyor");
            break;
        }

        if (!isle_komut(satir))
            break;
    }

    cleanup_all();
    return EXIT_SUCCESS;
}
