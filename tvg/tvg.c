// gcc -o test tvg.c -llgpio -lpthread -lm -Wall -Wextra

#include "tvg.h"

#include <lgpio.h>
#include <math.h>
#include <pthread.h>
#include <sched.h>    /* SCHED_FIFO — realtime öncelik için */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ─── Rampa LUT'u ───────────────────────────────────────────────────────────
 * Her 1 ms için MCP4651 Wiper1 step değeri.
 * tvg_lut_olustur() ile formülden otomatik hesaplanır.
 * Runtime'da tvg_lut_set() veya tvg_lut_yukle_dosya() ile override edilebilir.*/
static uint8_t tvg_lut[TVG_LUT_BOYUT];

/* ─── İç Durum Yapısı ───────────────────────────────────────────────────────
 * BUS PAYLAŞIMI NOTU:
 * Bu modül preamp ve phase_shift modülleriyle aynı I2C-1 hattını paylaşır.
 * İleride çakışmaları önlemek için her modülün ortak bir
 * extern pthread_mutex_t i2c_bus_mutex kullanması önerilir.
 * Şimdilik her modül kendi mutex'ini bağımsız yönetir.                     */
static struct {
    int     i2c_handle;
    uint8_t mevcut_step;            /* Son yazılan N değeri (önbellek)         */
    uint8_t max_step;               /* Güvenlik sınırı: 0 = sınırsız          */
    float   gain_db_min;            /* Runtime kazanç alt sınırı              */
    float   gain_db_max;            /* Runtime kazanç üst sınırı              */

    pthread_t    rampa_thread;
    volatile int rampa_aktif;       /* 1: thread çalışıyor, 0: dur sinyali    */
    int          rampa_thread_olusturuldu; /* join için takip                 */

    pthread_mutex_t mutex;
    int             baslandi;
} tvg;

/* ─── Log Sistemi ───────────────────────────────────────────────────────────*/
static TVG_LogLevel log_seviyesi = TVG_LOG_INFO;

static void tvg_log(TVG_LogLevel seviye, const char *fmt, ...) {
    if (seviye < log_seviyesi) return;
    static const char * const etiketler[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    FILE *hedef = (seviye >= TVG_LOG_WARN) ? stderr : stdout;
    fprintf(hedef, "[TVG][%s] ", etiketler[seviye]);
    va_list arglar;
    va_start(arglar, fmt);
    vfprintf(hedef, fmt, arglar);
    va_end(arglar);
    fputc('\n', hedef);
}

/* ─── Zaman Yardımcısı ──────────────────────────────────────────────────────*/
static void timespec_ekle_ns(struct timespec *ts, long ns) {
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec  += ts->tv_nsec / 1000000000L;
        ts->tv_nsec  = ts->tv_nsec % 1000000000L;
    }
}

/* ─── MCP4651 Sürücüsü — Wiper1'e özgü (sadece bu dosyada) ─────────────────*/

/* Wiper1'e step değeri yazar (2 byte: [0x10, step]).
 * Başarısız I2C işlemini TVG_I2C_MAX_DENEME kez dener.                    */
static int mcp4651_yaz(uint8_t step) {
    uint8_t buf[2];
    buf[0] = 0x10;  /* Wiper1 yazma komutu: (1 << 4) | 0x00 */
    buf[1] = step;

    for (int d = 0; d < TVG_I2C_MAX_DENEME; d++) {
        int ret = lgI2cWriteDevice(tvg.i2c_handle, (char *)buf, 2);
        if (ret == 0) return TVG_OK;
        tvg_log(TVG_LOG_WARN, "Wiper1 yaz hatasi step=%u deneme=%d/%d",
                step, d + 1, TVG_I2C_MAX_DENEME);
    }
    return TVG_ERR_I2C;
}

/* Wiper1'den step değeri okur (debug / doğrulama amaçlı).
 * Okuma komutu: 0x1C → 2 byte yanıt; D8 8-bit modda 0'dır.               */
static int mcp4651_oku(uint8_t *step_out) {
    if (!step_out) return TVG_ERR_PARAM;

    for (int d = 0; d < TVG_I2C_MAX_DENEME; d++) {
        uint8_t cmd = 0x1C; /* Wiper1 okuma komutu: (1 << 4) | 0x0C */
        int ret = lgI2cWriteDevice(tvg.i2c_handle, (char *)&cmd, 1);
        if (ret != 0) {
            tvg_log(TVG_LOG_WARN, "Wiper1 oku komutu hatasi deneme=%d", d + 1);
            continue;
        }
        uint8_t rbuf[2] = {0, 0};
        ret = lgI2cReadDevice(tvg.i2c_handle, (char *)rbuf, 2);
        if (ret == 2) {
            *step_out = rbuf[1]; /* D8 daima 0; step = D7..D0 */
            return TVG_OK;
        }
        tvg_log(TVG_LOG_WARN, "Wiper1 oku yaniti hatasi deneme=%d", d + 1);
    }
    return TVG_ERR_I2C;
}

/* ─── LUT Oluşturucu ─────────────────────────────────────────────────────── */

/* tvg.gain_db_min ve tvg.gain_db_max kullanarak dB-lineer rampa hesaplar.
 * t=0: N≈255 (min kazanç), t=TVG_RAMPA_MS: N≈0 (max kazanç).
 * mutex dışında da çağrılabilir — sadece tvg_lut[] ve tvg.gain_db_* kullanır. */
static void tvg_lut_olustur(void) {
    for (int t = 0; t < TVG_LUT_BOYUT; t++) {
        float oran  = (float)t / (float)TVG_RAMPA_MS;
        float G_dB  = tvg.gain_db_min + (tvg.gain_db_max - tvg.gain_db_min) * oran;
        float G_lin = powf(10.0f, G_dB / 20.0f);
        /* Ters formül: N = (TVG_INA_KATSAYI / (G_lin - 1) - TVG_R_SABIT) / TVG_R_STEP */
        float N_f   = (TVG_INA_KATSAYI / (G_lin - 1.0f) - TVG_R_SABIT_OHM) / TVG_R_STEP_OHM;
        if (N_f < 0.0f)   N_f = 0.0f;
        if (N_f > 255.0f) N_f = 255.0f;
        tvg_lut[t] = (uint8_t)(N_f + 0.5f);
    }
    tvg_log(TVG_LOG_DEBUG, "LUT hesaplandi: t=0 N=%u (%.2f dB), t=%d N=%u (%.2f dB)",
            tvg_lut[0], tvg.gain_db_min, TVG_RAMPA_MS, tvg_lut[TVG_RAMPA_MS], tvg.gain_db_max);
}

/* ─── Rampa Thread ───────────────────────────────────────────────────────── */

/* Dinleme süresi boyunca her 1 ms'de LUT'tan N okuyup MCP4651'e yazar.
 * TIMER_ABSTIME kullanımı sayesinde döngü boyunca drift birikmez.
 * Güvenlik sınırı: N max_step değerinin altına indirilemez.                */
static void *rampa_thread_fonk(void *arg) {
    (void)arg;

    /* Realtime öncelik — root yetkisi gerektirir:
     * struct sched_param sp = { .sched_priority = 85 };
     * pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp); */

    struct timespec hedef;
    clock_gettime(CLOCK_MONOTONIC, &hedef);

    tvg_log(TVG_LOG_INFO, "Rampa thread basladi (%d adim, %d ms)", TVG_LUT_BOYUT, TVG_RAMPA_MS);

    for (int t = 0; t < TVG_LUT_BOYUT && tvg.rampa_aktif; t++) {
        pthread_mutex_lock(&tvg.mutex);
        uint8_t N     = tvg_lut[t];
        uint8_t max_N = tvg.max_step;

        /* Güvenlik sınırı: N max_step'in altına inemez (tepe voltajı koruması) */
        if (max_N > 0 && N < max_N) N = max_N;

        int ret = mcp4651_yaz(N);
        if (ret == TVG_OK) tvg.mevcut_step = N;
        pthread_mutex_unlock(&tvg.mutex);

        if (ret != TVG_OK)
            tvg_log(TVG_LOG_WARN, "t=%d ms: I2C hatasi, N=%u yazılamadı", t, N);
        else
            tvg_log(TVG_LOG_DEBUG, "t=%d ms N=%u G=%.2f dB", t, N,
                    20.0f * log10f(TVG_GAIN_FORMUL(N)));

        /* Bir sonraki 1 ms adımına kadar bekle */
        timespec_ekle_ns(&hedef, 1000000L);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &hedef, NULL);
    }

    /* rampa_aktif bayrağını thread içinden sıfırla (normal bitiş) */
    pthread_mutex_lock(&tvg.mutex);
    tvg.rampa_aktif = 0;
    pthread_mutex_unlock(&tvg.mutex);

    tvg_log(TVG_LOG_INFO, "Rampa tamamlandi (N=%u)", tvg.mevcut_step);
    return NULL;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* I2C açar, MCP4651'i test eder, LUT'u formülden hesaplar,
 * Wiper1'i güvenli başlangıç değerine (N=255, minimum kazanç) çeker.      */
int tvg_init(void) {
    if (tvg.baslandi) {
        tvg_log(TVG_LOG_WARN, "tvg_init zaten cagirildi");
        return TVG_OK;
    }

    memset(&tvg, 0, sizeof(tvg));
    tvg.i2c_handle  = -1;
    tvg.gain_db_min = TVG_GAIN_DB_MIN;
    tvg.gain_db_max = TVG_GAIN_DB_MAX;

    if (pthread_mutex_init(&tvg.mutex, NULL) != 0) {
        tvg_log(TVG_LOG_ERROR, "Mutex baslatilamadi");
        return TVG_ERR_INIT;
    }

    tvg.i2c_handle = lgI2cOpen(TVG_I2C_BUS, TVG_MCP4651_ADDR, 0);
    if (tvg.i2c_handle < 0) {
        tvg_log(TVG_LOG_ERROR, "I2C bus %d adres 0x%02X acilamadi: hata=%d",
                TVG_I2C_BUS, TVG_MCP4651_ADDR, tvg.i2c_handle);
        pthread_mutex_destroy(&tvg.mutex);
        return TVG_ERR_I2C;
    }

    /* Bağlantı testi: güvenli başlangıç değeri N=255 yaz */
    int ret = mcp4651_yaz(255);
    if (ret != TVG_OK) {
        tvg_log(TVG_LOG_ERROR, "MCP4651 baglanti testi basarisiz");
        lgI2cClose(tvg.i2c_handle);
        pthread_mutex_destroy(&tvg.mutex);
        return ret;
    }
    tvg.mevcut_step = 255;

    /* LUT'u formülden oluştur */
    tvg_lut_olustur();

    tvg.baslandi = 1;
    tvg_log(TVG_LOG_INFO,
            "TVG hazir — adres=0x%02X Wiper%d bus=%d G=[%.2f,%.2f]dB",
            TVG_MCP4651_ADDR, TVG_MCP4651_WIPER, TVG_I2C_BUS,
            tvg.gain_db_min, tvg.gain_db_max);
    return TVG_OK;
}

/* Rampa thread'ini durdurur, Wiper1'i N=255'e (min kazanç) çeker,
 * I2C handle'ı kapatır ve mutex'i yok eder.                               */
void tvg_cleanup(void) {
    if (!tvg.baslandi) return;
    tvg_log(TVG_LOG_INFO, "TVG temizleniyor");

    tvg_rampa_durdur();

    pthread_mutex_lock(&tvg.mutex);
    mcp4651_yaz(255); /* Güvenli son durum: minimum kazanç */
    if (tvg.i2c_handle >= 0) {
        lgI2cClose(tvg.i2c_handle);
        tvg.i2c_handle = -1;
    }
    pthread_mutex_unlock(&tvg.mutex);
    pthread_mutex_destroy(&tvg.mutex);

    tvg.baslandi = 0;
    tvg_log(TVG_LOG_INFO, "TVG temizlendi");
}

/* Ham N değerini doğrudan MCP4651'e yazar (kalibrasyon ve test için). */
int tvg_set_step(uint8_t N) {
    if (!tvg.baslandi) return TVG_ERR_INIT;

    pthread_mutex_lock(&tvg.mutex);
    int ret = mcp4651_yaz(N);
    if (ret == TVG_OK) tvg.mevcut_step = N;
    pthread_mutex_unlock(&tvg.mutex);

    if (ret == TVG_OK)
        tvg_log(TVG_LOG_DEBUG, "Manuel step: N=%u (G=%.3fx / %.2f dB)",
                N, TVG_GAIN_FORMUL(N), 20.0f * log10f(TVG_GAIN_FORMUL(N)));
    return ret;
}

/* Son yazılan step değerini önbellekten döndürür (donanım okuma yapmaz). */
int tvg_get_step(uint8_t *N_out) {
    if (!tvg.baslandi)  return TVG_ERR_INIT;
    if (!N_out)         return TVG_ERR_PARAM;

    pthread_mutex_lock(&tvg.mutex);
    *N_out = tvg.mevcut_step;
    pthread_mutex_unlock(&tvg.mutex);
    return TVG_OK;
}

/* Anlık kazancı dB cinsinden döndürür. Hata durumunda -1.0 döner. */
float tvg_get_gain_db(void) {
    if (!tvg.baslandi) return -1.0f;

    pthread_mutex_lock(&tvg.mutex);
    uint8_t N = tvg.mevcut_step;
    pthread_mutex_unlock(&tvg.mutex);

    return 20.0f * log10f(TVG_GAIN_FORMUL(N));
}

/* Anlık lineer kazancı döndürür. Hata durumunda -1.0 döner. */
float tvg_get_gain_linear(void) {
    if (!tvg.baslandi) return -1.0f;

    pthread_mutex_lock(&tvg.mutex);
    uint8_t N = tvg.mevcut_step;
    pthread_mutex_unlock(&tvg.mutex);

    return TVG_GAIN_FORMUL(N);
}

/* PLL callback: INHIBIT LOW oldu (burst başladı).
 * Varsa önceki rampa durdurulur, Wiper1'e N=255 (minimum kazanç) yazılır.
 * Rampa bu aşamada başlatılmaz — dinleme süresi henüz başlamadı.          */
void tvg_on_burst_start(void) {
    if (!tvg.baslandi) return;

    /* Önceki rampa henüz bitmemişse durdur (genellikle çoktan tamamlanmıştır) */
    tvg_rampa_durdur();

    pthread_mutex_lock(&tvg.mutex);
    int ret = mcp4651_yaz(255);
    if (ret == TVG_OK) tvg.mevcut_step = 255;
    pthread_mutex_unlock(&tvg.mutex);

    tvg_log(TVG_LOG_DEBUG, "Burst basladi: N=255 (%.2f dB)", tvg_get_gain_db());
}

/* PLL callback: INHIBIT HIGH oldu (dinleme başladı).
 * Önceki rampa thread'i join edilir, yeni rampa thread'i başlatılır.      */
void tvg_on_listen_start(void) {
    if (!tvg.baslandi) return;

    /* Önceki rampa thread'ini düzgünce durdur ve join et */
    tvg_rampa_durdur();

    /* Yeni rampa thread'i başlat */
    tvg.rampa_aktif = 1;
    if (pthread_create(&tvg.rampa_thread, NULL, rampa_thread_fonk, NULL) == 0) {
        tvg.rampa_thread_olusturuldu = 1;
        tvg_log(TVG_LOG_DEBUG, "Dinleme basladi: rampa thread tetiklendi");
    } else {
        tvg_log(TVG_LOG_ERROR, "Rampa thread olusturulamadi");
        tvg.rampa_aktif = 0;
    }
}

/* Rampa thread'ini durdurur ve join eder.
 * Rampa çalışmıyorsa sessizce döner. Maksimum bekleme süresi ~1 ms.       */
void tvg_rampa_durdur(void) {
    if (!tvg.rampa_thread_olusturuldu) return;
    tvg.rampa_aktif = 0;
    pthread_join(tvg.rampa_thread, NULL);
    tvg.rampa_thread_olusturuldu = 0;
    tvg_log(TVG_LOG_DEBUG, "Rampa durduruldu");
}

/* LUT'ta belirtilen zamana (t_ms) karşılık gelen N değerini ayarlar.
 * Kalibrasyon sırasında rampa şeklini manuel olarak düzenlemek için.      */
int tvg_lut_set(int t_ms, uint8_t N) {
    if (t_ms < 0 || t_ms >= TVG_LUT_BOYUT) return TVG_ERR_LUT_ARALIK;

    pthread_mutex_lock(&tvg.mutex);
    tvg_lut[t_ms] = N;
    pthread_mutex_unlock(&tvg.mutex);
    return TVG_OK;
}

/* LUT'taki t_ms anına ait N değerini döndürür. */
int tvg_lut_get(int t_ms, uint8_t *N_out) {
    if (t_ms < 0 || t_ms >= TVG_LUT_BOYUT) return TVG_ERR_LUT_ARALIK;
    if (!N_out)                             return TVG_ERR_PARAM;

    pthread_mutex_lock(&tvg.mutex);
    *N_out = tvg_lut[t_ms];
    pthread_mutex_unlock(&tvg.mutex);
    return TVG_OK;
}

/* LUT'u formülden yeniden hesaplar (tvg_kalibre_* sonrası otomatik çağrılır). */
void tvg_lut_yeniden_hesapla(void) {
    pthread_mutex_lock(&tvg.mutex);
    tvg_lut_olustur();
    pthread_mutex_unlock(&tvg.mutex);
    tvg_log(TVG_LOG_INFO, "LUT yeniden hesaplandi");
}

/* Tüm LUT verilerini CSV dosyasına kaydeder. Format: t_ms,N başlığı + veriler.
 * Mutex kısa tutulur; dosya I/O mutex dışında gerçekleşir.                */
int tvg_lut_kaydet_dosya(const char *path) {
    uint8_t yerel_lut[TVG_LUT_BOYUT];

    pthread_mutex_lock(&tvg.mutex);
    memcpy(yerel_lut, tvg_lut, sizeof(tvg_lut));
    pthread_mutex_unlock(&tvg.mutex);

    FILE *fp = fopen(path, "w");
    if (!fp) {
        tvg_log(TVG_LOG_ERROR, "Dosya acilamadi (yazma): %s", path);
        return TVG_ERR_DOSYA;
    }

    fprintf(fp, "t_ms,N\n");
    for (int t = 0; t < TVG_LUT_BOYUT; t++)
        fprintf(fp, "%d,%u\n", t, yerel_lut[t]);
    fclose(fp);

    tvg_log(TVG_LOG_INFO, "LUT kaydedildi: %s (%d nokta)", path, TVG_LUT_BOYUT);
    return TVG_OK;
}

/* CSV dosyasından LUT verilerini yükler.
 * Eksik satırlar mevcut LUT değerini korur; geçersiz satırlar atlanır.    */
int tvg_lut_yukle_dosya(const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) {
        tvg_log(TVG_LOG_ERROR, "Dosya acilamadi (okuma): %s", path);
        return TVG_ERR_DOSYA;
    }

    /* Mevcut LUT'u kopyala; eksik satırlar mevcut değeri korur */
    uint8_t yeni_lut[TVG_LUT_BOYUT];
    pthread_mutex_lock(&tvg.mutex);
    memcpy(yeni_lut, tvg_lut, sizeof(tvg_lut));
    pthread_mutex_unlock(&tvg.mutex);

    char line[64];
    if (!fgets(line, sizeof(line), fp)) { /* başlık satırını atla */
        fclose(fp);
        return TVG_ERR_DOSYA;
    }

    int okunan = 0, atlanan = 0;
    while (fgets(line, sizeof(line), fp)) {
        int      t;
        unsigned N;
        if (sscanf(line, "%d,%u", &t, &N) != 2)  { atlanan++; continue; }
        if (t < 0 || t >= TVG_LUT_BOYUT)          { atlanan++; continue; }
        if (N > 255)                               { atlanan++; continue; }
        yeni_lut[t] = (uint8_t)N;
        okunan++;
    }
    fclose(fp);

    pthread_mutex_lock(&tvg.mutex);
    memcpy(tvg_lut, yeni_lut, sizeof(tvg_lut));
    pthread_mutex_unlock(&tvg.mutex);

    tvg_log(TVG_LOG_INFO, "LUT yuklendi: %s (%d nokta, %d atlandi)", path, okunan, atlanan);
    return TVG_OK;
}

/* Minimum kazanç sınırını runtime'da günceller ve LUT'u yeniden hesaplar. */
int tvg_kalibre_min_db(float yeni_dB) {
    if (!tvg.baslandi) return TVG_ERR_INIT;

    pthread_mutex_lock(&tvg.mutex);
    tvg.gain_db_min = yeni_dB;
    tvg_lut_olustur();
    pthread_mutex_unlock(&tvg.mutex);

    tvg_log(TVG_LOG_INFO, "Kalibrasyon: G_min=%.2f dB, LUT yeniden hesaplandi", yeni_dB);
    return TVG_OK;
}

/* Maksimum kazanç sınırını runtime'da günceller ve LUT'u yeniden hesaplar. */
int tvg_kalibre_max_db(float yeni_dB) {
    if (!tvg.baslandi) return TVG_ERR_INIT;

    pthread_mutex_lock(&tvg.mutex);
    tvg.gain_db_max = yeni_dB;
    tvg_lut_olustur();
    pthread_mutex_unlock(&tvg.mutex);

    tvg_log(TVG_LOG_INFO, "Kalibrasyon: G_max=%.2f dB, LUT yeniden hesaplandi", yeni_dB);
    return TVG_OK;
}

/* Rampa sırasında N'nin altına inemeyeceği sınırı ayarlar.
 * max_N=0: sınır yok (N=0'a kadar inebilir, maksimum kazanç).
 * max_N>0: N bu değerin altına inmez (tepe voltajı koruması).             */
void tvg_set_max_step(uint8_t max_N) {
    pthread_mutex_lock(&tvg.mutex);
    tvg.max_step = max_N;
    pthread_mutex_unlock(&tvg.mutex);
    if (max_N > 0)
        tvg_log(TVG_LOG_INFO, "Guvenlik siniri: N >= %u (G <= %.2f dB)",
                max_N, 20.0f * log10f(TVG_GAIN_FORMUL(max_N)));
    else
        tvg_log(TVG_LOG_INFO, "Guvenlik siniri: devre disi (N=0'a kadar)");
}

/* Mevcut güvenlik sınırını döndürür. */
uint8_t tvg_get_max_step(void) {
    pthread_mutex_lock(&tvg.mutex);
    uint8_t v = tvg.max_step;
    pthread_mutex_unlock(&tvg.mutex);
    return v;
}

/* Log seviyesini ayarlar. Varsayılan: TVG_LOG_INFO. */
void tvg_set_log_level(TVG_LogLevel level) {
    log_seviyesi = level;
}
