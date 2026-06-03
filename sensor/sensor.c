// gcc -o test sensor.c ../adc/adc.c -llgpio -lpthread -Wall -Wextra

#include "sensor.h"

#include <lgpio.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* ─── İç Durum Yapısı ───────────────────────────────────────────────────────*/
static struct {
    int             gpio_handle;           /* lgpio chip handle               */
    int             baslandi;

    pthread_mutex_t mutex;

    /* Kanal ölçekleme ayarları */
    SensorKanalAyar kanal_ayar[ADC_KANAL_SAYISI];

    /* Doppler LUT (vctrl'e göre sıralı tutulur) */
    DopplerLUT_Entry doppler_lut[DOPPLER_LUT_BOYUT_MAX];
    int              doppler_lut_boy;

    /* Echo tamponu */
    EchoOlay         echo_buf[SENSOR_ECHO_BUFFER_BOYUT];
    int              echo_sayisi;

    /* Burst başlangıç zamanı (Schmitt callback'i için) */
    struct timespec  burst_baslangic;
    int              burst_aktif;

    /* Önbelleğe alınmış fiziksel değerler */
    float            son_sicaklik_c;
    float            son_ses_hizi_m_s;

    /* Burst frekansı (Doppler hız hesabı için) */
    float            burst_frekans_hz;
} sensor;

/* ─── Log Sistemi ───────────────────────────────────────────────────────────*/
static SensorLogLevel log_seviyesi = SENSOR_LOG_INFO;

static void sensor_log(SensorLogLevel seviye, const char *fmt, ...) {
    if (seviye < log_seviyesi) return;
    static const char * const etiketler[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    FILE *hedef = (seviye >= SENSOR_LOG_WARN) ? stderr : stdout;
    fprintf(hedef, "[SENSOR][%s] ", etiketler[seviye]);
    va_list args;
    va_start(args, fmt);
    vfprintf(hedef, fmt, args);
    va_end(args);
    fputc('\n', hedef);
}

/* ─── Varsayılan Kanal Ayarları ─────────────────────────────────────────────
 * Tüm kanallar 5V devresinden gerilim bölücüyle 3.3V ADC'ye bağlı;
 * ölçekleme tüm kanallarda açık.                                           */
static void kanal_ayar_varsayilan(void) {
    for (int ch = 0; ch < ADC_KANAL_SAYISI; ch++) {
        sensor.kanal_ayar[ch].olcek_aktif   = true;
        sensor.kanal_ayar[ch].olcek_katsayi = SENSOR_VOLTAJ_OLCEK;
        sensor.kanal_ayar[ch].ofset_v       = 0.0f;
    }
}

/* ─── Schmitt Trigger Callback ──────────────────────────────────────────────
 * lgpio alert callback — lgpio'nun iç thread'inde çalışır.
 * Her yükselen kenar bir echo olayı olarak kaydedilir.
 * Gerçek lgpio imzası: (int num_alerts, lgGpioAlert_p alerts, void *userdata) */
static void schmitt_callback(int num_alerts, lgGpioAlert_p alerts, void *userdata) {
    (void)userdata;

    for (int i = 0; i < num_alerts; i++) {
        if (alerts[i].report.gpio  != SENSOR_SCHMITT_GPIO) continue;
        if (alerts[i].report.level != 1) continue;   /* sadece yükselen kenar */

        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);

        pthread_mutex_lock(&sensor.mutex);

        if (!sensor.burst_aktif || sensor.echo_sayisi >= SENSOR_ECHO_BUFFER_BOYUT) {
            pthread_mutex_unlock(&sensor.mutex);
            continue;
        }

        double gecikme_us =
            (double)(ts.tv_sec  - sensor.burst_baslangic.tv_sec)  * 1e6 +
            (double)(ts.tv_nsec - sensor.burst_baslangic.tv_nsec) / 1e3;

        float c = sensor.son_ses_hizi_m_s;   /* önbelleğe alınmış — mutex zaten tutuluyor */
        float mesafe_m = (float)(c * gecikme_us / 2e6);

        EchoOlay *slot = &sensor.echo_buf[sensor.echo_sayisi++];
        slot->tetiklendi  = true;
        slot->zaman       = ts;
        slot->gecikme_us  = gecikme_us;
        slot->mesafe_m    = mesafe_m;

        pthread_mutex_unlock(&sensor.mutex);
    }
}

/* ─── İç Yardımcı: Kanal Voltajı Oku (ölçeklendirilmiş) ───────────────────*/
static float voltaj_oku_kanaldan(int kanal) {
    /* Kanal ayarlarını mutex altında al */
    pthread_mutex_lock(&sensor.mutex);
    SensorKanalAyar ayar = sensor.kanal_ayar[kanal];
    pthread_mutex_unlock(&sensor.mutex);

    float v = 0.0f;
    if (adc_read_voltage((uint8_t)kanal, &v) != ADC_OK) return 0.0f;

    if (ayar.olcek_aktif)
        v = v * ayar.olcek_katsayi + ayar.ofset_v;
    else
        v = v + ayar.ofset_v;

    return v;
}

/* ─── İç Yardımcı: Doppler Frekans İnterpolasyonu ─────────────────────────
 * LUT vctrl'e göre sıralı tutulur. LUT boşsa kaba doğrusal model uygulanır.
 * Kaba model: merkez voltaj (VREF_DEVRE/2) → BURST_FREK_HZ_DEFAULT
 *             ±1V → ±1000 Hz                                               */
static float doppler_interpolasyon(float vctrl) {
    pthread_mutex_lock(&sensor.mutex);
    int n = sensor.doppler_lut_boy;

    if (n == 0) {
        /* Kaba doğrusal model */
        float merkez = SENSOR_VREF_DEVRE / 2.0f;
        float f = sensor.burst_frekans_hz + (vctrl - merkez) * 1000.0f;
        pthread_mutex_unlock(&sensor.mutex);
        return f;
    }

    /* Sınır kontrolleri */
    if (vctrl <= sensor.doppler_lut[0].vctrl) {
        float f = sensor.doppler_lut[0].frekans_hz;
        pthread_mutex_unlock(&sensor.mutex);
        return f;
    }
    if (vctrl >= sensor.doppler_lut[n - 1].vctrl) {
        float f = sensor.doppler_lut[n - 1].frekans_hz;
        pthread_mutex_unlock(&sensor.mutex);
        return f;
    }

    /* Doğrusal interpolasyon */
    float f = 0.0f;
    for (int i = 0; i < n - 1; i++) {
        if (vctrl >= sensor.doppler_lut[i].vctrl &&
            vctrl <  sensor.doppler_lut[i + 1].vctrl) {
            float dv = sensor.doppler_lut[i + 1].vctrl - sensor.doppler_lut[i].vctrl;
            float df = sensor.doppler_lut[i + 1].frekans_hz - sensor.doppler_lut[i].frekans_hz;
            float t  = (vctrl - sensor.doppler_lut[i].vctrl) / dv;
            f = sensor.doppler_lut[i].frekans_hz + t * df;
            break;
        }
    }

    pthread_mutex_unlock(&sensor.mutex);
    return f;
}

/* ─── Public API — Başlatma / Temizleme ────────────────────────────────────*/

int sensor_init(void) {
    if (sensor.baslandi) {
        sensor_log(SENSOR_LOG_WARN, "sensor_init zaten cagirildi");
        return SENSOR_OK;
    }

    memset(&sensor, 0, sizeof(sensor));
    sensor.gpio_handle      = -1;
    sensor.burst_frekans_hz = SENSOR_BURST_FREK_HZ_DEFAULT;

    /* Varsayılan ses hızı (0°C kabul) */
    sensor.son_sicaklik_c   = 20.0f;
    sensor.son_ses_hizi_m_s = SES_HIZI_OFSET + SES_HIZI_KATSAYI_T * 20.0f;

    if (pthread_mutex_init(&sensor.mutex, NULL) != 0) {
        sensor_log(SENSOR_LOG_ERROR, "Mutex baslatilamadi");
        return SENSOR_ERR_INIT;
    }

    kanal_ayar_varsayilan();

    /* ADC başlat */
    if (adc_init() != ADC_OK) {
        sensor_log(SENSOR_LOG_ERROR, "ADC baslatilamadi");
        pthread_mutex_destroy(&sensor.mutex);
        return SENSOR_ERR_ADC;
    }

    /* GPIO chip aç */
    sensor.gpio_handle = lgGpiochipOpen(0);
    if (sensor.gpio_handle < 0) {
        sensor_log(SENSOR_LOG_ERROR, "GPIO chip acilamadi: hata=%d", sensor.gpio_handle);
        adc_cleanup();
        pthread_mutex_destroy(&sensor.mutex);
        return SENSOR_ERR_GPIO;
    }

    /* GPIO 16 → giriş, alert (yükselen kenar) */
    int ret = lgGpioClaimAlert(sensor.gpio_handle, 0,
                                LG_RISING_EDGE, SENSOR_SCHMITT_GPIO, -1);
    if (ret < 0) {   /* lgpio: başarıda 0 veya pozitif, hatada negatif */
        sensor_log(SENSOR_LOG_ERROR, "GPIO %d alert kurulamadi: hata=%d",
                   SENSOR_SCHMITT_GPIO, ret);
        lgGpiochipClose(sensor.gpio_handle);
        sensor.gpio_handle = -1;
        adc_cleanup();
        pthread_mutex_destroy(&sensor.mutex);
        return SENSOR_ERR_GPIO;
    }

    lgGpioSetAlertsFunc(sensor.gpio_handle, SENSOR_SCHMITT_GPIO,
                         schmitt_callback, NULL);

    /* İlk sıcaklık ölçümü — ses hızı önbelleğini güncelle */
    sensor.baslandi = 1;
    sensor_get_sicaklik_c();

    sensor_log(SENSOR_LOG_INFO,
               "Sensor hazir — GPIO%d echo, ADC CH0-3, T=%.1fC, c=%.1fm/s",
               SENSOR_SCHMITT_GPIO,
               sensor.son_sicaklik_c, sensor.son_ses_hizi_m_s);
    return SENSOR_OK;
}

void sensor_cleanup(void) {
    if (!sensor.baslandi) return;
    sensor_log(SENSOR_LOG_INFO, "Sensor temizleniyor");

    /* Alert iptal ve GPIO kapat */
    if (sensor.gpio_handle >= 0) {
        lgGpioSetAlertsFunc(sensor.gpio_handle, SENSOR_SCHMITT_GPIO, NULL, NULL);
        lgGpioFree(sensor.gpio_handle, SENSOR_SCHMITT_GPIO);
        lgGpiochipClose(sensor.gpio_handle);
        sensor.gpio_handle = -1;
    }

    adc_cleanup();
    pthread_mutex_destroy(&sensor.mutex);
    sensor.baslandi = 0;
    sensor_log(SENSOR_LOG_INFO, "Sensor temizlendi");
}

/* ─── Voltaj Okumaları ──────────────────────────────────────────────────────*/

float sensor_envelope_voltaj(void) {
    return voltaj_oku_kanaldan(ADC_CH_ENVELOPE);
}

float sensor_ina_voltaj(void) {
    return voltaj_oku_kanaldan(ADC_CH_INA);
}

float sensor_doppler_voltaj(void) {
    return voltaj_oku_kanaldan(ADC_CH_DOPPLER);
}

float sensor_lm60_voltaj(void) {
    return voltaj_oku_kanaldan(ADC_CH_SICAKLIK);
}

/* ─── Fiziksel Değerler ─────────────────────────────────────────────────────*/

/* T(°C) = (Vout − OFSET_V) / KATSAYI_VPC; önbelleği günceller. */
float sensor_get_sicaklik_c(void) {
    float v = sensor_lm60_voltaj();
    float t = (v - SENSOR_LM60_OFSET_V) / SENSOR_LM60_KATSAYI_VPC;

    pthread_mutex_lock(&sensor.mutex);
    sensor.son_sicaklik_c   = t;
    sensor.son_ses_hizi_m_s = SES_HIZI_OFSET + SES_HIZI_KATSAYI_T * t;
    pthread_mutex_unlock(&sensor.mutex);

    sensor_log(SENSOR_LOG_DEBUG, "Sicaklik=%.2fC, SesHizi=%.2fm/s",
               t, sensor.son_ses_hizi_m_s);
    return t;
}

/* c(m/s) = 331.3 + 0.606 × T; sıcaklık ölçümü sonrası güncellenir. */
float sensor_get_ses_hizi(void) {
    pthread_mutex_lock(&sensor.mutex);
    float c = sensor.son_ses_hizi_m_s;
    pthread_mutex_unlock(&sensor.mutex);
    return c;
}

/* PLL loop filter voltajından kilitli frekansı döndürür. */
float sensor_get_doppler_frekans(void) {
    float v = sensor_doppler_voltaj();
    return doppler_interpolasyon(v);
}

/* Doppler hız hesabı: fd = f_echo − f_burst; v = fd × c / (2 × f_burst)
 * f_burst=0 → sensor.burst_frekans_hz kullanılır.                        */
float sensor_get_doppler_hiz(float f_burst) {
    if (f_burst <= 0.0f) {
        pthread_mutex_lock(&sensor.mutex);
        f_burst = sensor.burst_frekans_hz;
        pthread_mutex_unlock(&sensor.mutex);
    }

    float f_echo = sensor_get_doppler_frekans();

    pthread_mutex_lock(&sensor.mutex);
    float c = sensor.son_ses_hizi_m_s;
    pthread_mutex_unlock(&sensor.mutex);

    float fd  = f_echo - f_burst;
    float hiz = fd * c / (2.0f * f_burst);

    sensor_log(SENSOR_LOG_DEBUG,
               "Doppler: fecho=%.1fHz fburst=%.1fHz fd=%.1fHz hiz=%.3fm/s",
               f_echo, f_burst, fd, hiz);
    return hiz;
}

/* ─── Echo / Schmitt Trigger Yönetimi ──────────────────────────────────────*/

int sensor_burst_baslat(void) {
    if (!sensor.baslandi) return SENSOR_ERR_INIT;

    pthread_mutex_lock(&sensor.mutex);
    memset(sensor.echo_buf, 0, sizeof(sensor.echo_buf));
    sensor.echo_sayisi = 0;
    clock_gettime(CLOCK_MONOTONIC, &sensor.burst_baslangic);
    sensor.burst_aktif = 1;
    pthread_mutex_unlock(&sensor.mutex);

    sensor_log(SENSOR_LOG_DEBUG, "Burst basladi");
    return SENSOR_OK;
}

int sensor_burst_sonlandir(void) {
    if (!sensor.baslandi) return SENSOR_ERR_INIT;

    pthread_mutex_lock(&sensor.mutex);
    sensor.burst_aktif = 0;
    pthread_mutex_unlock(&sensor.mutex);

    sensor_log(SENSOR_LOG_DEBUG, "Burst sonlandi, echo_sayisi=%d",
               sensor.echo_sayisi);
    return SENSOR_OK;
}

int sensor_echo_sayisi(void) {
    pthread_mutex_lock(&sensor.mutex);
    int s = sensor.echo_sayisi;
    pthread_mutex_unlock(&sensor.mutex);
    return s;
}

int sensor_echo_oku(int index, EchoOlay *out) {
    if (!out) return SENSOR_ERR_NULL;

    pthread_mutex_lock(&sensor.mutex);
    if (index < 0 || index >= sensor.echo_sayisi) {
        pthread_mutex_unlock(&sensor.mutex);
        return SENSOR_ERR_PARAM;
    }
    *out = sensor.echo_buf[index];
    pthread_mutex_unlock(&sensor.mutex);
    return SENSOR_OK;
}

int sensor_ilk_echo_oku(EchoOlay *out) {
    return sensor_echo_oku(0, out);
}

/* ─── Kalibrasyon — Kanal Ölçekleme ────────────────────────────────────────*/

void sensor_set_olcek(int kanal, bool aktif, float katsayi, float ofset) {
    if (kanal < 0 || kanal >= ADC_KANAL_SAYISI) return;

    pthread_mutex_lock(&sensor.mutex);
    sensor.kanal_ayar[kanal].olcek_aktif   = aktif;
    sensor.kanal_ayar[kanal].olcek_katsayi = katsayi;
    sensor.kanal_ayar[kanal].ofset_v       = ofset;
    pthread_mutex_unlock(&sensor.mutex);
}

int sensor_get_olcek(int kanal, SensorKanalAyar *ayar_out) {
    if (!ayar_out) return SENSOR_ERR_NULL;
    if (kanal < 0 || kanal >= ADC_KANAL_SAYISI) return SENSOR_ERR_PARAM;

    pthread_mutex_lock(&sensor.mutex);
    *ayar_out = sensor.kanal_ayar[kanal];
    pthread_mutex_unlock(&sensor.mutex);
    return SENSOR_OK;
}

/* ─── Doppler LUT ───────────────────────────────────────────────────────────
 * LUT her ekleme sonrası vctrl'e göre sıralanır (bubble sort — maks 16 eleman).
 * Aynı vctrl varsa frekans güncellenir.                                    */

int sensor_doppler_lut_ekle(float vctrl, float frekans_hz) {
    pthread_mutex_lock(&sensor.mutex);

    /* Var olan girişi güncelle */
    for (int i = 0; i < sensor.doppler_lut_boy; i++) {
        if (fabsf(sensor.doppler_lut[i].vctrl - vctrl) < 1e-5f) {
            sensor.doppler_lut[i].frekans_hz = frekans_hz;
            pthread_mutex_unlock(&sensor.mutex);
            return SENSOR_OK;
        }
    }

    if (sensor.doppler_lut_boy >= DOPPLER_LUT_BOYUT_MAX) {
        pthread_mutex_unlock(&sensor.mutex);
        return SENSOR_ERR_LUT;
    }

    int pos = sensor.doppler_lut_boy++;
    sensor.doppler_lut[pos].vctrl      = vctrl;
    sensor.doppler_lut[pos].frekans_hz = frekans_hz;

    /* Bubble sort (küçük N için yeterli) */
    for (int i = 0; i < sensor.doppler_lut_boy - 1; i++) {
        for (int j = 0; j < sensor.doppler_lut_boy - 1 - i; j++) {
            if (sensor.doppler_lut[j].vctrl > sensor.doppler_lut[j + 1].vctrl) {
                DopplerLUT_Entry tmp = sensor.doppler_lut[j];
                sensor.doppler_lut[j]     = sensor.doppler_lut[j + 1];
                sensor.doppler_lut[j + 1] = tmp;
            }
        }
    }

    pthread_mutex_unlock(&sensor.mutex);
    return SENSOR_OK;
}

int sensor_doppler_lut_temizle(void) {
    pthread_mutex_lock(&sensor.mutex);
    memset(sensor.doppler_lut, 0, sizeof(sensor.doppler_lut));
    sensor.doppler_lut_boy = 0;
    pthread_mutex_unlock(&sensor.mutex);
    return SENSOR_OK;
}

/* LUT'u metin dosyasına yazar: her satır "vctrl frekans_hz\n" */
int sensor_doppler_lut_kaydet_dosya(const char *path) {
    if (!path) return SENSOR_ERR_PARAM;

    FILE *f = fopen(path, "w");
    if (!f) return SENSOR_ERR_DOSYA;

    pthread_mutex_lock(&sensor.mutex);
    for (int i = 0; i < sensor.doppler_lut_boy; i++)
        fprintf(f, "%.6f %.2f\n",
                sensor.doppler_lut[i].vctrl,
                sensor.doppler_lut[i].frekans_hz);
    pthread_mutex_unlock(&sensor.mutex);

    fclose(f);
    sensor_log(SENSOR_LOG_INFO, "Doppler LUT kaydedildi: %s", path);
    return SENSOR_OK;
}

/* Mevcut LUT'u temizleyip dosyadan yeniden yükler. */
int sensor_doppler_lut_yukle_dosya(const char *path) {
    if (!path) return SENSOR_ERR_PARAM;

    FILE *f = fopen(path, "r");
    if (!f) return SENSOR_ERR_DOSYA;

    sensor_doppler_lut_temizle();

    float v, hz;
    int   n = 0;
    while (fscanf(f, "%f %f", &v, &hz) == 2) {
        if (sensor_doppler_lut_ekle(v, hz) != SENSOR_OK) break;
        n++;
    }

    fclose(f);
    sensor_log(SENSOR_LOG_INFO, "Doppler LUT yuklendi: %s (%d giriş)", path, n);
    return SENSOR_OK;
}

/* ─── Burst Frekansı ────────────────────────────────────────────────────────*/

void sensor_set_burst_frekans(float hz) {
    if (hz <= 0.0f) return;
    pthread_mutex_lock(&sensor.mutex);
    sensor.burst_frekans_hz = hz;
    pthread_mutex_unlock(&sensor.mutex);
}

float sensor_get_burst_frekans(void) {
    pthread_mutex_lock(&sensor.mutex);
    float hz = sensor.burst_frekans_hz;
    pthread_mutex_unlock(&sensor.mutex);
    return hz;
}

/* ─── Log Seviyesi ──────────────────────────────────────────────────────────*/

void sensor_set_log_level(SensorLogLevel level) {
    log_seviyesi = level;
}
