// gcc -o test pll.c -llgpio -lpthread -lrt -Wall -Wextra

#include "pll.h"

#include <lgpio.h>
#include <pthread.h>
#include <sched.h>   /* SCHED_FIFO — realtime öncelik için */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ─── Kalibrasyon LUT'u ─────────────────────────────────────────────────────
 * V_ctrl (0–5V, VREF_DEVRE referanslı) → VCO çıkış frekansı (Hz)
 * pll_lut_yukle_dosya() ile çalışma zamanında yüklenebilir.
 * ─────────────────────────────────────────────────────────────────────────── */
static PLL_LUT_Entry pll_lut[PLL_LUT_BOYUT_MAX];
static int           pll_lut_boy = 0;

/* ─── İç Durum Yapısı ────────────────────────────────────────────────────── */
static struct {
    int gpio_handle;  /* lgpio GPIO chip handle   */
    int i2c_handle;   /* lgpio I2C handle         */

    int hedef_frekans;  /* Hedef VCO frekansı (Hz) — mutex korumalı */
    int n_divider;      /* N = hedef_frekans / REF_FREKANS_HZ        */

    pthread_t feedback_thread;
    pthread_t burst_thread;
    pthread_t sweep_thread;

    volatile int feedback_aktif;  /* 1: thread çalışıyor, 0: dur sinyali */
    volatile int burst_aktif;
    volatile int sweep_aktif;

    int burst_pulse_count;
    int burst_listen_ms;

    int sweep_min_hz;
    int sweep_max_hz;
    int sweep_step_hz;
    int sweep_dwell_ms;

    pthread_mutex_t mutex;
    int             baslandi; /* pll_init başarıyla tamamlandı mı */
} pll;

/* ─── Callback Pointer'ları ─────────────────────────────────────────────── */
static pll_event_callback_t burst_start_cb  = NULL;
static pll_event_callback_t listen_start_cb = NULL;

/* ─── Log Sistemi ────────────────────────────────────────────────────────── */
static PLL_LogLevel log_seviyesi = PLL_LOG_INFO;

static void pll_log(PLL_LogLevel seviye, const char *fmt, ...) {
    if (seviye < log_seviyesi) return;

    static const char * const etiketler[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    FILE *hedef = (seviye >= PLL_LOG_WARN) ? stderr : stdout;

    fprintf(hedef, "[PLL][%s] ", etiketler[seviye]);

    va_list arglar;
    va_start(arglar, fmt);
    vfprintf(hedef, fmt, arglar);
    va_end(arglar);

    fputc('\n', hedef);
}

/* ─── Zaman Yardımcısı ───────────────────────────────────────────────────── */

/* timespec yapısına nanosaniye ekler; tv_nsec taşmasını yönetir */
static void timespec_ekle_ns(struct timespec *ts, long ns) {
    ts->tv_nsec += ns;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec  += ts->tv_nsec / 1000000000L;
        ts->tv_nsec  = ts->tv_nsec % 1000000000L;
    }
}

/* ─── MCP3221 Sürücüsü (sadece bu dosyada kullanılır) ───────────────────── */

/* 12-bit ham ADC değeri okur (0–4095). Hata durumunda -1 döner. */
static int mcp3221_read_raw(void) {
    uint8_t buf[2] = {0, 0};
    int ret = lgI2cReadDevice(pll.i2c_handle, (char *)buf, 2);
    if (ret != 2) {
        pll_log(PLL_LOG_ERROR, "MCP3221 okuma hatasi (beklenen 2 byte, alinan %d)", ret);
        return -1;
    }
    /* D11..D8 = buf[0] & 0x0F,  D7..D0 = buf[1] */
    return ((buf[0] & 0x0F) << 8) | buf[1];
}

/* ADC girişindeki voltajı döndürür: 0–3.3V (Raspi VDD referanslı).
 * Hata durumunda -1.0 döner. */
static float mcp3221_read_voltage_adc(void) {
    int raw = mcp3221_read_raw();
    if (raw < 0) return -1.0f;
    return (raw / 4095.0f) * VREF_ADC;
}

/* Devre tarafındaki gerçek V_ctrl değerini döndürür: 0–5V (VREF_DEVRE referanslı).
 * Gerilim bölücü etkisi VOLTAJ_OLCEK ile giderilir. Hata durumunda -1.0 döner. */
static float mcp3221_read_voltage_real(void) {
    float adc_v = mcp3221_read_voltage_adc();
    if (adc_v < 0.0f) return -1.0f;
    return adc_v * VOLTAJ_OLCEK;
}

/* ─── LUT Interpolasyonu ─────────────────────────────────────────────────── */

/* V_ctrl (0–5V, VREF_DEVRE referanslı) → tahmini VCO frekansı (Hz).
 * LUT boşsa veya değer aralık dışındaysa hedef frekansı döndürür ve uyarı loglar. */
static int voltaj_to_frekans(float vctrl) {
    int n = pll_lut_boy;

    if (n == 0) {
        pll_log(PLL_LOG_WARN, "LUT bos — hedef frekans kullaniliyor");
        return pll.hedef_frekans; /* int okuma ARM'da hizalanmış olduğundan atomik */
    }

    if (vctrl <= pll_lut[0].vctrl) {
        if (vctrl < pll_lut[0].vctrl)
            pll_log(PLL_LOG_WARN, "Vctrl=%.3fV LUT alt siniri altinda (min=%.3fV)",
                    vctrl, pll_lut[0].vctrl);
        return pll_lut[0].frekans_hz;
    }

    if (vctrl >= pll_lut[n - 1].vctrl) {
        if (vctrl > pll_lut[n - 1].vctrl)
            pll_log(PLL_LOG_WARN, "Vctrl=%.3fV LUT ust siniri ustunde (max=%.3fV)",
                    vctrl, pll_lut[n - 1].vctrl);
        return pll_lut[n - 1].frekans_hz;
    }

    /* Lineer interpolasyon */
    for (int i = 0; i < n - 1; i++) {
        if (vctrl >= pll_lut[i].vctrl && vctrl <= pll_lut[i + 1].vctrl) {
            float t = (vctrl - pll_lut[i].vctrl)
                    / (pll_lut[i + 1].vctrl - pll_lut[i].vctrl);
            return (int)(pll_lut[i].frekans_hz
                       + t * (pll_lut[i + 1].frekans_hz - pll_lut[i].frekans_hz));
        }
    }

    return pll.hedef_frekans; /* ulaşılmamalı */
}

/* ─── Feedback Thread ────────────────────────────────────────────────────── */

/* FEEDBACK_PIN üzerinde kare dalga üretir.
 * Her ADC_GUNCELLEME_TOGGLE toggle'da V_ctrl okunur, VCO tahmini yapılır ve
 * feedback frekansı (= tahmini_VCO / N) güncellenir. TIMER_ABSTIME sayesinde
 * kümülatif drift oluşmaz. */
static void *feedback_thread_fonk(void *arg) {
    (void)arg;

    /* Realtime öncelik — root yetkisi gerektirir:
     * struct sched_param sp = { .sched_priority = 80 };
     * pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp); */

    int  pin    = 0;
    int  sayac  = 0;
    long yp_ns  = 5000000L;  /* Başlangıç yarı periyodu: 5 ms → 100 Hz */

    struct timespec sonraki;
    clock_gettime(CLOCK_MONOTONIC, &sonraki);

    pll_log(PLL_LOG_INFO, "Feedback thread basladi");

    while (pll.feedback_aktif) {
        /* Kare dalgayı toggle et */
        pin ^= 1;
        lgGpioWrite(pll.gpio_handle, FEEDBACK_PIN, pin);

        /* Her ADC_GUNCELLEME_TOGGLE toggle'da bir ADC'den oku ve frekansı güncelle */
        if (++sayac >= ADC_GUNCELLEME_TOGGLE) {
            sayac = 0;

            float vctrl = mcp3221_read_voltage_real();
            if (vctrl >= 0.0f) {
                int tahmini_vco = voltaj_to_frekans(vctrl);

                pthread_mutex_lock(&pll.mutex);
                int nd = pll.n_divider;
                pthread_mutex_unlock(&pll.mutex);

                if (nd > 0 && tahmini_vco > 0) {
                    float fb_hz = (float)tahmini_vco / (float)nd;
                    if (fb_hz > 0.1f) {
                        yp_ns = (long)(500000000.0f / fb_hz);
                        pll_log(PLL_LOG_DEBUG,
                                "Vctrl=%.3fV VCO=%dHz FB=%.2fHz yp=%ldus",
                                vctrl, tahmini_vco, fb_hz, yp_ns / 1000);
                    }
                }
            }
        }

        /* Bir sonraki toggle anını hesapla ve ABSTIME ile uyku — drift sıfır */
        timespec_ekle_ns(&sonraki, yp_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &sonraki, NULL);
    }

    lgGpioWrite(pll.gpio_handle, FEEDBACK_PIN, 0);
    pll_log(PLL_LOG_INFO, "Feedback thread durdu");
    return NULL;
}

/* ─── Burst Thread ───────────────────────────────────────────────────────── */

/* INHIBIT_PIN üzerinde burst döngüsü çalıştırır.
 *   LOW  → VCO aktif, ses gönderme fazı (pals_sayisi kadar pals)
 *   HIGH → VCO pasif, dinleme fazı (listen_ms)
 * LOW süresi her döngüde güncel hedef frekansa göre yeniden hesaplanır;
 * sweep aktifken frekans değişse bile doğru çalışır. */
static void *burst_thread_fonk(void *arg) {
    (void)arg;

    /* Realtime öncelik:
     * struct sched_param sp = { .sched_priority = 75 };
     * pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp); */

    pll_log(PLL_LOG_INFO, "Burst thread basladi");

    while (pll.burst_aktif) {
        pthread_mutex_lock(&pll.mutex);
        int frekans     = pll.hedef_frekans;
        int pals_sayisi = pll.burst_pulse_count;
        int dinleme_ms  = pll.burst_listen_ms;
        pthread_mutex_unlock(&pll.mutex);

        if (frekans <= 0) {
            /* Geçersiz frekans: kısa bekle ve tekrar dene */
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            timespec_ekle_ns(&ts, 1000000L); /* 1 ms */
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
            continue;
        }

        /* LOW fazı: ses gönderme — low_ns = (pals_sayisi / frekans) saniye */
        long low_ns = ((long)pals_sayisi * 1000000000L) / frekans;
        lgGpioWrite(pll.gpio_handle, INHIBIT_PIN, 0); /* LOW: VCO aktif */

        /* Burst başladı callback'i — snapshot alınır, mutex dışında çağrılır */
        {
            pthread_mutex_lock(&pll.mutex);
            pll_event_callback_t cb = burst_start_cb;
            pthread_mutex_unlock(&pll.mutex);
            if (cb) cb();
        }

        struct timespec bitis;
        clock_gettime(CLOCK_MONOTONIC, &bitis);
        timespec_ekle_ns(&bitis, low_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &bitis, NULL);

        if (!pll.burst_aktif) break;

        /* HIGH fazı: dinleme — VCO susturulmuş */
        lgGpioWrite(pll.gpio_handle, INHIBIT_PIN, 1); /* HIGH: VCO pasif */

        /* Listen başladı callback'i — snapshot alınır, mutex dışında çağrılır */
        {
            pthread_mutex_lock(&pll.mutex);
            pll_event_callback_t cb = listen_start_cb;
            pthread_mutex_unlock(&pll.mutex);
            if (cb) cb();
        }

        clock_gettime(CLOCK_MONOTONIC, &bitis);
        timespec_ekle_ns(&bitis, (long)dinleme_ms * 1000000L);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &bitis, NULL);
    }

    lgGpioWrite(pll.gpio_handle, INHIBIT_PIN, 1); /* Güvenli son durum: HIGH */
    pll_log(PLL_LOG_INFO, "Burst thread durdu");
    return NULL;
}

/* ─── Sweep Thread ───────────────────────────────────────────────────────── */

/* Hedef frekansı min→max arasında step_hz adımlarla döngüsel değiştirir.
 * Her adımda dwell_ms bekler (PLL kilitlenmesi + olası burst dinleme süresi).
 * Burst thread eş zamanlı çalışabilir; sweep sadece hedef frekansı yazar,
 * burst kendi döngüsünde LOW süresini güncel frekansa göre hesaplar. */
static void *sweep_thread_fonk(void *arg) {
    (void)arg;

    /* Realtime öncelik:
     * struct sched_param sp = { .sched_priority = 70 };
     * pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp); */

    pll_log(PLL_LOG_INFO, "Sweep thread basladi");

    while (pll.sweep_aktif) {
        pthread_mutex_lock(&pll.mutex);
        int min_hz  = pll.sweep_min_hz;
        int max_hz  = pll.sweep_max_hz;
        int adim_hz = pll.sweep_step_hz;
        int bekleme = pll.sweep_dwell_ms;
        pthread_mutex_unlock(&pll.mutex);

        for (int f = min_hz; f <= max_hz && pll.sweep_aktif; f += adim_hz) {
            pll_set_target_frequency(f);
            pll_log(PLL_LOG_DEBUG, "Sweep: %d Hz", f);

            struct timespec bitis;
            clock_gettime(CLOCK_MONOTONIC, &bitis);
            timespec_ekle_ns(&bitis, (long)bekleme * 1000000L);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &bitis, NULL);
        }
    }

    pll_log(PLL_LOG_INFO, "Sweep thread durdu");
    return NULL;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* GPIO chip ve I2C'yi başlatır; MCP3221'i test okuma ile doğrular.
 * Başarıda 0, hata durumunda negatif hata kodu döner. */
int pll_init(void) {
    if (pll.baslandi) {
        pll_log(PLL_LOG_WARN, "pll_init zaten cagirildi");
        return 0;
    }

    memset(&pll, 0, sizeof(pll));
    pll.gpio_handle = -1;
    pll.i2c_handle  = -1;

    if (pthread_mutex_init(&pll.mutex, NULL) != 0) {
        pll_log(PLL_LOG_ERROR, "Mutex baslatilamadi");
        return -1;
    }

    pll.hedef_frekans     = VARSAYILAN_HEDEF_HZ;
    pll.n_divider         = VARSAYILAN_HEDEF_HZ / REF_FREKANS_HZ;
    pll.burst_pulse_count = BURST_VARSAYILAN_PALS;
    pll.burst_listen_ms   = BURST_VARSAYILAN_DINLEME_MS;
    pll.sweep_min_hz      = SWEEP_VARSAYILAN_MIN;
    pll.sweep_max_hz      = SWEEP_VARSAYILAN_MAX;
    pll.sweep_step_hz     = SWEEP_VARSAYILAN_ADIM;
    pll.sweep_dwell_ms    = SWEEP_VARSAYILAN_BEKLEME;

    pll.gpio_handle = lgGpiochipOpen(0); /* /dev/gpiochip0 */
    if (pll.gpio_handle < 0) {
        pll_log(PLL_LOG_ERROR, "gpiochip0 acilamadi: hata=%d", pll.gpio_handle);
        pthread_mutex_destroy(&pll.mutex);
        return -2;
    }

    /* FEEDBACK_PIN → çıkış, başlangıç LOW */
    int ret = lgGpioClaimOutput(pll.gpio_handle, 0, FEEDBACK_PIN, 0);
    if (ret < 0) {
        pll_log(PLL_LOG_ERROR, "FEEDBACK_PIN %d alinamadi: hata=%d", FEEDBACK_PIN, ret);
        lgGpiochipClose(pll.gpio_handle);
        pthread_mutex_destroy(&pll.mutex);
        return -3;
    }

    /* INHIBIT_PIN → çıkış, başlangıç HIGH (VCO başlangıçta susturulmuş) */
    ret = lgGpioClaimOutput(pll.gpio_handle, 0, INHIBIT_PIN, 1);
    if (ret < 0) {
        pll_log(PLL_LOG_ERROR, "INHIBIT_PIN %d alinamadi: hata=%d", INHIBIT_PIN, ret);
        lgGpiochipClose(pll.gpio_handle);
        pthread_mutex_destroy(&pll.mutex);
        return -4;
    }

    /* I2C-1 üzerinde MCP3221 aç */
    pll.i2c_handle = lgI2cOpen(MCP3221_BUS, MCP3221_ADDR, 0);
    if (pll.i2c_handle < 0) {
        pll_log(PLL_LOG_ERROR, "I2C bus %d acilamadi (MCP3221 adres 0x%02X): hata=%d",
                MCP3221_BUS, MCP3221_ADDR, pll.i2c_handle);
        lgGpiochipClose(pll.gpio_handle);
        pthread_mutex_destroy(&pll.mutex);
        return -5;
    }

    /* Bağlantı doğrulama: bir test okuma yap */
    int test_raw = mcp3221_read_raw();
    if (test_raw < 0) {
        pll_log(PLL_LOG_ERROR, "MCP3221 baglanti testi basarisiz");
        lgI2cClose(pll.i2c_handle);
        lgGpiochipClose(pll.gpio_handle);
        pthread_mutex_destroy(&pll.mutex);
        return -6;
    }

    pll.baslandi = 1;
    pll_log(PLL_LOG_INFO, "PLL hazir — hedef=%dHz N=%d Vctrl=%.3fV",
            pll.hedef_frekans, pll.n_divider,
            (test_raw / 4095.0f) * VREF_ADC * VOLTAJ_OLCEK);
    return 0;
}

/* Tüm thread'leri durdurur (join ile bellek sızıntısı olmaz),
 * I2C ve GPIO kaynaklarını serbest bırakır. */
void pll_cleanup(void) {
    if (!pll.baslandi) return;

    pll_log(PLL_LOG_INFO, "PLL temizleniyor");

    /* Thread'leri sırayla durdur; her biri join edilmeden sonraki başlamaz */
    pll_sweep_stop();
    pll_burst_stop();
    pll_stop_feedback();

    if (pll.i2c_handle  >= 0) { lgI2cClose(pll.i2c_handle);      pll.i2c_handle  = -1; }
    if (pll.gpio_handle >= 0) { lgGpiochipClose(pll.gpio_handle); pll.gpio_handle = -1; }

    pthread_mutex_destroy(&pll.mutex);

    burst_start_cb  = NULL;
    listen_start_cb = NULL;

    pll.baslandi = 0;
    pll_log(PLL_LOG_INFO, "PLL temizlendi");
}

/* Hedef frekansı ve N_divider'ı atomik olarak günceller. */
void pll_set_target_frequency(int hz) {
    if (hz <= 0) {
        pll_log(PLL_LOG_WARN, "Gecersiz frekans: %d Hz", hz);
        return;
    }
    pthread_mutex_lock(&pll.mutex);
    pll.hedef_frekans = hz;
    pll.n_divider     = hz / REF_FREKANS_HZ;
    pthread_mutex_unlock(&pll.mutex);
}

/* Ayarlanmış hedef frekansı döndürür. */
int pll_get_target_frequency(void) {
    pthread_mutex_lock(&pll.mutex);
    int f = pll.hedef_frekans;
    pthread_mutex_unlock(&pll.mutex);
    return f;
}

/* V_ctrl okuyarak LUT'tan tahmini anlık VCO frekansını döndürür.
 * ADC okuma hatası durumunda -1 döner. */
int pll_get_current_frequency(void) {
    float v = pll_read_vctrl();
    if (v < 0.0f) return -1;
    return voltaj_to_frekans(v);
}

/* MCP3221'den okuyup devre tarafındaki gerçek V_ctrl değerini (0–5V) döndürür.
 * ADC okuma hatası durumunda -1.0 döner. */
float pll_read_vctrl(void) {
    return mcp3221_read_voltage_real();
}

/* Feedback kare dalgasını başlatır. Zaten aktifse uyarı loglanır. */
void pll_start_feedback(void) {
    if (pll.feedback_aktif) {
        pll_log(PLL_LOG_WARN, "Feedback zaten aktif");
        return;
    }
    pll.feedback_aktif = 1;
    if (pthread_create(&pll.feedback_thread, NULL, feedback_thread_fonk, NULL) != 0) {
        pll_log(PLL_LOG_ERROR, "Feedback thread olusturulamadi");
        pll.feedback_aktif = 0;
    }
}

/* Feedback kare dalgasını durdurur ve thread'i join eder. */
void pll_stop_feedback(void) {
    if (!pll.feedback_aktif) return;
    pll.feedback_aktif = 0;
    pthread_join(pll.feedback_thread, NULL);
    pll_log(PLL_LOG_INFO, "Feedback durduruldu");
}

/* Burst sinyalini başlatır.
 * pulse_count veya listen_ms ≤ 0 girilirse varsayılan değer kullanılır. */
void pll_burst_start(int pulse_count, int listen_ms) {
    if (pll.burst_aktif) {
        pll_log(PLL_LOG_WARN, "Burst zaten aktif");
        return;
    }
    pthread_mutex_lock(&pll.mutex);
    pll.burst_pulse_count = (pulse_count > 0) ? pulse_count : BURST_VARSAYILAN_PALS;
    pll.burst_listen_ms   = (listen_ms   > 0) ? listen_ms   : BURST_VARSAYILAN_DINLEME_MS;
    pthread_mutex_unlock(&pll.mutex);

    pll.burst_aktif = 1;
    if (pthread_create(&pll.burst_thread, NULL, burst_thread_fonk, NULL) != 0) {
        pll_log(PLL_LOG_ERROR, "Burst thread olusturulamadi");
        pll.burst_aktif = 0;
    }
}

/* Burst sinyalini durdurur ve thread'i join eder. */
void pll_burst_stop(void) {
    if (!pll.burst_aktif) return;
    pll.burst_aktif = 0;
    pthread_join(pll.burst_thread, NULL);
    pll_log(PLL_LOG_INFO, "Burst durduruldu");
}

/* Frekans taramayı başlatır.
 * Herhangi bir parametre ≤ 0 girilirse karşılık gelen varsayılan kullanılır. */
void pll_sweep_start(int min_hz, int max_hz, int step_hz, int dwell_ms) {
    if (pll.sweep_aktif) {
        pll_log(PLL_LOG_WARN, "Sweep zaten aktif");
        return;
    }
    pthread_mutex_lock(&pll.mutex);
    pll.sweep_min_hz   = (min_hz   > 0) ? min_hz   : SWEEP_VARSAYILAN_MIN;
    pll.sweep_max_hz   = (max_hz   > 0) ? max_hz   : SWEEP_VARSAYILAN_MAX;
    pll.sweep_step_hz  = (step_hz  > 0) ? step_hz  : SWEEP_VARSAYILAN_ADIM;
    pll.sweep_dwell_ms = (dwell_ms > 0) ? dwell_ms : SWEEP_VARSAYILAN_BEKLEME;
    pthread_mutex_unlock(&pll.mutex);

    pll.sweep_aktif = 1;
    if (pthread_create(&pll.sweep_thread, NULL, sweep_thread_fonk, NULL) != 0) {
        pll_log(PLL_LOG_ERROR, "Sweep thread olusturulamadi");
        pll.sweep_aktif = 0;
    }
}

/* Frekans taramayı durdurur ve thread'i join eder. */
void pll_sweep_stop(void) {
    if (!pll.sweep_aktif) return;
    pll.sweep_aktif = 0;
    pthread_join(pll.sweep_thread, NULL);
    pll_log(PLL_LOG_INFO, "Sweep durduruldu");
}

/* Log seviyesini ayarlar. Varsayılan: PLL_LOG_INFO. */
void pll_set_log_level(PLL_LogLevel level) {
    log_seviyesi = level;
}

/* ─── LUT Dosya Yükleme ──────────────────────────────────────────────────── */

/* CSV dosyasından VCO kalibrasyon noktalarını yükler.
 * Format (başlık yok): "vctrl frekans_hz\n" — boşlukla ayrılmış, artan vctrl sırası.
 * Mevcut LUT temizlenir; PLL_LUT_BOYUT_MAX kadar giriş kabul edilir.       */
int pll_lut_yukle_dosya(const char *path) {
    if (!path) return -1;

    FILE *f = fopen(path, "r");
    if (!f) {
        pll_log(PLL_LOG_ERROR, "PLL LUT dosyasi acilamadi: %s", path);
        return -1;
    }

    pll_lut_boy = 0;

    float v;
    int   hz;
    int   n = 0;
    while (fscanf(f, "%f %d", &v, &hz) == 2) {
        if (pll_lut_boy >= PLL_LUT_BOYUT_MAX) break;
        pll_lut[pll_lut_boy].vctrl      = v;
        pll_lut[pll_lut_boy].frekans_hz = hz;
        pll_lut_boy++;
        n++;
    }

    fclose(f);
    pll_log(PLL_LOG_INFO, "PLL LUT yuklendi: %s (%d giriş)", path, n);
    return 0;
}

/* ─── Callback Kaydetme ──────────────────────────────────────────────────── */

/* INHIBIT LOW anında (burst başladı) çağrılacak fonksiyonu kaydeder.
 * NULL geçilirse önceki kayıt silinir. */
void pll_set_burst_start_callback(pll_event_callback_t cb) {
    pthread_mutex_lock(&pll.mutex);
    burst_start_cb = cb;
    pthread_mutex_unlock(&pll.mutex);
}

/* INHIBIT HIGH anında (dinleme başladı) çağrılacak fonksiyonu kaydeder.
 * NULL geçilirse önceki kayıt silinir. */
void pll_set_listen_start_callback(pll_event_callback_t cb) {
    pthread_mutex_lock(&pll.mutex);
    listen_start_cb = cb;
    pthread_mutex_unlock(&pll.mutex);
}
