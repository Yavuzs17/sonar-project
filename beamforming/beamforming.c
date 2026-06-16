// gcc -o test beamforming.c -lpthread -lm -Wall -Wextra

#include "beamforming.h"

#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "../phase_shift/phase_shift.h"

/* ─── Runtime Tarama Aralığı ────────────────────────────────────────────────
 * Scan loop bu değerleri her turda bf_get_scan_range ile okur.
 * Cache adımı 5° olduğundan step_deg 5'in tam katı seçilmelidir;
 * ancak bf_set_direction anlık hesapladığından herhangi bir pozitif
 * tam sayı adım da teknik olarak geçerlidir.                                 */
#define BF_CACHE_STEP_DEG   1.0   /* Kabul edilen minimum adım (°)            */
#define BF_CACHE_RANGE_DEG 90.0   /* Kabul edilen maksimum açı sınırı (°)    */

static double s_az_max = 25.0;   /* Aktif azimuth yarı-aralık (derece)       */
                                 /* Varsayılan ±25°: TX kapsaması ±27° içinde */
                                 /* kalır, grating/SLL'nin en kötü olduğu uçlardan kaçınır */
static double s_el_max = 25.0;   /* Aktif elevation yarı-aralık (derece)     */
static double s_step   =  5.0;   /* Aktif tarama adımı (derece, HPBW 8.8° ile uyumlu) */

/* ─── Eleman Koordinatları ──────────────────────────────────────────────────
 * Normalize birim koordinatlar; gerçek metre = koordinat × eleman_mesafesi_m
 * Çevre elemanlar düzgün altıgen geometride 60° aralıklı, merkez (0,0).     */
static const BF_ElemanKonum eleman_konumlari[BF_ELEMAN_SAYISI] = {
    { 0.0f,       0.0f      },  /* Eleman 0: merkez             */
    { 1.0f,       0.0f      },  /* Eleman 1: sağ        (  0°) */
    { 0.5f,       0.866025f },  /* Eleman 2: sağ üst    ( 60°) */
    {-0.5f,       0.866025f },  /* Eleman 3: sol üst    (120°) */
    {-1.0f,       0.0f      },  /* Eleman 4: sol        (180°) */
    {-0.5f,      -0.866025f },  /* Eleman 5: sol alt    (240°) */
    { 0.5f,      -0.866025f },  /* Eleman 6: sağ alt    (300°) */
};

/* ─── Faz Cache'i ───────────────────────────────────────────────────────────
 * [frekans_idx][azimuth_idx][elevation_idx] → 7 alıcı fazı (0–360°)
 * Toplam boyut: 21 × 13 × 13 × 7 × 4 byte ≈ 99 KB                         */
static BF_FazSeti faz_cache[BF_SWEEP_NOKTA_SAYISI]
                            [BF_AZIMUTH_NOKTA_SAYISI]
                            [BF_ELEVATION_NOKTA_SAYISI];

/* ─── İç Durum Yapısı ───────────────────────────────────────────────────────*/
static struct {
    float eleman_mesafesi_m;      /* Gerçek eleman aralığı (metre)           */
    float aktif_azimuth_deg;      /* Son ayarlanan azimuth açısı             */
    float aktif_elevation_deg;    /* Son ayarlanan elevation açısı           */

    pthread_t    tarama_thread;
    volatile int tarama_aktif;
    int          tarama_thread_olusturuldu;
    int          tarama_dwell_ms;

    void (*tarama_callback)(int az_idx, int el_idx, int frek_idx);

    pthread_mutex_t mutex;
    int             baslandi;
} bf;

/* ─── Log Sistemi ───────────────────────────────────────────────────────────*/
static BF_LogLevel log_seviyesi = BF_LOG_INFO;

static void bf_log(BF_LogLevel seviye, const char *fmt, ...) {
    if (seviye < log_seviyesi) return;
    static const char * const etiketler[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    FILE *hedef = (seviye >= BF_LOG_WARN) ? stderr : stdout;
    fprintf(hedef, "[BF][%s] ", etiketler[seviye]);
    va_list arglar;
    va_start(arglar, fmt);
    vfprintf(hedef, fmt, arglar);
    va_end(arglar);
    fputc('\n', hedef);
}

/* ─── Zaman Yardımcısı ──────────────────────────────────────────────────────*/
static void timespec_ekle_ms(struct timespec *ts, int ms) {
    ts->tv_nsec += (long)ms * 1000000L;
    if (ts->tv_nsec >= 1000000000L) {
        ts->tv_sec  += ts->tv_nsec / 1000000000L;
        ts->tv_nsec  = ts->tv_nsec % 1000000000L;
    }
}

/* ─── İç Faz Hesaplama ───────────────────────────────────────────────────── */

/* Delay-and-sum formülü ile 7 alıcının faz değerlerini hesaplar.
 * d: gerçek eleman mesafesi (metre). Sonuç 0–360° aralığındadır.
 * Mutex almaz — yalnızca yerel matematiksel hesaplama yapar.                */
static void faz_hesapla_ic(float az_deg, float el_deg, float frek_hz, float d,
                            float fazlar_out[BF_ELEMAN_SAYISI]) {
    float az_rad = az_deg * (float)M_PI / 180.0f;
    float el_rad = el_deg * (float)M_PI / 180.0f;

    /* Hedef yönünün dizi düzlemindeki bileşenleri (z = dizi normali, ihmal edilir) */
    float ux = sinf(el_rad) * cosf(az_rad);
    float uy = sinf(el_rad) * sinf(az_rad);

    float lambda = BF_SES_HIZI_M_S / frek_hz;
    float k      = 2.0f * (float)M_PI / lambda;  /* Dalga sayısı (rad/m) */

    for (int i = 0; i < BF_ELEMAN_SAYISI; i++) {
        float xi = eleman_konumlari[i].x * d;
        float yi = eleman_konumlari[i].y * d;

        /* Negatif işaret: hedef yönünden gelen sinyaller konstruktif toplanır */
        float faz_rad    = -k * (xi * ux + yi * uy);
        float faz_derece = faz_rad * 180.0f / (float)M_PI;

        /* 0–360° aralığına normalize */
        faz_derece = fmodf(faz_derece, 360.0f);
        if (faz_derece < 0.0f) faz_derece += 360.0f;

        fazlar_out[i] = faz_derece;
    }
}

/* ─── Cache Oluşturucu ──────────────────────────────────────────────────────
 * Tüm (frekans × azimuth × elevation) kombinasyonları için faz tablosunu doldurur.
 * Init sırasında ~1–2 saniye sürebilir.
 * UYARI: Çağırılmadan önce tarama thread'i durdurulmuş olmalı —
 * cache dizisi mutex dışında yazılır, eş zamanlı okuma race condition yaratır. */
static void bf_cache_olustur(float d) {
    for (int fi = 0; fi < BF_SWEEP_NOKTA_SAYISI; fi++) {
        float frek = BF_SWEEP_MIN_HZ + fi * BF_SWEEP_ADIM_HZ;
        for (int ai = 0; ai < BF_AZIMUTH_NOKTA_SAYISI; ai++) {
            float az = BF_AZIMUTH_MIN_DEG + ai * BF_TARAMA_ADIM_DEG;
            for (int ei = 0; ei < BF_ELEVATION_NOKTA_SAYISI; ei++) {
                float el = BF_ELEVATION_MIN_DEG + ei * BF_TARAMA_ADIM_DEG;
                faz_hesapla_ic(az, el, frek, d, faz_cache[fi][ai][ei].faz_deg);
            }
        }
    }
    bf_log(BF_LOG_INFO,
           "Cache hazir: d=%.4fm, %d frekans x %d azimuth x %d elevation = %d kombinasyon",
           d, BF_SWEEP_NOKTA_SAYISI, BF_AZIMUTH_NOKTA_SAYISI, BF_ELEVATION_NOKTA_SAYISI,
           BF_SWEEP_NOKTA_SAYISI * BF_AZIMUTH_NOKTA_SAYISI * BF_ELEVATION_NOKTA_SAYISI);
}

/* ─── Phase Shift Yazma ──────────────────────────────────────────────────── */

static int fazlari_yaz(const float fazlar[BF_ELEMAN_SAYISI]) {
    /* Otomatik denge ofseti algoritmasıyla 7 fazı birden uygula.
     * Phase_shift modülü kör nokta yuvarlaması ve ortak ofset bulma işini halleder. */
    float gercek_aciler[BF_ELEMAN_SAYISI];
    int ret = ps_set_all_angles(fazlar, gercek_aciler);

    if (ret != PS_OK) {
        bf_log(BF_LOG_WARN, "ps_set_all_angles hata kodu: %d", ret);
        return BF_ERR_PHASE;
    }
    return BF_OK;
}
/* ─── Tarama Thread ──────────────────────────────────────────────────────── */

/* Tüm (azimuth × elevation × frekans) kombinasyonlarını sırayla tarar.
 * Her adımda phase_shift modülü güncellenir ve kayıtlı callback çağrılır.
 * Tarama sonu otomatik olarak başa döner (continuous mode).                  */
static void *tarama_thread_fonk(void *arg) {
    (void)arg;

    /* Realtime öncelik — root yetkisi gerektirir:
     * struct sched_param sp = { .sched_priority = 65 };
     * pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp); */

    bf_log(BF_LOG_INFO, "Tarama thread basladi (%d x %d x %d = %d kombinasyon/dongü)",
           BF_AZIMUTH_NOKTA_SAYISI, BF_ELEVATION_NOKTA_SAYISI, BF_SWEEP_NOKTA_SAYISI,
           BF_AZIMUTH_NOKTA_SAYISI * BF_ELEVATION_NOKTA_SAYISI * BF_SWEEP_NOKTA_SAYISI);

    while (bf.tarama_aktif) {
        /* Dwell ve callback snapshot'ını döngü başında al */
        pthread_mutex_lock(&bf.mutex);
        int   dwell_ms = bf.tarama_dwell_ms;
        void (*cb)(int, int, int) = bf.tarama_callback;
        pthread_mutex_unlock(&bf.mutex);

        for (int ai = 0; ai < BF_AZIMUTH_NOKTA_SAYISI && bf.tarama_aktif; ai++) {
            for (int ei = 0; ei < BF_ELEVATION_NOKTA_SAYISI && bf.tarama_aktif; ei++) {
                for (int fi = 0; fi < BF_SWEEP_NOKTA_SAYISI && bf.tarama_aktif; fi++) {

                    /* Cache'ten fazları al ve phase_shift modülüne yaz */
                    int ret = fazlari_yaz(faz_cache[fi][ai][ei].faz_deg);
                    if (ret == BF_OK) {
                        pthread_mutex_lock(&bf.mutex);
                        bf.aktif_azimuth_deg   = BF_AZIMUTH_MIN_DEG   + ai * BF_TARAMA_ADIM_DEG;
                        bf.aktif_elevation_deg = BF_ELEVATION_MIN_DEG + ei * BF_TARAMA_ADIM_DEG;
                        pthread_mutex_unlock(&bf.mutex);
                    } else {
                        bf_log(BF_LOG_WARN, "Faz yazma hatasi: az=%d el=%d frek=%d", ai, ei, fi);
                    }

                    /* Callback: hangi kombinasyonda olduğumuzu dış modüle bildir */
                    if (cb) cb(ai, ei, fi);

                    /* Dwell: bu yön+frekans kombinasyonunda bekleme süresi */
                    struct timespec hedef;
                    clock_gettime(CLOCK_MONOTONIC, &hedef);
                    timespec_ekle_ms(&hedef, dwell_ms);
                    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &hedef, NULL);
                }
            }
        }
    }

    bf_log(BF_LOG_INFO, "Tarama thread durdu");
    return NULL;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* Mutex başlatır, faz cache'ini hesaplar ve modülü kullanıma hazır hâle getirir.
 * Cache hesaplama ~1–2 saniye sürebilir; program başlangıcında çağrılmalı.  */
int bf_init(void) {
    if (bf.baslandi) {
        bf_log(BF_LOG_WARN, "bf_init zaten cagirildi");
        return BF_OK;
    }

    memset(&bf, 0, sizeof(bf));
    bf.eleman_mesafesi_m   = BF_ELEMAN_MESAFESI_M;
    bf.aktif_azimuth_deg   = 0.0f;
    bf.aktif_elevation_deg = 0.0f;
    bf.tarama_dwell_ms     = 100;  /* Varsayılan dwell: 100 ms */

    if (pthread_mutex_init(&bf.mutex, NULL) != 0) {
        bf_log(BF_LOG_ERROR, "Mutex baslatilamadi");
        return BF_ERR_INIT;
    }

    bf_log(BF_LOG_INFO, "Faz cache'i hesaplaniyor (%d kombinasyon)...",
           BF_SWEEP_NOKTA_SAYISI * BF_AZIMUTH_NOKTA_SAYISI * BF_ELEVATION_NOKTA_SAYISI);
    bf_cache_olustur(bf.eleman_mesafesi_m);

    bf.baslandi = 1;
    bf_log(BF_LOG_INFO,
           "Beamforming hazir — %d eleman, d=%.3fm, lambda=%.3fm @ %.0fHz",
           BF_ELEMAN_SAYISI, bf.eleman_mesafesi_m,
           BF_SES_HIZI_M_S / BF_VARSAYILAN_FREKANS_HZ, BF_VARSAYILAN_FREKANS_HZ);
    return BF_OK;
}

/* Tarama thread'ini durdurur ve mutex'i serbest bırakır. */
void bf_cleanup(void) {
    if (!bf.baslandi) return;
    bf_log(BF_LOG_INFO, "Beamforming temizleniyor");

    bf_tarama_durdur();

    pthread_mutex_destroy(&bf.mutex);
    bf.baslandi = 0;
    bf_log(BF_LOG_INFO, "Beamforming temizlendi");
}

/* Belirli bir (azimuth, elevation, frekans) için fazları anlık hesaplayıp
 * phase_shift modülüne yazar. Cache kullanmaz — her çağrıda formülden hesaplar.
 * Sınır dışı parametre → BF_ERR_PARAM.                                      */
int bf_set_direction(float azimuth_deg, float elevation_deg, float frekans_hz) {
    if (!bf.baslandi)                                                            return BF_ERR_INIT;
    if (azimuth_deg   < BF_AZIMUTH_MIN_DEG   || azimuth_deg   > BF_AZIMUTH_MAX_DEG)   return BF_ERR_PARAM;
    if (elevation_deg < BF_ELEVATION_MIN_DEG || elevation_deg > BF_ELEVATION_MAX_DEG) return BF_ERR_PARAM;
    if (frekans_hz    < BF_SWEEP_MIN_HZ      || frekans_hz    > BF_SWEEP_MAX_HZ)      return BF_ERR_PARAM;

    pthread_mutex_lock(&bf.mutex);
    float d = bf.eleman_mesafesi_m;
    pthread_mutex_unlock(&bf.mutex);

    float fazlar[BF_ELEMAN_SAYISI];
    faz_hesapla_ic(azimuth_deg, elevation_deg, frekans_hz, d, fazlar);

    int ret = fazlari_yaz(fazlar);

    pthread_mutex_lock(&bf.mutex);
    bf.aktif_azimuth_deg   = azimuth_deg;
    bf.aktif_elevation_deg = elevation_deg;
    pthread_mutex_unlock(&bf.mutex);

    if (ret == BF_OK)
        bf_log(BF_LOG_DEBUG, "Yon: az=%.1f° el=%.1f° f=%.0fHz",
               azimuth_deg, elevation_deg, frekans_hz);
    return ret;
}

/* Cache index'i ile doğrudan faz tablosuna erişir; hesaplama veya arama gerektirmez.
 * Tarama döngüsü için optimize edilmiştir (O(1) erişim).                    */
int bf_set_direction_indexed(int azimuth_idx, int elevation_idx, int frekans_idx) {
    if (!bf.baslandi)                                                      return BF_ERR_INIT;
    if (azimuth_idx   < 0 || azimuth_idx   >= BF_AZIMUTH_NOKTA_SAYISI)   return BF_ERR_PARAM;
    if (elevation_idx < 0 || elevation_idx >= BF_ELEVATION_NOKTA_SAYISI) return BF_ERR_PARAM;
    if (frekans_idx   < 0 || frekans_idx   >= BF_SWEEP_NOKTA_SAYISI)     return BF_ERR_PARAM;

    int ret = fazlari_yaz(faz_cache[frekans_idx][azimuth_idx][elevation_idx].faz_deg);

    pthread_mutex_lock(&bf.mutex);
    bf.aktif_azimuth_deg   = BF_AZIMUTH_MIN_DEG   + azimuth_idx   * BF_TARAMA_ADIM_DEG;
    bf.aktif_elevation_deg = BF_ELEVATION_MIN_DEG + elevation_idx * BF_TARAMA_ADIM_DEG;
    pthread_mutex_unlock(&bf.mutex);

    return ret;
}

/* Tarama thread'ini başlatır.
 * dwell_ms ≤ 0 ise varsayılan (100 ms) kullanılır.
 * Zaten aktifse uyarı loglanır ve BF_OK döner.                              */
int bf_tarama_baslat(int dwell_ms) {
    if (!bf.baslandi) return BF_ERR_INIT;
    if (bf.tarama_aktif) {
        bf_log(BF_LOG_WARN, "Tarama zaten aktif");
        return BF_OK;
    }

    pthread_mutex_lock(&bf.mutex);
    bf.tarama_dwell_ms = (dwell_ms > 0) ? dwell_ms : 100;
    pthread_mutex_unlock(&bf.mutex);

    bf.tarama_aktif = 1;
    if (pthread_create(&bf.tarama_thread, NULL, tarama_thread_fonk, NULL) != 0) {
        bf_log(BF_LOG_ERROR, "Tarama thread olusturulamadi");
        bf.tarama_aktif = 0;
        return BF_ERR_THREAD;
    }
    bf.tarama_thread_olusturuldu = 1;

    bf_log(BF_LOG_INFO, "Tarama basladi: dwell=%d ms, %d kombinasyon/dongü",
           bf.tarama_dwell_ms,
           BF_AZIMUTH_NOKTA_SAYISI * BF_ELEVATION_NOKTA_SAYISI * BF_SWEEP_NOKTA_SAYISI);
    return BF_OK;
}

/* Tarama thread'ini durdurur ve join eder. Aktif değilse sessizce döner.    */
void bf_tarama_durdur(void) {
    if (!bf.tarama_thread_olusturuldu) return;
    bf.tarama_aktif = 0;
    pthread_join(bf.tarama_thread, NULL);
    bf.tarama_thread_olusturuldu = 0;
    bf_log(BF_LOG_INFO, "Tarama durduruldu");
}

/* Her tarama adımında çağrılacak fonksiyonu kaydeder.
 * NULL geçilirse önceki kayıt silinir.                                       */
void bf_tarama_callback_kaydet(void (*callback)(int az_idx, int el_idx, int frek_idx)) {
    pthread_mutex_lock(&bf.mutex);
    bf.tarama_callback = callback;
    pthread_mutex_unlock(&bf.mutex);
}

/* Son ayarlanan yönü döndürür. NULL pointer'lar sessizce atlanır.           */
void bf_get_aktif_yon(float *azimuth_deg, float *elevation_deg) {
    pthread_mutex_lock(&bf.mutex);
    if (azimuth_deg)   *azimuth_deg   = bf.aktif_azimuth_deg;
    if (elevation_deg) *elevation_deg = bf.aktif_elevation_deg;
    pthread_mutex_unlock(&bf.mutex);
}

/* Eleman sayısını döndürür (derleme zamanı sabiti). */
int bf_get_eleman_sayisi(void) {
    return BF_ELEMAN_SAYISI;
}

/* Belirtilen elemanın metre cinsinden gerçek koordinatlarını döndürür.
 * Geçersiz eleman indeksi için (0, 0) döner.                                */
void bf_get_eleman_konum(int eleman, float *x_m, float *y_m) {
    if (eleman < 0 || eleman >= BF_ELEMAN_SAYISI) {
        if (x_m) *x_m = 0.0f;
        if (y_m) *y_m = 0.0f;
        return;
    }
    pthread_mutex_lock(&bf.mutex);
    float d = bf.eleman_mesafesi_m;
    pthread_mutex_unlock(&bf.mutex);

    if (x_m) *x_m = eleman_konumlari[eleman].x * d;
    if (y_m) *y_m = eleman_konumlari[eleman].y * d;
}

/* Verilen frekans için hava ses dalgaboyunu hesaplar (λ = c / f).
 * Geçersiz frekans için -1.0 döner.                                         */
float bf_get_lambda(float frekans_hz) {
    if (frekans_hz <= 0.0f) return -1.0f;
    return BF_SES_HIZI_M_S / frekans_hz;
}

/*
 * Belirli bir azimuth, elevation ve frekans için 7 alıcının faz değerlerini hesaplar.
 * Delay-and-sum beamforming klasik formülü kullanılır. Sonuç 0–360° aralığındadır.
 * Cache yerine anlık hesaplama — kalibrasyon ve doğrulama için idealdir.
 */
int bf_hesapla_fazlar(float az_deg, float el_deg, float frek_hz,
                      float fazlar_out[BF_ELEMAN_SAYISI]) {
    if (az_deg  < BF_AZIMUTH_MIN_DEG   || az_deg  > BF_AZIMUTH_MAX_DEG)   return BF_ERR_PARAM;
    if (el_deg  < BF_ELEVATION_MIN_DEG || el_deg  > BF_ELEVATION_MAX_DEG) return BF_ERR_PARAM;
    if (frek_hz < BF_SWEEP_MIN_HZ      || frek_hz > BF_SWEEP_MAX_HZ)      return BF_ERR_PARAM;
    if (!fazlar_out)                                                         return BF_ERR_PARAM;

    pthread_mutex_lock(&bf.mutex);
    float d = bf.eleman_mesafesi_m;
    pthread_mutex_unlock(&bf.mutex);

    faz_hesapla_ic(az_deg, el_deg, frek_hz, d, fazlar_out);
    return BF_OK;
}

/* Eleman aralığını günceller ve faz cache'ini yeniden hesaplar.
 * Tarama thread'i otomatik durdurulur — cache yazma sırasında race condition önlenir.
 * Kalibrasyon sonrası çağrılmalı; tarama yeniden başlatılması kullanıcıya bırakılır. */
void bf_set_eleman_mesafesi(float d_metre) {
    if (d_metre <= 0.0f) {
        bf_log(BF_LOG_WARN, "Gecersiz eleman mesafesi: %.4fm", d_metre);
        return;
    }

    /* Cache yeniden hesaplanacağından tarama thread'ini önce durdur */
    bf_tarama_durdur();

    pthread_mutex_lock(&bf.mutex);
    bf.eleman_mesafesi_m = d_metre;
    pthread_mutex_unlock(&bf.mutex);

    bf_log(BF_LOG_INFO, "Eleman mesafesi: %.4fm, cache yeniden hesaplaniyor...", d_metre);
    bf_cache_olustur(d_metre);
}

/* Mevcut eleman aralığını döndürür. */
float bf_get_eleman_mesafesi(void) {
    pthread_mutex_lock(&bf.mutex);
    float d = bf.eleman_mesafesi_m;
    pthread_mutex_unlock(&bf.mutex);
    return d;
}

/* Log seviyesini ayarlar. Varsayılan: BF_LOG_INFO. */
void bf_set_log_level(BF_LogLevel level) {
    log_seviyesi = level;
}

/* Scan loop'un iterasyon sınırlarını ve adımını çalışma anında günceller.
 * Parametre doğrulama:
 *   - az_max, el_max: (0, BF_CACHE_RANGE_DEG] aralığında olmalı
 *   - step_deg: en az BF_CACHE_STEP_DEG, tam sayıya yuvarlanır
 *   - az_max ve el_max step'in tam katı olmalı (tolerans 1e-6)
 * Geçersiz parametre → stderr log + -1 dönüş.                               */
int bf_set_scan_range(double az_max_deg, double el_max_deg, double step_deg) {
    /* Integer doğrulama — float comparison'dan kaçın */
    int istep = (int)round(step_deg);
    int iaz   = (int)round(az_max_deg);
    int iel   = (int)round(el_max_deg);

    if (istep < 1) {
        fprintf(stderr, "[bf] scan_range: step=%d gecersiz (min 1)\n", istep);
        return -1;
    }
    if (iaz <= 0 || iaz > (int)BF_CACHE_RANGE_DEG) {
        fprintf(stderr, "[bf] scan_range: az_max=%d aralik disi (1..%d)\n",
                iaz, (int)BF_CACHE_RANGE_DEG);
        return -1;
    }
    if (iel <= 0 || iel > (int)BF_CACHE_RANGE_DEG) {
        fprintf(stderr, "[bf] scan_range: el_max=%d aralik disi (1..%d)\n",
                iel, (int)BF_CACHE_RANGE_DEG);
        return -1;
    }
    if (iaz % istep != 0) {
        fprintf(stderr, "[bf] scan_range: az_max=%d step=%d'in tam kati degil\n",
                iaz, istep);
        return -1;
    }
    if (iel % istep != 0) {
        fprintf(stderr, "[bf] scan_range: el_max=%d step=%d'in tam kati degil\n",
                iel, istep);
        return -1;
    }

    s_az_max = (double)iaz;
    s_el_max = (double)iel;
    s_step   = (double)istep;

    fprintf(stderr, "[bf] scan_range set: +-%d deg / +-%d deg, step %d deg\n",
            iaz, iel, istep);
    return 0;
}

/* Aktif tarama sınırlarını pointer'lar üzerinden döndürür.
 * NULL pointer'lar sessizce atlanır.                                         */
void bf_get_scan_range(double *az_max, double *el_max, double *step) {
    if (az_max) *az_max = s_az_max;
    if (el_max) *el_max = s_el_max;
    if (step)   *step   = s_step;
}
