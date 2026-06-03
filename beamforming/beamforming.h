#ifndef BEAMFORMING_H
#define BEAMFORMING_H

/* ─── Fiziksel Parametreler ─────────────────────────────────────────────────
 * Eleman mesafesi üretim sonrası ölçülerek bf_set_eleman_mesafesi() ile
 * güncellenebilir; BF_ELEMAN_MESAFESI_M yalnızca başlangıç varsayılanıdır.  */
#define BF_ELEMAN_SAYISI            7
#define BF_SES_HIZI_M_S             343.0f    /* Havada ses hızı (20°C, kuru hava)   */
#define BF_VARSAYILAN_FREKANS_HZ    40000.0f  /* VCO nominal çalışma frekansı        */
#define BF_ELEMAN_MESAFESI_M        0.018f    /* 18 mm ≈ 2.1λ @ 40kHz (ölçülecek)  */

/* ─── Tarama Parametreleri ──────────────────────────────────────────────────
 * 13 × 13 = 169 yön noktası (-30° −25° … +25° +30°)                        */
#define BF_AZIMUTH_MIN_DEG          -30.0f
#define BF_AZIMUTH_MAX_DEG           30.0f
#define BF_ELEVATION_MIN_DEG        -30.0f
#define BF_ELEVATION_MAX_DEG         30.0f
#define BF_TARAMA_ADIM_DEG            5.0f
#define BF_AZIMUTH_NOKTA_SAYISI      13       /* (30 − (−30)) / 5 + 1 */
#define BF_ELEVATION_NOKTA_SAYISI    13

/* ─── Sweep (Frekans Tarama) Parametreleri ──────────────────────────────────
 * 21 frekans noktası (39–41 kHz, 100 Hz adım)                               */
#define BF_SWEEP_MIN_HZ             39000.0f
#define BF_SWEEP_MAX_HZ             41000.0f
#define BF_SWEEP_ADIM_HZ              100.0f
#define BF_SWEEP_NOKTA_SAYISI          21     /* (41000 − 39000) / 100 + 1 */

/* ─── Hata Kodları ──────────────────────────────────────────────────────────*/
#define BF_OK           0
#define BF_ERR_INIT    -1   /* Modül başlatılmamış veya başlatma hatası      */
#define BF_ERR_PARAM   -2   /* Aralık dışı açı veya frekans                  */
#define BF_ERR_PHASE   -3   /* phase_shift_set_angle hata döndürdü           */
#define BF_ERR_THREAD  -4   /* Thread oluşturma hatası                       */
#define BF_ERR_BELLEK  -5   /* Bellek hatası (ilerideki kullanım için)       */

/* ─── Log Seviyeleri ────────────────────────────────────────────────────────*/
typedef enum {
    BF_LOG_DEBUG = 0,
    BF_LOG_INFO,
    BF_LOG_WARN,
    BF_LOG_ERROR
} BF_LogLevel;

/* ─── Veri Yapıları ─────────────────────────────────────────────────────────*/

/* Eleman normalize koordinatı; gerçek metre = koord × eleman_mesafesi_m     */
typedef struct {
    float x;
    float y;
} BF_ElemanKonum;

/* Tek bir (azimuth, elevation, frekans) kombinasyonu için 7 alıcı fazı (°)  */
typedef struct {
    float faz_deg[BF_ELEMAN_SAYISI];
} BF_FazSeti;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* Başlatma / Temizleme */
int  bf_init(void);
void bf_cleanup(void);

/* Manuel Yön Ayarlama */
int  bf_set_direction(float azimuth_deg, float elevation_deg, float frekans_hz);
int  bf_set_direction_indexed(int azimuth_idx, int elevation_idx, int frekans_idx);

/* Otomatik Tarama */
int  bf_tarama_baslat(int dwell_ms);
void bf_tarama_durdur(void);
void bf_tarama_callback_kaydet(void (*callback)(int az_idx, int el_idx, int frek_idx));

/* Sorgu Fonksiyonları */
void  bf_get_aktif_yon(float *azimuth_deg, float *elevation_deg);
int   bf_get_eleman_sayisi(void);
void  bf_get_eleman_konum(int eleman, float *x_m, float *y_m);
float bf_get_lambda(float frekans_hz);

/* Hesaplama Yardımcısı — kalibrasyon ve test için anlık faz hesaplama */
int bf_hesapla_fazlar(float az_deg, float el_deg, float frek_hz,
                      float fazlar_out[BF_ELEMAN_SAYISI]);

/* Parametre Güncelleme */
void  bf_set_eleman_mesafesi(float d_metre);
float bf_get_eleman_mesafesi(void);

/* Log Seviyesi */
void bf_set_log_level(BF_LogLevel level);

/* ─── Runtime Tarama Aralığı ────────────────────────────────────────────────
 * Cache derleme zamanında ±BF_AZIMUTH_RANGE derecede 5° adımla kurulur;
 * bu fonksiyonlar yalnızca scan loop iterasyon aralığını ve adımını değiştirir.
 * step_deg en az 1° olmalı, az_max ve el_max step'in tam katı olmalı.        */
int  bf_set_scan_range(double az_max_deg, double el_max_deg, double step_deg);
/* 0 = OK, -1 = geçersiz parametre */

void bf_get_scan_range(double *az_max, double *el_max, double *step);

#endif /* BEAMFORMING_H */
