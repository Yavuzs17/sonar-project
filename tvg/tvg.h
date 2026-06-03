#ifndef TVG_H
#define TVG_H

#include <stdint.h>

/* ─── INA821 Kazanç Formülü Sabitleri ──────────────────────────────────────
 * R_G = TVG_R_SABIT_OHM + TVG_R_STEP_OHM × N
 * G   = 1 + TVG_INA_KATSAYI / R_G                                         */
#define TVG_R_SABIT_OHM    1075.0f    /* 1kΩ sabit direnç + wiper direnci (75Ω)  */
#define TVG_R_STEP_OHM     39.0625f   /* MCP4651 10kΩ için step başına direnç    */
#define TVG_INA_KATSAYI    49400.0f   /* INA821 datasheet katsayısı              */
#define TVG_GAIN_FORMUL(N) \
    (1.0f + TVG_INA_KATSAYI / (TVG_R_SABIT_OHM + TVG_R_STEP_OHM * (float)(N)))

/* ─── I2C / Çip Tanımları ───────────────────────────────────────────────────
 * I2C BUS PAYLAŞIMI:
 *   TVG         → bus 1 (donanımsal I2C-1, GPIO 2 SDA / GPIO 3 SCL)
 *   Preamp      → bus 1 (donanımsal I2C-1)
 *   Phase_shift → bus 1 (donanımsal I2C-1)
 *   PLL         → bus 3 (software I2C, GPIO 4/5)
 *
 * İleride çakışmaları önlemek için her modülün ortak bir
 * extern pthread_mutex_t i2c_bus_mutex kullanması önerilir.
 * Şimdilik her modül kendi mutex'ini bağımsız yönetir.                     */
#define TVG_I2C_BUS         1      /* Raspberry Pi donanımsal I2C-1              */
#define TVG_MCP4651_ADDR    0x2F   /* A2A1A0 = 111                              */
#define TVG_MCP4651_WIPER   1      /* Wiper1 (2. kanal); Wiper0 phase_shift'te  */
#define TVG_I2C_MAX_DENEME  3      /* Başarısız I2C için yeniden deneme sayısı  */

/* ─── Kazanç Sınır Değerleri ────────────────────────────────────────────────
 * Formülden hesaplanmıştır. Runtime'da tvg_kalibre_* ile değiştirilebilir. */
#define TVG_GAIN_DB_MIN    14.78f  /* N=255 için kazanç (dB) — minimum kazanç  */
#define TVG_GAIN_DB_MAX    33.43f  /* N=0 için kazanç (dB) — maksimum kazanç   */

/* ─── Rampa Parametreleri ───────────────────────────────────────────────────*/
#define TVG_RAMPA_MS       30                   /* Toplam rampa süresi (ms)     */
#define TVG_LUT_BOYUT      (TVG_RAMPA_MS + 1)  /* 0–30 ms → 31 nokta           */

/* ─── Hata Kodları ──────────────────────────────────────────────────────────*/
#define TVG_OK              0
#define TVG_ERR_INIT       -1   /* Başlatma hatası                              */
#define TVG_ERR_I2C        -2   /* I2C iletişim hatası                          */
#define TVG_ERR_PARAM      -3   /* Geçersiz parametre                           */
#define TVG_ERR_LUT_ARALIK -4   /* t_ms LUT aralığı dışında                    */
#define TVG_ERR_DOSYA      -5   /* Dosya açılamadı / okunamadı                  */
#define TVG_ERR_THREAD     -6   /* Thread oluşturma hatası                      */

/* ─── Log Seviyeleri ────────────────────────────────────────────────────────*/
typedef enum {
    TVG_LOG_DEBUG = 0,
    TVG_LOG_INFO,
    TVG_LOG_WARN,
    TVG_LOG_ERROR
} TVG_LogLevel;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* Başlatma / Temizleme */
int  tvg_init(void);
void tvg_cleanup(void);

/* Manuel Durum Kontrolü (kalibrasyon için) */
int   tvg_set_step(uint8_t N);
int   tvg_get_step(uint8_t *N_out);
float tvg_get_gain_db(void);
float tvg_get_gain_linear(void);

/* PLL Callback'leri — PLL modülü bu fonksiyonları çağırır */
void tvg_on_burst_start(void);   /* INHIBIT LOW: N=255 yaz (min kazanç)         */
void tvg_on_listen_start(void);  /* INHIBIT HIGH: rampa thread tetikle          */
void tvg_rampa_durdur(void);     /* Rampa thread'ini iptal et (manuel mod)      */

/* LUT Yönetimi */
int  tvg_lut_set(int t_ms, uint8_t N);
int  tvg_lut_get(int t_ms, uint8_t *N_out);
void tvg_lut_yeniden_hesapla(void);          /* Formülden LUT'u yeniden üret    */
int  tvg_lut_kaydet_dosya(const char *path); /* CSV formatı: t_ms,N             */
int  tvg_lut_yukle_dosya(const char *path);

/* Kalibrasyon — runtime'da min/max kazancı güncelle, LUT otomatik yenilenir */
int tvg_kalibre_min_db(float yeni_dB);
int tvg_kalibre_max_db(float yeni_dB);

/* Güvenlik Sınırı — N bu değerin altına inemez (kazanç sınırlanır) */
void    tvg_set_max_step(uint8_t max_N);  /* 0 = sınırsız (varsayılan)         */
uint8_t tvg_get_max_step(void);

/* Log Seviyesi */
void tvg_set_log_level(TVG_LogLevel level);

#endif /* TVG_H */
