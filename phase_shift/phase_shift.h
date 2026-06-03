#ifndef PHASE_SHIFT_H
#define PHASE_SHIFT_H

#include <stdbool.h>
#include <stdint.h>

/* ─── I2C / Çip Tanımları ───────────────────────────────────────────────────
 * BUS PAYLAŞIMI NOTU:
 * Bu modül preamp ve tvg modülleriyle aynı I2C-1 hattını paylaşır.
 * İleride çakışmaları önlemek için ortak bir
 * extern pthread_mutex_t i2c_bus_mutex kullanılması önerilir.
 * Şimdilik bu modül kendi ps_mutex'ini bağımsız yönetir.                    */
#define PS_I2C_BUS           1     /* Donanımsal I2C-1 (GPIO 2 SDA / GPIO 3 SCL) */
#define PS_I2C_MAX_DENEME    3     /* Başarısız I2C için yeniden deneme sayısı   */

/* MCP4651 dijital pot adresleri — APF faz kontrolü */
#define PS_MCP_CIP_5_ADDR   0x2C  /* A2A1A0=100 → Alıcı 1 (W0), Alıcı 2 (W1) */
#define PS_MCP_CIP_6_ADDR   0x2D  /* A2A1A0=101 → Alıcı 3 (W0), Alıcı 4 (W1) */
#define PS_MCP_CIP_7_ADDR   0x2E  /* A2A1A0=110 → Alıcı 5 (W0), Alıcı 6 (W1) */
#define PS_MCP_CIP_8_ADDR   0x2F  /* A2A1A0=111 → Alıcı 7 (W0), TVG W1 (dokunma) */

/* PCF8574 GPIO genişletici — inverter anahtarları */
#define PS_PCF_ADDR          0x20  /* A2A1A0=000; P0..P6 → Alıcı 1..7         */

/* ─── APF Formül Sabitleri ──────────────────────────────────────────────────
 * Teorik formül: faz(N,f) = -2 × atan(2π × f × R(N) × C)  (radyan)
 * R(N) = PS_R_WIPER_OHM + PS_R_STEP_OHM × N
 * Gerçek C değeri kalibrasyon ile fit edilir.                               */
#define PS_R_WIPER_OHM        75.0f       /* Wiper direnç (Ω)                 */
#define PS_R_STEP_OHM         39.0625f    /* MCP4651 step başına direnç (Ω)   */
#define PS_APF_KAPASITOR_F    1.0e-9f     /* 1nF — kalibrasyon ile düzeltilir */
#define PS_VARSAYILAN_FREK_HZ 40000.0f    /* APF hesaplama frekansı (Hz)      */

/* Teorik kapsama aralıkları (kalibre edilmemiş başlangıç değerleri)          */
#define PS_APF_OFF_MIN_DEG   -7.0f        /* Inverter OFF: minimum faz (°)    */
#define PS_APF_OFF_MAX_DEG  -166.3f       /* Inverter OFF: maksimum faz (°)   */
#define PS_APF_ON_MIN_DEG   -187.0f       /* Inverter ON:  minimum faz (°)    */
#define PS_APF_ON_MAX_DEG   -346.3f       /* Inverter ON:  maksimum faz (°)   */

/* ─── Alıcı Sayısı ──────────────────────────────────────────────────────────*/
#define PS_ALICI_SAYISI      7

/* ─── Hata Kodları ──────────────────────────────────────────────────────────*/
#define PS_OK                0
#define PS_ERR_INIT         -1   /* Başlatma hatası                           */
#define PS_ERR_I2C          -2   /* I2C iletişim hatası                       */
#define PS_ERR_KANAL        -3   /* Geçersiz alıcı numarası (0–6 dışı)        */
#define PS_ERR_KALIB_YOK    -4   /* Alıcı kalibre edilmemiş (nominal formül)  */
#define PS_ERR_KOR_NOKTA    -5   /* Hedef açı kör noktada; en yakın uygulandı */
#define PS_ERR_FIT          -6   /* Formül fit edilemedi (yetersiz nokta vb.)  */
#define PS_ERR_DOSYA        -7   /* Dosya açılamadı / okunamadı               */
#define PS_ERR_PARAM        -8   /* Geçersiz parametre                        */

/* ─── Log Seviyeleri ────────────────────────────────────────────────────────*/
typedef enum {
    PS_LOG_DEBUG = 0,
    PS_LOG_INFO,
    PS_LOG_WARN,
    PS_LOG_ERROR
} PS_LogLevel;

/* ─── Kalibrasyon Veri Yapıları ─────────────────────────────────────────────*/

/* Tek bir ölçüm noktası: osiloskopla ölçülmüş (step, inverter) → gerçek faz */
typedef struct {
    uint8_t step;
    bool    inverter_on;
    float   olculen_aci;   /* Osiloskopla ölçülen gerçek faz (derece)        */
} PS_KalibPunto;

#define PS_KALIB_NOKTA_MAX 16

/* Bir alıcının kalibrasyon durumu ve fit edilmiş parametreler                */
typedef struct {
    PS_KalibPunto noktalar[PS_KALIB_NOKTA_MAX];
    int           nokta_sayisi;

    /* Fit edilmiş parametreler — ps_kalib_fit() sonrası geçerli             */
    float kapasitor_f;      /* Gerçek C değeri (tolerans düzeltmeli, F)      */
    float ofset_apf_off;    /* Inverter OFF için ek faz ofseti (derece)      */
    float ofset_apf_on;     /* Inverter ON için ek faz ofseti (derece)       */

    /* Çalışma sınırları — kör nokta tespiti için ölçülen değerler           */
    float min_aci_off;      /* Inverter OFF: en küçük negatif faz (°, ≈ -7°) */
    float max_aci_off;      /* Inverter OFF: en büyük negatif faz (°, ≈ -166°) */
    float min_aci_on;       /* Inverter ON:  en küçük negatif faz (°, ≈ -187°) */
    float max_aci_on;       /* Inverter ON:  en büyük negatif faz (°, ≈ -346°) */

    bool kalibre_edildi;    /* ps_kalib_fit() başarıyla tamamlandı mı         */
} PS_AliciKalib;

/* Tek bir alıcı için bulunan (step, inverter) ayarı ve sapma bilgisi        */
typedef struct {
    uint8_t step;
    bool    inverter_on;
    float   gercek_aci;    /* Bu ayarın gerçekte üreteceği faz (0–360°)     */
    float   sapma_deg;     /* Hedeften açısal fark (mutlak değer, derece)   */
} PS_AyarSonuc;

/* Toplu ayarlama için denge ofseti ve tüm alıcı ayarları                   */
typedef struct {
    float        ofset_deg;                        /* Bulunan en iyi ofset   */
    float        toplam_sapma;                     /* Tüm alıcıların sapma Σ */
    PS_AyarSonuc ayarlar[PS_ALICI_SAYISI];         /* Her alıcının seçilen ayarı */
} PS_DengeAyari;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* Başlatma / Temizleme */
int  ps_init(void);
void ps_cleanup(void);

/* Düşük Seviye Kontrol (kalibrasyon ve test için) */
int ps_set_step(int alici, uint8_t step, bool inverter_on);
int ps_get_step(int alici, uint8_t *step_out, bool *inverter_out);

/* Yüksek Seviye Kontrol (beamforming için) */
int ps_set_angle(int alici, float hedef_aci, float *gercek_aci_out);

/* Toplu Ayarlama — otomatik denge ofseti uygular */
int ps_set_all_angles(const float hedef_aciler[PS_ALICI_SAYISI],
                      float gercek_aciler_out[PS_ALICI_SAYISI]);

/* Kalibrasyon Yönetimi */
int  ps_kalib_nokta_ekle(int alici, uint8_t step, bool inverter_on, float olculen_aci);
int  ps_kalib_nokta_temizle(int alici);
int  ps_kalib_fit(int alici);
int  ps_kalib_kaydet_dosya(const char *path);
int  ps_kalib_yukle_dosya(const char *path);

/* Kalibrasyon Sınırları (kör nokta tanımı) */
int ps_kalib_set_sinir(int alici, bool inverter_on, float min_aci, float max_aci);
int ps_kalib_get_sinir(int alici, bool inverter_on, float *min_out, float *max_out);

/* Sorgu / Bilgi */
int   ps_get_kalib_durum(int alici, bool *kalibre_out, int *nokta_sayisi_out);
float ps_teorik_aci(uint8_t step, float frekans_hz);

/* Beamforming uyumluluk fonksiyonu — beamforming.c extern bildirimiyle çağırır */
int phase_shift_set_angle(int alici, float derece);

/* Log Seviyesi */
void ps_set_log_level(PS_LogLevel level);

#endif /* PHASE_SHIFT_H */
