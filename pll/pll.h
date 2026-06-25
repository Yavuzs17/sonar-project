#ifndef PLL_H
#define PLL_H

/* ─── Pin Tanımları ─────────────────────────────────────────────────────────
 * BCM numaraları. Değiştirmek için yalnızca bu define'ları güncelle.        */
#define FEEDBACK_PIN    17   /* Feedback kare dalga çıkışı                   */
#define INHIBIT_PIN     27   /* VCO inhibit/burst çıkışı                     */

/* ─── ADC / I2C Tanımları ───────────────────────────────────────────────────*/
#define MCP3221_ADDR    0x4D  /* MCP3221 sabit I2C adresi                    */
/*
 * I2C BUS DAĞITIMI:
 *   - PLL modülü       → bus 3 (software I2C, GPIO 4 SDA / GPIO 5 SCL)
 *   - Preamp modülü    → bus 1 (donanımsal I2C-1, GPIO 2 SDA / GPIO 3 SCL)
 *   - Phase_shift mod. → bus 1 (donanımsal I2C-1, GPIO 2 SDA / GPIO 3 SCL)
 *
 * Software I2C aktivasyonu için /boot/config.txt dosyasına şu satırı ekleyin:
 *   dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=4,i2c_gpio_scl=5
 * Sonrasında Raspberry Pi'yi yeniden başlatın.
 */
#define MCP3221_BUS     3     /* Software I2C hattı (GPIO 4 SDA, GPIO 5 SCL)  */
                              /* Donanımsal I2C-1 (GPIO 2/3) preamp ve        */
                              /* phase_shift modülleri için ayrıldı.          */

/* ─── Voltaj Tanımları ──────────────────────────────────────────────────────
 * Gerilim bölücü oranı değişirse yalnızca VREF_DEVRE'yi güncelle.          */
#define VREF_ADC        3.3f                     /* MCP3221 referans voltajı (Raspi VDD)     */
#define VREF_DEVRE      5.0f                     /* Devrenin gerçek çalışma voltajı (LUT bu cinsten) */
#define VOLTAJ_OLCEK    (VREF_DEVRE / VREF_ADC)  /* Ölçeklendirme katsayısı (~1.515)         */

/* ─── Frekans Parametreleri ─────────────────────────────────────────────────*/
#define REF_FREKANS_HZ          100    /* Faz karşılaştırıcı referans frekansı (Hz)  */
#define VARSAYILAN_HEDEF_HZ   40000    /* Başlangıç hedef VCO frekansı (Hz)          */

/* ─── Feedback ADC Güncelleme ───────────────────────────────────────────────*/
#define ADC_GUNCELLEME_TOGGLE   2      /* Her kaç toggle'da bir ADC okunup frekans güncellenir */

/* ─── Burst Varsayılan Parametreleri ────────────────────────────────────────*/
#define BURST_VARSAYILAN_PALS         60   /* Varsayılan burst pals sayısı             */
#define BURST_VARSAYILAN_DINLEME_MS   30   /* Varsayılan dinleme (HIGH) süresi (ms)    */

/* ─── Sweep Varsayılan Parametreleri ────────────────────────────────────────*/
#define SWEEP_VARSAYILAN_MIN      39000  /* Minimum tarama frekansı (Hz)              */
#define SWEEP_VARSAYILAN_MAX      41000  /* Maksimum tarama frekansı (Hz)             */
#define SWEEP_VARSAYILAN_ADIM       100  /* Tarama adım büyüklüğü (Hz)               */
#define SWEEP_VARSAYILAN_BEKLEME     31  /* Her frekansta bekleme süresi (ms)         */

/* ─── LUT Yapısı ─────────────────────────────────────────────────────────── */
#define PLL_LUT_BOYUT_MAX  32

typedef struct {
    float vctrl;      /* Kontrol voltajı — 0–5V cinsinden (VREF_DEVRE referanslı) */
    int   frekans_hz; /* Karşılık gelen VCO çıkış frekansı (Hz)                   */
} PLL_LUT_Entry;

/* ─── Log Seviyeleri ─────────────────────────────────────────────────────── */
typedef enum {
    PLL_LOG_DEBUG = 0,
    PLL_LOG_INFO,
    PLL_LOG_WARN,
    PLL_LOG_ERROR
} PLL_LogLevel;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* Başlatma / Temizleme */
int   pll_init(void);
void  pll_cleanup(void);

/* Frekans Kontrolü */
void  pll_set_target_frequency(int hz);
int   pll_get_target_frequency(void);
int   pll_get_current_frequency(void);

/* Voltaj Okuma */
float pll_read_vctrl(void);

/* Feedback Sinyali */
void  pll_start_feedback(void);
void  pll_stop_feedback(void);

/* Burst (Inhibit) Sinyali */
void  pll_burst_start(int pulse_count, int listen_ms);
void  pll_burst_stop(void);

/* Manuel INHIBIT — burst yokken VCO'yu sustur/aç (frekans ölçümü için) */
void  pll_mute(void);    /* INHIBIT HIGH → VCO pasif  */
void  pll_unmute(void);  /* INHIBIT LOW  → VCO aktif   */

/* Sweep (Frekans Tarama) */
void  pll_sweep_start(int min_hz, int max_hz, int step_hz, int dwell_ms);
void  pll_sweep_stop(void);

/* Log Seviyesi */
void  pll_set_log_level(PLL_LogLevel level);

/* LUT Yönetimi */
int   pll_lut_yukle_dosya(const char *path);

/* ─── Harici Modül Callback'leri ────────────────────────────────────────── */
typedef void (*pll_event_callback_t)(void);

/*
 * PLL modülü, burst ve listen geçişlerinde dış modülleri (örn. TVG) tetikleyebilir.
 * Callback'ler nullable — kayıt yapılmazsa hiçbir şey çağrılmaz.
 *
 * Çağrılma anı:
 *   - burst_start_callback : INHIBIT_PIN LOW olduktan hemen sonra (ses başladı)
 *   - listen_start_callback: INHIBIT_PIN HIGH olduktan hemen sonra (dinleme başladı)
 *
 * UYARI: Callback'ler burst thread context'inde çalışır. Uzun süren işlemler
 * yapılmamalı. TVG gibi modüller bu callback içinde sadece kendi worker thread'ini
 * tetiklemeli.
 */
void pll_set_burst_start_callback(pll_event_callback_t cb);
void pll_set_listen_start_callback(pll_event_callback_t cb);

#endif /* PLL_H */
