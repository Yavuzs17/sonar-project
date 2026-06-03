#ifndef PREAMP_H
#define PREAMP_H

#include <stdint.h>

/* ─── Devre Parametreleri ───────────────────────────────────────────────────
 * Farklı bir direnç konfigürasyonu için yalnızca bu define'ları güncelle.  */
/*
 * Giriş direnci R_in nominal 2200Ω, fakat fiziksel direncin %1–%5 toleransı vardır.
 * Bu yüzden formül üzerinden hesaplanan kazanç sadece kaba tahmindir —
 * gerçek kazanç değeri her kanal için ayrı ayrı LUT ile kalibre edilmelidir.
 * Eğer giriş direnci farklı bir değerle değiştirilirse sadece bu define güncellenir.
 */
#define PREAMP_R_GIRIS_OHM   2200.0f   /* Sabit giriş direnci R_in (Ω)      */
#define PREAMP_R_STEP_OHM    39.0f     /* MCP4651 step başına direnç (Ω)    */
/* Non-inverting amplifier: G = 1 + (R_pot / R_in)                          */
#define PREAMP_GAIN_FORMUL(step) \
    (1.0f + ((float)(step) * PREAMP_R_STEP_OHM / PREAMP_R_GIRIS_OHM))

/* ─── I2C / Çip Tanımları ───────────────────────────────────────────────────*/
/*
 * NOT: Bu modül, phase_shift modülü ile aynı I2C-1 hattını paylaşır.
 * I2C bus erişimi mutex ile korunur.
 * PLL modülü ayrı bir bus (software I2C, GPIO 4/5) kullandığı için
 * preamp ile çakışmaz.
 */
#define PREAMP_I2C_BUS  1   /* Raspberry Pi donanımsal I2C-1 hattı            */
                            /* (GPIO 2 = SDA, GPIO 3 = SCL)                   */
                            /* PLL modülü ayrı software I2C kullanır (GPIO 4/5) */
#define PREAMP_CIP_SAYISI       4      /* Toplam MCP4651 çip sayısı              */
#define PREAMP_KANAL_SAYISI     7      /* Aktif alıcı kanal sayısı               */
#define PREAMP_LUT_MAX_NOKTA    32     /* Kanal başına maksimum LUT noktası      */

/* MCP4651 I2C adresleri (A2A1A0 pin kombinasyonuna göre)                   */
#define PREAMP_CIP_1_ADDR  0x28  /* A2A1A0=000 — Alıcı 1 (W0), Alıcı 2 (W1) */
#define PREAMP_CIP_2_ADDR  0x29  /* A2A1A0=001 — Alıcı 3 (W0), Alıcı 4 (W1) */
#define PREAMP_CIP_3_ADDR  0x2A  /* A2A1A0=010 — Alıcı 5 (W0), Alıcı 6 (W1) */
#define PREAMP_CIP_4_ADDR  0x2B  /* A2A1A0=011 — Alıcı 7 (W0), boş (W1)     */

/* ─── MCP4651 Protokol Sabitleri ───────────────────────────────────────────*/
#define MCP4651_CMD_YAZ       0x00  /* Komut nibble: yazma (00)              */
#define MCP4651_CMD_OKU       0x0C  /* Komut nibble: okuma (11)              */
#define MCP4651_MAX_STEP      255   /* Maksimum step (8-bit mod)             */
#define PREAMP_I2C_MAX_DENEME 3     /* Başarısız I2C için yeniden deneme     */

/* ─── Hata Kodları ──────────────────────────────────────────────────────────*/
#define PREAMP_OK            0
#define PREAMP_ERR_INIT     -1   /* Başlatma hatası                          */
#define PREAMP_ERR_I2C      -2   /* I2C iletişim hatası                      */
#define PREAMP_ERR_KANAL    -3   /* Geçersiz kanal (0–6 dışı)                */
#define PREAMP_ERR_STEP     -4   /* Step 255 üstünde                         */
#define PREAMP_ERR_LUT_BOS  -5   /* LUT boş                                  */
#define PREAMP_ERR_LUT_DOLU -6   /* LUT kapasitesi dolu                      */
#define PREAMP_ERR_DOSYA    -7   /* Dosya açılamadı / okunamadı              */

/* ─── Log Seviyeleri ────────────────────────────────────────────────────────*/
typedef enum {
    PREAMP_LOG_DEBUG = 0,
    PREAMP_LOG_INFO,
    PREAMP_LOG_WARN,
    PREAMP_LOG_ERROR
} PREAMP_LogLevel;

/* ─── LUT Veri Yapısı ───────────────────────────────────────────────────────
 * Kalibrasyon programı tarafından runtime'da doldurulur.                   */
typedef struct {
    float   kazanc_db;  /* Kalibre edilen kazanç (dB)                       */
    uint8_t step;       /* Bu kazanca karşılık gelen MCP4651 step değeri    */
} Preamp_LUT_Entry;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* Başlatma / Temizleme */
int  preamp_init(void);
void preamp_cleanup(void);

/* Kazanç Ayarlama — tek kanal */
int  preamp_set_step(int kanal, uint8_t step);
int  preamp_set_gain_db(int kanal, float kazanc_db);
int  preamp_set_gain_linear(int kanal, float gain);

/* Kazanç Ayarlama — toplu (tüm 7 kanal) */
int  preamp_set_all_step(uint8_t step);
int  preamp_set_all_gain_db(float kazanc_db);

/* Okuma (önbellekten, son yazılan değer) */
int   preamp_get_step(int kanal, uint8_t *step_out);
float preamp_get_gain_db(int kanal);
float preamp_get_gain_linear(int kanal);

/* LUT Yönetimi */
int  preamp_lut_ekle(int kanal, float kazanc_db, uint8_t step);
void preamp_lut_temizle(int kanal);
int  preamp_lut_kaydet_dosya(const char *path);   /* CSV: kanal,kazanc_db,step */
int  preamp_lut_yukle_dosya(const char *path);

/* Log Seviyesi */
void preamp_set_log_level(PREAMP_LogLevel level);

#endif /* PREAMP_H */
