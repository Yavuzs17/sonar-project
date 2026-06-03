#ifndef SENSOR_H
#define SENSOR_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>    /* struct timespec */

#include "../adc/adc.h"

/* ─── GPIO Tanımı ───────────────────────────────────────────────────────────*/
#define SENSOR_SCHMITT_GPIO       16   /* Schmitt trigger echo girişi (BCM)  */
                                        /* Devre 5V → gerilim bölücü → 3.3V  */

/* ─── Gerilim Ölçekleme ─────────────────────────────────────────────────────
 * 5V devresinden gelen kanallar gerilim bölücüyle ADC'ye bağlı.
 * Gerçek voltajı bulmak için: v_gercek = v_adc × SENSOR_VOLTAJ_OLCEK.
 * Her kanal için ölçekleme ayrı ayarlanabilir (sensor_set_olcek).           */
#define SENSOR_VREF_DEVRE         5.0f
#define SENSOR_VREF_ADC           3.3f
#define SENSOR_VOLTAJ_OLCEK       (SENSOR_VREF_DEVRE / SENSOR_VREF_ADC)

/* ─── LM60 Sıcaklık Sensörü ─────────────────────────────────────────────────
 * Formül: T(°C) = (Vout − OFSET_V) / KATSAYI_VPC
 * Vout = KATSAYI_VPC × T + OFSET_V   (ör. 25°C → 0.581V)
 * LM60, 5V ile besleniyor → gerilim bölücüyle 3.3V ADC'ye bağlı.
 * sensor_lm60_voltaj() SENSOR_VOLTAJ_OLCEK uygulanmış gerçek devre voltajını
 * döndürür; sensor_get_sicaklik_c() bu değeri doğrudan formüle verir.      */
#define SENSOR_LM60_OFSET_V       0.424f    /* 0°C'de LM60 çıkışı (V)       */
#define SENSOR_LM60_KATSAYI_VPC   0.00625f  /* 6.25 mV/°C                   */
#define SENSOR_LM60_VBESLEME_V    5.0f      /* LM60 besleme voltajı         */

/* ─── Ses Hızı ──────────────────────────────────────────────────────────────
 * c(m/s) = SES_HIZI_OFSET + SES_HIZI_KATSAYI_T × T(°C)                   */
#define SES_HIZI_OFSET            331.3f    /* 0°C'de ses hızı (m/s)        */
#define SES_HIZI_KATSAYI_T        0.606f    /* °C başına değişim (m/s/°C)   */

/* ─── Doppler Parametreleri ─────────────────────────────────────────────────
 * LUT boş olursa uygulanan kaba varsayım:
 *   merkez voltajı (VREF_DEVRE/2) → BURST_FREK_HZ_DEFAULT
 *   ±1V → ±1000 Hz                                                          */
#define SENSOR_BURST_FREK_HZ_DEFAULT  40000.0f
#define DOPPLER_LUT_BOYUT_MAX         16

/* ─── Echo / Schmitt Trigger ────────────────────────────────────────────────*/
#define SENSOR_ECHO_BUFFER_BOYUT      32    /* Bir burst'te max echo sayısı  */

/* ─── Hata Kodları ──────────────────────────────────────────────────────────*/
#define SENSOR_OK          0
#define SENSOR_ERR_INIT   -1   /* Başlatma hatası                            */
#define SENSOR_ERR_ADC    -2   /* ADC okuma hatası                           */
#define SENSOR_ERR_GPIO   -3   /* GPIO / lgpio hatası                        */
#define SENSOR_ERR_PARAM  -4   /* Geçersiz parametre                         */
#define SENSOR_ERR_LUT    -5   /* Doppler LUT dolu / boş                     */
#define SENSOR_ERR_NULL   -6   /* NULL pointer                               */
#define SENSOR_ERR_DOSYA  -7   /* Dosya açılamadı / okunamadı               */

/* ─── Log Seviyeleri ────────────────────────────────────────────────────────*/
typedef enum {
    SENSOR_LOG_DEBUG = 0,
    SENSOR_LOG_INFO,
    SENSOR_LOG_WARN,
    SENSOR_LOG_ERROR
} SensorLogLevel;

/* ─── Veri Yapıları ─────────────────────────────────────────────────────────*/

/* Kanal ölçekleme ve kalibrasyon ofseti                                      */
typedef struct {
    bool  olcek_aktif;    /* Gerilim bölücü ölçeklemesi uygulanacak mı      */
    float olcek_katsayi;  /* Ölçek katsayısı (varsayılan SENSOR_VOLTAJ_OLCEK) */
    float ofset_v;        /* Kalibrasyon voltaj ofseti (V)                  */
} SensorKanalAyar;

/* Doppler LUT noktası: ölçülen voltaj → PLL kilitlenme frekansı             */
typedef struct {
    float vctrl;       /* Loop filter voltajı, devre tarafı (0–5V)          */
    float frekans_hz;  /* Bu voltajda PLL'in kilitli olduğu frekans (Hz)    */
} DopplerLUT_Entry;

/* Yakalanmış tek bir echo olayı                                              */
typedef struct {
    bool            tetiklendi;   /* Bu yuvada geçerli bir echo var mı      */
    struct timespec zaman;        /* CLOCK_MONOTONIC zaman damgası           */
    double          gecikme_us;   /* Burst başından itibaren gecikme (µs)   */
    float           mesafe_m;     /* Hesaplanan tek yön mesafe (m)          */
} EchoOlay;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/*
 * Örnek kullanım:
 *
 *   sensor_init();
 *
 *   float t = sensor_get_sicaklik_c();
 *   float c = sensor_get_ses_hizi();
 *   printf("Sicaklik: %.1f C, Ses hizi: %.1f m/s\n", t, c);
 *
 *   sensor_burst_baslat();
 *   // ... PLL burst ...
 *   usleep(30000);
 *   sensor_burst_sonlandir();
 *
 *   EchoOlay echo;
 *   if (sensor_ilk_echo_oku(&echo) == SENSOR_OK)
 *       printf("Hedef: %.2f m, gecikme: %.1f us\n", echo.mesafe_m, echo.gecikme_us);
 *
 *   float hiz = sensor_get_doppler_hiz(0);
 *   printf("Hedef hizi: %.2f m/s\n", hiz);
 */

/* Başlatma / Temizleme */
int  sensor_init(void);
void sensor_cleanup(void);

/* Voltaj Okumaları (ölçeklendirilmiş, gerçek devre voltajı) */
float sensor_envelope_voltaj(void);
float sensor_ina_voltaj(void);
float sensor_doppler_voltaj(void);
float sensor_lm60_voltaj(void);

/* Anlamlı Fiziksel Değerler */
float sensor_get_sicaklik_c(void);
float sensor_get_ses_hizi(void);
float sensor_get_doppler_frekans(void);
float sensor_get_doppler_hiz(float f_burst);   /* f_burst=0 → varsayılan 40kHz */

/* Echo / Schmitt Trigger Yönetimi */
int sensor_burst_baslat(void);
int sensor_burst_sonlandir(void);
int sensor_echo_sayisi(void);
int sensor_echo_oku(int index, EchoOlay *out);
int sensor_ilk_echo_oku(EchoOlay *out);

/* Kalibrasyon — Kanal Ölçekleme */
void sensor_set_olcek(int kanal, bool aktif, float katsayi, float ofset);
int  sensor_get_olcek(int kanal, SensorKanalAyar *ayar_out);

/* Doppler LUT */
int  sensor_doppler_lut_ekle(float vctrl, float frekans_hz);
int  sensor_doppler_lut_temizle(void);
int  sensor_doppler_lut_kaydet_dosya(const char *path);
int  sensor_doppler_lut_yukle_dosya(const char *path);

/* Burst Frekansı (Doppler hız hesabı için referans) */
void  sensor_set_burst_frekans(float hz);
float sensor_get_burst_frekans(void);

/* Log Seviyesi */
void sensor_set_log_level(SensorLogLevel level);

#endif /* SENSOR_H */
