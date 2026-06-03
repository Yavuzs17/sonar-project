#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/* ─── SPI Bağlantı Tanımları ────────────────────────────────────────────────
 * SPI0 pin atamaları (Raspberry Pi standart):
 *   GPIO 8  → CE0  (Chip Enable / Chip Select)
 *   GPIO 9  → MISO (Master In Slave Out)
 *   GPIO 10 → MOSI (Master Out Slave In)
 *   GPIO 11 → SCLK (Clock)
 *
 * Etkinleştirme: sudo raspi-config → Interface → SPI → Enable             */
#define ADC_SPI_BUS         0         /* SPI0                               */
#define ADC_SPI_CHANNEL     0         /* CE0 (chip select 0)                */
#define ADC_SPI_SPEED_HZ    1000000   /* 1 MHz (maks 16 MHz desteklenir)    */
#define ADC_SPI_FLAGS       0         /* SPI Mode 0 (CPOL=0, CPHA=0)       */
#define ADC_SPI_MAX_DENEME  3         /* Başarısız SPI için yeniden deneme  */

/* ─── ADC128S102 Protokol Sabiti ────────────────────────────────────────────
 * 16-bit frame'in yüksek byte'ında kanal seçim bitleri.
 * ADC pipelined çalışır: gönderilen kanal bir sonraki dönüşüme aittir.
 * TX: [0 0 0 ADD2 ADD1 ADD0 0 0] [0 0 0 0 0 0 0 0]
 * RX: [0 0 0 0 D11 D10 D9 D8] [D7 D6 D5 D4 D3 D2 D1 D0]                   */
#define ADC_CMD_BYTE(ch)    ((uint8_t)(((ch) & 0x07) << 3))

/* ─── Kanal Tanımları ───────────────────────────────────────────────────────*/
#define ADC_CH_ENVELOPE     0   /* Envelope detector çıkışı                  */
#define ADC_CH_DOPPLER      1   /* Doppler PLL loop filter çıkışı            */
#define ADC_CH_INA          2   /* INA821 çıkışı (TVG'li)                    */
#define ADC_CH_SICAKLIK     3   /* LM60 sıcaklık sensörü çıkışı              */
#define ADC_CH_RESERVED_4   4   /* İleride kullanım için ayrıldı             */
#define ADC_CH_RESERVED_5   5
#define ADC_CH_RESERVED_6   6
#define ADC_CH_RESERVED_7   7
#define ADC_KANAL_SAYISI    8

/* ─── Referans Voltajı ve Ölçeklendirme ─────────────────────────────────────
 * NOT: Gerilim bölücü ölçeklendirmesi bu modülde yapılmaz — üst seviye
 * sensor modüllerinin sorumluluğundadır.                                     */
#define ADC_VREF            3.3f   /* VA pin voltajı (Raspberry Pi 3.3V)     */
#define ADC_MAX_HAM         4095   /* 12-bit tam ölçek (2^12 - 1)            */

/* ─── Hata Kodları ──────────────────────────────────────────────────────────*/
#define ADC_OK              0
#define ADC_ERR_INIT       -1   /* Başlatma hatası                           */
#define ADC_ERR_SPI        -2   /* SPI iletişim hatası                       */
#define ADC_ERR_PARAM      -3   /* Geçersiz kanal numarası (0–7 dışı)        */
#define ADC_ERR_NULL       -4   /* NULL pointer parametresi                  */

/* ─── Log Seviyeleri ────────────────────────────────────────────────────────*/
typedef enum {
    ADC_LOG_DEBUG = 0,
    ADC_LOG_INFO,
    ADC_LOG_WARN,
    ADC_LOG_ERROR
} ADC_LogLevel;

/* ─── Public API ─────────────────────────────────────────────────────────── */

/*
 * Örnek kullanım:
 *
 *   // Tek kanal voltaj okuma:
 *   float v;
 *   adc_read_voltage(ADC_CH_ENVELOPE, &v);
 *
 *   // 16 örnek ortalaması (gürültü azaltma):
 *   float v_doppler;
 *   adc_read_averaged(ADC_CH_DOPPLER, 16, &v_doppler);
 *
 *   // Tüm kanallar:
 *   float voltajlar[ADC_KANAL_SAYISI];
 *   adc_read_all_voltage(voltajlar);
 */

/* Başlatma / Temizleme */
int  adc_init(void);
void adc_cleanup(void);

/* Düşük Seviye Okuma */
int adc_read_raw(uint8_t channel, uint16_t *raw_out);
int adc_read_voltage(uint8_t channel, float *voltage_out);

/* Toplu Okuma */
int adc_read_all_raw(uint16_t raw_array[ADC_KANAL_SAYISI]);
int adc_read_all_voltage(float voltage_array[ADC_KANAL_SAYISI]);

/* Çoklu Örnekleme (Gürültü Azaltma) */
int adc_read_averaged(uint8_t channel, int sample_count, float *voltage_out);

/* SPI Hızı Ayarı */
int adc_set_spi_speed(uint32_t hz);

/* Log Seviyesi */
void adc_set_log_level(ADC_LogLevel level);

#endif /* ADC_H */
