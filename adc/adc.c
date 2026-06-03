// gcc -o test adc.c -llgpio -lpthread -Wall -Wextra

#include "adc.h"

#include <lgpio.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ─── İç Durum Yapısı ───────────────────────────────────────────────────────*/
static struct {
    int      spi_handle;
    uint32_t spi_hiz_hz;
    pthread_mutex_t mutex;
    int             baslandi;
} adc;

/* ─── Log Sistemi ───────────────────────────────────────────────────────────*/
static ADC_LogLevel log_seviyesi = ADC_LOG_INFO;

static void adc_log(ADC_LogLevel seviye, const char *fmt, ...) {
    if (seviye < log_seviyesi) return;
    static const char * const etiketler[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    FILE *hedef = (seviye >= ADC_LOG_WARN) ? stderr : stdout;
    fprintf(hedef, "[ADC][%s] ", etiketler[seviye]);
    va_list arglar;
    va_start(arglar, fmt);
    vfprintf(hedef, fmt, arglar);
    va_end(arglar);
    fputc('\n', hedef);
}

/* ─── SPI 16-bit Transfer ───────────────────────────────────────────────────
 * ADC128S102 protokolü: 2 byte gönder, 2 byte al.
 * TX byte0 = kanal seçim komutu; RX = bir önceki dönüşümün 12-bit sonucu.  */
static int spi_xfer(uint8_t cmd, uint16_t *result_out) {
    uint8_t tx[2] = {cmd, 0x00};
    uint8_t rx[2] = {0x00, 0x00};

    int ret = lgSpiXfer(adc.spi_handle, (char *)tx, (char *)rx, 2);
    if (ret != 2) return ADC_ERR_SPI;

    /* RX format: [0 0 0 0 D11..D8] [D7..D0] */
    *result_out = ((uint16_t)(rx[0] & 0x0F) << 8) | rx[1];
    return ADC_OK;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* SPI bus'ı açar ve ADC128S102 bağlantısını test okuma ile doğrular.       */
int adc_init(void) {
    if (adc.baslandi) {
        adc_log(ADC_LOG_WARN, "adc_init zaten cagirildi");
        return ADC_OK;
    }

    memset(&adc, 0, sizeof(adc));
    adc.spi_handle = -1;
    adc.spi_hiz_hz = ADC_SPI_SPEED_HZ;

    if (pthread_mutex_init(&adc.mutex, NULL) != 0) {
        adc_log(ADC_LOG_ERROR, "Mutex baslatilamadi");
        return ADC_ERR_INIT;
    }

    adc.spi_handle = lgSpiOpen(ADC_SPI_BUS, ADC_SPI_CHANNEL,
                                ADC_SPI_SPEED_HZ, ADC_SPI_FLAGS);
    if (adc.spi_handle < 0) {
        adc_log(ADC_LOG_ERROR, "SPI bus %d kanal %d acilamadi: hata=%d",
                ADC_SPI_BUS, ADC_SPI_CHANNEL, adc.spi_handle);
        pthread_mutex_destroy(&adc.mutex);
        return ADC_ERR_INIT;
    }

    /* Bağlantı testi: kanal 0'ı seçip iki transfer yap (pipeline gerektirir) */
    uint16_t dummy;
    if (spi_xfer(ADC_CMD_BYTE(0), &dummy) != ADC_OK ||
        spi_xfer(ADC_CMD_BYTE(0), &dummy) != ADC_OK) {
        adc_log(ADC_LOG_ERROR, "ADC128S102 baglanti testi basarisiz");
        lgSpiClose(adc.spi_handle);
        pthread_mutex_destroy(&adc.mutex);
        return ADC_ERR_SPI;
    }

    adc.baslandi = 1;
    adc_log(ADC_LOG_INFO,
            "ADC128S102 hazir — SPI%d CE%d @ %u Hz, Vref=%.2fV",
            ADC_SPI_BUS, ADC_SPI_CHANNEL, ADC_SPI_SPEED_HZ, ADC_VREF);
    return ADC_OK;
}

/* SPI bus'ı kapatır. */
void adc_cleanup(void) {
    if (!adc.baslandi) return;
    adc_log(ADC_LOG_INFO, "ADC temizleniyor");

    pthread_mutex_lock(&adc.mutex);
    if (adc.spi_handle >= 0) {
        lgSpiClose(adc.spi_handle);
        adc.spi_handle = -1;
    }
    pthread_mutex_unlock(&adc.mutex);
    pthread_mutex_destroy(&adc.mutex);

    adc.baslandi = 0;
    adc_log(ADC_LOG_INFO, "ADC temizlendi");
}

/* Belirtilen kanaldan ham 12-bit değer okur.
 * ADC128S102 pipelined çalışır: iki transfer gereklidir.
 *   Transfer 1: kanal seçilir, önceki kanalın sonucu atılır.
 *   Transfer 2: seçilen kanalın dönüşüm sonucu okunur.
 * Başarısız transferler ADC_SPI_MAX_DENEME kez yeniden denenir.            */
int adc_read_raw(uint8_t channel, uint16_t *raw_out) {
    if (!adc.baslandi)              return ADC_ERR_INIT;
    if (channel >= ADC_KANAL_SAYISI) return ADC_ERR_PARAM;
    if (!raw_out)                   return ADC_ERR_NULL;

    uint8_t cmd = ADC_CMD_BYTE(channel);

    pthread_mutex_lock(&adc.mutex);

    int ret = ADC_ERR_SPI;
    for (int d = 0; d < ADC_SPI_MAX_DENEME; d++) {
        uint16_t dummy;
        /* Transfer 1: kanal seçimi (pipeline priming) */
        if (spi_xfer(cmd, &dummy) != ADC_OK) {
            adc_log(ADC_LOG_WARN, "SPI prime transfer hatasi, deneme %d/%d",
                    d + 1, ADC_SPI_MAX_DENEME);
            continue;
        }
        /* Transfer 2: kanalın dönüşüm sonucu */
        if (spi_xfer(cmd, raw_out) != ADC_OK) {
            adc_log(ADC_LOG_WARN, "SPI okuma transfer hatasi, deneme %d/%d",
                    d + 1, ADC_SPI_MAX_DENEME);
            continue;
        }
        ret = ADC_OK;
        break;
    }

    pthread_mutex_unlock(&adc.mutex);

    if (ret == ADC_OK)
        adc_log(ADC_LOG_DEBUG, "CH%u raw=%u (%.3fV)",
                channel, *raw_out, (*raw_out / (float)ADC_MAX_HAM) * ADC_VREF);
    return ret;
}

/* Belirtilen kanalı okuyup voltaja çevirir (0.0 – ADC_VREF arası).        */
int adc_read_voltage(uint8_t channel, float *voltage_out) {
    if (!voltage_out) return ADC_ERR_NULL;

    uint16_t raw;
    int ret = adc_read_raw(channel, &raw);
    if (ret == ADC_OK)
        *voltage_out = (raw / (float)ADC_MAX_HAM) * ADC_VREF;
    return ret;
}

/* 8 kanalı sırayla okur; pipeline doğru yönetilir (9 transfer).
 * Herhangi bir transfer hata verirse tüm dizi yeniden denenir.             */
int adc_read_all_raw(uint16_t raw_array[ADC_KANAL_SAYISI]) {
    if (!adc.baslandi) return ADC_ERR_INIT;
    if (!raw_array)    return ADC_ERR_NULL;

    pthread_mutex_lock(&adc.mutex);

    int ret = ADC_ERR_SPI;
    for (int d = 0; d < ADC_SPI_MAX_DENEME; d++) {
        /* Transfer 0: CH0 seç, pipeline'ı hazırla (sonuç atılır) */
        uint16_t dummy;
        if (spi_xfer(ADC_CMD_BYTE(0), &dummy) != ADC_OK) {
            adc_log(ADC_LOG_WARN, "Toplu okuma prime hatasi, deneme %d/%d",
                    d + 1, ADC_SPI_MAX_DENEME);
            continue;
        }

        /* Transfer 1-7: CHi'yi oku, CH(i+1)'i seç */
        int hata = 0;
        for (int ch = 0; ch < ADC_KANAL_SAYISI - 1; ch++) {
            if (spi_xfer(ADC_CMD_BYTE(ch + 1), &raw_array[ch]) != ADC_OK) {
                adc_log(ADC_LOG_WARN, "Toplu okuma CH%d hatasi, deneme %d/%d",
                        ch, d + 1, ADC_SPI_MAX_DENEME);
                hata = 1;
                break;
            }
        }
        if (hata) continue;

        /* Transfer 8: CH7'yi oku (CH0 seçilerek pipeline sıfırlanır) */
        if (spi_xfer(ADC_CMD_BYTE(0), &raw_array[ADC_KANAL_SAYISI - 1]) != ADC_OK) {
            adc_log(ADC_LOG_WARN, "Toplu okuma CH7 hatasi, deneme %d/%d",
                    d + 1, ADC_SPI_MAX_DENEME);
            continue;
        }

        ret = ADC_OK;
        break;
    }

    pthread_mutex_unlock(&adc.mutex);
    return ret;
}

/* 8 kanalı okuyup tüm değerleri voltaja çevirir.                           */
int adc_read_all_voltage(float voltage_array[ADC_KANAL_SAYISI]) {
    if (!voltage_array) return ADC_ERR_NULL;

    uint16_t raw[ADC_KANAL_SAYISI];
    int ret = adc_read_all_raw(raw);
    if (ret == ADC_OK) {
        for (int ch = 0; ch < ADC_KANAL_SAYISI; ch++)
            voltage_array[ch] = (raw[ch] / (float)ADC_MAX_HAM) * ADC_VREF;
    }
    return ret;
}

/* Belirli kanaldan sample_count kez okuma yapıp ortalama voltajı döndürür.
 * Başarısız örnekler ortalamadan çıkarılır; hiç başarılı yoksa ADC_ERR_SPI. */
int adc_read_averaged(uint8_t channel, int sample_count, float *voltage_out) {
    if (!adc.baslandi)               return ADC_ERR_INIT;
    if (channel >= ADC_KANAL_SAYISI) return ADC_ERR_PARAM;
    if (!voltage_out)                return ADC_ERR_NULL;
    if (sample_count <= 0)           return ADC_ERR_PARAM;

    uint32_t toplam  = 0;
    int      basarili = 0;

    for (int s = 0; s < sample_count; s++) {
        uint16_t raw;
        if (adc_read_raw(channel, &raw) == ADC_OK) {
            toplam += raw;
            basarili++;
        }
    }

    if (basarili == 0) {
        adc_log(ADC_LOG_ERROR, "CH%u: %d ornekten hicbiri basarisiz olmadi",
                channel, sample_count);
        return ADC_ERR_SPI;
    }

    *voltage_out = ((float)toplam / basarili / ADC_MAX_HAM) * ADC_VREF;
    adc_log(ADC_LOG_DEBUG, "CH%u ortalama: %.4fV (%d/%d ornek)",
            channel, *voltage_out, basarili, sample_count);
    return ADC_OK;
}

/* SPI hızını runtime'da değiştirir (maks 16 MHz).
 * Mevcut handle kapatılır, yeni hızla yeniden açılır.                      */
int adc_set_spi_speed(uint32_t hz) {
    if (!adc.baslandi)  return ADC_ERR_INIT;
    if (hz == 0 || hz > 16000000) return ADC_ERR_PARAM;

    pthread_mutex_lock(&adc.mutex);

    lgSpiClose(adc.spi_handle);
    adc.spi_handle = lgSpiOpen(ADC_SPI_BUS, ADC_SPI_CHANNEL, hz, ADC_SPI_FLAGS);

    int ret;
    if (adc.spi_handle < 0) {
        adc_log(ADC_LOG_ERROR, "SPI yeniden acilamadi %u Hz: hata=%d", hz, adc.spi_handle);
        ret = ADC_ERR_SPI;
    } else {
        adc.spi_hiz_hz = hz;
        ret = ADC_OK;
        adc_log(ADC_LOG_INFO, "SPI hizi guncellendi: %u Hz", hz);
    }

    pthread_mutex_unlock(&adc.mutex);
    return ret;
}

/* Log seviyesini ayarlar. Varsayılan: ADC_LOG_INFO. */
void adc_set_log_level(ADC_LogLevel level) {
    log_seviyesi = level;
}
