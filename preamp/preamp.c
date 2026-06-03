// gcc -o test preamp.c -llgpio -lpthread -lm -Wall -Wextra

#include "preamp.h"

#include <lgpio.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ─── Kanal → Çip / Wiper Eşleme Tablosu ───────────────────────────────────
 * Alıcı 1–7'yi MCP4651 çipine ve wiper kanalına bağlar.
 * Çip başına iki wiper: wiper=0 (kanal A) ve wiper=1 (kanal B).           */
typedef struct {
    uint8_t i2c_addr; /* Çip I2C adresi                                     */
    uint8_t wiper;    /* Wiper kanalı: 0 veya 1                             */
} Preamp_KanalMap;

static const Preamp_KanalMap kanal_map[PREAMP_KANAL_SAYISI] = {
    { PREAMP_CIP_1_ADDR, 0 },  /* Alıcı 1 */
    { PREAMP_CIP_1_ADDR, 1 },  /* Alıcı 2 */
    { PREAMP_CIP_2_ADDR, 0 },  /* Alıcı 3 */
    { PREAMP_CIP_2_ADDR, 1 },  /* Alıcı 4 */
    { PREAMP_CIP_3_ADDR, 0 },  /* Alıcı 5 */
    { PREAMP_CIP_3_ADDR, 1 },  /* Alıcı 6 */
    { PREAMP_CIP_4_ADDR, 0 },  /* Alıcı 7 */
};

/* Çip indeksi → I2C adresi tablosu (sıra: cip_handle[] ile eşleşir) */
static const uint8_t cip_adresler[PREAMP_CIP_SAYISI] = {
    PREAMP_CIP_1_ADDR, PREAMP_CIP_2_ADDR,
    PREAMP_CIP_3_ADDR, PREAMP_CIP_4_ADDR
};

/* ─── Kalibrasyon LUT'ları ──────────────────────────────────────────────────
 * Her kanal için bağımsız (üretim toleransı, ofset farkı nedeniyle).
 * Runtime'da preamp_lut_ekle() ile doldurulur; const değil — değiştirilebilir.
 * Sıralama: kazanc_db artan sırada tutulur (eklemede korunur).             */
static Preamp_LUT_Entry lut[PREAMP_KANAL_SAYISI][PREAMP_LUT_MAX_NOKTA];
static int              lut_boyut[PREAMP_KANAL_SAYISI];

/* ─── İç Durum Yapısı ───────────────────────────────────────────────────────
 *
 * BUS PAYLAŞIMI NOTU:
 * Bu modül PLL modülüyle aynı I2C-1 hattını (GPIO 2/3) paylaşır.
 * İleride çakışmaları önlemek için preamp.mutex yerine her iki modülün
 * ortak bir  extern pthread_mutex_t i2c_bus_mutex  kullanması önerilir.
 * Şimdilik her modül kendi mutex'ini bağımsız yönetir.                     */
static struct {
    int             cip_handle[PREAMP_CIP_SAYISI];    /* lgpio I2C handle'ları */
    uint8_t         mevcut_step[PREAMP_KANAL_SAYISI]; /* Son yazılan step önbelleği */
    pthread_mutex_t mutex;
    int             baslandi;
} preamp;

/* ─── Log Sistemi ───────────────────────────────────────────────────────────*/
static PREAMP_LogLevel log_seviyesi = PREAMP_LOG_INFO;

static void preamp_log(PREAMP_LogLevel seviye, const char *fmt, ...) {
    if (seviye < log_seviyesi) return;
    static const char * const etiketler[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    FILE *hedef = (seviye >= PREAMP_LOG_WARN) ? stderr : stdout;
    fprintf(hedef, "[PREAMP][%s] ", etiketler[seviye]);
    va_list arglar;
    va_start(arglar, fmt);
    vfprintf(hedef, fmt, arglar);
    va_end(arglar);
    fputc('\n', hedef);
}

/* ─── MCP4651 Sürücüsü (sadece bu dosyada kullanılır) ──────────────────────*/

/* MCP4651 wiper'ına step değeri yazar.
 * Protokol: 2 byte — [komut byte: (wiper<<4)|yazma] [veri byte: step].
 * Başarısız I2C işlemini PREAMP_I2C_MAX_DENEME kez dener.                 */
static int mcp4651_yaz(uint8_t i2c_addr, uint8_t wiper, uint16_t step) {
    int cip_idx = (int)(i2c_addr - PREAMP_CIP_1_ADDR);
    if (cip_idx < 0 || cip_idx >= PREAMP_CIP_SAYISI) return PREAMP_ERR_I2C;
    if (step > MCP4651_MAX_STEP) step = MCP4651_MAX_STEP;

    uint8_t buf[2];
    buf[0] = (uint8_t)((wiper << 4) | MCP4651_CMD_YAZ); /* adres nibble + yazma komutu */
    buf[1] = (uint8_t)(step & 0xFF);                      /* veri byte                   */

    for (int d = 0; d < PREAMP_I2C_MAX_DENEME; d++) {
        int ret = lgI2cWriteDevice(preamp.cip_handle[cip_idx], (char *)buf, 2);
        if (ret == 0) return PREAMP_OK;
        preamp_log(PREAMP_LOG_WARN,
                   "I2C yaz hatasi cip=0x%02X wiper=%d step=%u deneme=%d/%d",
                   i2c_addr, wiper, (unsigned)step, d + 1, PREAMP_I2C_MAX_DENEME);
    }
    return PREAMP_ERR_I2C;
}

/* MCP4651 wiper'ından step değeri okur (debug / doğrulama amaçlı).
 * Protokol: 1 byte okuma komutu yaz, 2 byte yanıt oku.
 * D8 8-bit modda daima 0'dır; step = byte1.                               */
static int mcp4651_oku(uint8_t i2c_addr, uint8_t wiper, uint16_t *step_out) {
    int cip_idx = (int)(i2c_addr - PREAMP_CIP_1_ADDR);
    if (cip_idx < 0 || cip_idx >= PREAMP_CIP_SAYISI || !step_out) return PREAMP_ERR_I2C;

    for (int d = 0; d < PREAMP_I2C_MAX_DENEME; d++) {
        uint8_t cmd = (uint8_t)((wiper << 4) | MCP4651_CMD_OKU);
        int ret = lgI2cWriteDevice(preamp.cip_handle[cip_idx], (char *)&cmd, 1);
        if (ret != 0) {
            preamp_log(PREAMP_LOG_WARN, "I2C oku komutu hatasi cip=0x%02X wiper=%d deneme=%d",
                       i2c_addr, wiper, d + 1);
            continue;
        }
        uint8_t rbuf[2] = {0, 0};
        ret = lgI2cReadDevice(preamp.cip_handle[cip_idx], (char *)rbuf, 2);
        if (ret == 2) {
            *step_out = (uint16_t)(((rbuf[0] & 0x01) << 8) | rbuf[1]);
            return PREAMP_OK;
        }
        preamp_log(PREAMP_LOG_WARN, "I2C oku yaniti hatasi cip=0x%02X wiper=%d deneme=%d",
                   i2c_addr, wiper, d + 1);
    }
    return PREAMP_ERR_I2C;
}

/* ─── LUT Interpolasyonu ─────────────────────────────────────────────────── */

/* Kazanç (dB) → MCP4651 step değeri dönüşümü.
 * Kanalın LUT'u doluysa lineer interpolasyon kullanır.
 * LUT boşsa formülü ters çevirerek step hesaplar:
 *   G  = 10^(dB/20)
 *   step = (G - 1) × R_in / R_step                                        */
static uint8_t kazanc_db_to_step(int kanal, float kazanc_db) {
    int n = lut_boyut[kanal];

    if (n == 0) {
        /* LUT boş: ters formülden hesapla */
        float G      = powf(10.0f, kazanc_db / 20.0f);
        float step_f = (G - 1.0f) * PREAMP_R_GIRIS_OHM / PREAMP_R_STEP_OHM;
        if (step_f < 0.0f)   step_f = 0.0f;
        if (step_f > 255.0f) step_f = 255.0f;
        return (uint8_t)(step_f + 0.5f); /* en yakın tamsayıya yuvarlama */
    }

    /* Sınır kontrolü */
    if (kazanc_db <= lut[kanal][0].kazanc_db) {
        if (kazanc_db < lut[kanal][0].kazanc_db)
            preamp_log(PREAMP_LOG_WARN, "Kanal %d: %.2f dB LUT alt siniri altinda (%.2f dB)",
                       kanal + 1, kazanc_db, lut[kanal][0].kazanc_db);
        return lut[kanal][0].step;
    }
    if (kazanc_db >= lut[kanal][n - 1].kazanc_db) {
        if (kazanc_db > lut[kanal][n - 1].kazanc_db)
            preamp_log(PREAMP_LOG_WARN, "Kanal %d: %.2f dB LUT ust siniri ustunde (%.2f dB)",
                       kanal + 1, kazanc_db, lut[kanal][n - 1].kazanc_db);
        return lut[kanal][n - 1].step;
    }

    /* Lineer interpolasyon */
    for (int i = 0; i < n - 1; i++) {
        if (kazanc_db >= lut[kanal][i].kazanc_db && kazanc_db <= lut[kanal][i + 1].kazanc_db) {
            float t = (kazanc_db - lut[kanal][i].kazanc_db)
                    / (lut[kanal][i + 1].kazanc_db - lut[kanal][i].kazanc_db);
            float s = lut[kanal][i].step + t * (lut[kanal][i + 1].step - lut[kanal][i].step);
            return (uint8_t)(s + 0.5f);
        }
    }

    return lut[kanal][n - 1].step; /* ulaşılmamalı */
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* I2C açar, 4 çipin bağlantısını test okuma ile doğrular,
 * tüm kanalları step=0 (minimum kazanç) ile başlatır.                      */
int preamp_init(void) {
    if (preamp.baslandi) {
        preamp_log(PREAMP_LOG_WARN, "preamp_init zaten cagirildi");
        return PREAMP_OK;
    }

    memset(&preamp, 0, sizeof(preamp));
    for (int i = 0; i < PREAMP_CIP_SAYISI; i++) preamp.cip_handle[i] = -1;
    memset(lut_boyut, 0, sizeof(lut_boyut));

    if (pthread_mutex_init(&preamp.mutex, NULL) != 0) {
        preamp_log(PREAMP_LOG_ERROR, "Mutex baslatilamadi");
        return PREAMP_ERR_INIT;
    }

    /* Her çip için I2C handle aç */
    for (int i = 0; i < PREAMP_CIP_SAYISI; i++) {
        preamp.cip_handle[i] = lgI2cOpen(PREAMP_I2C_BUS, cip_adresler[i], 0);
        if (preamp.cip_handle[i] < 0) {
            preamp_log(PREAMP_LOG_ERROR, "Cip %d I2C acilamadi (adres=0x%02X hata=%d)",
                       i + 1, cip_adresler[i], preamp.cip_handle[i]);
            for (int j = 0; j < i; j++) {
                lgI2cClose(preamp.cip_handle[j]);
                preamp.cip_handle[j] = -1;
            }
            pthread_mutex_destroy(&preamp.mutex);
            return PREAMP_ERR_I2C;
        }
    }

    /* Bağlantı testi: tüm kanalları step=0 ile başlat */
    for (int k = 0; k < PREAMP_KANAL_SAYISI; k++) {
        int ret = mcp4651_yaz(kanal_map[k].i2c_addr, kanal_map[k].wiper, 0);
        if (ret != PREAMP_OK) {
            preamp_log(PREAMP_LOG_ERROR, "Kanal %d baslangic yazimi basarisiz", k + 1);
            for (int i = 0; i < PREAMP_CIP_SAYISI; i++) {
                lgI2cClose(preamp.cip_handle[i]);
                preamp.cip_handle[i] = -1;
            }
            pthread_mutex_destroy(&preamp.mutex);
            return ret;
        }
        preamp.mevcut_step[k] = 0;
    }

    preamp.baslandi = 1;
    preamp_log(PREAMP_LOG_INFO, "Preamp hazir — %d kanal, %d cip, kazanc=1x / 0dB",
               PREAMP_KANAL_SAYISI, PREAMP_CIP_SAYISI);
    return PREAMP_OK;
}

/* Tüm wiper'ları step=0'a çeker (minimum kazanç / güvenli konum),
 * I2C handle'larını kapatır ve mutex'i yok eder.                          */
void preamp_cleanup(void) {
    if (!preamp.baslandi) return;
    preamp_log(PREAMP_LOG_INFO, "Preamp temizleniyor");

    pthread_mutex_lock(&preamp.mutex);
    for (int k = 0; k < PREAMP_KANAL_SAYISI; k++)
        mcp4651_yaz(kanal_map[k].i2c_addr, kanal_map[k].wiper, 0);
    for (int i = 0; i < PREAMP_CIP_SAYISI; i++) {
        if (preamp.cip_handle[i] >= 0) {
            lgI2cClose(preamp.cip_handle[i]);
            preamp.cip_handle[i] = -1;
        }
    }
    pthread_mutex_unlock(&preamp.mutex);
    pthread_mutex_destroy(&preamp.mutex);

    preamp.baslandi = 0;
    preamp_log(PREAMP_LOG_INFO, "Preamp temizlendi");
}

/* Ham step değeriyle belirtilen kanalın kazancını ayarlar (0–255).
 * Kalibrasyon sırasında doğrudan step kontrolü için kullanılır.           */
int preamp_set_step(int kanal, uint8_t step) {
    if (!preamp.baslandi)                           return PREAMP_ERR_INIT;
    if (kanal < 0 || kanal >= PREAMP_KANAL_SAYISI) return PREAMP_ERR_KANAL;

    pthread_mutex_lock(&preamp.mutex);
    int ret = mcp4651_yaz(kanal_map[kanal].i2c_addr, kanal_map[kanal].wiper, step);
    if (ret == PREAMP_OK) preamp.mevcut_step[kanal] = step;
    pthread_mutex_unlock(&preamp.mutex);

    if (ret == PREAMP_OK)
        preamp_log(PREAMP_LOG_DEBUG, "Kanal %d: step=%u (%.3fx / %.2f dB)",
                   kanal + 1, step, PREAMP_GAIN_FORMUL(step),
                   20.0f * log10f(PREAMP_GAIN_FORMUL(step)));
    return ret;
}

/* dB cinsinden hedef kazancı kanalın LUT'u (veya formül) ile step'e çevirip yazar. */
int preamp_set_gain_db(int kanal, float kazanc_db) {
    if (!preamp.baslandi)                           return PREAMP_ERR_INIT;
    if (kanal < 0 || kanal >= PREAMP_KANAL_SAYISI) return PREAMP_ERR_KANAL;

    pthread_mutex_lock(&preamp.mutex);
    uint8_t step = kazanc_db_to_step(kanal, kazanc_db);
    int ret = mcp4651_yaz(kanal_map[kanal].i2c_addr, kanal_map[kanal].wiper, step);
    if (ret == PREAMP_OK) preamp.mevcut_step[kanal] = step;
    pthread_mutex_unlock(&preamp.mutex);
    return ret;
}

/* Lineer kazanç (örn. 2.5×) → dB → step dönüşümü yaparak yazar.
 * Minimum desteklenen kazanç 1.0× (0 dB, step=0).                        */
int preamp_set_gain_linear(int kanal, float gain) {
    if (!preamp.baslandi)                           return PREAMP_ERR_INIT;
    if (kanal < 0 || kanal >= PREAMP_KANAL_SAYISI) return PREAMP_ERR_KANAL;
    if (gain < 1.0f) {
        preamp_log(PREAMP_LOG_WARN, "Kazanc 1.0'dan kucuk olamaz (%.3f) — 1.0 kullaniliyor", gain);
        gain = 1.0f;
    }

    float kazanc_db = 20.0f * log10f(gain);

    pthread_mutex_lock(&preamp.mutex);
    uint8_t step = kazanc_db_to_step(kanal, kazanc_db);
    int ret = mcp4651_yaz(kanal_map[kanal].i2c_addr, kanal_map[kanal].wiper, step);
    if (ret == PREAMP_OK) preamp.mevcut_step[kanal] = step;
    pthread_mutex_unlock(&preamp.mutex);
    return ret;
}

/* Tüm 7 kanala aynı ham step değerini yazar.
 * İlk hata karşılaşılsa da tüm kanallar denenir; son hata kodu döner.    */
int preamp_set_all_step(uint8_t step) {
    if (!preamp.baslandi) return PREAMP_ERR_INIT;
    int ret = PREAMP_OK;

    pthread_mutex_lock(&preamp.mutex);
    for (int k = 0; k < PREAMP_KANAL_SAYISI; k++) {
        int r = mcp4651_yaz(kanal_map[k].i2c_addr, kanal_map[k].wiper, step);
        if (r == PREAMP_OK) preamp.mevcut_step[k] = step;
        else                ret = r;
    }
    pthread_mutex_unlock(&preamp.mutex);
    return ret;
}

/* Tüm 7 kanala aynı dB hedefini yazar.
 * Her kanal kendi LUT'unu kullanır: kanallar arası tolerans otomatik giderilir. */
int preamp_set_all_gain_db(float kazanc_db) {
    if (!preamp.baslandi) return PREAMP_ERR_INIT;
    int ret = PREAMP_OK;

    pthread_mutex_lock(&preamp.mutex);
    for (int k = 0; k < PREAMP_KANAL_SAYISI; k++) {
        uint8_t step = kazanc_db_to_step(k, kazanc_db);
        int r = mcp4651_yaz(kanal_map[k].i2c_addr, kanal_map[k].wiper, step);
        if (r == PREAMP_OK) preamp.mevcut_step[k] = step;
        else                ret = r;
    }
    pthread_mutex_unlock(&preamp.mutex);
    return ret;
}

/* Son yazılan step değerini önbellekten döndürür (donanım okuma yapmaz). */
int preamp_get_step(int kanal, uint8_t *step_out) {
    if (!preamp.baslandi)                           return PREAMP_ERR_INIT;
    if (kanal < 0 || kanal >= PREAMP_KANAL_SAYISI) return PREAMP_ERR_KANAL;
    if (!step_out)                                  return PREAMP_ERR_KANAL;

    pthread_mutex_lock(&preamp.mutex);
    *step_out = preamp.mevcut_step[kanal];
    pthread_mutex_unlock(&preamp.mutex);
    return PREAMP_OK;
}

/* Önbellekteki step'ten hesaplanan kazancı dB cinsinden döndürür.
 * Hata durumunda -1.0 döner (gerçek kazanç daima ≥ 0 dB).               */
float preamp_get_gain_db(int kanal) {
    if (!preamp.baslandi || kanal < 0 || kanal >= PREAMP_KANAL_SAYISI) return -1.0f;

    pthread_mutex_lock(&preamp.mutex);
    uint8_t step = preamp.mevcut_step[kanal];
    pthread_mutex_unlock(&preamp.mutex);

    return 20.0f * log10f(PREAMP_GAIN_FORMUL(step));
}

/* Önbellekteki step'ten hesaplanan lineer kazancı döndürür.
 * Hata durumunda -1.0 döner (gerçek kazanç daima ≥ 1.0×).               */
float preamp_get_gain_linear(int kanal) {
    if (!preamp.baslandi || kanal < 0 || kanal >= PREAMP_KANAL_SAYISI) return -1.0f;

    pthread_mutex_lock(&preamp.mutex);
    uint8_t step = preamp.mevcut_step[kanal];
    pthread_mutex_unlock(&preamp.mutex);

    return PREAMP_GAIN_FORMUL(step);
}

/* LUT'a (kazanc_db, step) çifti ekler; artan kazanc_db sırası korunur.
 * LUT'u kalibrasyon sırasında adım adım doldurmak için kullanılır.       */
int preamp_lut_ekle(int kanal, float kazanc_db, uint8_t step) {
    if (kanal < 0 || kanal >= PREAMP_KANAL_SAYISI) return PREAMP_ERR_KANAL;

    pthread_mutex_lock(&preamp.mutex);
    if (lut_boyut[kanal] >= PREAMP_LUT_MAX_NOKTA) {
        pthread_mutex_unlock(&preamp.mutex);
        preamp_log(PREAMP_LOG_WARN, "Kanal %d LUT dolu (%d)", kanal + 1, PREAMP_LUT_MAX_NOKTA);
        return PREAMP_ERR_LUT_DOLU;
    }

    /* Sıralı ekleme konumunu bul */
    int pos = lut_boyut[kanal];
    for (int i = 0; i < lut_boyut[kanal]; i++) {
        if (kazanc_db < lut[kanal][i].kazanc_db) { pos = i; break; }
    }

    /* Ekleme noktasından itibaren sağa kaydır */
    for (int i = lut_boyut[kanal]; i > pos; i--) lut[kanal][i] = lut[kanal][i - 1];

    lut[kanal][pos].kazanc_db = kazanc_db;
    lut[kanal][pos].step      = step;
    lut_boyut[kanal]++;
    int boyut = lut_boyut[kanal];
    pthread_mutex_unlock(&preamp.mutex);

    preamp_log(PREAMP_LOG_DEBUG, "Kanal %d LUT: %.2f dB → step %u eklendi (toplam %d)",
               kanal + 1, kazanc_db, step, boyut);
    return PREAMP_OK;
}

/* Belirtilen kanalın LUT'unu tamamen siler. */
void preamp_lut_temizle(int kanal) {
    if (kanal < 0 || kanal >= PREAMP_KANAL_SAYISI) return;
    pthread_mutex_lock(&preamp.mutex);
    lut_boyut[kanal] = 0;
    pthread_mutex_unlock(&preamp.mutex);
    preamp_log(PREAMP_LOG_DEBUG, "Kanal %d LUT temizlendi", kanal + 1);
}

/* Tüm kanalların LUT verilerini CSV dosyasına kaydeder.
 * Format: başlık satırı + "kanal,kazanc_db,step" satırları.
 * I2C mutex kısa tutulur; dosya I/O mutex dışında gerçekleşir.            */
int preamp_lut_kaydet_dosya(const char *path) {
    /* LUT verisinin yerel kopyasını al (mutex korumali, kısa kritik bölge) */
    Preamp_LUT_Entry yerel_lut[PREAMP_KANAL_SAYISI][PREAMP_LUT_MAX_NOKTA];
    int yerel_boyut[PREAMP_KANAL_SAYISI];

    pthread_mutex_lock(&preamp.mutex);
    for (int k = 0; k < PREAMP_KANAL_SAYISI; k++) {
        yerel_boyut[k] = lut_boyut[k];
        for (int i = 0; i < lut_boyut[k]; i++) yerel_lut[k][i] = lut[k][i];
    }
    pthread_mutex_unlock(&preamp.mutex);

    /* Dosya yazma (mutex dışında — I/O yavaş olabilir) */
    FILE *fp = fopen(path, "w");
    if (!fp) {
        preamp_log(PREAMP_LOG_ERROR, "Dosya acilamadi (yazma): %s", path);
        return PREAMP_ERR_DOSYA;
    }

    fprintf(fp, "kanal,kazanc_db,step\n");
    int toplam = 0;
    for (int k = 0; k < PREAMP_KANAL_SAYISI; k++) {
        for (int i = 0; i < yerel_boyut[k]; i++) {
            fprintf(fp, "%d,%.4f,%u\n", k, yerel_lut[k][i].kazanc_db, yerel_lut[k][i].step);
            toplam++;
        }
    }
    fclose(fp);

    preamp_log(PREAMP_LOG_INFO, "LUT kaydedildi: %s (%d nokta)", path, toplam);
    return PREAMP_OK;
}

/* CSV dosyasından LUT verilerini yükler.
 * Mevcut LUT'lar sıfırlanır. CSV manuel düzenlenmişse otomatik sıralanır. */
int preamp_lut_yukle_dosya(const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) {
        preamp_log(PREAMP_LOG_ERROR, "Dosya acilamadi (okuma): %s", path);
        return PREAMP_ERR_DOSYA;
    }

    /* Geçici LUT — mutex dışında doldurulur */
    Preamp_LUT_Entry yeni_lut[PREAMP_KANAL_SAYISI][PREAMP_LUT_MAX_NOKTA];
    int yeni_boyut[PREAMP_KANAL_SAYISI];
    memset(yeni_boyut, 0, sizeof(yeni_boyut));

    char line[128];
    if (!fgets(line, sizeof(line), fp)) { /* başlık satırını atla */
        fclose(fp);
        return PREAMP_ERR_DOSYA;
    }

    int okunan = 0, atlanan = 0;
    while (fgets(line, sizeof(line), fp)) {
        int      kanal;
        float    db;
        unsigned step;
        if (sscanf(line, "%d,%f,%u", &kanal, &db, &step) != 3)     { atlanan++; continue; }
        if (kanal < 0 || kanal >= PREAMP_KANAL_SAYISI)              { atlanan++; continue; }
        if (step > MCP4651_MAX_STEP)                                 { atlanan++; continue; }
        if (yeni_boyut[kanal] >= PREAMP_LUT_MAX_NOKTA)              { atlanan++; continue; }

        yeni_lut[kanal][yeni_boyut[kanal]].kazanc_db = db;
        yeni_lut[kanal][yeni_boyut[kanal]].step       = (uint8_t)step;
        yeni_boyut[kanal]++;
        okunan++;
    }
    fclose(fp);

    /* Manuel düzenlemeye karşı her kanalı kazanc_db'ye göre kabarcık sırala */
    for (int k = 0; k < PREAMP_KANAL_SAYISI; k++) {
        for (int i = 0; i < yeni_boyut[k] - 1; i++) {
            for (int j = 0; j < yeni_boyut[k] - 1 - i; j++) {
                if (yeni_lut[k][j].kazanc_db > yeni_lut[k][j + 1].kazanc_db) {
                    Preamp_LUT_Entry tmp = yeni_lut[k][j];
                    yeni_lut[k][j]      = yeni_lut[k][j + 1];
                    yeni_lut[k][j + 1]  = tmp;
                }
            }
        }
    }

    /* LUT'ları güncelle (mutex korumali) */
    pthread_mutex_lock(&preamp.mutex);
    for (int k = 0; k < PREAMP_KANAL_SAYISI; k++) {
        lut_boyut[k] = yeni_boyut[k];
        for (int i = 0; i < yeni_boyut[k]; i++) lut[k][i] = yeni_lut[k][i];
    }
    pthread_mutex_unlock(&preamp.mutex);

    preamp_log(PREAMP_LOG_INFO, "LUT yuklendi: %s (%d nokta, %d satir atlandi)",
               path, okunan, atlanan);
    return PREAMP_OK;
}

/* Log seviyesini ayarlar. Varsayılan: PREAMP_LOG_INFO. */
void preamp_set_log_level(PREAMP_LogLevel level) {
    log_seviyesi = level;
}
