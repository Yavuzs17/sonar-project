// gcc -o test phase_shift.c -llgpio -lpthread -lm -Wall -Wextra

#include "phase_shift.h"

#include <lgpio.h>
#include <math.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ─── Alıcı Eşleme Tablosu ──────────────────────────────────────────────────
 * Her alıcı: hangi MCP4651 chip + hangi wiper + hangi PCF8574 pin           */
typedef struct {
    uint8_t mcp_addr;   /* APF için MCP4651 I2C adresi    */
    uint8_t mcp_wiper;  /* 0 = Wiper0, 1 = Wiper1         */
    uint8_t pcf_pin;    /* PCF8574 pin numarası (0–6)     */
} PS_KanalMap;

static const PS_KanalMap kanal_map[PS_ALICI_SAYISI] = {
    { PS_MCP_CIP_5_ADDR, 0, 0 },  /* Alıcı 1 */
    { PS_MCP_CIP_5_ADDR, 1, 1 },  /* Alıcı 2 */
    { PS_MCP_CIP_6_ADDR, 0, 2 },  /* Alıcı 3 */
    { PS_MCP_CIP_6_ADDR, 1, 3 },  /* Alıcı 4 */
    { PS_MCP_CIP_7_ADDR, 0, 4 },  /* Alıcı 5 */
    { PS_MCP_CIP_7_ADDR, 1, 5 },  /* Alıcı 6 */
    { PS_MCP_CIP_8_ADDR, 0, 6 },  /* Alıcı 7 — Wiper1 TVG'de, dokunma */
};

/* ─── Kalibrasyon Verisi ────────────────────────────────────────────────────*/
static PS_AliciKalib alici_kalib[PS_ALICI_SAYISI];

/* ─── İç Durum Yapısı ───────────────────────────────────────────────────────*/
static struct {
    int     mcp_handle[4];              /* CIP 5..8 için I2C handle'lar       */
    int     pcf_handle;                 /* PCF8574 için I2C handle            */
    uint8_t pcf_durum;                  /* PCF8574 çıkış byte'ı (tüm pinler) */
    uint8_t mevcut_step[PS_ALICI_SAYISI]; /* Her kanalın son yazılan step'i  */
    bool    mevcut_inv[PS_ALICI_SAYISI];  /* Her kanalın inverter durumu     */
    pthread_mutex_t mutex;
    int             baslandi;
} ps;

/* ─── Log Sistemi ───────────────────────────────────────────────────────────*/
static PS_LogLevel log_seviyesi = PS_LOG_INFO;

static void ps_log(PS_LogLevel seviye, const char *fmt, ...) {
    if (seviye < log_seviyesi) return;
    static const char * const etiketler[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    FILE *hedef = (seviye >= PS_LOG_WARN) ? stderr : stdout;
    fprintf(hedef, "[PS][%s] ", etiketler[seviye]);
    va_list arglar;
    va_start(arglar, fmt);
    vfprintf(hedef, fmt, arglar);
    va_end(arglar);
    fputc('\n', hedef);
}

/* ─── I2C Yardımcısı ────────────────────────────────────────────────────────*/

/* MCP4651 adresinden I2C handle döndürür. Tanımsız adres için -1.          */
static int mcp_handle_al(uint8_t addr) {
    switch (addr) {
        case PS_MCP_CIP_5_ADDR: return ps.mcp_handle[0];
        case PS_MCP_CIP_6_ADDR: return ps.mcp_handle[1];
        case PS_MCP_CIP_7_ADDR: return ps.mcp_handle[2];
        case PS_MCP_CIP_8_ADDR: return ps.mcp_handle[3];
        default: return -1;
    }
}

/* ─── MCP4651 Sürücüsü ──────────────────────────────────────────────────────*/

/* Belirtilen wiper'a step değeri yazar (2 byte I2C write).
 * Komut byte: Wiper0=0x00, Wiper1=0x10.                                     */
static int mcp4651_yaz(uint8_t addr, uint8_t wiper, uint8_t step) {
    int handle = mcp_handle_al(addr);
    if (handle < 0) return PS_ERR_I2C;

    uint8_t buf[2];
    buf[0] = (wiper == 0) ? 0x00 : 0x10;
    buf[1] = step;

    for (int d = 0; d < PS_I2C_MAX_DENEME; d++) {
        if (lgI2cWriteDevice(handle, (char *)buf, 2) == 0) return PS_OK;
        ps_log(PS_LOG_WARN, "MCP 0x%02X W%u yaz hatasi, deneme %d/%d",
               addr, wiper, d + 1, PS_I2C_MAX_DENEME);
    }
    return PS_ERR_I2C;
}

/* Wiper step değerini okur (okuma komutu: Wiper0=0x0C, Wiper1=0x1C).       */
static int mcp4651_oku(uint8_t addr, uint8_t wiper, uint8_t *step_out) {
    int handle = mcp_handle_al(addr);
    if (handle < 0 || !step_out) return PS_ERR_I2C;

    uint8_t cmd = (wiper == 0) ? 0x0C : 0x1C;

    for (int d = 0; d < PS_I2C_MAX_DENEME; d++) {
        if (lgI2cWriteDevice(handle, (char *)&cmd, 1) != 0) continue;
        uint8_t rbuf[2] = {0, 0};
        if (lgI2cReadDevice(handle, (char *)rbuf, 2) == 2) {
            *step_out = rbuf[1];  /* D8 daima 0; step = D7..D0 */
            return PS_OK;
        }
    }
    return PS_ERR_I2C;
}

/* ─── PCF8574 Sürücüsü ──────────────────────────────────────────────────────*/

/* Tek pin'i günceller; sonra tüm byte'ı PCF8574'e yazar.
 * Quasi-bidirectional yapı gereği tüm pinler bir seferde yazılır.           */
static int pcf8574_pin_yaz(uint8_t pin_no, bool on) {
    if (on)
        ps.pcf_durum |=  (uint8_t)(1u << pin_no);
    else
        ps.pcf_durum &= (uint8_t)~(1u << pin_no);

    uint8_t buf = ps.pcf_durum;
    for (int d = 0; d < PS_I2C_MAX_DENEME; d++) {
        if (lgI2cWriteDevice(ps.pcf_handle, (char *)&buf, 1) == 0) return PS_OK;
        ps_log(PS_LOG_WARN, "PCF8574 pin %u yaz hatasi, deneme %d/%d",
               pin_no, d + 1, PS_I2C_MAX_DENEME);
    }
    return PS_ERR_I2C;
}

/* PCF8574 çıkış durumunu tüm byte olarak okur.                              */
static int pcf8574_oku(uint8_t *durum_out) {
    if (!durum_out) return PS_ERR_PARAM;
    for (int d = 0; d < PS_I2C_MAX_DENEME; d++) {
        uint8_t buf = 0;
        if (lgI2cReadDevice(ps.pcf_handle, (char *)&buf, 1) == 1) {
            *durum_out = buf;
            return PS_OK;
        }
    }
    return PS_ERR_I2C;
}

/* ─── Kalibrasyon Başlatma ──────────────────────────────────────────────────*/

/* Alıcı kalibrasyonunu fabrika değerlerine sıfırlar.                        */
static void kalib_sifirla(int alici) {
    PS_AliciKalib *k = &alici_kalib[alici];
    memset(k, 0, sizeof(*k));
    k->kapasitor_f    = PS_APF_KAPASITOR_F;
    k->min_aci_off    = PS_APF_OFF_MIN_DEG;
    k->max_aci_off    = PS_APF_OFF_MAX_DEG;
    k->min_aci_on     = PS_APF_ON_MIN_DEG;
    k->max_aci_on     = PS_APF_ON_MAX_DEG;
    k->kalibre_edildi = false;
}

/* ─── APF Formül Yardımcıları ───────────────────────────────────────────────*/

/* Teorik APF faz hesabı: faz = -2 × atan(ω × R(N) × C) × (180/π)
 * Sonuç [-166°, 0°] aralığında (inverter olmadan).                          */
static float apf_aci_hesapla(uint8_t step, float C) {
    float R       = PS_R_WIPER_OHM + PS_R_STEP_OHM * step;
    float omega   = 2.0f * (float)M_PI * PS_VARSAYILAN_FREK_HZ;
    float faz_rad = -2.0f * atanf(omega * R * C);
    return faz_rad * 180.0f / (float)M_PI;
}

/* APF hedef faz açısından step hesaplar (ters formül).
 * target_apf: [-166°, 0°] aralığında, derece cinsinden.                    */
static uint8_t step_hesapla(float target_apf, float C) {
    /* -2 × atan(ωRC) = target_apf  →  ωRC = tan(-target_apf / 2 × π/180)  */
    float omega   = 2.0f * (float)M_PI * PS_VARSAYILAN_FREK_HZ;
    float arg     = -target_apf * (float)M_PI / 360.0f;  /* -target/2 rad  */
    float tan_val = tanf(arg);
    if (tan_val < 0.0f) tan_val = 0.0f;                  /* Negatif R olmaz */
    float R       = tan_val / (omega * C);
    float N       = (R - PS_R_WIPER_OHM) / PS_R_STEP_OHM;
    if (N < 0.0f)   N = 0.0f;
    if (N > 255.0f) N = 255.0f;
    return (uint8_t)(N + 0.5f);
}

/* ─── Fit Fonksiyonu ────────────────────────────────────────────────────────
 * Kalibrasyon noktalarından C ve ofset parametrelerini grid search ile fit eder.
 * OFF noktaları: C ve ofset_apf_off; ON noktaları: ofset_apf_on.            */
static int ps_fit_alici(int alici) {
    PS_AliciKalib *kalib = &alici_kalib[alici];

    int n_off = 0, n_on = 0;
    for (int k = 0; k < kalib->nokta_sayisi; k++) {
        if (kalib->noktalar[k].inverter_on) n_on++;
        else                                n_off++;
    }

    if (n_off < 2) {
        ps_log(PS_LOG_WARN, "Alici %d: fit icin en az 2 OFF nokta gerekli (%d var)",
               alici, n_off);
        return PS_ERR_FIT;
    }

    /* Grid search: C ∈ [0.05nF, 10nF], 200 adım, 0.05nF çözünürlük         */
    float best_C   = PS_APF_KAPASITOR_F;
    float best_off = 0.0f;
    float best_rss = 1e30f;

    for (int ci = 1; ci <= 200; ci++) {
        float C = ci * 0.05e-9f;

        /* Bu C için ortalama offset hesapla */
        float sum = 0.0f;
        for (int k = 0; k < kalib->nokta_sayisi; k++) {
            if (kalib->noktalar[k].inverter_on) continue;
            float pred = apf_aci_hesapla(kalib->noktalar[k].step, C);
            sum += kalib->noktalar[k].olculen_aci - pred;
        }
        float ofset = sum / n_off;

        /* Kalıntı kareler toplamı */
        float rss = 0.0f;
        for (int k = 0; k < kalib->nokta_sayisi; k++) {
            if (kalib->noktalar[k].inverter_on) continue;
            float pred = apf_aci_hesapla(kalib->noktalar[k].step, C) + ofset;
            float err  = kalib->noktalar[k].olculen_aci - pred;
            rss += err * err;
        }

        if (rss < best_rss) {
            best_rss = rss;
            best_C   = C;
            best_off = ofset;
        }
    }

    kalib->kapasitor_f   = best_C;
    kalib->ofset_apf_off = best_off;

    /* ON noktalarından ofset_apf_on: ölçüm = apf_pred + ofset_off - 180 + ofset_on */
    if (n_on >= 1) {
        float sum_on = 0.0f;
        for (int k = 0; k < kalib->nokta_sayisi; k++) {
            if (!kalib->noktalar[k].inverter_on) continue;
            float apf_pred = apf_aci_hesapla(kalib->noktalar[k].step, best_C) + best_off;
            sum_on += kalib->noktalar[k].olculen_aci - (apf_pred - 180.0f);
        }
        kalib->ofset_apf_on = sum_on / n_on;
    } else {
        kalib->ofset_apf_on = 0.0f;
    }

    /* Çalışma sınırlarını ölçüm noktalarından güncelle */
    kalib->min_aci_off = kalib->max_aci_off = 0.0f;  /* ilk OFF ölçümüyle başlatılacak */
    kalib->min_aci_on  = kalib->max_aci_on  = 0.0f;
    bool ilk_off = true, ilk_on = true;

    for (int k = 0; k < kalib->nokta_sayisi; k++) {
        float a = kalib->noktalar[k].olculen_aci;
        if (!kalib->noktalar[k].inverter_on) {
            if (ilk_off) { kalib->min_aci_off = kalib->max_aci_off = a; ilk_off = false; }
            else {
                if (a > kalib->min_aci_off) kalib->min_aci_off = a;  /* en küçük negatif */
                if (a < kalib->max_aci_off) kalib->max_aci_off = a;  /* en büyük negatif */
            }
        } else {
            if (ilk_on)  { kalib->min_aci_on  = kalib->max_aci_on  = a; ilk_on  = false; }
            else {
                if (a > kalib->min_aci_on)  kalib->min_aci_on  = a;
                if (a < kalib->max_aci_on)  kalib->max_aci_on  = a;
            }
        }
    }
    /* Nokta yoksa fabrika değerini koru */
    if (ilk_off) { kalib->min_aci_off = PS_APF_OFF_MIN_DEG; kalib->max_aci_off = PS_APF_OFF_MAX_DEG; }
    if (ilk_on)  { kalib->min_aci_on  = PS_APF_ON_MIN_DEG;  kalib->max_aci_on  = PS_APF_ON_MAX_DEG;  }

    kalib->kalibre_edildi = true;
    ps_log(PS_LOG_INFO,
           "Alici %d fit: C=%.3fnF ofset_off=%.2f° ofset_on=%.2f° RSS=%.4f",
           alici, best_C * 1e9f, best_off, kalib->ofset_apf_on, best_rss);
    return PS_OK;
}

/* ─── Açıdan (Step, Inverter) Hesaplama ────────────────────────────────────
 * hedef_aci: 0–360° (beamforming çıktısı).
 * Devre domenine çevrilir (−360° ile 0°), uygun aralık seçilir.
 * Kör noktada: en yakın geçerli açıya yuvarlanır; PS_ERR_KOR_NOKTA döner.  */
static int ps_aci_to_ayar(int alici, float hedef_aci, PS_AyarSonuc *sonuc) {
    PS_AliciKalib *kalib = &alici_kalib[alici];

    /* 0–360° normalize */
    float hedef = fmodf(hedef_aci, 360.0f);
    if (hedef < 0.0f) hedef += 360.0f;

    /* Devre domenine çevir: APF gecikme eşdeğeri → negatif açı */
    float circuit = (hedef > 0.0f) ? (hedef - 360.0f) : 0.0f;

    /* Geçerli aralıklar (circuit domain, negatif, max < min) */
    float off_min = kalib->min_aci_off;   /* ≈ -7°    */
    float off_max = kalib->max_aci_off;   /* ≈ -166°  */
    float on_min  = kalib->min_aci_on;    /* ≈ -187°  */
    float on_max  = kalib->max_aci_on;    /* ≈ -346°  */

    bool try_off = (circuit >= off_max && circuit <= off_min);
    bool try_on  = (circuit >= on_max  && circuit <= on_min);

    int   hata    = PS_OK;
    bool  use_inv = false;
    float target_circuit = circuit;

    if (!try_off && !try_on) {
        /* Kör nokta: en yakın geçerli sınıra yuvarla */
        hata = PS_ERR_KOR_NOKTA;

        float d[4] = {
            fabsf(circuit - off_min),
            fabsf(circuit - off_max),
            fabsf(circuit - on_min),
            fabsf(circuit - on_max)
        };
        /* Çembersel sarma için: 0° ile -346° arasındaki mesafe de kontrol */
        if (circuit > off_min) d[0] = fabsf(circuit - (off_min - 360.0f));
        if (circuit > on_min)  d[2] = fabsf(circuit - (on_min  - 360.0f));

        float best = d[0]; use_inv = false; target_circuit = off_min;
        if (d[1] < best) { best = d[1]; use_inv = false; target_circuit = off_max; }
        if (d[2] < best) { best = d[2]; use_inv = true;  target_circuit = on_min;  }
        if (d[3] < best) {              use_inv = true;  target_circuit = on_max;  }
    } else if (try_off) {
        use_inv = false;
    } else {
        use_inv = true;
    }

    /* Kalibrasyon değerleri */
    float C        = kalib->kalibre_edildi ? kalib->kapasitor_f    : PS_APF_KAPASITOR_F;
    float ofset    = use_inv
                     ? (kalib->kalibre_edildi ? kalib->ofset_apf_on  : 0.0f)
                     : (kalib->kalibre_edildi ? kalib->ofset_apf_off : 0.0f);

    /* APF hedef açısı (inverter bileşeni çıkarılmış, ofset düzeltilmiş)     */
    float apf_target;
    if (use_inv) {
        /* target_circuit = apf + ofset_off - 180 + ofset_on
         * → apf = target - ofset_on + 180 - ofset_off               */
        float ofset_off = kalib->kalibre_edildi ? kalib->ofset_apf_off : 0.0f;
        apf_target = target_circuit - ofset + 180.0f - ofset_off;
    } else {
        apf_target = target_circuit - ofset;
    }

    /* [-166°, 0°] aralığına kırp */
    if (apf_target >   0.0f) apf_target =   0.0f;
    if (apf_target < -166.3f) apf_target = -166.3f;

    /* Step hesapla */
    uint8_t step = step_hesapla(apf_target, C);

    /* Gerçek üretilen açıyı hesapla */
    float apf_actual = apf_aci_hesapla(step, C);
    float actual_circuit = use_inv ? (apf_actual + ofset - 180.0f) : (apf_actual + ofset);
    float actual_0_360   = actual_circuit + 360.0f;
    if (actual_0_360 >= 360.0f) actual_0_360 -= 360.0f;
    if (actual_0_360 < 0.0f)    actual_0_360 += 360.0f;

    /* Açısal sapma (çembersel) */
    float sapma = fabsf(hedef - actual_0_360);
    if (sapma > 180.0f) sapma = 360.0f - sapma;

    sonuc->step        = step;
    sonuc->inverter_on = use_inv;
    sonuc->gercek_aci  = actual_0_360;
    sonuc->sapma_deg   = sapma;

    return hata;
}

/* ─── Denge Ofseti Algoritması ─────────────────────────────────────────────
 * 7 alıcının ham fazlarına ortak bir ofset ekleyerek kör nokta çakışmasını
 * minimuma indirir. Kaba arama (5° adım) + ince arama (0.5° adım).         */
static int ps_denge_bul(const float ham_aciler[PS_ALICI_SAYISI], PS_DengeAyari *sonuc) {
    float best_ofset = 0.0f;
    float best_sapma = 1e30f;

    /* Kaba arama: 0°–355°, 5° adım (72 deneme) */
    for (int o = 0; o < 360; o += 5) {
        float ofset         = (float)o;
        float toplam_sapma  = 0.0f;

        for (int i = 0; i < PS_ALICI_SAYISI; i++) {
            float hedef = fmodf(ham_aciler[i] + ofset, 360.0f);
            if (hedef < 0.0f) hedef += 360.0f;
            PS_AyarSonuc ayar;
            ps_aci_to_ayar(i, hedef, &ayar);
            toplam_sapma += ayar.sapma_deg;
        }

        if (toplam_sapma < best_sapma) {
            best_sapma = toplam_sapma;
            best_ofset = ofset;
        }
    }

    /* İnce arama: best_ofset ± 5°, 0.5° adım (21 deneme) */
    for (int oi = -10; oi <= 10; oi++) {
        float ofset = fmodf(best_ofset + oi * 0.5f, 360.0f);
        if (ofset < 0.0f) ofset += 360.0f;
        float toplam_sapma = 0.0f;

        for (int i = 0; i < PS_ALICI_SAYISI; i++) {
            float hedef = fmodf(ham_aciler[i] + ofset, 360.0f);
            if (hedef < 0.0f) hedef += 360.0f;
            PS_AyarSonuc ayar;
            ps_aci_to_ayar(i, hedef, &ayar);
            toplam_sapma += ayar.sapma_deg;
        }

        if (toplam_sapma < best_sapma) {
            best_sapma = toplam_sapma;
            best_ofset = ofset;
        }
    }

    /* Seçilen ofset ile nihai ayarları hesapla */
    sonuc->ofset_deg    = best_ofset;
    sonuc->toplam_sapma = best_sapma;
    for (int i = 0; i < PS_ALICI_SAYISI; i++) {
        float hedef = fmodf(ham_aciler[i] + best_ofset, 360.0f);
        if (hedef < 0.0f) hedef += 360.0f;
        ps_aci_to_ayar(i, hedef, &sonuc->ayarlar[i]);
    }

    return PS_OK;
}

/* ─── Public API ─────────────────────────────────────────────────────────── */

/* 4 MCP4651 ve 1 PCF8574 I2C bağlantısını açar, bağlantı testleri yapar,
 * tüm wiper'ları orta konuma (step=128) ve inverter'ları OFF'a çeker.       */
int ps_init(void) {
    if (ps.baslandi) {
        ps_log(PS_LOG_WARN, "ps_init zaten cagirildi");
        return PS_OK;
    }

    memset(&ps, 0, sizeof(ps));
    for (int i = 0; i < 4; i++) ps.mcp_handle[i] = -1;
    ps.pcf_handle = -1;

    if (pthread_mutex_init(&ps.mutex, NULL) != 0) {
        ps_log(PS_LOG_ERROR, "Mutex baslatilamadi");
        return PS_ERR_INIT;
    }

    /* MCP4651 chip'lerini aç */
    uint8_t mcp_adresleri[4] = {
        PS_MCP_CIP_5_ADDR, PS_MCP_CIP_6_ADDR,
        PS_MCP_CIP_7_ADDR, PS_MCP_CIP_8_ADDR
    };
    for (int i = 0; i < 4; i++) {
        ps.mcp_handle[i] = lgI2cOpen(PS_I2C_BUS, mcp_adresleri[i], 0);
        if (ps.mcp_handle[i] < 0) {
            ps_log(PS_LOG_ERROR, "MCP 0x%02X acilamadi: hata=%d",
                   mcp_adresleri[i], ps.mcp_handle[i]);
            goto hata_cikis;
        }
    }

    /* PCF8574'ü aç */
    ps.pcf_handle = lgI2cOpen(PS_I2C_BUS, PS_PCF_ADDR, 0);
    if (ps.pcf_handle < 0) {
        ps_log(PS_LOG_ERROR, "PCF8574 0x%02X acilamadi: hata=%d",
               PS_PCF_ADDR, ps.pcf_handle);
        goto hata_cikis;
    }

    /* Tüm inverter'ları OFF yap (0x00 → tüm P0..P7 LOW) */
    ps.pcf_durum = 0x00;
    uint8_t sifir = 0x00;
    if (lgI2cWriteDevice(ps.pcf_handle, (char *)&sifir, 1) != 0) {
        ps_log(PS_LOG_ERROR, "PCF8574 baslangic yazma hatasi");
        goto hata_cikis;
    }

    /* Tüm wiper'ları orta konuma çek (step=128) ve önbelleği doldur */
    for (int alici = 0; alici < PS_ALICI_SAYISI; alici++) {
        const PS_KanalMap *km = &kanal_map[alici];
        if (mcp4651_yaz(km->mcp_addr, km->mcp_wiper, 128) != PS_OK) {
            ps_log(PS_LOG_ERROR, "Alici %d MCP baslangic yazma hatasi", alici);
            goto hata_cikis;
        }
        ps.mevcut_step[alici] = 128;
        ps.mevcut_inv[alici]  = false;
        kalib_sifirla(alici);
    }

    ps.baslandi = 1;
    ps_log(PS_LOG_INFO,
           "PhaseShift hazir — %d alici, 4 MCP4651 + PCF8574 (bus %d)",
           PS_ALICI_SAYISI, PS_I2C_BUS);
    return PS_OK;

hata_cikis:
    for (int i = 0; i < 4; i++)
        if (ps.mcp_handle[i] >= 0) { lgI2cClose(ps.mcp_handle[i]); ps.mcp_handle[i] = -1; }
    if (ps.pcf_handle >= 0) { lgI2cClose(ps.pcf_handle); ps.pcf_handle = -1; }
    pthread_mutex_destroy(&ps.mutex);
    return PS_ERR_INIT;
}

/* Tüm wiper'ları step=128'e, inverter'ları OFF'a çeker, I2C kapatır.       */
void ps_cleanup(void) {
    if (!ps.baslandi) return;
    ps_log(PS_LOG_INFO, "PhaseShift temizleniyor");

    pthread_mutex_lock(&ps.mutex);

    /* Güvenli son durum: wiper orta, inverter OFF */
    for (int alici = 0; alici < PS_ALICI_SAYISI; alici++) {
        const PS_KanalMap *km = &kanal_map[alici];
        mcp4651_yaz(km->mcp_addr, km->mcp_wiper, 128);
    }
    uint8_t sifir = 0x00;
    lgI2cWriteDevice(ps.pcf_handle, (char *)&sifir, 1);

    for (int i = 0; i < 4; i++)
        if (ps.mcp_handle[i] >= 0) { lgI2cClose(ps.mcp_handle[i]); ps.mcp_handle[i] = -1; }
    if (ps.pcf_handle >= 0) { lgI2cClose(ps.pcf_handle); ps.pcf_handle = -1; }

    pthread_mutex_unlock(&ps.mutex);
    pthread_mutex_destroy(&ps.mutex);
    ps.baslandi = 0;
    ps_log(PS_LOG_INFO, "PhaseShift temizlendi");
}

/* Ham donanım erişimi: step ve inverter durumunu doğrudan yazar (test için). */
int ps_set_step(int alici, uint8_t step, bool inverter_on) {
    if (!ps.baslandi)                              return PS_ERR_INIT;
    if (alici < 0 || alici >= PS_ALICI_SAYISI)    return PS_ERR_KANAL;

    pthread_mutex_lock(&ps.mutex);
    const PS_KanalMap *km = &kanal_map[alici];
    int ret = mcp4651_yaz(km->mcp_addr, km->mcp_wiper, step);
    if (ret == PS_OK) {
        ret = pcf8574_pin_yaz(km->pcf_pin, inverter_on);
    }
    if (ret == PS_OK) {
        ps.mevcut_step[alici] = step;
        ps.mevcut_inv[alici]  = inverter_on;
    }
    pthread_mutex_unlock(&ps.mutex);

    ps_log(PS_LOG_DEBUG, "Alici %d: step=%u inv=%d", alici, step, inverter_on);
    return ret;
}

/* Önbellekten son yazılan step ve inverter değerini döndürür.               */
int ps_get_step(int alici, uint8_t *step_out, bool *inverter_out) {
    if (!ps.baslandi)                              return PS_ERR_INIT;
    if (alici < 0 || alici >= PS_ALICI_SAYISI)    return PS_ERR_KANAL;

    pthread_mutex_lock(&ps.mutex);
    if (step_out)    *step_out    = ps.mevcut_step[alici];
    if (inverter_out) *inverter_out = ps.mevcut_inv[alici];
    pthread_mutex_unlock(&ps.mutex);
    return PS_OK;
}

/* Hedef açıyı (0–360°) en uygun (step, inverter) çiftine dönüştürüp yazar.
 * Kalibrasyon yoksa nominal formül kullanılır → PS_ERR_KALIB_YOK döner.
 * Kör noktada en yakın geçerli açı uygulanır → PS_ERR_KOR_NOKTA döner.     */
int ps_set_angle(int alici, float hedef_aci, float *gercek_aci_out) {
    if (!ps.baslandi)                              return PS_ERR_INIT;
    if (alici < 0 || alici >= PS_ALICI_SAYISI)    return PS_ERR_KANAL;

    pthread_mutex_lock(&ps.mutex);

    PS_AyarSonuc ayar;
    int hata = ps_aci_to_ayar(alici, hedef_aci, &ayar);

    /* Kalibrasyon uyarısı (donanım yazma yine de yapılır) */
    if (hata == PS_OK && !alici_kalib[alici].kalibre_edildi)
        hata = PS_ERR_KALIB_YOK;

    const PS_KanalMap *km = &kanal_map[alici];
    int i2c_ret = mcp4651_yaz(km->mcp_addr, km->mcp_wiper, ayar.step);
    if (i2c_ret == PS_OK)
        i2c_ret = pcf8574_pin_yaz(km->pcf_pin, ayar.inverter_on);

    if (i2c_ret == PS_OK) {
        ps.mevcut_step[alici] = ayar.step;
        ps.mevcut_inv[alici]  = ayar.inverter_on;
        if (gercek_aci_out) *gercek_aci_out = ayar.gercek_aci;
    }

    pthread_mutex_unlock(&ps.mutex);

    if (i2c_ret != PS_OK) return PS_ERR_I2C;

    ps_log(PS_LOG_DEBUG, "Alici %d: hedef=%.1f° gercek=%.1f° sapma=%.2f° step=%u inv=%d",
           alici, hedef_aci, ayar.gercek_aci, ayar.sapma_deg, ayar.step, ayar.inverter_on);
    return hata;
}

/* 7 alıcının tüm fazlarını otomatik denge ofseti ile birlikte ayarlar.
 * gercek_aciler_out NULL değilse, her alıcının uygulanan açısını yazar.     */
int ps_set_all_angles(const float hedef_aciler[PS_ALICI_SAYISI],
                      float gercek_aciler_out[PS_ALICI_SAYISI]) {
    if (!ps.baslandi) return PS_ERR_INIT;
    if (!hedef_aciler) return PS_ERR_PARAM;

    pthread_mutex_lock(&ps.mutex);

    PS_DengeAyari denge;
    ps_denge_bul(hedef_aciler, &denge);

    int ilk_hata = PS_OK;
    for (int i = 0; i < PS_ALICI_SAYISI; i++) {
        const PS_KanalMap *km = &kanal_map[i];
        int ret = mcp4651_yaz(km->mcp_addr, km->mcp_wiper, denge.ayarlar[i].step);
        if (ret == PS_OK)
            ret = pcf8574_pin_yaz(km->pcf_pin, denge.ayarlar[i].inverter_on);

        if (ret == PS_OK) {
            ps.mevcut_step[i] = denge.ayarlar[i].step;
            ps.mevcut_inv[i]  = denge.ayarlar[i].inverter_on;
            if (gercek_aciler_out) gercek_aciler_out[i] = denge.ayarlar[i].gercek_aci;
        } else if (ilk_hata == PS_OK) {
            ilk_hata = PS_ERR_I2C;
            ps_log(PS_LOG_WARN, "Alici %d toplu faz yazma hatasi", i);
        }
    }

    pthread_mutex_unlock(&ps.mutex);

    ps_log(PS_LOG_DEBUG, "Toplu faz: ofset=%.1f° toplam_sapma=%.2f°",
           denge.ofset_deg, denge.toplam_sapma);
    return ilk_hata;
}

/* Belirli bir alıcıya kalibrasyon ölçüm noktası ekler.
 * PS_KALIB_NOKTA_MAX sınırına ulaşıldığında PS_ERR_PARAM döner.            */
int ps_kalib_nokta_ekle(int alici, uint8_t step, bool inverter_on, float olculen_aci) {
    if (!ps.baslandi)                              return PS_ERR_INIT;
    if (alici < 0 || alici >= PS_ALICI_SAYISI)    return PS_ERR_KANAL;

    pthread_mutex_lock(&ps.mutex);
    PS_AliciKalib *kalib = &alici_kalib[alici];
    int ret = PS_OK;

    if (kalib->nokta_sayisi >= PS_KALIB_NOKTA_MAX) {
        ps_log(PS_LOG_WARN, "Alici %d: kalibrasyon LUT dolu (%d nokta)", alici, PS_KALIB_NOKTA_MAX);
        ret = PS_ERR_PARAM;
    } else {
        PS_KalibPunto *p = &kalib->noktalar[kalib->nokta_sayisi++];
        p->step        = step;
        p->inverter_on = inverter_on;
        p->olculen_aci = olculen_aci;
        ps_log(PS_LOG_DEBUG, "Alici %d kalib nokta: step=%u inv=%d aci=%.2f°",
               alici, step, inverter_on, olculen_aci);
    }

    pthread_mutex_unlock(&ps.mutex);
    return ret;
}

/* Alıcının tüm kalibrasyon noktalarını siler ve fabrika değerlerine döner. */
int ps_kalib_nokta_temizle(int alici) {
    if (!ps.baslandi)                              return PS_ERR_INIT;
    if (alici < 0 || alici >= PS_ALICI_SAYISI)    return PS_ERR_KANAL;

    pthread_mutex_lock(&ps.mutex);
    kalib_sifirla(alici);
    pthread_mutex_unlock(&ps.mutex);

    ps_log(PS_LOG_INFO, "Alici %d kalibrasyon temizlendi", alici);
    return PS_OK;
}

/* Mevcut ölçüm noktalarından formül parametrelerini fit eder.
 * Başarı: kalibre_edildi=true; Hata: PS_ERR_FIT.                           */
int ps_kalib_fit(int alici) {
    if (!ps.baslandi)                              return PS_ERR_INIT;
    if (alici < 0 || alici >= PS_ALICI_SAYISI)    return PS_ERR_KANAL;

    pthread_mutex_lock(&ps.mutex);
    int ret = ps_fit_alici(alici);
    pthread_mutex_unlock(&ps.mutex);
    return ret;
}

/* Tüm alıcıların kalibrasyon noktalarını CSV dosyasına kaydeder.
 * Format: alici,step,inverter,olculen_aci  (başlık satırı dahil)            */
int ps_kalib_kaydet_dosya(const char *path) {
    if (!path) return PS_ERR_PARAM;

    /* Mutex kısa tutulur; I/O dışında yapılır */
    PS_AliciKalib yerel[PS_ALICI_SAYISI];
    pthread_mutex_lock(&ps.mutex);
    memcpy(yerel, alici_kalib, sizeof(yerel));
    pthread_mutex_unlock(&ps.mutex);

    FILE *fp = fopen(path, "w");
    if (!fp) {
        ps_log(PS_LOG_ERROR, "Dosya acilamadi (yazma): %s", path);
        return PS_ERR_DOSYA;
    }

    fprintf(fp, "alici,step,inverter,olculen_aci\n");
    int toplam = 0;
    for (int alici = 0; alici < PS_ALICI_SAYISI; alici++) {
        for (int k = 0; k < yerel[alici].nokta_sayisi; k++) {
            PS_KalibPunto *p = &yerel[alici].noktalar[k];
            fprintf(fp, "%d,%u,%d,%.4f\n",
                    alici, p->step, p->inverter_on ? 1 : 0, p->olculen_aci);
            toplam++;
        }
    }
    fclose(fp);

    ps_log(PS_LOG_INFO, "Kalibrasyon kaydedildi: %s (%d nokta)", path, toplam);
    return PS_OK;
}

/* CSV dosyasından kalibrasyon noktalarını yükler.
 * Mevcut verinin üstüne yazar. Geçersiz satırlar atlanır.                   */
int ps_kalib_yukle_dosya(const char *path) {
    if (!path) return PS_ERR_PARAM;

    FILE *fp = fopen(path, "r");
    if (!fp) {
        ps_log(PS_LOG_ERROR, "Dosya acilamadi (okuma): %s", path);
        return PS_ERR_DOSYA;
    }

    /* Mevcut kalibrasyon verilerini temizle */
    pthread_mutex_lock(&ps.mutex);
    for (int i = 0; i < PS_ALICI_SAYISI; i++) kalib_sifirla(i);
    pthread_mutex_unlock(&ps.mutex);

    char line[128];
    if (!fgets(line, sizeof(line), fp)) { fclose(fp); return PS_ERR_DOSYA; } /* başlık */

    int okunan = 0, atlanan = 0;
    while (fgets(line, sizeof(line), fp)) {
        int      alici, inv_int;
        unsigned step;
        float    aci;
        if (sscanf(line, "%d,%u,%d,%f", &alici, &step, &inv_int, &aci) != 4) { atlanan++; continue; }
        if (alici < 0 || alici >= PS_ALICI_SAYISI) { atlanan++; continue; }
        if (step  > 255)                           { atlanan++; continue; }

        pthread_mutex_lock(&ps.mutex);
        PS_AliciKalib *kalib = &alici_kalib[alici];
        if (kalib->nokta_sayisi < PS_KALIB_NOKTA_MAX) {
            PS_KalibPunto *p = &kalib->noktalar[kalib->nokta_sayisi++];
            p->step        = (uint8_t)step;
            p->inverter_on = (inv_int != 0);
            p->olculen_aci = aci;
            okunan++;
        } else { atlanan++; }
        pthread_mutex_unlock(&ps.mutex);
    }
    fclose(fp);

    ps_log(PS_LOG_INFO, "Kalibrasyon yuklendi: %s (%d nokta, %d atlandi)", path, okunan, atlanan);
    return PS_OK;
}

/* Alıcı için kör nokta sınırlarını günceller.
 * inverter_on=false: APF OFF aralığı; inverter_on=true: APF ON aralığı.    */
int ps_kalib_set_sinir(int alici, bool inverter_on, float min_aci, float max_aci) {
    if (!ps.baslandi)                              return PS_ERR_INIT;
    if (alici < 0 || alici >= PS_ALICI_SAYISI)    return PS_ERR_KANAL;

    pthread_mutex_lock(&ps.mutex);
    if (inverter_on) {
        alici_kalib[alici].min_aci_on = min_aci;
        alici_kalib[alici].max_aci_on = max_aci;
    } else {
        alici_kalib[alici].min_aci_off = min_aci;
        alici_kalib[alici].max_aci_off = max_aci;
    }
    pthread_mutex_unlock(&ps.mutex);
    return PS_OK;
}

/* Alıcı için kör nokta sınırlarını döndürür.                                */
int ps_kalib_get_sinir(int alici, bool inverter_on, float *min_out, float *max_out) {
    if (!ps.baslandi)                              return PS_ERR_INIT;
    if (alici < 0 || alici >= PS_ALICI_SAYISI)    return PS_ERR_KANAL;

    pthread_mutex_lock(&ps.mutex);
    if (inverter_on) {
        if (min_out) *min_out = alici_kalib[alici].min_aci_on;
        if (max_out) *max_out = alici_kalib[alici].max_aci_on;
    } else {
        if (min_out) *min_out = alici_kalib[alici].min_aci_off;
        if (max_out) *max_out = alici_kalib[alici].max_aci_off;
    }
    pthread_mutex_unlock(&ps.mutex);
    return PS_OK;
}

/* Alıcının kalibrasyon durumu ve nokta sayısını döndürür.                   */
int ps_get_kalib_durum(int alici, bool *kalibre_out, int *nokta_sayisi_out) {
    if (!ps.baslandi)                              return PS_ERR_INIT;
    if (alici < 0 || alici >= PS_ALICI_SAYISI)    return PS_ERR_KANAL;

    pthread_mutex_lock(&ps.mutex);
    if (kalibre_out)     *kalibre_out     = alici_kalib[alici].kalibre_edildi;
    if (nokta_sayisi_out) *nokta_sayisi_out = alici_kalib[alici].nokta_sayisi;
    pthread_mutex_unlock(&ps.mutex);
    return PS_OK;
}

/* Teorik APF faz hesabı — kalibrasyon ofseti uygulanmaz, saf formül.       */
float ps_teorik_aci(uint8_t step, float frekans_hz) {
    float R       = PS_R_WIPER_OHM + PS_R_STEP_OHM * step;
    float omega   = 2.0f * (float)M_PI * frekans_hz;
    float faz_rad = -2.0f * atanf(omega * R * PS_APF_KAPASITOR_F);
    return faz_rad * 180.0f / (float)M_PI;
}

/* Beamforming modülüne uyumluluk: extern int phase_shift_set_angle() bildirimiyle
 * çağrılan ince sarmalayıcı; gerçek açı geri dönüşü ihmal edilir.          */
int phase_shift_set_angle(int alici, float derece) {
    return ps_set_angle(alici, derece, NULL);
}

/* Log seviyesini ayarlar. Varsayılan: PS_LOG_INFO. */
void ps_set_log_level(PS_LogLevel level) {
    log_seviyesi = level;
}
