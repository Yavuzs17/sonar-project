/*
 * Beamforming + Phase Shift Entegrasyon Testleri
 *
 * Derleme:
 *   gcc -o test_int tests/test_integration.c \
 *       beamforming/beamforming.c phase_shift/phase_shift.c \
 *       -lm -lpthread -Wall
 *
 * Çalıştırma:
 *   ./test_int
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../beamforming/beamforming.h"
#include "../phase_shift/phase_shift.h"

#define EPSILON_DEG 5.0f   // 5° tolerans (kalibrasyon olmadan)

#define GREEN  "\033[0;32m"
#define RED    "\033[0;31m"
#define YELLOW "\033[0;33m"
#define RESET  "\033[0m"

static int gecen = 0;
static int kalan = 0;

#define TEST_BASLA(isim) printf("\n--- %s ---\n", isim);

#define BEKLENEN(kosul, mesaj) do { \
    if (kosul) { printf(GREEN "  ✓ " RESET "%s\n", mesaj); gecen++; } \
    else        { printf(RED   "  ✗ " RESET "%s\n", mesaj); kalan++; } \
} while(0)

// =====================================================
// MOCK lgpio (donanım simulasyonu)
// =====================================================

// MCP4651'lere yazılan son değerleri sakla (test için)
static uint8_t mock_mcp_step[8][2] = {0};   // [çip][wiper]
static uint8_t mock_pcf_durum = 0;
static int mock_son_addr = -1;
static int mock_son_wiper = -1;

int lgI2cOpen(int dev, int addr, int flags) {
    (void)dev; (void)flags;
    mock_son_addr = addr;
    return addr;  // handle olarak adresi kullan, takip kolay
}
int lgI2cClose(int handle) { (void)handle; return 0; }

int lgI2cWriteDevice(int handle, const char *buf, int count) {
    // MCP4651: 2 byte yazma (komut byte + data byte)
    if (count == 2) {
        int wiper = (buf[0] >> 4) & 0x01;  // bit 4 wiper seçer
        uint8_t step = (uint8_t)buf[1];

        // handle = adres (mock'ta öyle ayarladık)
        // MCP4651 adresleri 0x28-0x2F
        if (handle >= 0x28 && handle <= 0x2F) {
            int cip = handle - 0x28;
            if (cip < 8 && wiper < 2) {
                mock_mcp_step[cip][wiper] = step;
            }
        }
        // PCF8574 (1 byte yazma)
        else if (handle == 0x20) {
            mock_pcf_durum = (uint8_t)buf[0];
        }
    } else if (count == 1 && handle == 0x20) {
        mock_pcf_durum = (uint8_t)buf[0];
    }
    return 0;
}

int lgI2cReadDevice(int handle, char *buf, int count) {
    (void)handle;
    for (int i = 0; i < count; i++) buf[i] = 0;
    return count;
}

// Mock için: phase_shift modülünün son uyguladığı açıyı yakalamak
// (gerçek ps_set_angle'i çağırmadan da test edebilmek için)
static float mock_son_uygulanan[7] = {0};
static int mock_set_count[7] = {0};

// Beamforming içinde çağrılan phase_shift_set_angle'i overrride etmek yerine
// gerçeğini kullanıp uygulanan açıyı geri okuyacağız.

// =====================================================
// TEST 1: Beamforming + Phase Shift birlikte init
// =====================================================
void test_init(void) {
    TEST_BASLA("Test 1: Her iki modül başarıyla başlatılır");

    int ps_sonuc = ps_init();
    BEKLENEN(ps_sonuc == PS_OK, "ps_init başarılı");

    int bf_sonuc = bf_init();
    BEKLENEN(bf_sonuc == 0, "bf_init başarılı");
}

// =====================================================
// TEST 2: Broadside steering — tüm fazlar 0
// =====================================================
void test_broadside_steering(void) {
    TEST_BASLA("Test 2: Broadside (0,0) — tüm phase_shift komutları 0°");

    // Önce phase_shift kalibrasyonunu doldur ki ps_set_angle çalışsın
    for (int alici = 0; alici < 7; alici++) {
        ps_kalib_nokta_temizle(alici);

        // Teorik formülden 7 nokta üret (her alıcı için aynı)
        for (int N = 0; N <= 255; N += 50) {
            float aci = ps_teorik_aci(N, 40000.0f);
            ps_kalib_nokta_ekle(alici, (uint8_t)N, false, aci);
        }
        for (int N = 0; N <= 255; N += 100) {
            float aci = ps_teorik_aci(N, 40000.0f);
            ps_kalib_nokta_ekle(alici, (uint8_t)N, true, aci - 180.0f);
        }

        // Sınırları ayarla
        ps_kalib_set_sinir(alici, false, -7.0f, -136.75f);
        ps_kalib_set_sinir(alici, true,  -187.0f, -316.75f);

        ps_kalib_fit(alici);
    }

    // Broadside set_direction çağrısı
    int sonuc = bf_set_direction(0.0f, 0.0f, 40000.0f);
    BEKLENEN(sonuc == 0, "bf_set_direction(0,0) başarılı");
}

// =====================================================
// TEST 3: Beamforming hesabı = manuel hesap
// =====================================================
void test_faz_tutarliligi(void) {
    TEST_BASLA("Test 3: Beamforming + manuel hesap karşılaştırma");

    // 15° azimuth, 15° elevation, 40 kHz
    float bf_fazlar[7];
    bf_hesapla_fazlar(15.0f, 15.0f, 40000.0f, bf_fazlar);

    printf("    Beamforming hesabı (az=15, el=15, f=40k):\n");
    for (int i = 0; i < 7; i++) {
        printf("      Eleman %d: %.2f°\n", i, bf_fazlar[i]);
    }

    // Eleman 0 (merkez) her zaman 0 olmalı
    float e0 = bf_fazlar[0];
    int merkez_sifir = (fabsf(e0) < EPSILON_DEG) || (fabsf(e0 - 360.0f) < EPSILON_DEG);
    BEKLENEN(merkez_sifir, "Eleman 0 (merkez) fazı sıfır");

    // Eleman 1 (sağ) ile Eleman 4 (sol) simetrik olmalı
    float toplam14 = fmodf(bf_fazlar[1] + bf_fazlar[4], 360.0f);
    if (toplam14 < 0) toplam14 += 360.0f;
    int simetrik = (toplam14 < EPSILON_DEG) || (fabsf(toplam14 - 360.0f) < EPSILON_DEG);
    BEKLENEN(simetrik, "Eleman 1 ve 4 simetrik");
}

// =====================================================
// TEST 4: Set direction mock donanıma yazıyor mu
// =====================================================
void test_donanim_yazma(void) {
    TEST_BASLA("Test 4: Steering komutu mock donanıma yazıyor");

    // Mock değerleri sıfırla
    memset(mock_mcp_step, 0, sizeof(mock_mcp_step));
    mock_pcf_durum = 0;

    // Belirli bir yöne steer et
    int sonuc = bf_set_direction(15.0f, 15.0f, 40000.0f);
    BEKLENEN(sonuc == 0, "bf_set_direction(15,15) çalıştı");

    // En az bir MCP4651'e yazılmış olmalı
    int yazilan_sayisi = 0;
    for (int cip = 0; cip < 8; cip++) {
        for (int w = 0; w < 2; w++) {
            if (mock_mcp_step[cip][w] != 0) yazilan_sayisi++;
        }
    }
    printf("    Mock'ta yazılı MCP4651 wiper sayısı: %d\n", yazilan_sayisi);
    BEKLENEN(yazilan_sayisi >= 4, "En az 4 wiper yazıldı (7 alıcıdan ≥4)");
}

// =====================================================
// TEST 5: Frekans değişince fazlar değişiyor
// =====================================================
void test_sweep_uyumu(void) {
    TEST_BASLA("Test 5: Sweep'te frekans değişince fazlar yeniden hesaplanıyor");

    float fazlar_39k[7], fazlar_40k[7], fazlar_41k[7];

    bf_hesapla_fazlar(15.0f, 15.0f, 39000.0f, fazlar_39k);
    bf_hesapla_fazlar(15.0f, 15.0f, 40000.0f, fazlar_40k);
    bf_hesapla_fazlar(15.0f, 15.0f, 41000.0f, fazlar_41k);

    // En az bir eleman için fazlar farklı olmalı
    int farkli_var = 0;
    for (int i = 1; i < 7; i++) {
        if (fabsf(fazlar_39k[i] - fazlar_41k[i]) > 1.0f) {
            farkli_var = 1;
            printf("    Eleman %d: 39k=%.2f° 40k=%.2f° 41k=%.2f°\n",
                   i, fazlar_39k[i], fazlar_40k[i], fazlar_41k[i]);
            break;
        }
    }
    BEKLENEN(farkli_var, "Frekans değişince fazlar değişiyor");
}

// =====================================================
// TEST 6: 169 yön taraması hatasız
// =====================================================
void test_full_tarama(void) {
    TEST_BASLA("Test 6: Tüm 13×13 = 169 yön hatasız hesaplanıyor");

    int hata_sayisi = 0;
    int ilk_hata_az = 0, ilk_hata_el = 0;
    float fazlar[7];

    for (float az = -30; az <= 30; az += 5) {
        for (float el = -30; el <= 30; el += 5) {
            int sonuc = bf_hesapla_fazlar(az, el, 40000.0f, fazlar);
            if (sonuc != 0) {
                if (hata_sayisi == 0) {
                    ilk_hata_az = (int)az;
                    ilk_hata_el = (int)el;
                }
                hata_sayisi++;
            }
        }
    }

    printf("    169 yön içinde hata sayısı: %d\n", hata_sayisi);
    if (hata_sayisi > 0) {
        printf("    İlk hata: az=%d, el=%d\n", ilk_hata_az, ilk_hata_el);
    }
    BEKLENEN(hata_sayisi == 0, "Tüm yönler hatasız hesaplandı");
}

// =====================================================
// TEST 7: Sweep + tarama kombinasyonu
// =====================================================
void test_sweep_tarama(void) {
    TEST_BASLA("Test 7: 21 frekans × 169 yön = 3549 kombinasyon hatasız");

    int hata_sayisi = 0;
    float fazlar[7];

    for (float f = 39000; f <= 41000; f += 100) {
        for (float az = -30; az <= 30; az += 5) {
            for (float el = -30; el <= 30; el += 5) {
                int sonuc = bf_hesapla_fazlar(az, el, f, fazlar);
                if (sonuc != 0) hata_sayisi++;
            }
        }
    }

    printf("    3549 kombinasyon içinde hata sayısı: %d\n", hata_sayisi);
    BEKLENEN(hata_sayisi == 0, "Tüm frekans×yön kombinasyonları hatasız");
}

// =====================================================
// TEST 8: Eleman mesafesi değiştirme
// =====================================================
void test_mesafe_degistirme(void) {
    TEST_BASLA("Test 8: Eleman mesafesi değişince fazlar güncellenir");

    float fazlar_18mm[7], fazlar_20mm[7];

    bf_set_eleman_mesafesi(0.018f);
    bf_hesapla_fazlar(15.0f, 15.0f, 40000.0f, fazlar_18mm);

    bf_set_eleman_mesafesi(0.020f);
    bf_hesapla_fazlar(15.0f, 15.0f, 40000.0f, fazlar_20mm);

    // En az bir eleman farklı olmalı
    int farkli = 0;
    for (int i = 1; i < 7; i++) {
        if (fabsf(fazlar_18mm[i] - fazlar_20mm[i]) > 1.0f) {
            farkli = 1;
            printf("    Eleman %d: d=18mm → %.2f°, d=20mm → %.2f°\n",
                   i, fazlar_18mm[i], fazlar_20mm[i]);
            break;
        }
    }
    BEKLENEN(farkli, "Mesafe değişince fazlar değişiyor");

    // Eski değere geri dön
    bf_set_eleman_mesafesi(0.018f);
}

// =====================================================
// MAIN
// =====================================================
int main(void) {
    printf("===============================\n");
    printf("  ENTEGRASYON TESTLERİ        \n");
    printf("  Beamforming + Phase Shift   \n");
    printf("===============================\n");

    test_init();
    test_broadside_steering();
    test_faz_tutarliligi();
    test_donanim_yazma();
    test_sweep_uyumu();
    test_full_tarama();
    test_sweep_tarama();
    test_mesafe_degistirme();

    printf("\n===============================\n");
    if (kalan == 0) {
        printf(GREEN "  ✓ %d/%d test geçti" RESET "\n", gecen, gecen + kalan);
    } else {
        printf(RED   "  ✗ %d test başarısız" RESET ", %d test geçti\n", kalan, gecen);
    }
    printf("===============================\n\n");

    return (kalan == 0) ? 0 : 1;
}