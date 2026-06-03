/*
 * Phase shift modülü mantık testleri
 *
 * Derleme:
 *   gcc -o test_ps tests/test_phase_shift.c phase_shift/phase_shift.c -lm -llgpio -lpthread -Wall
 *
 * Çalıştırma:
 *   ./test_ps
 *
 * NOT: Donanım fonksiyonları (mcp4651_yaz, pcf8574_pin_yaz) lgpio'ya bağlı.
 *      Test sırasında bu fonksiyonlar I2C hatası verecek ama mantık testleri
 *      donanımdan bağımsız çalışacak.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../phase_shift/phase_shift.h"

// === MOCK lgpio fonksiyonları (donanım simulasyonu) ===
int lgI2cOpen(int dev, int addr, int flags) {
    (void)dev; (void)addr; (void)flags;
    return 1;  // başarılı handle
}
int lgI2cClose(int handle) { (void)handle; return 0; }
int lgI2cWriteDevice(int handle, const char *buf, int count) {
    (void)handle; (void)buf; (void)count;
    return 0;  // başarılı yazma
}
int lgI2cReadDevice(int handle, char *buf, int count) {
    (void)handle;
    for (int i = 0; i < count; i++) buf[i] = 0;
    return count;  // başarılı okuma
}

#define EPSILON 0.01f
#define EPSILON_DEG 1.0f

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
// TEST 1: APF teorik formülü — bilinen N değerleri için faz
// =====================================================
void test_apf_formulu(void) {
    TEST_BASLA("Test 1: APF teorik formülü ps_teorik_aci()");

    // Manuel hesap (C = 1nF varsayımıyla, 40kHz):
    // R(N) = 75 + 39.0625 × N
    // faz = -2 × atan(2π × f × R(N) × C) (radyan), derece için ×180/π

    // N=0:    R=75      → atan(2π × 40000 × 75 × 1e-9) = atan(0.01885) = 1.080° → faz = -2.16°
    // N=128:  R=5075    → atan(2π × 40000 × 5075 × 1e-9) = atan(1.275) = 51.88° → faz = -103.77°
    // N=255:  R=10038.94 → atan(2.523) = 68.39° → faz = -136.78°

    float faz0   = ps_teorik_aci(0,   40000.0f);
    float faz128 = ps_teorik_aci(128, 40000.0f);
    float faz255 = ps_teorik_aci(255, 40000.0f);

    printf("    Faz(N=0)   = %.2f°  (manuel: ~-2.16°)\n", faz0);
    printf("    Faz(N=128) = %.2f°  (manuel: ~-103.77°)\n", faz128);
    printf("    Faz(N=255) = %.2f°  (manuel: ~-136.78°)\n", faz255);

    BEKLENEN(fabsf(faz0   - (-2.16f))   < 1.0f, "N=0 → faz ~-2.16°");
    BEKLENEN(fabsf(faz128 - (-103.77f)) < 2.0f, "N=128 → faz ~-103.77°");
    BEKLENEN(fabsf(faz255 - (-136.78f)) < 2.0f, "N=255 → faz ~-136.78°");
}

// =====================================================
// TEST 2: Faz monotonluğu — N arttıkça faz NEGATİFE doğru artar
// =====================================================
void test_faz_monotonluk(void) {
    TEST_BASLA("Test 2: N arttıkça faz daha negatif olur (mutlak değer artar)");

    float onceki = ps_teorik_aci(0, 40000.0f);
    int monoton = 1;
    int hata_n = -1;

    for (int N = 1; N <= 255; N++) {
        float simdi = ps_teorik_aci(N, 40000.0f);
        // Faz negatif değer alıyor, monoton azalmalı (daha negatif olmalı)
        if (simdi >= onceki) {
            monoton = 0;
            hata_n = N;
            break;
        }
        onceki = simdi;
    }

    BEKLENEN(monoton, "Tüm N için faz monoton azalıyor (negatife gidiyor)");
    if (!monoton) printf("    HATA: N=%d'de monotonluk bozuldu\n", hata_n);
}

// =====================================================
// TEST 3: Frekans etkisi — yüksek frekansta daha büyük faz
// =====================================================
void test_frekans_etkisi(void) {
    TEST_BASLA("Test 3: Yüksek frekansta faz mutlak değeri büyür");

    float faz_39k = ps_teorik_aci(128, 39000.0f);
    float faz_40k = ps_teorik_aci(128, 40000.0f);
    float faz_41k = ps_teorik_aci(128, 41000.0f);

    printf("    N=128 @ 39kHz: %.2f°\n", faz_39k);
    printf("    N=128 @ 40kHz: %.2f°\n", faz_40k);
    printf("    N=128 @ 41kHz: %.2f°\n", faz_41k);

    BEKLENEN(faz_41k < faz_40k && faz_40k < faz_39k,
             "Frekans arttıkça faz daha negatif");
}

// =====================================================
// TEST 4: Kalibrasyon nokta ekleme/silme
// =====================================================
void test_kalibrasyon_yonetimi(void) {
    TEST_BASLA("Test 4: Kalibrasyon noktaları ekle/sil/sorgula");

    int sonuc = ps_kalib_nokta_ekle(0, 0,   false, -2.0f);
    BEKLENEN(sonuc == PS_OK, "Nokta ekleme: N=0, inv=OFF, -2°");

    sonuc = ps_kalib_nokta_ekle(0, 128, false, -100.0f);
    BEKLENEN(sonuc == PS_OK, "Nokta ekleme: N=128, inv=OFF, -100°");

    sonuc = ps_kalib_nokta_ekle(0, 255, false, -160.0f);
    BEKLENEN(sonuc == PS_OK, "Nokta ekleme: N=255, inv=OFF, -160°");

    bool kalibre;
    int nokta_sayisi;
    sonuc = ps_get_kalib_durum(0, &kalibre, &nokta_sayisi);
    printf("    Alıcı 0 — kalibre=%d, nokta=%d\n", kalibre, nokta_sayisi);
    BEKLENEN(sonuc == PS_OK && nokta_sayisi == 3, "3 nokta eklendi");
    BEKLENEN(kalibre == false, "Henüz fit edilmedi → kalibre=false");

    // Geçersiz alıcı
    sonuc = ps_kalib_nokta_ekle(99, 0, false, 0.0f);
    BEKLENEN(sonuc < 0, "Geçersiz alıcı (99) hata döndürdü");

    // Geçersiz step
    sonuc = ps_kalib_nokta_ekle(1, 0, false, -500.0f);
    BEKLENEN(sonuc < 0 || sonuc == PS_OK, "Aşırı açı eklenebilir veya reddedilebilir");

    // Temizleme
    sonuc = ps_kalib_nokta_temizle(0);
    BEKLENEN(sonuc == PS_OK, "Alıcı 0 kalibrasyonu temizlendi");

    ps_get_kalib_durum(0, &kalibre, &nokta_sayisi);
    BEKLENEN(nokta_sayisi == 0, "Temizleme sonrası 0 nokta");
}

// =====================================================
// TEST 5: Formül fit — sentetik veri ile
// =====================================================
void test_formul_fit(void) {
    TEST_BASLA("Test 5: Formül fit — teorik veriyle");

    // Önce mevcut kalibrasyonu temizle
    ps_kalib_nokta_temizle(1);

    // Teorik formülden 4 nokta üret
    float aci_n0   = ps_teorik_aci(0,   40000.0f);
    float aci_n50  = ps_teorik_aci(50,  40000.0f);
    float aci_n128 = ps_teorik_aci(128, 40000.0f);
    float aci_n255 = ps_teorik_aci(255, 40000.0f);

    ps_kalib_nokta_ekle(1, 0,   false, aci_n0);
    ps_kalib_nokta_ekle(1, 50,  false, aci_n50);
    ps_kalib_nokta_ekle(1, 128, false, aci_n128);
    ps_kalib_nokta_ekle(1, 255, false, aci_n255);

    // Inverter ON için 3 nokta
    ps_kalib_nokta_ekle(1, 0,   true, aci_n0   - 180.0f);
    ps_kalib_nokta_ekle(1, 128, true, aci_n128 - 180.0f);
    ps_kalib_nokta_ekle(1, 255, true, aci_n255 - 180.0f);

    int sonuc = ps_kalib_fit(1);
    BEKLENEN(sonuc == PS_OK, "Fit başarılı");

    bool kalibre;
    int nokta_sayisi;
    ps_get_kalib_durum(1, &kalibre, &nokta_sayisi);
    BEKLENEN(kalibre == true, "Fit sonrası kalibre=true");
    BEKLENEN(nokta_sayisi == 7, "7 nokta fit edildi");
}

// =====================================================
// TEST 6: Min/Max sınır ayarları
// =====================================================
void test_sinir_yonetimi(void) {
    TEST_BASLA("Test 6: Kör nokta sınırları (min/max açı)");

    // Inverter OFF sınırı: -7° ile -166.3°
    int sonuc = ps_kalib_set_sinir(2, false, -7.0f, -166.3f);
    BEKLENEN(sonuc == PS_OK, "Inverter OFF sınırı set edildi");

    float min, max;
    sonuc = ps_kalib_get_sinir(2, false, &min, &max);
    BEKLENEN(sonuc == PS_OK, "Sınır okundu");
    BEKLENEN(fabsf(min - (-7.0f))     < EPSILON, "min_aci_off = -7°");
    BEKLENEN(fabsf(max - (-166.3f))   < EPSILON, "max_aci_off = -166.3°");

    // Inverter ON sınırı: -187° ile -346.3°
    ps_kalib_set_sinir(2, true, -187.0f, -346.3f);
    ps_kalib_get_sinir(2, true, &min, &max);
    BEKLENEN(fabsf(min - (-187.0f))   < EPSILON, "min_aci_on = -187°");
    BEKLENEN(fabsf(max - (-346.3f))   < EPSILON, "max_aci_on = -346.3°");
}

// =====================================================
// MAIN
// =====================================================
int main(void) {
    printf("===============================\n");
    printf(" PHASE SHIFT MODÜL TESTLERİ   \n");
    printf("===============================\n");
    // Modülü başlat (mock'lanmış lgpio ile)
    if (ps_init() != PS_OK) {
        printf("HATA: ps_init() basarisiz\n");
        return 1;
    }
    test_apf_formulu();
    test_faz_monotonluk();
    test_frekans_etkisi();
    test_kalibrasyon_yonetimi();
    test_formul_fit();
    test_sinir_yonetimi();

    printf("\n===============================\n");
    if (kalan == 0) {
        printf(GREEN "  ✓ %d/%d test geçti" RESET "\n", gecen, gecen + kalan);
    } else {
        printf(RED   "  ✗ %d test başarısız" RESET ", %d test geçti\n", kalan, gecen);
    }
    printf("===============================\n\n");

    return (kalan == 0) ? 0 : 1;
}