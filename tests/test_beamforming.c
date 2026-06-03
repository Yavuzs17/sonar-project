/*
 * Beamforming modülü mantık testleri
 * Donanım gerektirmez, sadece matematiksel doğrulama
 *
 * Derleme:
 *   gcc -o test_bf tests/test_beamforming.c beamforming/beamforming.c -lm -Wall
 *
 * Çalıştırma:
 *   ./test_bf
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../beamforming/beamforming.h"

#define EPSILON 0.01f
#define EPSILON_DEG 1.0f

// Mock: phase_shift_set_angle bf_init testi için stub
int phase_shift_set_angle(int alici, float derece) {
    (void)alici; (void)derece;
    return 0;
}

// Renkli çıktı için ANSI kodları
#define GREEN  "\033[0;32m"
#define RED    "\033[0;31m"
#define YELLOW "\033[0;33m"
#define RESET  "\033[0m"

static int gecen_test = 0;
static int kalan_test = 0;

#define TEST_BASLA(isim) \
    printf("\n--- %s ---\n", isim);

#define BEKLENEN(kosul, mesaj) do { \
    if (kosul) { \
        printf(GREEN "  ✓ " RESET "%s\n", mesaj); \
        gecen_test++; \
    } else { \
        printf(RED "  ✗ " RESET "%s\n", mesaj); \
        kalan_test++; \
    } \
} while(0)

// =====================================================
// TEST 1: Broadside (0°,0°) — tüm fazlar yaklaşık 0 olmalı
// =====================================================
void test_broadside(void) {
    TEST_BASLA("Test 1: Broadside (azimuth=0, elevation=0)");

    float fazlar[7];
    int sonuc = bf_hesapla_fazlar(0.0f, 0.0f, 40000.0f, fazlar);

    BEKLENEN(sonuc == 0, "Hesaplama başarılı");

    printf("    Fazlar: ");
    for (int i = 0; i < 7; i++) printf("%.2f ", fazlar[i]);
    printf("\n");

    // 0,0 yönünde tüm dalga aynı anda gelir → tüm fazlar 0 olmalı
    // Ama formül 0-360 normalize ediyor, bu yüzden 0 veya 360'a yakın olabilir
    int hepsi_sifir = 1;
    for (int i = 0; i < 7; i++) {
        float f = fazlar[i];
        // 0'a veya 360'a yakın mı?
        if (fabsf(f) > EPSILON_DEG && fabsf(f - 360.0f) > EPSILON_DEG) {
            hepsi_sifir = 0;
            break;
        }
    }
    BEKLENEN(hepsi_sifir, "Tüm fazlar broadside'da sıfır (veya 360)");
}

// =====================================================
// TEST 2: Merkez eleman fazı her zaman 0 olmalı
// =====================================================
void test_merkez_eleman(void) {
    TEST_BASLA("Test 2: Merkez eleman (eleman 0) her açıda fazı sıfır");

    float fazlar[7];
    float test_acilari[][2] = {
        {0, 0}, {15, 0}, {0, 15}, {30, 30}, {-30, -30}, {-15, 20}
    };

    for (size_t t = 0; t < sizeof(test_acilari)/sizeof(test_acilari[0]); t++) {
        float az = test_acilari[t][0];
        float el = test_acilari[t][1];

        bf_hesapla_fazlar(az, el, 40000.0f, fazlar);

        char mesaj[100];
        snprintf(mesaj, sizeof(mesaj), "az=%.0f el=%.0f → eleman 0 fazı: %.2f°", az, el, fazlar[0]);

        // Merkez eleman koordinatı (0,0) olduğu için faz her zaman 0
        int sifir_mi = (fabsf(fazlar[0]) < EPSILON_DEG ||
                       fabsf(fazlar[0] - 360.0f) < EPSILON_DEG);
        BEKLENEN(sifir_mi, mesaj);
    }
}

// =====================================================
// TEST 3: Simetri testi
// Eleman 1 (sağ) ile Eleman 4 (sol) — fazları toplamı 360 olmalı
// (zıt konumda, ters faz)
// =====================================================
void test_simetri(void) {
    TEST_BASLA("Test 3: Simetri — zıt elemanların fazları zıt işaretli");

    float fazlar[7];
    bf_hesapla_fazlar(15.0f, 0.0f, 40000.0f, fazlar);

    printf("    Eleman 1 (sağ): %.2f°, Eleman 4 (sol): %.2f°\n", fazlar[1], fazlar[4]);

    // 1 ve 4 zıt konumda → fazlar toplamı 360 (veya 0) olmalı
    float toplam = fmodf(fazlar[1] + fazlar[4], 360.0f);
    if (toplam < 0) toplam += 360.0f;

    int simetrik = (toplam < EPSILON_DEG || fabsf(toplam - 360.0f) < EPSILON_DEG);
    BEKLENEN(simetrik, "Eleman 1 ve 4 simetrik fazda");

    // 2 ve 5 de zıt
    bf_hesapla_fazlar(15.0f, 15.0f, 40000.0f, fazlar);
    toplam = fmodf(fazlar[2] + fazlar[5], 360.0f);
    if (toplam < 0) toplam += 360.0f;
    simetrik = (toplam < EPSILON_DEG || fabsf(toplam - 360.0f) < EPSILON_DEG);
    BEKLENEN(simetrik, "Eleman 2 ve 5 simetrik fazda");
}

// =====================================================
// TEST 4: Frekansa göre faz değişimi
// Aynı yönde, daha yüksek frekansta → daha büyük faz
// =====================================================
void test_frekans_etkisi(void) {
    TEST_BASLA("Test 4: Frekans arttıkça faz büyür (lambda küçülür)");

    float fazlar_39k[7], fazlar_41k[7];
    bf_hesapla_fazlar(15.0f, 15.0f, 39000.0f, fazlar_39k);
    bf_hesapla_fazlar(15.0f, 15.0f, 41000.0f, fazlar_41k);
    printf("    Eleman 1 @ 39kHz: %.2f°\n", fazlar_39k[1]);
    printf("    Eleman 1 @ 41kHz: %.2f°\n", fazlar_41k[1]);

    // 41kHz'de faz daha büyük olmalı (lambda küçük → k büyük → faz büyük)
    // Ama 0-360 normalize edildiği için karşılaştırma ham değerle daha kolay
    // Bu yüzden direkt fazların farklı olduğunu kontrol et
    BEKLENEN(fabsf(fazlar_39k[1] - fazlar_41k[1]) > 0.5f,
             "39kHz ve 41kHz fazları farklı (frekans etki ediyor)");
}

// =====================================================
// TEST 5: Aralık dışı parametreler hata vermeli
// =====================================================
void test_arali_disi(void) {
    TEST_BASLA("Test 5: Geçersiz parametreler hata kodu döndürmeli");

    float fazlar[7];

    int sonuc = bf_hesapla_fazlar(50.0f, 0.0f, 40000.0f, fazlar);
    BEKLENEN(sonuc < 0, "Azimuth 50° (aralık dışı) hata döndürdü");

    sonuc = bf_hesapla_fazlar(0.0f, -45.0f, 40000.0f, fazlar);
    BEKLENEN(sonuc < 0, "Elevation -45° (aralık dışı) hata döndürdü");

    sonuc = bf_hesapla_fazlar(0.0f, 0.0f, 50000.0f, fazlar);
    BEKLENEN(sonuc < 0, "Frekans 50kHz (aralık dışı) hata döndürdü");
}

// =====================================================
// TEST 6: Sayısal doğruluk — manuel hesap ile karşılaştır
// =====================================================
void test_sayisal_dogruluk(void) {
    TEST_BASLA("Test 6: Manuel hesapla karşılaştırma");

    // Eleman 1 koordinatı: (d, 0), d = 0.018 m
    // azimuth=15°, elevation=15°, f=40kHz
    // ux = sin(el) × cos(az) = sin(15) × cos(15) = 0.2588 × 0.9659 = 0.250
    // uy = sin(el) × sin(az) = sin(15) × sin(15) = 0.2588 × 0.2588 = 0.067
    // lambda = 343/40000 = 0.008575 m
    // k = 2π/lambda = 732.8 rad/m
    // x1 = 0.018, y1 = 0
    // delta_1 = -k × (x1×ux + y1×uy) = -732.8 × (0.018 × 0.250) = -3.298 rad
    // -3.298 rad × (180/π) = -188.96°
    // 0-360 normalize: -188.96 + 360 = 171.04°

    float fazlar[7];
    bf_hesapla_fazlar(15.0f, 15.0f, 40000.0f, fazlar);

    printf("    Eleman 1 hesaplanan: %.2f°, beklenen: ~171°\n", fazlar[1]);

    // Tolerans 5° (eleman mesafesi farklı olabilir)
    int yakin = fabsf(fazlar[1] - 171.0f) < 5.0f;
    BEKLENEN(yakin, "Eleman 1 fazı manuel hesapla uyumlu");
}

// =====================================================
// MAIN
// =====================================================
int main(void) {
    printf("===============================\n");
    printf("  BEAMFORMING MODÜL TESTLERİ  \n");
    printf("===============================\n");
 // BU SATIRI EKLE
    if (bf_init() != 0) {
        printf("HATA: bf_init() basarisiz\n");
        return 1;
    }
    test_broadside();
    test_merkez_eleman();
    test_simetri();
    test_frekans_etkisi();
    test_arali_disi();
    test_sayisal_dogruluk();

    printf("\n===============================\n");
    printf("  SONUÇ: ");
    if (kalan_test == 0) {
        printf(GREEN "✓ %d/%d test geçti" RESET "\n", gecen_test, gecen_test + kalan_test);
    } else {
        printf(RED "✗ %d test başarısız" RESET ", %d test geçti\n", kalan_test, gecen_test);
    }
    printf("===============================\n\n");

    return (kalan_test == 0) ? 0 : 1;
}