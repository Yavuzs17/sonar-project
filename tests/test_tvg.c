/*
 * TVG modülü mantık testleri
 *
 * Derleme:
 *   gcc -o test_tvg tests/test_tvg.c tvg/tvg.c -lm -Wall
 *
 * Çalıştırma:
 *   ./test_tvg
 *
 * NOT: tvg_init() I2C donanımına erişmeye çalıştığı için doğrudan çağırmıyoruz.
 *      Sadece donanım gerektirmeyen public/static fonksiyonları test ediyoruz.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../tvg/tvg.h"

#define EPSILON 0.01f
#define EPSILON_DB 0.5f

#define GREEN  "\033[0;32m"
#define RED    "\033[0;31m"
#define RESET  "\033[0m"

static int gecen = 0;
static int kalan = 0;

#define TEST_BASLA(isim) printf("\n--- %s ---\n", isim);

#define BEKLENEN(kosul, mesaj) do { \
    if (kosul) { printf(GREEN "  ✓ " RESET "%s\n", mesaj); gecen++; } \
    else        { printf(RED   "  ✗ " RESET "%s\n", mesaj); kalan++; } \
} while(0)

// =====================================================
// TEST 1: Kazanç formülü — bilinen N değerleri için doğru kazanç
// =====================================================
void test_kazanc_formulu(void) {
    TEST_BASLA("Test 1: G(N) = 1 + 49400/(1075 + 39.0625×N)");

    // Manuel hesaplar:
    // N=0   → R=1075     → G = 1 + 49400/1075      = 46.95
    // N=128 → R=6075     → G = 1 + 49400/6075      = 9.13
    // N=255 → R=11036.94 → G = 1 + 49400/11036.94  = 5.476

    float G0   = TVG_GAIN_FORMUL(0);
    float G128 = TVG_GAIN_FORMUL(128);
    float G255 = TVG_GAIN_FORMUL(255);

    printf("    G(N=0)   = %.3f  (beklenen ~46.95)\n", G0);
    printf("    G(N=128) = %.3f  (beklenen ~9.13)\n",  G128);
    printf("    G(N=255) = %.3f  (beklenen ~5.48)\n",  G255);

    BEKLENEN(fabsf(G0   - 46.95f) < 0.05f, "N=0 → kazanç doğru (~46.95×)");
    BEKLENEN(fabsf(G128 -  9.13f) < 0.05f, "N=128 → kazanç doğru (~9.13×)");
    BEKLENEN(fabsf(G255 -  5.48f) < 0.05f, "N=255 → kazanç doğru (~5.48×)");
}

// =====================================================
// TEST 2: Kazanç monotonluğu — N büyürse kazanç AZALIR
// =====================================================
void test_monotonluk(void) {
    TEST_BASLA("Test 2: N arttıkça kazanç monoton azalır");

    float onceki = TVG_GAIN_FORMUL(0);
    int monoton = 1;
    int hata_n = -1;

    for (int N = 1; N <= 255; N++) {
        float simdi = TVG_GAIN_FORMUL(N);
        if (simdi >= onceki) {
            monoton = 0;
            hata_n = N;
            break;
        }
        onceki = simdi;
    }

    if (monoton) {
        BEKLENEN(1, "Tüm N değerleri için kazanç azalıyor");
    } else {
        printf("    HATA: N=%d'de monotonluk bozuldu\n", hata_n);
        BEKLENEN(0, "Monotonluk bozuk");
    }
}

// =====================================================
// TEST 3: dB ↔ lineer dönüşüm tutarlılığı
// =====================================================
void test_db_donusum(void) {
    TEST_BASLA("Test 3: dB ↔ lineer dönüşüm matematiği");

    // 20 dB = 10×
    float G_lin = powf(10.0f, 20.0f / 20.0f);
    BEKLENEN(fabsf(G_lin - 10.0f) < EPSILON, "20 dB = 10×");

    // 6 dB ≈ 2×
    G_lin = powf(10.0f, 6.0f / 20.0f);
    BEKLENEN(fabsf(G_lin - 2.0f) < 0.01f, "6 dB ≈ 2×");

    // 0 dB = 1×
    G_lin = powf(10.0f, 0.0f / 20.0f);
    BEKLENEN(fabsf(G_lin - 1.0f) < EPSILON, "0 dB = 1×");

    // 14.78 dB ≈ 5.48× (TVG min kazancı)
    G_lin = powf(10.0f, 14.78f / 20.0f);
    BEKLENEN(fabsf(G_lin - 5.48f) < 0.05f, "14.78 dB ≈ 5.48× (TVG min)");

    // 33.43 dB ≈ 46.95× (TVG max kazancı)
    G_lin = powf(10.0f, 33.43f / 20.0f);
    BEKLENEN(fabsf(G_lin - 46.95f) < 0.5f, "33.43 dB ≈ 46.95× (TVG max)");
}

// =====================================================
// TEST 4: TVG min/max sabitlerinin tutarlılığı
// =====================================================
void test_min_max_sabitler(void) {
    TEST_BASLA("Test 4: TVG_GAIN_DB_MIN ve TVG_GAIN_DB_MAX formülle uyumlu");

    // TVG_GAIN_DB_MIN N=255 için olmalı
    float G_n255 = TVG_GAIN_FORMUL(255);
    float G_db_n255 = 20.0f * log10f(G_n255);
    printf("    N=255 → %.2f dB  (TVG_GAIN_DB_MIN = %.2f)\n", G_db_n255, TVG_GAIN_DB_MIN);
    BEKLENEN(fabsf(G_db_n255 - TVG_GAIN_DB_MIN) < EPSILON_DB,
             "TVG_GAIN_DB_MIN değeri formülle uyumlu");

    // TVG_GAIN_DB_MAX N=0 için olmalı
    float G_n0 = TVG_GAIN_FORMUL(0);
    float G_db_n0 = 20.0f * log10f(G_n0);
    printf("    N=0   → %.2f dB  (TVG_GAIN_DB_MAX = %.2f)\n", G_db_n0, TVG_GAIN_DB_MAX);
    BEKLENEN(fabsf(G_db_n0 - TVG_GAIN_DB_MAX) < EPSILON_DB,
             "TVG_GAIN_DB_MAX değeri formülle uyumlu");
}

// =====================================================
// TEST 5: Direnç hesabı — R(N) formülü
// =====================================================
void test_direnc_formulu(void) {
    TEST_BASLA("Test 5: R(N) = 1075 + 39.0625 × N");

    // N=0 → R = 1075
    float R0 = TVG_R_SABIT_OHM + TVG_R_STEP_OHM * 0;
    BEKLENEN(fabsf(R0 - 1075.0f) < EPSILON, "N=0 → R = 1075 Ω");

    // N=128 → R = 1075 + 5000 = 6075
    float R128 = TVG_R_SABIT_OHM + TVG_R_STEP_OHM * 128;
    printf("    R(N=128) = %.2f Ω (beklenen 6075)\n", R128);
    BEKLENEN(fabsf(R128 - 6075.0f) < 0.5f, "N=128 → R ≈ 6075 Ω");

    // N=255 → R ≈ 11036.9
    float R255 = TVG_R_SABIT_OHM + TVG_R_STEP_OHM * 255;
    printf("    R(N=255) = %.2f Ω (beklenen 11035.94)\n", R255);
    BEKLENEN(fabsf(R255 - 11035.94f) < 0.5f, "N=255 → R ≈ 11036 Ω");
}

// =====================================================
// TEST 6: Rampa eğrisi — tüm 30ms boyunca
// dB lineer artıyor mu manuel doğrulama
// =====================================================
void test_rampa_egrisi(void) {
    TEST_BASLA("Test 6: Rampa eğrisi — t=0'da min kazanç, t=30'da max kazanç");

    // Manuel rampa hesabı
    // t=0  → G_dB = 14.78  → G_lin = 5.48  → N ≈ 255
    // t=15 → G_dB = 24.10  → G_lin ≈ 16.05 → N ≈ ?
    // t=30 → G_dB = 33.43  → G_lin = 46.95 → N ≈ 0

    float t = 0.0f;
    float oran = t / 30.0f;
    float G_dB = TVG_GAIN_DB_MIN + (TVG_GAIN_DB_MAX - TVG_GAIN_DB_MIN) * oran;
    float G_lin = powf(10.0f, G_dB / 20.0f);
    float N = (TVG_INA_KATSAYI / (G_lin - 1.0f) - TVG_R_SABIT_OHM) / TVG_R_STEP_OHM;

    printf("    t=0  → G_dB=%.2f → N=%.1f  (beklenen ~255)\n", G_dB, N);
    BEKLENEN(N > 250.0f, "t=0 anında N ≈ 255 (min kazanç)");

    t = 30.0f;
    oran = t / 30.0f;
    G_dB = TVG_GAIN_DB_MIN + (TVG_GAIN_DB_MAX - TVG_GAIN_DB_MIN) * oran;
    G_lin = powf(10.0f, G_dB / 20.0f);
    N = (TVG_INA_KATSAYI / (G_lin - 1.0f) - TVG_R_SABIT_OHM) / TVG_R_STEP_OHM;

    printf("    t=30 → G_dB=%.2f → N=%.1f  (beklenen ~0)\n", G_dB, N);
    BEKLENEN(N < 5.0f, "t=30 anında N ≈ 0 (max kazanç)");

    // Ortada bir değer
    t = 15.0f;
    oran = t / 30.0f;
    G_dB = TVG_GAIN_DB_MIN + (TVG_GAIN_DB_MAX - TVG_GAIN_DB_MIN) * oran;
    G_lin = powf(10.0f, G_dB / 20.0f);
    N = (TVG_INA_KATSAYI / (G_lin - 1.0f) - TVG_R_SABIT_OHM) / TVG_R_STEP_OHM;

    printf("    t=15 → G_dB=%.2f → N=%.1f  (orta değer)\n", G_dB, N);
    BEKLENEN(N > 5 && N < 250, "t=15 anında N orta aralıkta");
}

// =====================================================
// MAIN
// =====================================================
int main(void) {
    printf("===============================\n");
    printf("    TVG MODÜL TESTLERİ        \n");
    printf("===============================\n");

    test_kazanc_formulu();
    test_monotonluk();
    test_db_donusum();
    test_min_max_sabitler();
    test_direnc_formulu();
    test_rampa_egrisi();

    printf("\n===============================\n");
    if (kalan == 0) {
        printf(GREEN "  ✓ %d/%d test geçti" RESET "\n", gecen, gecen + kalan);
    } else {
        printf(RED   "  ✗ %d test başarısız" RESET ", %d test geçti\n", kalan, gecen);
    }
    printf("===============================\n\n");

    return (kalan == 0) ? 0 : 1;
}
