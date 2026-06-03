/*
 * Preamp modülü mantık testleri
 *
 * Derleme:
 *   gcc -o test_preamp tests/test_preamp.c preamp/preamp.c -lm -lpthread -Wall
 *
 * Çalıştırma:
 *   ./test_preamp
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../preamp/preamp.h"

#define EPSILON 0.01f

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

// Mock lgpio
int lgI2cOpen(int dev, int addr, int flags) { (void)dev; (void)addr; (void)flags; return 1; }
int lgI2cClose(int handle) { (void)handle; return 0; }
int lgI2cWriteDevice(int handle, const char *buf, int count) { (void)handle; (void)buf; (void)count; return 0; }
int lgI2cReadDevice(int handle, char *buf, int count) {
    (void)handle;
    for (int i = 0; i < count; i++) buf[i] = 0;
    return count;
}

// =====================================================
// TEST 1: Kazanç formülü — G = 1 + (R_pot / R_giris)
// =====================================================
void test_kazanc_formulu(void) {
    TEST_BASLA("Test 1: Kazanç formülü G = 1 + (step × 39 / 2200)");

    // N=0   → R_pot=0     → G = 1.0
    // N=64  → R_pot=2496  → G = 1 + 2496/2200 = 2.135
    // N=128 → R_pot=4992  → G = 1 + 4992/2200 = 3.269
    // N=255 → R_pot=9945  → G = 1 + 9945/2200 = 5.520

    float G0   = PREAMP_GAIN_FORMUL(0);
    float G64  = PREAMP_GAIN_FORMUL(64);
    float G128 = PREAMP_GAIN_FORMUL(128);
    float G255 = PREAMP_GAIN_FORMUL(255);

    printf("    G(N=0)   = %.3f  (beklenen 1.000)\n", G0);
    printf("    G(N=64)  = %.3f  (beklenen ~2.135)\n", G64);
    printf("    G(N=128) = %.3f  (beklenen ~3.269)\n", G128);
    printf("    G(N=255) = %.3f  (beklenen ~5.520)\n", G255);

    BEKLENEN(fabsf(G0   - 1.000f) < 0.01f, "N=0 → kazanç 1.0 (giriş = çıkış)");
    BEKLENEN(fabsf(G64  - 2.135f) < 0.01f, "N=64 → kazanç ~2.135");
    BEKLENEN(fabsf(G128 - 3.269f) < 0.01f, "N=128 → kazanç ~3.269");
    BEKLENEN(fabsf(G255 - 5.520f) < 0.01f, "N=255 → kazanç ~5.520");
}

// =====================================================
// TEST 2: Monotonluk — N büyürse kazanç ARTAR
// =====================================================
void test_monotonluk(void) {
    TEST_BASLA("Test 2: N arttıkça kazanç monoton artar");

    float onceki = PREAMP_GAIN_FORMUL(0);
    int monoton = 1;

    for (int N = 1; N <= 255; N++) {
        float simdi = PREAMP_GAIN_FORMUL(N);
        if (simdi <= onceki) { monoton = 0; break; }
        onceki = simdi;
    }

    BEKLENEN(monoton, "Tüm N için kazanç artıyor");
}

// =====================================================
// TEST 3: dB dönüşümü
// =====================================================
void test_db_donusum(void) {
    TEST_BASLA("Test 3: dB cinsinden kazanç");

    // G=1   → 0 dB
    // G=2.135 → 6.59 dB
    // G=5.52 → 14.83 dB

    float G = PREAMP_GAIN_FORMUL(0);
    float G_db = 20.0f * log10f(G);
    BEKLENEN(fabsf(G_db) < 0.01f, "N=0 → 0 dB");

    G = PREAMP_GAIN_FORMUL(255);
    G_db = 20.0f * log10f(G);
    printf("    N=255 → %.2f dB (beklenen ~14.83 dB)\n", G_db);
    BEKLENEN(fabsf(G_db - 14.83f) < 0.1f, "N=255 → ~14.83 dB");
}

// =====================================================
// TEST 4: Kanal mapping doğruluğu
// =====================================================
void test_kanal_mapping(void) {
    TEST_BASLA("Test 4: 7 alıcı için kanal mapping");

    if (preamp_init() != PREAMP_OK) {
        printf("    HATA: preamp_init başarısız\n");
        kalan++;
        return;
    }

    // Her kanal için step yazılabiliyor mu (mock yazma her zaman başarılı)
    for (int i = 0; i < 7; i++) {
        int sonuc = preamp_set_step(i, 128);
        char mesaj[80];
        snprintf(mesaj, sizeof(mesaj), "Alıcı %d step yazma başarılı", i);
        BEKLENEN(sonuc == PREAMP_OK, mesaj);
    }

    // Geçersiz kanal
    int sonuc = preamp_set_step(7, 128);
    BEKLENEN(sonuc < 0, "Alıcı 7 (geçersiz) hata döndürdü");

    sonuc = preamp_set_step(99, 128);
    BEKLENEN(sonuc < 0, "Alıcı 99 (geçersiz) hata döndürdü");
}

// =====================================================
// TEST 5: Toplu ayarlama
// =====================================================
void test_toplu_ayar(void) {
    TEST_BASLA("Test 5: preamp_set_all_step toplu yazma");

    int sonuc = preamp_set_all_step(100);
    BEKLENEN(sonuc == PREAMP_OK, "Tüm 7 alıcıya step=100 yazıldı");

    sonuc = preamp_set_all_gain_db(10.0f);
    // LUT boş, formül kullanılır
    BEKLENEN(sonuc == PREAMP_OK || sonuc == PREAMP_ERR_LUT_BOS,
             "Tüm 7 alıcıya 10 dB ayarlandı (LUT boşsa formül)");
}

// =====================================================
// MAIN
// =====================================================
int main(void) {
    printf("===============================\n");
    printf("   PREAMP MODÜL TESTLERİ      \n");
    printf("===============================\n");

    test_kazanc_formulu();
    test_monotonluk();
    test_db_donusum();
    test_kanal_mapping();
    test_toplu_ayar();

    printf("\n===============================\n");
    if (kalan == 0) {
        printf(GREEN "  ✓ %d/%d test geçti" RESET "\n", gecen, gecen + kalan);
    } else {
        printf(RED   "  ✗ %d test başarısız" RESET ", %d test geçti\n", kalan, gecen);
    }
    printf("===============================\n\n");

    return (kalan == 0) ? 0 : 1;
}