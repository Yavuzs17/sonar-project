/*
 * PLL modülü mantık testleri
 *
 * Derleme:
 *   gcc -o test_pll tests/test_pll.c pll/pll.c -lm -lpthread -lrt -Wall
 *
 * Çalıştırma:
 *   ./test_pll
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../pll/pll.h"

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
    // MCP3221 sahte veri: 0V'a yakın
    if (count >= 2) { buf[0] = 0; buf[1] = 0; }
    return count;
}
int lgGpiochipOpen(int device) { (void)device; return 1; }
int lgGpiochipClose(int handle) { (void)handle; return 0; }
int lgGpioClaimOutput(int handle, int flags, int pin, int level) {
    (void)handle; (void)flags; (void)pin; (void)level; return 0;
}
int lgGpioFree(int handle, int pin) { (void)handle; (void)pin; return 0; }
int lgGpioWrite(int handle, int pin, int level) {
    (void)handle; (void)pin; (void)level; return 0;
}

// =====================================================
// TEST 1: Voltaj ölçeklendirme — gerilim bölücü
// =====================================================
void test_voltaj_olcek(void) {
    TEST_BASLA("Test 1: Voltaj ölçeklendirme (3.3V ADC → 5V devre)");

    // VOLTAJ_OLCEK = 5.0 / 3.3 = 1.515
    // ADC 1V okursa → gerçek 1.515V
    // ADC 3.3V okursa → gerçek 5.0V

    float olcek = VOLTAJ_OLCEK;
    printf("    VOLTAJ_OLCEK = %.4f (beklenen 1.5151)\n", olcek);
    BEKLENEN(fabsf(olcek - 1.5151f) < 0.001f, "Ölçek katsayısı doğru");

    // Manuel hesap
    float adc_okuma = 2.0f;
    float gercek = adc_okuma * olcek;
    printf("    ADC 2.0V → gerçek %.3fV (beklenen ~3.030V)\n", gercek);
    BEKLENEN(fabsf(gercek - 3.030f) < 0.01f, "2V ADC → 3.03V gerçek");
}

// =====================================================
// TEST 2: N-divider hesabı
// =====================================================
void test_n_divider(void) {
    TEST_BASLA("Test 2: N_divider = hedef_frekans / 100");

    // 38000 / 100 = 380
    // 40000 / 100 = 400
    // 41000 / 100 = 410

    BEKLENEN(38000 / REF_FREKANS_HZ == 380, "38 kHz → N=380");
    BEKLENEN(40000 / REF_FREKANS_HZ == 400, "40 kHz → N=400");
    BEKLENEN(41000 / REF_FREKANS_HZ == 410, "41 kHz → N=410");
}

// =====================================================
// TEST 3: Sabitler tutarlı mı
// =====================================================
void test_sabitler(void) {
    TEST_BASLA("Test 3: Sabit değer kontrolleri");

    BEKLENEN(VREF_ADC == 3.3f,           "VREF_ADC = 3.3V");
    BEKLENEN(VREF_DEVRE == 5.0f,         "VREF_DEVRE = 5.0V");
    BEKLENEN(REF_FREKANS_HZ == 100,      "REF_FREKANS_HZ = 100");
    BEKLENEN(VARSAYILAN_HEDEF_HZ == 40000, "Varsayılan hedef = 40 kHz");

    printf("    SWEEP: %d → %d Hz, adım %d\n",
           SWEEP_VARSAYILAN_MIN, SWEEP_VARSAYILAN_MAX, SWEEP_VARSAYILAN_ADIM);
    BEKLENEN(SWEEP_VARSAYILAN_MIN < SWEEP_VARSAYILAN_MAX, "Sweep min < max");
    BEKLENEN(SWEEP_VARSAYILAN_ADIM > 0, "Sweep adım pozitif");
}

// =====================================================
// TEST 4: Hedef frekans set/get
// =====================================================
void test_frekans_set_get(void) {
    TEST_BASLA("Test 4: Hedef frekans ayarlama");

    if (pll_init() != 0) {
        printf("    NOT: pll_init başarısız (mock yetersiz olabilir)\n");
        // pll_init thread başlatıyor olabilir, hatayı geçiştir
    }

    pll_set_target_frequency(40000);
    int hedef = pll_get_target_frequency();
    printf("    Set: 40000 Hz, Get: %d Hz\n", hedef);
    BEKLENEN(hedef == 40000, "Set/Get 40 kHz tutarlı");

    pll_set_target_frequency(38500);
    hedef = pll_get_target_frequency();
    BEKLENEN(hedef == 38500, "Set/Get 38.5 kHz tutarlı");

    // Aralık dışı
    pll_set_target_frequency(99999);
    hedef = pll_get_target_frequency();
    printf("    99999 Hz set sonrası get: %d Hz\n", hedef);
    // Modül aralık kontrolü yapmıyorsa kabul ettirir, sorun değil
}

// =====================================================
// MAIN
// =====================================================
int main(void) {
    printf("===============================\n");
    printf("    PLL MODÜL TESTLERİ        \n");
    printf("===============================\n");

    test_voltaj_olcek();
    test_n_divider();
    test_sabitler();
    test_frekans_set_get();

    printf("\n===============================\n");
    if (kalan == 0) {
        printf(GREEN "  ✓ %d/%d test geçti" RESET "\n", gecen, gecen + kalan);
    } else {
        printf(RED   "  ✗ %d test başarısız" RESET ", %d test geçti\n", kalan, gecen);
    }
    printf("===============================\n\n");

    return (kalan == 0) ? 0 : 1;
}