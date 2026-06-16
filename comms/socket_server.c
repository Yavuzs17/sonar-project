/*
 * Derleme:
 *   gcc -c socket_server.c -lcjson -lpthread -Wall -Wextra
 *
 * Kurulum (Raspbian/Debian):
 *   sudo apt install libcjson-dev
 */

#include "socket_server.h"

#include <cjson/cJSON.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

/* ─── İç Durum ──────────────────────────────────────────────────────────────*/
static struct {
    int  server_fd;
    int  client_fd;          /* bağlı istemci fd; -1 ise yok                 */
    char socket_yolu[108];   /* AF_UNIX yol limiti 108 karakter               */

    pthread_t accept_thread;
    int       accept_aktif;

    volatile int calisiyor;  /* 0 → kapatma sinyali                           */

    pthread_mutex_t mutex;   /* client_fd ve yazma operasyonlarını korur       */

    comms_cmd_handler_t handler;

    int baslandi;
} srv;

/* ─── Log ───────────────────────────────────────────────────────────────────*/
static CommsLogLevel log_seviyesi = COMMS_LOG_INFO;

static void comms_log(CommsLogLevel sev, const char *fmt, ...) {
    if (sev < log_seviyesi) return;
    static const char * const etiket[] = {"DEBUG", "INFO", "WARN", "ERROR"};
    FILE *h = (sev >= COMMS_LOG_WARN) ? stderr : stdout;
    fprintf(h, "[COMMS][%s] ", etiket[sev]);
    va_list ap;
    va_start(ap, fmt);
    vfprintf(h, fmt, ap);
    va_end(ap);
    fputc('\n', h);
}

/* ─── İç Yardımcı: fd'ye JSON satırı yaz ───────────────────────────────────
 * msg + '\n' yazar. ÇAĞRI ÖNCESINDE mutex alınmış olmalıdır.              */
static int fd_yaz_kilitli(int fd, const char *msg) {
    size_t len = strlen(msg);
    if (write(fd, msg, len) < 0) return COMMS_ERR_SOCKET;
    if (write(fd, "\n", 1)  < 0) return COMMS_ERR_SOCKET;
    return COMMS_OK;
}

/* ─── İç Yardımcı: cJSON nesnesini serialize et ve istemciye gönder ────────
 * Mutex'i içeride alır; mutex tutulmadan çağrılmalıdır.                   */
static int json_gonder(cJSON *root) {
    char *str = cJSON_PrintUnformatted(root);
    if (!str) return COMMS_ERR_BUFFER;

    pthread_mutex_lock(&srv.mutex);
    int ret = COMMS_ERR_NO_CLIENT;
    if (srv.client_fd >= 0)
        ret = fd_yaz_kilitli(srv.client_fd, str);
    pthread_mutex_unlock(&srv.mutex);

    cJSON_free(str);
    return ret;
}

/* ─── İç Yardımcı: Event nesnesi oluştur ve gönder ─────────────────────────
 * fields (cJSON nesnesi, NULL olabilir) alanlarını type/name ile birleştirir.
 * fields'ın sahipliği bu fonksiyona devredilmez; çağıran temizler.        */
static int event_gonder(const char *event_name, cJSON *fields) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return COMMS_ERR_BUFFER;

    cJSON_AddStringToObject(root, "type", "event");
    cJSON_AddStringToObject(root, "name", event_name);

    if (fields) {
        for (cJSON *item = fields->child; item; item = item->next) {
            cJSON *copy = cJSON_Duplicate(item, 1);
            if (copy) cJSON_AddItemToObject(root, item->string, copy);
        }
    }

    int ret = json_gonder(root);
    cJSON_Delete(root);
    return ret;
}

/* ─── Komut İşleyici ────────────────────────────────────────────────────────
 * Gelen JSON satırını parse eder, kayıtlı handler'ı çağırır,
 * {"type":"response","cmd":"...",...} formatında yanıt gönderir.
 * istemci_read_thread'inden çağrılır; mutex TUTULMADAN çalışır.           */
/* Plain text komutun yanıtını oluşturup gönder.
 * JSON döndürmeyen ya da "gecersiz JSON" hatasını gizleyen bir yardımcı.   */
static void plain_yanit_gonder(const char *cmd_str, const char *resp_govde) {
    cJSON *yanit = cJSON_CreateObject();
    if (!yanit) return;

    cJSON_AddStringToObject(yanit, "type", "response");
    cJSON_AddStringToObject(yanit, "cmd",  cmd_str);

    cJSON *govde = cJSON_Parse(resp_govde);
    if (govde) {
        for (cJSON *item = govde->child; item; item = item->next) {
            cJSON *copy = cJSON_Duplicate(item, 1);
            if (copy) cJSON_AddItemToObject(yanit, item->string, copy);
        }
        cJSON_Delete(govde);
    } else {
        cJSON_AddStringToObject(yanit, "status", "error");
        cJSON_AddStringToObject(yanit, "message", "handler gecersiz JSON donurdu");
    }

    json_gonder(yanit);
    cJSON_Delete(yanit);
}

static void komut_isle(const char *satir, int fd) {
    (void)fd;  /* yanıtlar json_gonder üzerinden gidiyor; fd artık kullanılmıyor */

    cJSON *istek = cJSON_Parse(satir);
    if (!istek) {
        /* JSON değil — "cmd arg1 arg2 ..." plain text formatını dene */
        char plain_cmd[64] = "";
        if (sscanf(satir, "%63s", plain_cmd) != 1 || plain_cmd[0] == '\0') {
            comms_log(COMMS_LOG_WARN, "Gecersiz satir: %.80s", satir);
            const char *err = "{\"type\":\"response\",\"status\":\"error\","
                               "\"message\":\"gecersiz komut\"}\n";
            pthread_mutex_lock(&srv.mutex);
            write(srv.client_fd >= 0 ? srv.client_fd : -1, err, strlen(err));
            pthread_mutex_unlock(&srv.mutex);
            return;
        }

        comms_log(COMMS_LOG_INFO, "Plain text komut: %s", plain_cmd);

        pthread_mutex_lock(&srv.mutex);
        comms_cmd_handler_t h = srv.handler;
        pthread_mutex_unlock(&srv.mutex);

        char resp_govde[COMMS_BUFFER_BOYUT];
        resp_govde[0] = '\0';
        int handler_ret = h ? h(plain_cmd, satir, resp_govde, sizeof(resp_govde)) : -1;

        if (resp_govde[0] == '\0') {
            strncpy(resp_govde,
                    handler_ret == 0 ? "{\"status\":\"ok\"}" :
                    (h ? "{\"status\":\"error\",\"message\":\"bilinmeyen hata\"}" :
                         "{\"status\":\"error\",\"message\":\"handler kayitli degil\"}"),
                    sizeof(resp_govde) - 1);
        }

        plain_yanit_gonder(plain_cmd, resp_govde);
        return;
    }

    const char *cmd = cJSON_GetStringValue(cJSON_GetObjectItem(istek, "cmd"));
    if (!cmd) {
        comms_log(COMMS_LOG_WARN, "JSON'da cmd alani eksik");
        const char *err = "{\"type\":\"response\",\"status\":\"error\","
                           "\"message\":\"cmd alani eksik\"}\n";
        pthread_mutex_lock(&srv.mutex);
        write(fd, err, strlen(err));
        pthread_mutex_unlock(&srv.mutex);
        cJSON_Delete(istek);
        return;
    }

    /* Handler'ı kilitsiz çağır — handler içinden comms_emit_event çağrılabilir */
    char resp_govde[COMMS_BUFFER_BOYUT];
    resp_govde[0] = '\0';
    int handler_ret = -1;

    pthread_mutex_lock(&srv.mutex);
    comms_cmd_handler_t h = srv.handler;
    pthread_mutex_unlock(&srv.mutex);

    if (h) {
        handler_ret = h(cmd, satir, resp_govde, sizeof(resp_govde));
    }

    /* Varsayılan yanıt gövdesi — handler boş bıraktıysa */
    if (resp_govde[0] == '\0') {
        if (handler_ret == 0) {
            strncpy(resp_govde, "{\"status\":\"ok\"}", sizeof(resp_govde) - 1);
        } else if (!h) {
            strncpy(resp_govde,
                    "{\"status\":\"error\",\"message\":\"handler kayitli degil\"}",
                    sizeof(resp_govde) - 1);
        } else {
            strncpy(resp_govde,
                    "{\"status\":\"error\",\"message\":\"bilinmeyen hata\"}",
                    sizeof(resp_govde) - 1);
        }
    }

    /* Tam yanıt nesnesini oluştur: type + cmd + handler gövde alanları */
    cJSON *yanit = cJSON_CreateObject();
    if (yanit) {
        cJSON_AddStringToObject(yanit, "type", "response");
        cJSON_AddStringToObject(yanit, "cmd",  cmd);

        cJSON *govde = cJSON_Parse(resp_govde);
        if (govde) {
            for (cJSON *item = govde->child; item; item = item->next) {
                cJSON *copy = cJSON_Duplicate(item, 1);
                if (copy) cJSON_AddItemToObject(yanit, item->string, copy);
            }
            cJSON_Delete(govde);
        } else {
            cJSON_AddStringToObject(yanit, "status", "error");
            cJSON_AddStringToObject(yanit, "message", "handler gecersiz JSON donurdu");
        }

        json_gonder(yanit);
        cJSON_Delete(yanit);
    }

    cJSON_Delete(istek);
}

/* ─── İstemci Okuma Thread'i ────────────────────────────────────────────────
 * Bağlı istemciden satır satır JSON okur, her satırı komut_isle'ye verir.
 * İstemci ayrıldığında veya srv.calisiyor = 0 olduğunda çıkar.            */
static void *istemci_read_thread(void *arg) {
    int fd = *(int *)arg;
    free(arg);

    char buf[COMMS_BUFFER_BOYUT * 2];
    size_t pos = 0;

    comms_log(COMMS_LOG_INFO, "Okuma thread'i basladi (fd=%d)", fd);

    while (srv.calisiyor) {
        ssize_t n = read(fd, buf + pos, sizeof(buf) - pos - 1);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }
        if (n == 0) break;   /* istemci kapandı (EOF) */

        pos += (size_t)n;
        buf[pos] = '\0';

        /* Tamamlanmış satırları ('\n' ile biten) işle */
        char *baslangic = buf;
        char *nl;
        while ((nl = memchr(baslangic, '\n',
                            (size_t)(buf + pos - baslangic))) != NULL) {
            *nl = '\0';
            if (nl > baslangic)
                komut_isle(baslangic, fd);
            baslangic = nl + 1;
        }

        /* Kısmi satırı tamponun başına taşı */
        size_t kalan = (size_t)(buf + pos - baslangic);
        if (kalan && baslangic != buf)
            memmove(buf, baslangic, kalan);
        pos = kalan;

        /* Tampon doldu ama satır sonu yok → mesaj çok büyük, sıfırla */
        if (pos >= sizeof(buf) - 1) {
            comms_log(COMMS_LOG_WARN, "Satir tamponu doldu, temizlendi");
            pos = 0;
        }
    }

    comms_log(COMMS_LOG_INFO, "Istemci ayrildi (fd=%d)", fd);

    /* client_fd'yi temizle — cleanup shutdown çağırmış olabilir, double-close önle */
    pthread_mutex_lock(&srv.mutex);
    if (srv.client_fd == fd) {
        close(fd);
        srv.client_fd = -1;
    }
    pthread_mutex_unlock(&srv.mutex);

    return NULL;
}

/* ─── Kabul Thread'i ─────────────────────────────────────────────────────────
 * Yeni bağlantıları kabul eder ve her biri için bir okuma thread'i başlatır.
 * Okuma thread'ini join ederek sıralı çalışır (şimdilik tek istemci).     */
static void *kabul_thread(void *arg) {
    (void)arg;
    comms_log(COMMS_LOG_INFO, "Kabul thread'i basladi");

    while (srv.calisiyor) {
        struct sockaddr_un addr;
        socklen_t alen = sizeof(addr);

        int cfd = accept(srv.server_fd, (struct sockaddr *)&addr, &alen);
        if (cfd < 0) {
            if (!srv.calisiyor) break;
            if (errno == EINTR) continue;
            comms_log(COMMS_LOG_WARN, "accept hatasi: %s", strerror(errno));
            continue;
        }

        pthread_mutex_lock(&srv.mutex);
        if (srv.client_fd >= 0) {
            comms_log(COMMS_LOG_WARN, "Istemci zaten bagli, yeni baglanti reddedildi");
            close(cfd);
            pthread_mutex_unlock(&srv.mutex);
            continue;
        }
        srv.client_fd = cfd;
        pthread_mutex_unlock(&srv.mutex);

        comms_log(COMMS_LOG_INFO, "Istemci baglandi (fd=%d)", cfd);

        int *fdp = malloc(sizeof(int));
        if (!fdp) {
            comms_log(COMMS_LOG_ERROR, "malloc hatasi");
            pthread_mutex_lock(&srv.mutex);
            close(cfd);
            srv.client_fd = -1;
            pthread_mutex_unlock(&srv.mutex);
            continue;
        }
        *fdp = cfd;

        pthread_t rt;
        if (pthread_create(&rt, NULL, istemci_read_thread, fdp) != 0) {
            comms_log(COMMS_LOG_ERROR, "Okuma thread'i olusturulamadi");
            free(fdp);
            pthread_mutex_lock(&srv.mutex);
            close(cfd);
            srv.client_fd = -1;
            pthread_mutex_unlock(&srv.mutex);
            continue;
        }

        /* Bu istemci ayrılana kadar bekle, sonra yeni bağlantı kabul et */
        pthread_join(rt, NULL);
    }

    comms_log(COMMS_LOG_INFO, "Kabul thread'i sonlandi");
    return NULL;
}

/* ─── Public API — Başlatma ─────────────────────────────────────────────────*/

int comms_init(const char *socket_path) {
    if (srv.baslandi) {
        comms_log(COMMS_LOG_WARN, "comms_init zaten cagirildi");
        return COMMS_OK;
    }

    /* SIGPIPE'ı yoksay — istemci kapanırken write crash yapmasın */
    signal(SIGPIPE, SIG_IGN);

    memset(&srv, 0, sizeof(srv));
    srv.server_fd = -1;
    srv.client_fd = -1;
    srv.calisiyor = 1;

    strncpy(srv.socket_yolu,
            socket_path ? socket_path : COMMS_SOCKET_PATH,
            sizeof(srv.socket_yolu) - 1);

    if (pthread_mutex_init(&srv.mutex, NULL) != 0) {
        comms_log(COMMS_LOG_ERROR, "Mutex baslatilamadi");
        return COMMS_ERR_INIT;
    }

    unlink(srv.socket_yolu);   /* eski socket dosyasını sil */

    srv.server_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (srv.server_fd < 0) {
        comms_log(COMMS_LOG_ERROR, "socket() hatasi: %s", strerror(errno));
        pthread_mutex_destroy(&srv.mutex);
        return COMMS_ERR_SOCKET;
    }

    struct sockaddr_un saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sun_family = AF_UNIX;
    strncpy(saddr.sun_path, srv.socket_yolu, sizeof(saddr.sun_path) - 1);

    if (bind(srv.server_fd, (struct sockaddr *)&saddr, sizeof(saddr)) < 0) {
        comms_log(COMMS_LOG_ERROR, "bind() hatasi: %s", strerror(errno));
        close(srv.server_fd);
        pthread_mutex_destroy(&srv.mutex);
        return COMMS_ERR_BIND;
    }

    if (listen(srv.server_fd, COMMS_MAX_CLIENTS) < 0) {
        comms_log(COMMS_LOG_ERROR, "listen() hatasi: %s", strerror(errno));
        close(srv.server_fd);
        unlink(srv.socket_yolu);
        pthread_mutex_destroy(&srv.mutex);
        return COMMS_ERR_LISTEN;
    }

    if (pthread_create(&srv.accept_thread, NULL, kabul_thread, NULL) != 0) {
        comms_log(COMMS_LOG_ERROR, "Kabul thread'i olusturulamadi");
        close(srv.server_fd);
        unlink(srv.socket_yolu);
        pthread_mutex_destroy(&srv.mutex);
        return COMMS_ERR_THREAD;
    }

    srv.accept_aktif = 1;
    srv.baslandi     = 1;

    comms_log(COMMS_LOG_INFO, "Hazir — %s dinleniyor", srv.socket_yolu);
    return COMMS_OK;
}

/* ─── Public API — Temizleme ────────────────────────────────────────────────*/

void comms_cleanup(void) {
    if (!srv.baslandi) return;
    comms_log(COMMS_LOG_INFO, "Kapatiliyor");

    srv.calisiyor = 0;

    /* server_fd'yi kapat → accept() hata döner, kabul_thread döngüden çıkar */
    if (srv.server_fd >= 0) {
        close(srv.server_fd);
        srv.server_fd = -1;
    }

    /* shutdown → istemci_read_thread'deki read() EOF görür, çıkar
     * → kabul_thread'deki pthread_join döner → kabul_thread çıkar
     * shutdown kullanmak close'tan güvenli: fd numarası hemen serbest kalmaz */
    pthread_mutex_lock(&srv.mutex);
    int cfd = srv.client_fd;
    pthread_mutex_unlock(&srv.mutex);

    if (cfd >= 0)
        shutdown(cfd, SHUT_RDWR);

    if (srv.accept_aktif) {
        pthread_join(srv.accept_thread, NULL);
        srv.accept_aktif = 0;
    }
    /* kabul_thread, read thread'i join etti; client_fd'yi read thread kapattı */

    unlink(srv.socket_yolu);
    pthread_mutex_destroy(&srv.mutex);
    srv.baslandi = 0;
    comms_log(COMMS_LOG_INFO, "Kapatildi");
}

/* ─── Public API — Handler Kaydı ───────────────────────────────────────────*/

int comms_register_handler(comms_cmd_handler_t handler) {
    if (!handler) return COMMS_ERR_NULL;
    pthread_mutex_lock(&srv.mutex);
    srv.handler = handler;
    pthread_mutex_unlock(&srv.mutex);
    return COMMS_OK;
}

/* ─── Public API — Event Yayınlama ─────────────────────────────────────────
 * params_json: ek alanları içeren JSON nesnesi (ör. {"dist":1.45,"az":15})
 * NULL veya boş string geçilebilir.                                        */
int comms_emit_event(const char *event_name, const char *params_json) {
    if (!event_name) return COMMS_ERR_NULL;
    if (!srv.baslandi) return COMMS_ERR_INIT;

    cJSON *fields = NULL;
    if (params_json && strlen(params_json) > 2)
        fields = cJSON_Parse(params_json);

    int ret = event_gonder(event_name, fields);
    if (fields) cJSON_Delete(fields);
    return ret;
}

/* ─── Kısayol Event Fonksiyonları ───────────────────────────────────────────*/

int comms_emit_echo(float dist, float delay_us, float amp, float az, float el) {
    cJSON *p = cJSON_CreateObject();
    if (!p) return COMMS_ERR_BUFFER;
    cJSON_AddNumberToObject(p, "dist",     (double)dist);
    cJSON_AddNumberToObject(p, "delay_us", (double)delay_us);
    cJSON_AddNumberToObject(p, "amp",      (double)amp);
    cJSON_AddNumberToObject(p, "az",       (double)az);
    cJSON_AddNumberToObject(p, "el",       (double)el);
    int ret = event_gonder("echo", p);
    cJSON_Delete(p);
    return ret;
}

int comms_emit_echo_batch(const float *dist, const float *delay_us,
                          const float *amp, int n, float az, float el) {
    if (n <= 0) return COMMS_OK;            /* boş batch — no-op */
    if (!dist || !delay_us || !amp) return COMMS_ERR_NULL;

    cJSON *p = cJSON_CreateObject();
    if (!p) return COMMS_ERR_BUFFER;
    cJSON_AddNumberToObject(p, "az", (double)az);
    cJSON_AddNumberToObject(p, "el", (double)el);

    cJSON *arr = cJSON_CreateArray();
    if (!arr) {
        cJSON_Delete(p);
        return COMMS_ERR_BUFFER;
    }

    for (int i = 0; i < n; i++) {
        cJSON *item = cJSON_CreateObject();
        if (!item) {
            cJSON_Delete(arr);
            cJSON_Delete(p);
            return COMMS_ERR_BUFFER;
        }
        cJSON_AddNumberToObject(item, "dist",     (double)dist[i]);
        cJSON_AddNumberToObject(item, "delay_us", (double)delay_us[i]);
        cJSON_AddNumberToObject(item, "amp",      (double)amp[i]);
        cJSON_AddItemToArray(arr, item);
    }
    cJSON_AddItemToObject(p, "echoes", arr);

    int ret = event_gonder("echo_batch", p);
    cJSON_Delete(p);
    return ret;
}

int comms_emit_doppler(float freq, float velocity) {
    cJSON *p = cJSON_CreateObject();
    if (!p) return COMMS_ERR_BUFFER;
    cJSON_AddNumberToObject(p, "freq",     (double)freq);
    cJSON_AddNumberToObject(p, "velocity", (double)velocity);
    int ret = event_gonder("doppler", p);
    cJSON_Delete(p);
    return ret;
}

int comms_emit_scan_progress(int step, int total, float az, float el) {
    cJSON *p = cJSON_CreateObject();
    if (!p) return COMMS_ERR_BUFFER;
    cJSON_AddNumberToObject(p, "step",  (double)step);
    cJSON_AddNumberToObject(p, "total", (double)total);
    cJSON_AddNumberToObject(p, "az",    (double)az);
    cJSON_AddNumberToObject(p, "el",    (double)el);
    int ret = event_gonder("scan_progress", p);
    cJSON_Delete(p);
    return ret;
}

int comms_emit_temperature(float celsius, float sound_speed) {
    cJSON *p = cJSON_CreateObject();
    if (!p) return COMMS_ERR_BUFFER;
    cJSON_AddNumberToObject(p, "celsius",     (double)celsius);
    cJSON_AddNumberToObject(p, "sound_speed", (double)sound_speed);
    int ret = event_gonder("temperature", p);
    cJSON_Delete(p);
    return ret;
}

int comms_emit_error(const char *module, const char *message) {
    if (!module || !message) return COMMS_ERR_NULL;
    cJSON *p = cJSON_CreateObject();
    if (!p) return COMMS_ERR_BUFFER;
    cJSON_AddStringToObject(p, "module",  module);
    cJSON_AddStringToObject(p, "message", message);
    int ret = event_gonder("error", p);
    cJSON_Delete(p);
    return ret;
}

/* ─── Public API — Bağlantı Durumu ─────────────────────────────────────────*/

int comms_is_client_connected(void) {
    pthread_mutex_lock(&srv.mutex);
    int c = (srv.client_fd >= 0) ? 1 : 0;
    pthread_mutex_unlock(&srv.mutex);
    return c;
}

int comms_get_connected_count(void) {
    return comms_is_client_connected();
}

/* ─── Public API — Log Seviyesi ─────────────────────────────────────────────*/

void comms_set_log_level(CommsLogLevel level) {
    log_seviyesi = level;
}
