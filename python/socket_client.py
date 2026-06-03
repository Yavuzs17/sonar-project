"""
SonarClient — Unix domain socket üzerinden C sonar sistemiyle haberleşen Python istemcisi.

Bağımlılıklar: Sadece Python standart kütüphanesi (socket, json, threading, queue).
Kullanım:      from socket_client import SonarClient
"""

import json
import logging
import queue
import socket
import threading
import time
from typing import Any, Callable, Dict, List, Optional


# ─── Özel İstisnalar ──────────────────────────────────────────────────────────

class SonarConnectionError(Exception):
    """Bağlantı hatalarını (bağlı değil, bağlantı kesildi) temsil eder."""


class SonarTimeoutError(Exception):
    """Komut yanıtı bekleme süresi dolduğunda fırlatılır."""


# ─── İstemci Sınıfı ───────────────────────────────────────────────────────────

class SonarClient:
    """
    Unix domain socket üzerinden C sonar sistemine bağlanan istemci.

    Komutlar senkron (send_command), asenkron event'ler ise on_event ile
    kayıtlı callback'lere dağıtılır. Arka planda bir okuma thread'i çalışır:
      - type:"response" → senkron send_command'ın beklediği kuyruğa
      - type:"event"    → kayıtlı callback'lere

    Not: C tarafı cmd_id desteklemediğinden gönderimler seri hâle getirilir
    (_send_lock), böylece yanıt–komut eşleşmesi garanti edilir.
    """

    def __init__(
        self,
        socket_path: str = "/tmp/sonar.sock",
        auto_reconnect: bool = False,
        reconnect_interval: float = 2.0,
    ):
        """
        socket_path        : Unix socket dosyasının yolu.
        auto_reconnect     : Bağlantı kopunca otomatik yeniden bağlan.
        reconnect_interval : Yeniden deneme aralığı (saniye).
        """
        self.socket_path = socket_path
        self.auto_reconnect = auto_reconnect
        self.reconnect_interval = reconnect_interval

        self.logger = logging.getLogger("SonarClient")

        self._sock: Optional[socket.socket] = None
        self._connected = False
        self._running = False

        self._reader_thread: Optional[threading.Thread] = None

        # Senkron komut/yanıt sıralaması
        self._send_lock = threading.Lock()
        self._response_queue: queue.Queue = queue.Queue()

        # Asenkron event callback'leri: event_name → [callback, ...]
        self._event_callbacks: Dict[str, List[Callable]] = {}
        self._callback_lock = threading.Lock()

    # ─── Bağlantı Yönetimi ────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        C tarafına bağlanır. Başarılıysa True, değilse False döner.
        Exception fırlatmaz; hata log'a yazılır.
        """
        if self._connected:
            self.logger.warning("Zaten bağlı.")
            return True

        if not hasattr(socket, "AF_UNIX"):
            self.logger.error("AF_UNIX is not supported on this platform (Windows).")
            return False

        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.connect(self.socket_path)
        except (FileNotFoundError, ConnectionRefusedError, OSError) as exc:
            self.logger.error("Bağlantı kurulamadı (%s): %s", self.socket_path, exc)
            return False

        self._sock = sock
        self._connected = True
        self._running = True

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="SonarReader",
            daemon=True,
        )
        self._reader_thread.start()

        self.logger.info("Bağlı: %s", self.socket_path)
        return True

    def disconnect(self):
        """Bağlantıyı kapatır ve okuma thread'ini temiz biçimde sonlandırır."""
        self._running = False
        self._connected = False

        # Socket'i kapat → reader thread'deki recv() blokesi kalkar
        if self._sock:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

        # Bekleyen send_command çağrılarını serbest bırak
        self._response_queue.put({"type": "disconnect", "message": "bağlantı kapatıldı"})

        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=3.0)
            self._reader_thread = None

        self.logger.info("Bağlantı kapatıldı.")

    def is_connected(self) -> bool:
        """Bağlantı aktif mi?"""
        return self._connected

    # ─── Okuma Thread'i ───────────────────────────────────────────────────────

    def _reader_loop(self):
        """
        Arka planda socket'ten bayt okur, satırlara böler ve mesajları yönlendirir.
        Bağlantı kapanınca (EOF veya hata) temiz biçimde çıkar.
        """
        self.logger.debug("Okuma thread'i başladı.")
        buf = b""

        try:
            while self._running:
                try:
                    chunk = self._sock.recv(4096)
                except OSError:
                    break

                if not chunk:
                    break  # EOF — C tarafı kapandı

                buf += chunk

                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    line = raw_line.decode("utf-8", errors="replace").strip()
                    if line:
                        self._handle_message(line)

        except Exception as exc:  # noqa: BLE001
            self.logger.debug("Okuma thread'i beklenmedik hata: %s", exc)
        finally:
            self._connected = False
            # Bekleyen send_command'ları serbest bırak
            self._response_queue.put(
                {"type": "disconnect", "message": "bağlantı kesildi"}
            )
            self.logger.info("Okuma thread'i sonlandı.")

            if self.auto_reconnect and self._running:
                self._schedule_reconnect()

    def _handle_message(self, line: str):
        """Gelen JSON satırını parse edip response veya event olarak yönlendirir."""
        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            self.logger.warning("Geçersiz JSON: %.80s", line)
            return

        self.logger.debug("← %s", line[:200])

        msg_type = msg.get("type")
        if msg_type == "response":
            self._response_queue.put(msg)
        elif msg_type == "event":
            self._dispatch_event(msg)
        else:
            self.logger.debug("Bilinmeyen mesaj tipi: %s", msg_type)

    def _dispatch_event(self, event_data: Dict[str, Any]):
        """Kayıtlı event callback'lerini sırayla çağırır."""
        event_name = event_data.get("name", "")
        with self._callback_lock:
            specific = self._event_callbacks.get(event_name, []).copy()
            wildcard = self._event_callbacks.get("*", []).copy()

        for cb in specific + wildcard:
            try:
                cb(event_data)
            except Exception as exc:  # noqa: BLE001
                self.logger.error("Event callback hatası (%s): %s", event_name, exc)

    # ─── Yeniden Bağlanma ─────────────────────────────────────────────────────

    def _schedule_reconnect(self):
        """auto_reconnect aktifse arka planda yeniden bağlanma döngüsü başlatır."""
        def _loop():
            self.logger.info(
                "Yeniden bağlanma bekleniyor (%.1fs arayla)...",
                self.reconnect_interval,
            )
            while self._running:
                time.sleep(self.reconnect_interval)
                if not self._running:
                    break
                self.logger.info("Yeniden bağlanma deneniyor...")
                if self.connect():
                    self.logger.info("Yeniden bağlandı.")
                    break

        threading.Thread(target=_loop, name="SonarReconnect", daemon=True).start()

    # ─── Komut Gönderme ───────────────────────────────────────────────────────

    def send_command(
        self,
        cmd: str,
        params: Optional[Dict[str, Any]] = None,
        timeout: float = 2.0,
    ) -> Dict[str, Any]:
        """
        C tarafına JSON komutu gönderir ve yanıtı bekler (bloke edici).

        cmd     : Komut adı (ör. "steer", "freq", "read_temp")
        params  : Ek parametreler (ör. {"az": 15.0, "el": 20.0})
        timeout : Yanıt bekleme süresi (saniye)

        Dönüş   : Yanıt dict'i ({"type":"response","cmd":"...","status":"ok",...})

        Raises:
            SonarConnectionError : Bağlantı yok veya koptu.
            SonarTimeoutError    : Yanıt timeout süresi içinde gelmedi.
        """
        if not self._connected:
            raise SonarConnectionError("Bağlı değil.")

        payload: Dict[str, Any] = {"cmd": cmd}
        if params:
            payload.update(params)

        data = json.dumps(payload, ensure_ascii=False) + "\n"

        with self._send_lock:
            # Önceki çağrıdan kalmış bayat yanıtları temizle
            while not self._response_queue.empty():
                try:
                    self._response_queue.get_nowait()
                except queue.Empty:
                    break

            try:
                self.logger.debug("→ %s", data.strip())
                self._sock.sendall(data.encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError, OSError) as exc:
                self._connected = False
                raise SonarConnectionError(f"Gönderim hatası: {exc}") from exc

            try:
                response = self._response_queue.get(timeout=timeout)
            except queue.Empty:
                raise SonarTimeoutError(
                    f"'{cmd}' komutuna {timeout:.1f}s içinde yanıt gelmedi."
                )

        if response.get("type") == "disconnect":
            raise SonarConnectionError("Komut beklenirken bağlantı kesildi.")

        return response

    # ─── Event Callback Kaydı ─────────────────────────────────────────────────

    def on_event(self, event_name: str, callback: Callable[[Dict[str, Any]], None]):
        """
        Belirtilen event adı için callback kaydeder.

        event_name : Event adı ("echo", "scan_progress", "temperature", "doppler",
                     "error") veya "*" (tüm event'ler için yakalayıcı)
        callback   : dict alıp None dönen callable; okuma thread'inden çağrılır
        """
        with self._callback_lock:
            self._event_callbacks.setdefault(event_name, []).append(callback)
        self.logger.debug("Event dinleyici eklendi: '%s'", event_name)

    def remove_event_listener(
        self, event_name: str, callback: Callable[[Dict[str, Any]], None]
    ):
        """Daha önce kayıtlı bir event callback'ini kaldırır."""
        with self._callback_lock:
            listeners = self._event_callbacks.get(event_name, [])
            try:
                listeners.remove(callback)
            except ValueError:
                self.logger.warning("Callback bulunamadı: '%s'", event_name)

    # ─── Kısa Yol Metodları ───────────────────────────────────────────────────

    def steer(self, az: float, el: float) -> Dict[str, Any]:
        """Anten yönünü ayarla. az: azimut, el: elevasyon (derece, -30..+30)."""
        return self.send_command("steer", {"az": az, "el": el})

    def set_freq(self, hz: int) -> Dict[str, Any]:
        """PLL çıkış frekansını ayarla (Hz, ör. 40000)."""
        return self.send_command("freq", {"hz": hz})

    def set_gain(self, kanal: int, db: float) -> Dict[str, Any]:
        """Belirli bir preamp kanalının kazancını ayarla (kanal 0–6, dB)."""
        return self.send_command("gain", {"kanal": kanal, "db": db})

    def set_gain_all(self, db: float) -> Dict[str, Any]:
        """Tüm preamp kanallarına aynı kazancı uygula (dB)."""
        return self.send_command("gain_all", {"db": db})

    def burst_start(self) -> Dict[str, Any]:
        """PLL burst gönderimini başlat (60 pals, 30 ms dinleme)."""
        return self.send_command("burst_start")

    def burst_stop(self) -> Dict[str, Any]:
        """PLL burst gönderimini durdur."""
        return self.send_command("burst_stop")

    def scan_start(self, dwell_ms: int = 50) -> Dict[str, Any]:
        """13×13 otomatik taramayı başlat. dwell_ms: her konumda bekleme süresi (ms)."""
        return self.send_command("scan_start", {"dwell_ms": dwell_ms})

    def scan_stop(self) -> Dict[str, Any]:
        """Devam eden otomatik taramayı durdur."""
        return self.send_command("scan_stop")

    def read_temp(self) -> Dict[str, Any]:
        """LM60 sensöründen sıcaklık (°C) ve ses hızı (m/s) oku."""
        return self.send_command("read_temp")

    def read_echo(self) -> Dict[str, Any]:
        """Son yakalanan echo bilgisini oku (mesafe ve gecikme)."""
        return self.send_command("read_echo")

    def read_doppler(self) -> Dict[str, Any]:
        """Doppler frekansı ve hedef hızını hesapla."""
        return self.send_command("read_doppler")

    def status(self) -> Dict[str, Any]:
        """Sistem durumunu oku (frekans, yön, sıcaklık, tarama durumu)."""
        return self.send_command("status")

    def freq_sweep_start(
        self,
        min_hz:   int = 39000,
        max_hz:   int = 41000,
        step_hz:  int = 100,
        dwell_ms: int = 31,
    ) -> Dict[str, Any]:
        """PLL frekans taramasını başlat (min_hz–max_hz, step_hz adım, dwell_ms bekleme)."""
        return self.send_command("freq_sweep_start", {
            "min_hz": min_hz, "max_hz": max_hz,
            "step_hz": step_hz, "dwell_ms": dwell_ms,
        })

    def freq_sweep_stop(self) -> Dict[str, Any]:
        """PLL frekans taramasını durdur."""
        return self.send_command("freq_sweep_stop")

    def scan_range(self, az_max: float, el_max: float, step: float = 5.0) -> Dict[str, Any]:
        """Tarama aralığını ayarla (±az_max, ±el_max derece, step adım)."""
        return self.send_command(
            "scan_range", {"az_max": az_max, "el_max": el_max, "step": step}
        )

    # ─── Context Manager ──────────────────────────────────────────────────────

    def __enter__(self) -> "SonarClient":
        """with SonarClient() as client: kullanımını destekler."""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    def __repr__(self) -> str:
        durum = "bağlı" if self._connected else "bağlı değil"
        return f"SonarClient(path={self.socket_path!r}, {durum})"


# ─── Demo ─────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import logging
    import time

    logging.basicConfig(
        level=logging.DEBUG,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    client = SonarClient()

    if not client.connect():
        print("HATA: C tarafına bağlanılamadı. ./sonar çalışıyor mu?")
        raise SystemExit(1)

    # Event dinleyicileri kaydet
    def on_echo(data: dict):
        dist = data.get("dist", 0)
        print(f"Echo: {dist:.2f}m @ az={data.get('az')} el={data.get('el')}")

    def on_scan_progress(data: dict):
        step  = data.get("step", "?")
        total = data.get("total", "?")
        print(f"Tarama: {step}/{total}  az={data.get('az')} el={data.get('el')}")

    def on_temperature(data: dict):
        c  = data.get("celsius", 0)
        ss = data.get("sound_speed", 0)
        print(f"Sicaklik: {c:.1f} C  ses hizi: {ss:.1f} m/s")

    client.on_event("echo",          on_echo)
    client.on_event("scan_progress", on_scan_progress)
    client.on_event("temperature",   on_temperature)

    try:
        print("\n--- Status ---")
        print(client.status())

        print("\n--- Sicaklik ---")
        print(client.read_temp())

        print("\n--- Steering az=15 el=20 ---")
        print(client.steer(15.0, 20.0))

        print("\n--- Frekans 40000 Hz ---")
        print(client.set_freq(40000))

        print("\n--- Kazanc kanal=0 db=12 ---")
        print(client.set_gain(0, 12.0))

        print("\n--- 5 saniye event bekleniyor ---")
        time.sleep(5)

    except SonarConnectionError as exc:
        print(f"Baglanti hatasi: {exc}")
    except SonarTimeoutError as exc:
        print(f"Zaman asimi: {exc}")
    finally:
        client.disconnect()
