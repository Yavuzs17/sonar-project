import os
import sys

# ── Raspberry Pi / Wayland ortam değişkenleri ──────────────────────────────
# Wayland oturumunda Qt'nin X11 (xcb) backend'ini kullanmasını zorla.
# GLViewWidget, Wayland/EGL yerine GLX bağlamıyla düzgün çalışır.
os.environ.setdefault("XDG_RUNTIME_DIR", "/run/user/1000")
os.environ.setdefault("DISPLAY", ":0")
# Wayland varsa bile OpenGL için X11 tercih edilsin
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")
# PyOpenGL'in GLX (masaüstü OpenGL) kullanmasını sağla
os.environ.setdefault("PYOPENGL_PLATFORM", "glx")

from PyQt6.QtWidgets import QApplication

from ui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
