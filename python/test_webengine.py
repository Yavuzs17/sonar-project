import os
os.environ["QTWEBENGINE_DISABLE_SANDBOX"] = "1"

import sys
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtWebEngineWidgets import QWebEngineView

app = QApplication(sys.argv)
win = QMainWindow()
web = QWebEngineView()
web.setHtml("<html><body style='background:#0d1b2a;color:white;font-size:48px;text-align:center;padding-top:200px'>Test çalışıyor ✓</body></html>")
win.setCentralWidget(web)
win.resize(800, 480)
win.show()
sys.exit(app.exec())
