"""
UI ekran görüntüsüne numaralı callout annotasyonu ekler.

Kullanım:
    python annotate_ui.py screenshot.png

Çıktı: screenshot_annotated.png

Gereksinim: pip install Pillow
"""

import sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
import math

# ─── Annotasyon tanımları ────────────────────────────────────────────────────
#
# Her giriş:  (numara, açıklama, ok_baslangic_xy, ok_bitis_xy)
# ok_baslangic_xy : dairenin yerleşeceği konum (görüntü piksel koordinatı)
# ok_bitis_xy     : okun gösterdiği nokta (None ise ok yok, sadece daire)
#
# Koordinatları kendi ekran görüntünüze göre düzenleyin.
# Referans çözünürlük: 1456 × 816 piksel (tam ekran)
# Ölçekleme için SCALE_X / SCALE_Y kullanılır.
# ─────────────────────────────────────────────────────────────────────────────

ANNOTATIONS = [
    # num  label_text (tabloda kullanılır)         callout_xy     arrow_tip_xy
    (1,  "Bağlantı durumu ve telemetri",           (130,  24),    (85,   24)),
    (2,  "Tarama aralığı ayarları",                (100, 110),    (100,  95)),
    (3,  "Engel tespiti (DBSCAN) parametreleri",   (100, 225),    (100, 220)),
    (4,  "Harita temizle butonu",                  (100, 325),    (100, 330)),
    (5,  "2D / 3D harita sekme seçici",            (310,  52),    (260,  52)),
    (6,  "Azimuth polar haritası",                 (490, 400),    (490, 380)),
    (7,  "Elevation polar haritası",               (1010, 400),   (1010, 380)),
    (8,  "Tespit edilen engeller (cluster) listesi",(1380, 120),  (1360, 100)),
    (9,  "Aktif tarama hüzmesi",                   (720, 230),    (700, 270)),
    (10, "DBSCAN cluster merkezi (turuncu elmas)",  (570, 390),    (555, 400)),
    (11, "Kontrol sekmeleri (Steering/Frekans/...)", (730, 707),   (730, 710)),
    (12, "Tarama ilerleme göstergesi",             (730, 790),    (730, 783)),
]

# ─── Stil sabitleri ───────────────────────────────────────────────────────────

CIRCLE_RADIUS  = 14        # callout dairesinin yarıçapı (piksel)
CIRCLE_FILL    = (0, 100, 220, 220)    # RGBA — mavi
CIRCLE_OUTLINE = (255, 255, 255, 255)  # beyaz kenar
TEXT_COLOR     = (255, 255, 255, 255)
ARROW_COLOR    = (255, 220, 0, 230)    # sarı ok
ARROW_WIDTH    = 2
OUTLINE_WIDTH  = 2


def draw_arrow(draw: ImageDraw.ImageDraw,
               start: tuple, end: tuple,
               color, width: int, head_size: int = 10):
    """Oklu çizgi çizer (basit ok ucu)."""
    draw.line([start, end], fill=color, width=width)

    # ok ucu hesapla
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length = math.hypot(dx, dy)
    if length < 1:
        return
    ux, uy = dx / length, dy / length
    px, py = -uy, ux  # dikey

    tip = end
    b1 = (tip[0] - head_size * ux + head_size * 0.4 * px,
          tip[1] - head_size * uy + head_size * 0.4 * py)
    b2 = (tip[0] - head_size * ux - head_size * 0.4 * px,
          tip[1] - head_size * uy - head_size * 0.4 * py)
    draw.polygon([tip, b1, b2], fill=color)


def annotate(input_path: str):
    img = Image.open(input_path).convert("RGBA")
    W, H = img.size

    # Üzerine çizilecek katman (şeffaf)
    overlay = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    draw    = ImageDraw.Draw(overlay)

    # Font — sistem fontu yoksa PIL varsayılanına düş
    try:
        font = ImageFont.truetype("arial.ttf", CIRCLE_RADIUS + 4)
    except OSError:
        try:
            font = ImageFont.truetype(
                "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
                CIRCLE_RADIUS + 4)
        except OSError:
            font = ImageFont.load_default()

    for num, _, callout_xy, arrow_tip_xy in ANNOTATIONS:
        cx, cy = callout_xy

        # Ok çiz (callout → hedef)
        if arrow_tip_xy:
            draw_arrow(draw, callout_xy, arrow_tip_xy,
                       color=ARROW_COLOR, width=ARROW_WIDTH)

        # Daire (outline → fill)
        r = CIRCLE_RADIUS
        bbox = [cx - r, cy - r, cx + r, cy + r]
        draw.ellipse(bbox, fill=CIRCLE_FILL, outline=CIRCLE_OUTLINE,
                     width=OUTLINE_WIDTH)

        # Numara
        text = str(num)
        try:
            tw = draw.textlength(text, font=font)
        except AttributeError:
            tw = draw.textsize(text, font=font)[0]
        th = CIRCLE_RADIUS * 1.4
        draw.text((cx - tw / 2, cy - th / 2), text,
                  fill=TEXT_COLOR, font=font)

    # Birleştir
    combined = Image.alpha_composite(img, overlay).convert("RGB")
    out = Path(input_path).with_stem(Path(input_path).stem + "_annotated")
    combined.save(str(out), "PNG")
    print(f"Kaydedildi: {out}")
    return str(out)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Kullanim: python annotate_ui.py <screenshot.png>")
        sys.exit(1)
    annotate(sys.argv[1])
