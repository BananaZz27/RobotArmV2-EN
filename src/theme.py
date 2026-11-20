# Window look: colours, fonts and the shared stylesheet (qt_app.py, viewer3d.py)
#   buttons are styled by the "kind" property · labels by the "role" property
#   two themes: "scifi" (default — futuristic HUD: black background, cyan neon lines, cut-corner cards, glowing numbers)
#               "light" (white cards on grey) — pick it with the environment variable ROBOTARM_THEME=light
import os
from datetime import datetime

from PyQt6.QtCore import QPointF, QRectF
from PyQt6.QtGui import QColor, QFont, QLinearGradient, QPainter, QPainterPath, QPalette, QPen, QRadialGradient
from PyQt6.QtWidgets import (QFrame, QGraphicsDropShadowEffect, QHBoxLayout, QLabel, QPushButton, QVBoxLayout,
                             QWidget)

STYLE = os.environ.get("ROBOTARM_THEME", "scifi")
SCIFI = STYLE == "scifi"

FONT_UI = "Leelawadee UI"                   # Windows UI font
FONT_HEAD = ["Bahnschrift", FONT_UI]        # headings/big numbers: squared instrument-panel font
FONT_MONO = ["Cascadia Mono", "Consolas", FONT_UI]

if SCIFI:
    BG, CARD, LINE = "#02060b", "#06121c", "#14465a"
    TEXT, MUTED = "#cdeffd", "#5d8799"
    ACCENT, OK, WARN, BAD = "#00e5ff", "#00ffa3", "#ffb347", "#ff3b5c"
    TARGET, ACTUAL = "#ff8c42", "#00e5ff"   # same colours as the 3D view: orange = Target, cyan = actual
    # 3D view: background (bottom, top), floor grid, base, reach rings
    VIEW = dict(bg="#010409", top="#062033", grid="#0b4f63", base="#16303f", ring="#00e5ff")
else:
    BG, CARD, LINE = "#f3f5f8", "#ffffff", "#e2e6ec"
    TEXT, MUTED = "#1f2933", "#6b7685"
    ACCENT, OK, WARN, BAD = "#2563eb", "#2a9d8f", "#c77d14", "#e63946"
    TARGET, ACTUAL = "#ff8c42", "#3a86ff"
    VIEW = dict(bg="#f4f5f7", top="#dfe3ea", grid="#c9ced6", base="#5b6270", ring=None)


def rgba(color, alpha):
    c = QColor(color)
    return f"rgba({c.red()},{c.green()},{c.blue()},{alpha})"


LIGHT_QSS = f"""
QMainWindow, QDialog {{ background:{BG}; }}
QWidget {{ color:{TEXT}; }}
QToolTip {{ background:#1f2933; color:white; border:none; padding:6px; border-radius:4px; }}
QLabel {{ background:transparent; }}
QLabel[role="title"] {{ font-size:11pt; font-weight:600; }}
QLabel[role="app"] {{ font-size:14pt; font-weight:700; }}
QLabel[role="hint"] {{ color:{MUTED}; font-size:9pt; }}
QLabel[role="name"] {{ font-weight:600; }}
QLabel[role="range"] {{ color:{MUTED}; font-size:8pt; }}
QLabel[role="tile-name"] {{ color:{MUTED}; font-size:9pt; }}
QLabel[role="actual"] {{ color:{ACTUAL}; background:#eaf1ff; border-radius:6px; padding:3px 8px; }}

QPushButton {{ background:{CARD}; border:1px solid #cfd5de; border-radius:7px; padding:6px 14px; }}
QPushButton:hover {{ background:#f0f4fa; border-color:#9fb0c7; }}
QPushButton:pressed {{ background:#e1e8f2; }}
QPushButton[kind="primary"] {{ background:{ACCENT}; color:white; border:1px solid {ACCENT}; font-weight:600; }}
QPushButton[kind="primary"]:hover {{ background:#1d4fd7; }}
QPushButton[kind="danger"] {{ background:{BAD}; color:white; border:1px solid #c92f3c; font-weight:700;
                             font-size:11pt; padding:8px 22px; }}
QPushButton[kind="danger"]:hover {{ background:#d42f3d; }}
QPushButton[kind="ghost"] {{ background:transparent; border:1px dashed #b8c1cd; color:{MUTED}; }}
QPushButton[kind="jog"] {{ font-size:13pt; font-weight:700; min-width:64px; min-height:34px; padding:2px; }}
QPushButton[kind="jog"]:pressed {{ background:{ACCENT}; color:white; border-color:{ACCENT}; }}

QTabWidget::pane {{ border:none; background:transparent; top:-1px; }}
QTabBar::tab {{ background:transparent; color:{MUTED}; padding:8px 16px; margin-right:2px;
               border-bottom:3px solid transparent; font-weight:600; }}
QTabBar::tab:selected {{ color:{ACCENT}; border-bottom:3px solid {ACCENT}; }}
QTabBar::tab:hover:!selected {{ color:{TEXT}; }}
QDoubleSpinBox {{ min-height:26px; }}
QSlider::groove:horizontal {{ height:6px; background:#e4e8ee; border-radius:3px; }}
QSlider::sub-page:horizontal {{ background:{TARGET}; border-radius:3px; }}
QSlider::handle:horizontal {{ background:white; border:2px solid {TARGET}; width:14px; height:14px;
                             margin:-6px 0; border-radius:9px; }}
QTableWidget {{ background:{CARD}; border:1px solid {LINE}; border-radius:6px; gridline-color:#eef1f5;
               alternate-background-color:#f8fafc; selection-background-color:#dbe7ff; selection-color:{TEXT}; }}
QHeaderView::section {{ background:#f1f4f8; color:{MUTED}; border:none; border-bottom:1px solid {LINE};
                        padding:5px; font-weight:600; }}
QTextEdit {{ background:{CARD}; border:1px solid {LINE}; border-radius:6px; padding:4px; }}
QPlainTextEdit#log {{ background:#1f2430; color:#d8dee9; border:none; border-radius:8px; padding:6px; }}
QSplitter::handle {{ background:transparent; }}
QSplitter::handle:hover {{ background:#d5dce6; }}
QScrollBar:vertical {{ background:transparent; width:10px; margin:2px; }}
QScrollBar::handle:vertical {{ background:#c5ccd6; border-radius:4px; min-height:24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height:0; }}
"""

# futuristic theme: square corners (radius 2), faint neon borders, cyan text on black
SCIFI_QSS = f"""
QMainWindow, QDialog {{ background:{BG}; }}
QWidget {{ color:{TEXT}; }}
QToolTip {{ background:#031018; color:{ACCENT}; border:1px solid {ACCENT}; padding:6px; }}
QLabel {{ background:transparent; }}
QLabel[role="title"] {{ color:{ACCENT}; }}
QLabel[role="app"] {{ color:{ACCENT}; }}
QLabel[role="hint"] {{ color:{MUTED}; font-size:9pt; }}
QLabel[role="name"] {{ color:{TEXT}; font-weight:600; }}
QLabel[role="range"] {{ color:{MUTED}; font-size:8pt; }}
QLabel[role="tile-name"] {{ color:{MUTED}; }}
QLabel[role="tile-value"] {{ color:{ACCENT}; }}
QLabel[role="actual"] {{ color:{ACTUAL}; background:{rgba(ACTUAL, 18)}; border:1px solid {rgba(ACTUAL, 90)};
                        border-radius:2px; padding:3px 8px; }}
QLabel[role="clock"] {{ color:{ACCENT}; }}

QPushButton {{ background:{rgba(ACCENT, 10)}; color:{ACCENT}; border:1px solid {rgba(ACCENT, 110)};
              border-radius:2px; padding:6px 14px; }}
QPushButton:hover {{ background:{rgba(ACCENT, 40)}; border-color:{ACCENT}; color:white; }}
QPushButton:pressed {{ background:{rgba(ACCENT, 90)}; }}
QPushButton[kind="primary"] {{ background:{rgba(ACCENT, 200)}; color:#00141b; border:1px solid {ACCENT}; font-weight:700; }}
QPushButton[kind="primary"]:hover {{ background:{ACCENT}; color:#00141b; }}
QPushButton[kind="danger"] {{ background:{rgba(BAD, 45)}; color:#ff8fa3; border:1px solid {BAD}; font-weight:700;
                             font-size:11pt; padding:8px 22px; }}
QPushButton[kind="danger"]:hover {{ background:{rgba(BAD, 110)}; color:white; }}
QPushButton[kind="danger"]:pressed {{ background:{BAD}; color:white; }}
QPushButton[kind="ghost"] {{ background:transparent; border:1px dashed {rgba(ACCENT, 80)}; color:{MUTED}; }}
QPushButton[kind="ghost"]:hover {{ color:{ACCENT}; }}
QPushButton[kind="jog"] {{ font-size:13pt; font-weight:700; min-width:64px; min-height:34px; padding:2px; }}
QPushButton[kind="jog"]:pressed {{ background:{ACCENT}; color:#00141b; }}

QTabWidget::pane {{ border:none; background:transparent; top:-1px; }}
QTabBar::tab {{ background:transparent; color:{MUTED}; padding:8px 16px; margin-right:2px;
               border-bottom:2px solid {rgba(ACCENT, 30)}; font-weight:600; }}
QTabBar::tab:selected {{ color:{ACCENT}; border-bottom:2px solid {ACCENT};
                        background:qlineargradient(x1:0,y1:0,x2:0,y2:1, stop:0 transparent, stop:1 {rgba(ACCENT, 45)}); }}
QTabBar::tab:hover:!selected {{ color:{TEXT}; }}

QDoubleSpinBox {{ min-height:26px; }}
QSlider::groove:horizontal {{ height:2px; background:{rgba(ACCENT, 60)}; }}
QSlider::sub-page:horizontal {{ background:{TARGET}; height:2px; }}
QSlider::handle:horizontal {{ background:{TARGET}; border:1px solid #ffd1b0; width:8px; height:18px; margin:-8px 0; }}
QSlider::handle:horizontal:hover {{ background:#ffb27d; }}

QTableWidget {{ background:{rgba('#000000', 90)}; border:1px solid {LINE}; gridline-color:{rgba(ACCENT, 25)};
               alternate-background-color:{rgba(ACCENT, 10)}; selection-background-color:{rgba(ACCENT, 60)};
               selection-color:white; }}
QHeaderView::section {{ background:{rgba(ACCENT, 22)}; color:{ACCENT}; border:none;
                        border-bottom:1px solid {rgba(ACCENT, 120)}; padding:5px; font-weight:600; }}
QTextEdit {{ background:{rgba('#000000', 110)}; border:1px solid {LINE}; padding:4px; }}
QPlainTextEdit#log {{ background:#000306; color:#6dffc8; border:1px solid {rgba(OK, 50)}; padding:6px;
                      selection-background-color:{rgba(OK, 60)}; }}
QSplitter::handle {{ background:transparent; }}
QSplitter::handle:hover {{ background:{rgba(ACCENT, 60)}; }}
QCheckBox {{ spacing:8px; }}
QScrollBar:vertical {{ background:transparent; width:8px; margin:2px; }}
QScrollBar::handle:vertical {{ background:{rgba(ACCENT, 90)}; min-height:24px; }}
QScrollBar::add-line, QScrollBar::sub-line {{ height:0; }}
"""
QSS = SCIFI_QSS if SCIFI else LIGHT_QSS


def apply(app):
    """apply the theme to the whole program (call once after creating QApplication)"""
    app.setStyle("Fusion")                      # same look on every machine + full stylesheet support
    app.setFont(QFont(FONT_UI, 10))
    pal = app.palette()                         # spin boxes/checkboxes are drawn from the palette (no QSS → the arrows stay)
    for role, color in ((QPalette.ColorRole.Window, BG), (QPalette.ColorRole.WindowText, TEXT),
                        (QPalette.ColorRole.Base, "#020a10" if SCIFI else CARD), (QPalette.ColorRole.Text, TEXT),
                        (QPalette.ColorRole.Button, CARD), (QPalette.ColorRole.ButtonText, TEXT),
                        (QPalette.ColorRole.Highlight, ACCENT),
                        (QPalette.ColorRole.HighlightedText, "#00141b" if SCIFI else "#ffffff")):
        pal.setColor(role, QColor(color))
    app.setPalette(pal)
    app.setStyleSheet(QSS)


def font(families, size, bold=False, caps=False, spacing=0.0):
    f = QFont()
    f.setFamilies(families)
    f.setPointSize(size)
    f.setBold(bold)
    if caps:
        f.setCapitalization(QFont.Capitalization.AllUppercase)
    if spacing:
        f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, spacing)
    return f


def mono(size=10, bold=False):
    return font(FONT_MONO, size, bold)


def glow(widget, color=None, radius=18):
    """glow around the text (sci-fi theme only)"""
    if SCIFI:
        fx = QGraphicsDropShadowEffect(widget)
        fx.setBlurRadius(radius)
        fx.setColor(QColor(color or ACCENT))
        fx.setOffset(0, 0)
        widget.setGraphicsEffect(fx)
    return widget


# font per label role (QSS only sets colours — letter spacing/caps must come from QFont)
_ROLE_FONT = {
    "title": lambda: font(FONT_HEAD, 10, True, caps=SCIFI, spacing=1.6 if SCIFI else 0),
    "app": lambda: font(FONT_HEAD, 15, True, caps=SCIFI, spacing=3.0 if SCIFI else 0),
    "tile-name": lambda: font(FONT_HEAD, 9, caps=SCIFI, spacing=1.2 if SCIFI else 0),
    "tile-value": lambda: font(FONT_HEAD if SCIFI else FONT_MONO, 17 if SCIFI else 15, True),
    "clock": lambda: font(FONT_MONO, 10, True, spacing=1.0),
}


def label(text="", role=None, **kw):
    if SCIFI and role == "title":
        text = "// " + text
    lbl = QLabel(text, **kw)
    if role:
        lbl.setProperty("role", role)
        if role in _ROLE_FONT:
            f = _ROLE_FONT[role]()
            if any("฀" <= ch <= "๿" for ch in text):   # Thai text: no letter spacing (it splits vowels/tone marks from consonants)
                f.setLetterSpacing(QFont.SpacingType.AbsoluteSpacing, 0)
            lbl.setFont(f)
    return lbl


def button(text, kind=None, tip=""):
    b = QPushButton(text)
    if kind:
        b.setProperty("kind", kind)
    if tip:
        b.setToolTip(tip)
    if kind == "danger":
        glow(b, BAD, 22)
    return b


class Card(QFrame):
    """card frame — sci-fi: cut top-right/bottom-left corners + faint neon border + bright corner brackets · light: rounded white card"""
    CUT = 12

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        r = QRectF(self.rect()).adjusted(0.5, 0.5, -0.5, -0.5)
        if not SCIFI:
            path = QPainterPath()
            path.addRoundedRect(r, 10, 10)
            p.fillPath(path, QColor(CARD))
            p.setPen(QPen(QColor(LINE), 1))
            p.drawPath(path)
            return
        c = self.CUT
        path = QPainterPath(QPointF(r.left(), r.top()))
        for x, y in ((r.right() - c, r.top()), (r.right(), r.top() + c), (r.right(), r.bottom()),
                     (r.left() + c, r.bottom()), (r.left(), r.bottom() - c)):
            path.lineTo(x, y)
        path.closeSubpath()
        fill = QLinearGradient(0, r.top(), 0, r.bottom())
        fill.setColorAt(0, QColor(8, 30, 44, 235))
        fill.setColorAt(1, QColor(3, 12, 20, 235))
        p.fillPath(path, fill)
        p.setPen(QPen(QColor(LINE), 1))
        p.drawPath(path)
        p.setPen(QPen(QColor(ACCENT), 2))                       # corner brackets top-left + bottom-right
        k = 16
        p.drawLine(QPointF(r.left(), r.top() + k), QPointF(r.left(), r.top()))
        p.drawLine(QPointF(r.left(), r.top()), QPointF(r.left() + k, r.top()))
        p.drawLine(QPointF(r.right(), r.bottom() - k), QPointF(r.right(), r.bottom()))
        p.drawLine(QPointF(r.right(), r.bottom()), QPointF(r.right() - k, r.bottom()))
        p.setPen(QPen(QColor(ACCENT), 1))                       # small tick on the cut corner
        p.drawLine(QPointF(r.right() - c + 3, r.top() + 3), QPointF(r.right() - 3, r.top() + c - 3))


def card(title=None, hint=None):
    """card → (frame, inner layout) · title/hint = optional card header"""
    frame = Card()
    lay = QVBoxLayout(frame)
    lay.setContentsMargins(14, 12, 14, 12)
    lay.setSpacing(8)
    if title:
        head = QHBoxLayout()
        head.addWidget(label(title, "title"))
        head.addStretch(1)
        if hint:
            head.addWidget(label(hint, "hint"))
        lay.addLayout(head)
    return frame, lay


class Backdrop(QWidget):
    """window background — sci-fi: faint grid + cyan glow in the middle (HUD style)"""

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(BG))
        if not SCIFI:
            return
        w, h = self.width(), self.height()
        g = QRadialGradient(QPointF(w * 0.5, h * 0.35), max(w, h) * 0.7)
        g.setColorAt(0, QColor(0, 229, 255, 22))
        g.setColorAt(1, QColor(0, 0, 0, 0))
        p.fillRect(self.rect(), g)
        p.setPen(QPen(QColor(0, 229, 255, 14), 1))
        for x in range(0, w, 32):
            p.drawLine(x, 0, x, h)
        for y in range(0, h, 32):
            p.drawLine(0, y, w, y)


def pill(color):
    """stylesheet for a status label (text colour on a faint tint of it)"""
    c = QColor(color).name()
    if SCIFI:
        return (f"color:{c}; background:{rgba(c, 30)}; border:1px solid {rgba(c, 140)}; border-radius:2px; "
                "padding:3px 12px; font-weight:600")
    return f"color:{c}; background:{rgba(c, 30)}; border-radius:11px; padding:3px 12px; font-weight:600"


def clock_text():
    return f"SYS {datetime.now():%H:%M:%S}" if SCIFI else ""
