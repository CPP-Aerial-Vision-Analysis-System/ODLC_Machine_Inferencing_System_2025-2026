"""
sim_demo/panel_widgets.py -- PURPOSE: the building blocks of the demo window, drawn with PyQt5:
    VideoView    the big live camera picture with a heads-up display (height, speed, heading,
                 mode), a crosshair and a white "PHOTO" flash whenever a photo is taken
    PhotoView    the newest photo with the detector's boxes and labels drawn on it
    PhaseStrip   "Waiting > Taking off > Searching > ... > Landed" with the current phase lit
    StatusPill   small coloured status labels (MAVROS, camera, photos, detector, Mission Planner)
    EventLog     the scrolling, colour-coded list of what is happening
plus the colour theme (THEME, STYLE_SHEET) and ros_image_to_qimage(), which turns the raw
bytes of a sensor_msgs/Image into a Qt image WITHOUT OpenCV (the pip OpenCV build changes Qt
plugin paths and breaks PyQt5, so the panel never imports cv2).
Nothing here talks to ROS; demo_panel.py feeds these widgets.
"""
from __future__ import annotations

import html
import time

from PyQt5 import QtCore, QtGui, QtWidgets

THEME = {
    'bg': '#0d1117', 'panel': '#151b23', 'panel2': '#1b2330', 'border': '#2a3441',
    'text': '#e6edf3', 'muted': '#8b98a5', 'accent': '#2f81f7', 'cyan': '#39c5ff',
    'green': '#3fb950', 'amber': '#f0b429', 'red': '#f85149', 'orange': '#ff9f1a',
    'person': '#ff4d4d', 'tent': '#ffa31a', 'done': '#1f3b2c',
}
CLASS_COLORS = {'person': THEME['person'], 'tent': THEME['tent']}
CLASS_LABELS = {'person': 'PERSON', 'tent': 'TENT'}

STYLE_SHEET = f"""
QWidget {{ background: {THEME['bg']}; color: {THEME['text']}; font-family: 'DejaVu Sans';
          font-size: 13px; }}
QFrame#card {{ background: {THEME['panel']}; border: 1px solid {THEME['border']};
              border-radius: 8px; }}
QLabel#cardTitle {{ color: {THEME['muted']}; font-size: 12px; font-weight: bold;
                   letter-spacing: 1px; background: transparent; }}
QLabel#title {{ font-size: 20px; font-weight: bold; background: transparent; }}
QLabel#subtitle {{ color: {THEME['muted']}; font-size: 13px; background: transparent; }}
QLabel#teleName {{ color: {THEME['muted']}; font-size: 12px; background: transparent; }}
QLabel#teleValue {{ font-size: 19px; font-weight: bold; background: transparent; }}
QLabel#bestLine {{ font-size: 14px; background: transparent; }}
QLabel#cmdStatus {{ color: {THEME['muted']}; font-size: 12px; background: transparent; }}
QPushButton {{ background: {THEME['panel2']}; border: 1px solid {THEME['border']};
              border-radius: 8px; padding: 8px 12px; font-size: 14px; font-weight: bold; }}
QPushButton:hover {{ border-color: {THEME['cyan']}; }}
QPushButton:pressed {{ background: {THEME['border']}; }}
QPushButton:disabled {{ color: {THEME['muted']}; border-color: {THEME['panel2']}; }}
QPushButton#photoBtn {{ background: #1f4fa8; border-color: #2f81f7; font-size: 17px; }}
QPushButton#startBtn {{ background: #1d5b30; border-color: {THEME['green']}; }}
QPushButton#rtlBtn {{ background: #6b1f1c; border-color: {THEME['red']}; }}
QPushButton#modeBtn {{ padding: 3px 10px; font-size: 12px; border-radius: 6px; }}
QPushButton#modeBtn:checked {{ background: {THEME['accent']}; border-color: {THEME['accent']}; }}
QPushButton#mpBtn {{ padding: 4px 6px; font-size: 12px; }}
QLabel#mpHint {{ color: {THEME['muted']}; font-size: 12px; background: transparent; }}
QTableWidget {{ background: {THEME['panel']}; border: none; gridline-color: {THEME['border']};
               selection-background-color: #24456e; font-size: 13px; }}
QHeaderView::section {{ background: {THEME['panel2']}; color: {THEME['muted']}; border: none;
                       padding: 4px; font-weight: bold; }}
QPlainTextEdit {{ background: {THEME['panel']}; border: none; font-family: 'DejaVu Sans Mono';
                 font-size: 12px; }}
QSplitter::handle {{ background: {THEME['bg']}; }}
QScrollBar:vertical {{ background: {THEME['panel']}; width: 10px; }}
QScrollBar::handle:vertical {{ background: {THEME['border']}; border-radius: 4px; }}
"""


def ros_image_to_qimage(encoding: str, width: int, height: int, step: int,
                        data: bytes) -> QtGui.QImage | None:
    """Convert raw sensor_msgs/Image fields to a QImage that owns its pixels.

    Supports rgb8, bgr8, rgba8, bgra8, mono8 (what Gazebo cameras and the bridge produce).
    Returns None for unsupported encodings or inconsistent sizes.
    """
    enc = (encoding or '').lower()
    fmt_bpp = {'rgb8': (QtGui.QImage.Format_RGB888, 3), 'bgr8': (QtGui.QImage.Format_RGB888, 3),
               'rgba8': (QtGui.QImage.Format_RGBA8888, 4), 'bgra8': (QtGui.QImage.Format_ARGB32, 4),
               'mono8': (QtGui.QImage.Format_Grayscale8, 1), '8uc1': (QtGui.QImage.Format_Grayscale8, 1),
               '8uc3': (QtGui.QImage.Format_RGB888, 3)}
    if enc not in fmt_bpp or width <= 0 or height <= 0:
        return None
    fmt, bpp = fmt_bpp[enc]
    step = step or width * bpp
    if len(data) < step * height or step < width * bpp:
        return None
    img = QtGui.QImage(bytes(data), width, height, step, fmt).copy()   # copy: own the pixels
    if enc in ('bgr8', '8uc3'):
        img = img.rgbSwapped()
    return img


def card(title: str, widget: QtWidgets.QWidget, right: QtWidgets.QWidget | None = None,
         margins: int = 8) -> QtWidgets.QFrame:
    """Wrap a widget in a rounded dark card with a small upper-case title."""
    frame = QtWidgets.QFrame()
    frame.setObjectName('card')
    lay = QtWidgets.QVBoxLayout(frame)
    lay.setContentsMargins(margins, 6, margins, margins)
    lay.setSpacing(4)
    head = QtWidgets.QHBoxLayout()
    lab = QtWidgets.QLabel(title.upper())
    lab.setObjectName('cardTitle')
    head.addWidget(lab)
    head.addStretch(1)
    if right is not None:
        head.addWidget(right)
    lay.addLayout(head)
    lay.addWidget(widget, 1)
    return frame


def _font(px: int, bold: bool = False, family: str = 'DejaVu Sans') -> QtGui.QFont:
    f = QtGui.QFont(family)
    f.setPixelSize(px)
    f.setBold(bold)
    return f


def _draw_tag(p: QtGui.QPainter, x: float, y: float, text: str, fg: QtGui.QColor,
              bg: QtGui.QColor, px: int = 14, bold: bool = True) -> QtCore.QRectF:
    """Draw text on a rounded background at (x, y) = top-left. Returns the rectangle used."""
    p.setFont(_font(px, bold))
    fm = QtGui.QFontMetrics(p.font())
    w, h = fm.horizontalAdvance(text) + 12, fm.height() + 4
    r = QtCore.QRectF(x, y, w, h)
    p.setPen(QtCore.Qt.NoPen)
    p.setBrush(bg)
    p.drawRoundedRect(r, 4, 4)
    p.setPen(fg)
    p.drawText(r, QtCore.Qt.AlignCenter, text)
    return r


def _fit(src_w: int, src_h: int, dst: QtCore.QRect) -> QtCore.QRectF:
    """Largest rectangle with the source aspect ratio centred in dst."""
    if src_w <= 0 or src_h <= 0:
        return QtCore.QRectF(dst)
    s = min(dst.width() / src_w, dst.height() / src_h)
    w, h = src_w * s, src_h * s
    return QtCore.QRectF(dst.x() + (dst.width() - w) / 2, dst.y() + (dst.height() - h) / 2, w, h)


class VideoView(QtWidgets.QWidget):
    """The live camera picture (keeps its aspect ratio) with a heads-up display."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(480, 270)
        self._img: QtGui.QImage | None = None
        self._fps = 0.0
        self._hud: dict = {}
        self._flash_t = 0.0
        self._flash_text = ''
        self._placeholder = 'Waiting for the camera picture ...'
        self._last_frame_t = 0.0

    def set_placeholder(self, text: str) -> None:
        """Text shown while there is no picture."""
        self._placeholder = text
        self.update()

    def set_frame(self, img: QtGui.QImage, fps: float) -> None:
        """Show a new camera frame."""
        self._img, self._fps, self._last_frame_t = img, fps, time.time()
        self.update()

    def set_hud(self, hud: dict) -> None:
        """hud keys: alt_m, speed, heading, mode, armed, cam_pitch, cam_zoom (any may be missing)."""
        self._hud = dict(hud)

    def flash(self, text: str = 'PHOTO') -> None:
        """White border flash: 'a photo was just taken'."""
        self._flash_t, self._flash_text = time.time(), text
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt naming
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#000000'))
        stale = self._img is None or time.time() - self._last_frame_t > 3.0
        if self._img is not None:
            r = _fit(self._img.width(), self._img.height(), self.rect())
            p.drawImage(r, self._img)
        else:
            r = QtCore.QRectF(self.rect())
        if self._img is None:
            p.setPen(QtGui.QColor(THEME['muted']))
            p.setFont(_font(18, True))
            p.drawText(self.rect(), QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap,
                       self._placeholder)
        # Crosshair (only over a picture: it would cross out the waiting text).
        if self._img is not None:
            cx, cy = r.center().x(), r.center().y()
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, 150), 1.5))
            for dx, dy in ((-30, 0), (30, 0), (0, -30), (0, 30)):
                p.drawLine(QtCore.QPointF(cx + dx * 0.33, cy + dy * 0.33),
                           QtCore.QPointF(cx + dx, cy + dy))
        # Top badges.
        live_col = QtGui.QColor(THEME['red'] if not stale else THEME['muted'])
        _draw_tag(p, r.x() + 10, r.y() + 10, 'LIVE CAMERA POV' if not stale else 'NO SIGNAL',
                  QtGui.QColor('white'), live_col, 14)
        if self._img is not None:
            info = f'{self._img.width()}x{self._img.height()}  {self._fps:4.1f} fps'
            tag_w = QtGui.QFontMetrics(_font(12)).horizontalAdvance(info) + 12
            _draw_tag(p, r.right() - tag_w - 10, r.y() + 10, info, QtGui.QColor(THEME['text']),
                      QtGui.QColor(0, 0, 0, 150), 12, False)
        # HUD bar.
        h = self._hud
        parts = []
        if h.get('alt_m') is not None:
            parts.append(f"ALT {h['alt_m']:.1f} m ({h['alt_m'] * 3.28084:.0f} ft)")
        if h.get('speed') is not None:
            parts.append(f"SPD {h['speed']:4.1f} m/s")
        if h.get('heading') is not None:
            parts.append(f"HDG {h['heading']:03.0f}°")
        if h.get('mode'):
            parts.append(h['mode'])
        if 'armed' in h:
            parts.append('ARMED' if h['armed'] else 'DISARMED')
        if h.get('cam_pitch') is not None:
            cam = f"CAM {h['cam_pitch']:+.0f}°"
            if h.get('cam_zoom') is not None:
                cam += f" {h['cam_zoom']:.1f}x"
            parts.append(cam)
        if parts:
            text = '   |   '.join(parts)
            p.setFont(_font(15, True))
            fm = QtGui.QFontMetrics(p.font())
            bar = QtCore.QRectF(r.x(), r.bottom() - fm.height() - 14, r.width(), fm.height() + 14)
            p.fillRect(bar, QtGui.QColor(0, 0, 0, 150))
            p.setPen(QtGui.QColor('#d6f5ff'))
            p.drawText(bar, QtCore.Qt.AlignCenter, text)
        # Photo flash.
        age = time.time() - self._flash_t
        if age < 0.9:
            a = int(255 * (1.0 - age / 0.9))
            p.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255, a), 10))
            p.setBrush(QtCore.Qt.NoBrush)
            p.drawRect(r.adjusted(5, 5, -5, -5))
            _draw_tag(p, r.center().x() - 70, r.y() + 50, f'● {self._flash_text}',
                      QtGui.QColor(0, 0, 0, a), QtGui.QColor(255, 255, 255, a), 18)
            QtCore.QTimer.singleShot(40, self.update)
        p.end()


class PhotoView(QtWidgets.QWidget):
    """A photo with detection boxes, plus a caption bar."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 180)
        self._img: QtGui.QImage | None = None
        self._orig = (0, 0)
        self._boxes: list = []
        self._caption = ''
        self._status = ''
        self._status_color = THEME['muted']
        self._highlight = False
        self._extra_lines: list[str] = []
        self._placeholder = 'No photos yet.\nPhotos appear here when the drone takes them.'

    def set_placeholder(self, text: str) -> None:
        """Text shown while there is no photo."""
        self._placeholder = text
        self.update()

    def set_photo(self, img: QtGui.QImage | None, orig_size: tuple[int, int], caption: str,
                  boxes: list, status: str, status_color: str, highlight: bool = False,
                  extra_lines: list[str] | None = None) -> None:
        """Show a photo.

        Args:
            img: the (possibly downscaled) photo. orig_size: full-resolution (w, h) in which
            the boxes are given. caption: file name line. boxes: detections.Box list.
            status: right-hand status text. highlight: draw a 'target found' frame.
            extra_lines: text listed on the photo when the detector gave no boxes.
        """
        self._img, self._orig, self._caption = img, orig_size, caption
        self._boxes, self._status, self._status_color = list(boxes), status, status_color
        self._highlight = highlight
        self._extra_lines = list(extra_lines or [])
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt naming
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.SmoothPixmapTransform)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor('#05080c'))
        cap_h = 28
        area = self.rect().adjusted(0, 0, 0, -cap_h)
        if self._img is None:
            p.setPen(QtGui.QColor(THEME['muted']))
            p.setFont(_font(15))
            p.drawText(area, QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap, self._placeholder)
        else:
            r = _fit(self._img.width(), self._img.height(), area)
            p.drawImage(r, self._img)
            ow, oh = self._orig if self._orig[0] > 0 else (self._img.width(), self._img.height())
            sx, sy = r.width() / max(1, ow), r.height() / max(1, oh)
            for b in self._boxes:
                col = QtGui.QColor(CLASS_COLORS.get(b.cls, THEME['cyan']))
                x1, y1, x2, y2 = b.xyxy
                br = QtCore.QRectF(r.x() + x1 * sx, r.y() + y1 * sy, max(6.0, (x2 - x1) * sx),
                                   max(6.0, (y2 - y1) * sy))
                # Small objects get a bigger ring so they stay visible after downscaling.
                if br.width() < 18 or br.height() < 18:
                    c = br.center()
                    p.setPen(QtGui.QPen(col, 2, QtCore.Qt.DashLine))
                    p.setBrush(QtCore.Qt.NoBrush)
                    p.drawEllipse(c, 22, 22)
                p.setPen(QtGui.QPen(col, 3))
                p.setBrush(QtCore.Qt.NoBrush)
                p.drawRect(br)
                label = f'{CLASS_LABELS.get(b.cls, b.cls.upper())} {b.score * 100:.0f}%'
                ty = br.y() - 24 if br.y() - 24 > r.y() else br.bottom() + 2
                _draw_tag(p, br.x(), ty, label, QtGui.QColor('black'), col, 14)
            if self._extra_lines:
                y = r.y() + 8
                for line in self._extra_lines[:8]:
                    rect = _draw_tag(p, r.x() + 8, y, line, QtGui.QColor(THEME['text']),
                                     QtGui.QColor(0, 0, 0, 170), 13, False)
                    y = rect.bottom() + 3
            if self._highlight:
                p.setPen(QtGui.QPen(QtGui.QColor(THEME['green']), 5))
                p.setBrush(QtCore.Qt.NoBrush)
                p.drawRect(r.adjusted(2, 2, -2, -2))
                _draw_tag(p, r.right() - 150, r.y() + 8, 'TARGET FOUND', QtGui.QColor('black'),
                          QtGui.QColor(THEME['green']), 14)
        # Caption bar (only under a photo).
        if self._img is None:
            p.end()
            return
        bar = QtCore.QRectF(0, self.height() - cap_h, self.width(), cap_h)
        p.fillRect(bar, QtGui.QColor(THEME['panel2']))
        # Status on the right first; the caption gets the rest of the width (shortened in the
        # middle with "..." when it does not fit, so the two never overlap).
        p.setFont(_font(12, True))
        status_w = QtGui.QFontMetrics(p.font()).horizontalAdvance(self._status)
        p.setPen(QtGui.QColor(self._status_color))
        p.drawText(bar.adjusted(8, 0, -8, 0), QtCore.Qt.AlignVCenter | QtCore.Qt.AlignRight,
                   self._status)
        p.setFont(_font(12))
        avail = int(bar.width() - status_w - 32)
        caption = QtGui.QFontMetrics(p.font()).elidedText(self._caption, QtCore.Qt.ElideMiddle,
                                                          max(40, avail))
        p.setPen(QtGui.QColor(THEME['text']))
        p.drawText(bar.adjusted(8, 0, -8, 0), QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, caption)
        p.end()


class PhaseStrip(QtWidgets.QWidget):
    """The row of mission phases; the current one is lit, past ones are ticked."""

    def __init__(self, phases: list[str], parent=None):
        super().__init__(parent)
        self.phases = list(phases)
        self.current = 0
        self.visited: set[int] = {0}
        self.detail = ''
        self.since = ''
        self.setMinimumHeight(74)

    def set_state(self, current: int, visited: set[int], detail: str, since: str = '') -> None:
        """Update the lit phase, the set of phases already passed and the detail line."""
        self.current, self.visited, self.detail, self.since = current, set(visited), detail, since
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802 - Qt naming
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        n = len(self.phases)
        gap = 18
        top, chip_h = 4, 38
        w = (self.width() - gap * (n - 1) - 4) / n
        for i, name in enumerate(self.phases):
            x = 2 + i * (w + gap)
            rect = QtCore.QRectF(x, top, w, chip_h)
            if i == self.current:
                bg, fg, border = QtGui.QColor(THEME['accent']), QtGui.QColor('white'), THEME['cyan']
            elif i < self.current and i in self.visited:
                bg, fg, border = QtGui.QColor(THEME['done']), QtGui.QColor('#9be9a8'), THEME['done']
            elif i < self.current:
                bg, fg, border = QtGui.QColor(THEME['panel']), QtGui.QColor('#4d5966'), THEME['panel2']
            else:
                bg, fg, border = QtGui.QColor(THEME['panel2']), QtGui.QColor(THEME['muted']), THEME['border']
            p.setPen(QtGui.QPen(QtGui.QColor(border), 1.5))
            p.setBrush(bg)
            p.drawRoundedRect(rect, 8, 8)
            label = name
            if i < self.current and i in self.visited:
                label = '✓ ' + name
            elif i < self.current:
                label = name + ' (skipped)'
            px = 14 if i == self.current else 13
            p.setFont(_font(px, i == self.current))
            fm = QtGui.QFontMetrics(p.font())
            while fm.horizontalAdvance(label) > w - 8 and px > 9:
                px -= 1
                p.setFont(_font(px, i == self.current))
                fm = QtGui.QFontMetrics(p.font())
            p.setPen(fg)
            p.drawText(rect, QtCore.Qt.AlignCenter, label)
            if i < n - 1:
                ax = x + w + 4
                ay = top + chip_h / 2
                p.setPen(QtGui.QPen(QtGui.QColor(THEME['muted']), 2))
                p.drawLine(QtCore.QPointF(ax, ay), QtCore.QPointF(ax + gap - 8, ay))
                p.drawLine(QtCore.QPointF(ax + gap - 12, ay - 4), QtCore.QPointF(ax + gap - 8, ay))
                p.drawLine(QtCore.QPointF(ax + gap - 12, ay + 4), QtCore.QPointF(ax + gap - 8, ay))
        p.setFont(_font(15, True))
        p.setPen(QtGui.QColor(THEME['cyan']))
        text = f'{self.phases[self.current]}: {self.detail}' if self.detail else self.phases[self.current]
        if self.since:
            text += f'   ({self.since})'
        p.drawText(QtCore.QRectF(4, top + chip_h + 4, self.width() - 8, 26),
                   QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft, text)
        p.end()


class StatusPill(QtWidgets.QLabel):
    """Small rounded label: a name and a coloured state ('ok', 'warn', 'bad', 'off')."""

    COLORS = {'ok': THEME['green'], 'warn': THEME['amber'], 'bad': THEME['red'],
              'off': THEME['muted']}

    def __init__(self, name: str, parent=None):
        super().__init__(parent)
        self.name = name
        self.set_state('off', 'waiting')

    def set_state(self, state: str, text: str, tooltip: str = '') -> None:
        """Change colour and text."""
        col = self.COLORS.get(state, THEME['muted'])
        self.setText(f'● {self.name}: {text}')
        self.setToolTip(tooltip or text)
        self.setStyleSheet(f'QLabel {{ color: {col}; background: {THEME["panel2"]}; '
                           f'border: 1px solid {THEME["border"]}; border-radius: 10px; '
                           f'padding: 3px 10px; font-size: 12px; font-weight: bold; }}')


class EventLog(QtWidgets.QPlainTextEdit):
    """Read-only, colour-coded event list (newest at the bottom, auto-scrolling)."""

    SOURCE_COLORS = {'autopilot': THEME['cyan'], 'team': THEME['text'], 'camera': THEME['amber'],
                     'panel': THEME['green'], 'detector': THEME['orange'], 'warn': THEME['amber'],
                     'error': THEME['red']}

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setReadOnly(True)
        self.setMaximumBlockCount(400)
        self.setLineWrapMode(QtWidgets.QPlainTextEdit.WidgetWidth)

    def add(self, t: float, source: str, text: str, level: str = '') -> None:
        """Append one line: time, [source], text."""
        col = self.SOURCE_COLORS.get(level or source, THEME['text'])
        stamp = time.strftime('%H:%M:%S', time.localtime(t))
        self.appendHtml(f'<span style="color:{THEME["muted"]}">{stamp}</span> '
                        f'<span style="color:{col}"><b>[{html.escape(source)}]</b> '
                        f'{html.escape(text)}</span>')
        sb = self.verticalScrollBar()
        sb.setValue(sb.maximum())
