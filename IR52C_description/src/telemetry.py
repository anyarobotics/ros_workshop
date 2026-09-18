#!/usr/bin/env python3
"""
motor_telemetry.py — 5-Motor Telemetry Monitor
Start/Stop logging, per-tab Freeze, display decimation.

Usage:
  python motor_telemetry.py --port COM3 --baud 115200
"""

import sys
import struct
import threading
import argparse
import queue
import time
import csv
from datetime import datetime

import serial
from PyQt6 import QtWidgets, QtCore, QtGui
from PyQt6.QtCore import Qt

# ── Packet constants ──────────────────────────────────────────────────────────
HEADER      = bytes([0xAA, 0x55])
PKT_FMT     = '<2sIfffBBfffBBfffBBfffBBfffBBB'
PKT_SIZE    = struct.calcsize(PKT_FMT)
STATE_NAMES = {0: 'IDLE', 1: 'RUNNING', 2: 'MANUAL', 3: 'BRAKE', 4: 'FAULT'}

BG         = '#0f1117'
SURFACE    = '#1a1d27'
SURFACE2   = '#22263a'
BORDER     = '#2e3350'
TEXT       = '#e8eaf0'
TEXT_MUTED = '#8b8fa8'
ACCENT     = ['#4fc3f7', '#81c784', '#ffb74d', '#f06292', '#ce93d8']
GREEN      = '#4caf50'
AMBER      = '#ff9800'
RED        = '#f44336'
STATE_COLOR = {0: TEXT_MUTED, 1: GREEN, 2: AMBER, 3: AMBER, 4: RED}

# ── Packet parser ─────────────────────────────────────────────────────────────
def parse_packet(data: bytes):
    if len(data) < PKT_SIZE:
        return None
    fields = struct.unpack_from(PKT_FMT, data)
    chk = 0
    for b in data[:PKT_SIZE - 1]:
        chk ^= b
    motors = []
    for i in range(5):
        motors.append({
            'angle':  fields[2 + i*5],
            'target': fields[2 + i*5 + 1],
            'rpm':    fields[2 + i*5 + 2],
            'state':  fields[2 + i*5 + 3],
            'crc_ok': fields[2 + i*5 + 4],
        })
    return {
        'timestamp_ms': fields[1],
        'motors':       motors,
        'chk_ok':       chk == data[PKT_SIZE - 1],
    }

# ── Serial reader ─────────────────────────────────────────────────────────────
class SerialReader(threading.Thread):
    def __init__(self, port, baud, pkt_q):
        super().__init__(daemon=True)
        self.port = port; self.baud = baud
        self.pkt_q = pkt_q; self.running = True

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except serial.SerialException as e:
            self.pkt_q.put({'error': str(e)}); return
        buf = bytearray()
        while self.running:
            chunk = ser.read(256)
            if not chunk: continue
            buf.extend(chunk)
            while True:
                idx = buf.find(HEADER)
                if idx < 0: buf = buf[-1:]; break
                if len(buf) - idx < PKT_SIZE: break
                pkt = parse_packet(bytes(buf[idx: idx + PKT_SIZE]))
                buf = buf[idx + PKT_SIZE:]
                if pkt: self.pkt_q.put(pkt)

# ── Helpers ───────────────────────────────────────────────────────────────────
def _mini_label(text, color):
    lbl = QtWidgets.QLabel(text)
    lbl.setStyleSheet(
        f'color:{color}; font-size:11px; background:transparent; border:none;')
    lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
    return lbl

def _sep_widget():
    w = QtWidgets.QLabel('|')
    w.setStyleSheet(f'color:{BORDER}; font-size:11px;')
    return w

# ── Motor position card ───────────────────────────────────────────────────────
class MotorCard(QtWidgets.QFrame):
    def __init__(self, motor_id):
        super().__init__()
        self.motor_id = motor_id
        color = ACCENT[motor_id]
        self.setStyleSheet(f"""
            MotorCard {{
                background:{SURFACE}; border:1px solid {color}44;
                border-left:3px solid {color}; border-radius:8px;
            }}
        """)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 12); lay.setSpacing(4)

        hdr = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel(f'Motor {motor_id}')
        title.setStyleSheet(
            f'color:{color}; font-size:13px; font-weight:700;'
            f' background:transparent; border:none;')
        self.state_badge = QtWidgets.QLabel('IDLE')
        self.state_badge.setStyleSheet(
            f'color:{TEXT_MUTED}; background:{SURFACE2}; border-radius:4px;'
            f' padding:1px 7px; font-size:11px; font-weight:600; border:none;')
        self.crc_dot = QtWidgets.QLabel('●')
        self.crc_dot.setStyleSheet(
            f'color:{GREEN}; font-size:14px; background:transparent; border:none;')
        hdr.addWidget(title); hdr.addStretch()
        hdr.addWidget(self.crc_dot); hdr.addWidget(self.state_badge)
        lay.addLayout(hdr)

        self.angle_lbl = QtWidgets.QLabel('—')
        self.angle_lbl.setStyleSheet(
            f'color:{TEXT}; font-size:32px; font-weight:300;'
            f' background:transparent; border:none;')
        self.angle_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(self.angle_lbl)

        sub = QtWidgets.QHBoxLayout()
        self.tgt_lbl = _mini_label('Tgt —°', TEXT_MUTED)
        self.err_lbl = _mini_label('Err —°', TEXT_MUTED)
        self.rpm_lbl = _mini_label('— RPM', TEXT_MUTED)
        for w in [self.tgt_lbl, self.err_lbl, self.rpm_lbl]:
            sub.addWidget(w, 1, Qt.AlignmentFlag.AlignCenter)
        lay.addLayout(sub)

        self.bar = QtWidgets.QProgressBar()
        self.bar.setRange(0, 3600); self.bar.setTextVisible(False)
        self.bar.setFixedHeight(4)
        self.bar.setStyleSheet(f"""
            QProgressBar {{ background:{SURFACE2}; border-radius:2px; border:none; }}
            QProgressBar::chunk {{ background:{color}; border-radius:2px; }}
        """)
        lay.addWidget(self.bar)
        self._last_update = 0.0; self._pending = None

    def buffer(self, data): self._pending = data

    def flush_if_due(self, now):
        if self._pending is None or now - self._last_update < 2.0: return
        self._last_update = now
        d = self._pending
        angle, target = d['angle'], d['target']
        err = angle - target; state = d['state']; crc_ok = d['crc_ok']
        self.angle_lbl.setText(f'{angle:.2f}°')
        self.tgt_lbl.setText(f'Tgt {target:.1f}°')
        self.err_lbl.setText(f'Err {err:+.2f}°')
        self.rpm_lbl.setText(f'{d["rpm"]:.1f} RPM')
        self.bar.setValue(int(min(3600, max(0, angle * 10))))
        sc = STATE_COLOR.get(state, TEXT_MUTED)
        self.state_badge.setText(STATE_NAMES.get(state, str(state)))
        self.state_badge.setStyleSheet(
            f'color:{sc}; background:{SURFACE2}; border-radius:4px;'
            f' padding:1px 7px; font-size:11px; font-weight:600; border:none;')
        self.crc_dot.setStyleSheet(
            f'color:{GREEN if crc_ok else RED};'
            f' font-size:14px; background:transparent; border:none;')

# ── Per-motor log table ───────────────────────────────────────────────────────
LOG_COLS   = ['Time', 'ms', 'Angle°', 'Target°', 'Err°', 'RPM', 'State', 'CRC']
LOG_WIDTHS = [80, 70, 80, 80, 72, 70, 70, 40]

class MotorLogTable(QtWidgets.QTableWidget):
    MAX_ROWS = 2000

    def __init__(self, motor_id):
        super().__init__(0, len(LOG_COLS))
        self.motor_id = motor_id
        self.frozen   = False          # freeze scroll when True
        self._row_counter = 0          # counts received rows (for decimation)
        color = ACCENT[motor_id]
        self.setHorizontalHeaderLabels(LOG_COLS)
        self.horizontalHeader().setStretchLastSection(True)
        self.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.setEditTriggers(
            QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.setAlternatingRowColors(True)
        self.verticalHeader().setVisible(False)
        self.verticalHeader().setDefaultSectionSize(20)
        self.setShowGrid(False)
        for i, w in enumerate(LOG_WIDTHS):
            self.setColumnWidth(i, w)
        self.setStyleSheet(f"""
            QTableWidget {{
                background:{SURFACE}; color:{TEXT};
                font-size:11px; font-family:monospace;
                border:none; alternate-background-color:{SURFACE2};
            }}
            QHeaderView::section {{
                background:{SURFACE2}; color:{color};
                font-size:11px; padding:4px 6px; font-weight:600;
                border:none; border-bottom:2px solid {color};
            }}
            QTableWidget::item {{ padding:1px 4px; }}
            QScrollBar:vertical {{
                background:{SURFACE}; width:6px; border:none;
            }}
            QScrollBar::handle:vertical {{
                background:{BORDER}; border-radius:3px;
            }}
        """)

    def add_row(self, ts_ms, m, decimate=1):
        """
        Add a row. decimate=N means show 1 in every N packets in the table.
        CSV logging is handled separately (always every packet).
        """
        self._row_counter += 1
        if self._row_counter % decimate != 0:
            return

        now_str = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        err     = m['angle'] - m['target']
        state   = m['state']; crc_ok = m['crc_ok']
        color   = ACCENT[self.motor_id]

        autoscroll = (not self.frozen and
                      self.verticalScrollBar().value() >=
                      self.verticalScrollBar().maximum() - 30)

        row = self.rowCount()
        if row >= self.MAX_ROWS:
            self.removeRow(0); row = self.rowCount()
        self.insertRow(row)

        err_color = (GREEN if abs(err) < 1.0
                     else AMBER if abs(err) < 5.0 else RED)
        values = [now_str, str(ts_ms),
                  f"{m['angle']:.3f}", f"{m['target']:.3f}",
                  f"{err:+.3f}", f"{m['rpm']:.2f}",
                  STATE_NAMES.get(state, str(state)),
                  '✓' if crc_ok else '✗']
        colors = [TEXT_MUTED, TEXT_MUTED, color, TEXT_MUTED,
                  err_color, TEXT,
                  STATE_COLOR.get(state, TEXT_MUTED),
                  GREEN if crc_ok else RED]

        for col, (val, c) in enumerate(zip(values, colors)):
            item = QtWidgets.QTableWidgetItem(val)
            item.setForeground(QtGui.QColor(c))
            self.setItem(row, col, item)
        if autoscroll:
            self.scrollToBottom()

# ── Status bar ────────────────────────────────────────────────────────────────
class StatusBar(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(10, 4, 10, 4); lay.setSpacing(6)
        self.conn_lbl  = self._mk('● Disconnected', RED, bold=True)
        self.rate_lbl  = self._mk('0 pkt/s', TEXT_MUTED)
        self.count_lbl = self._mk('0 pkts', TEXT_MUTED)
        self.drop_lbl  = self._mk('', TEXT_MUTED)
        self.log_lbl   = self._mk('', TEXT_MUTED)
        for w in [self.conn_lbl, _sep_widget(),
                  self.rate_lbl, _sep_widget(),
                  self.count_lbl, _sep_widget(),
                  self.drop_lbl]:
            lay.addWidget(w)
        lay.addStretch()
        lay.addWidget(self.log_lbl)
        self.setStyleSheet(
            f'background:{SURFACE2}; border-top:1px solid {BORDER};')

    @staticmethod
    def _mk(text, color, bold=False):
        w = QtWidgets.QLabel(text)
        w.setStyleSheet(
            f'color:{color}; font-size:11px;'
            f' {"font-weight:700;" if bold else ""}')
        return w

    def set_connected(self, desc):
        self.conn_lbl.setText(f'● {desc}')
        self.conn_lbl.setStyleSheet(
            f'color:{GREEN}; font-size:11px; font-weight:700;')

# ── Main window ───────────────────────────────────────────────────────────────
class TelemetryWindow(QtWidgets.QMainWindow):
    def __init__(self, port, baud):
        super().__init__()
        self.port = port; self.baud = baud
        self.pkt_q = queue.Queue()
        self.pkt_count = 0; self.drop_count = 0
        self._rate_buf = []
        self._csv_files   = [None] * 5
        self._csv_writers = [None] * 5
        self._logging     = False      # master log gate
        self._decimate    = 5          # show 1 in N in table (CSV gets all)

        self.setWindowTitle('Motor Telemetry Monitor')
        self.resize(1280, 880)
        self._apply_theme(); self._build_ui(); self._start_reader()

        QtCore.QTimer(self, interval=30,   timeout=self._poll).start()
        QtCore.QTimer(self, interval=200,  timeout=self._flush_cards).start()
        QtCore.QTimer(self, interval=1000, timeout=self._update_rate).start()

    # ── Theme ─────────────────────────────────────────────────────────────────
    def _apply_theme(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background:{BG}; color:{TEXT}; }}
            QTabWidget::pane  {{ border:1px solid {BORDER}; background:{SURFACE}; }}
            QTabBar::tab {{
                background:{SURFACE2}; color:{TEXT_MUTED};
                padding:6px 18px; font-size:12px;
                border:1px solid {BORDER}; border-bottom:none;
                border-radius:4px 4px 0 0; margin-right:2px;
            }}
            QTabBar::tab:selected {{ color:{TEXT}; font-weight:700; background:{SURFACE}; }}
            QTabBar::tab:hover    {{ color:{TEXT}; }}
            QPushButton {{
                background:{SURFACE2}; color:{TEXT};
                border:1px solid {BORDER}; border-radius:5px;
                padding:5px 14px; font-size:12px; font-weight:600;
            }}
            QPushButton:hover   {{ background:{BORDER}; }}
            QPushButton:pressed {{ background:{SURFACE}; }}
            QPushButton#start_btn {{
                background:#1a3a1a; border-color:{GREEN}; color:{GREEN};
                font-size:13px; padding:6px 22px;
            }}
            QPushButton#start_btn:hover {{ background:#1f4a1f; }}
            QPushButton#start_btn[active="true"] {{
                background:#3a1a1a; border-color:{RED}; color:{RED};
            }}
            QPushButton#start_btn[active="true"]:hover {{ background:#4a1f1f; }}
            QPushButton#freeze_btn[frozen="true"] {{
                background:#2a2a1a; border-color:{AMBER}; color:{AMBER};
            }}
            QComboBox {{
                background:{SURFACE}; color:{TEXT};
                border:1px solid {BORDER}; border-radius:4px;
                padding:3px 8px; font-size:12px;
            }}
            QLabel {{ background:transparent; }}
        """)

    # ── UI ────────────────────────────────────────────────────────────────────
    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0); root.setSpacing(0)

        # ── Toolbar ───────────────────────────────────────────────────────────
        toolbar = QtWidgets.QWidget()
        toolbar.setFixedHeight(56)
        toolbar.setStyleSheet(
            f'background:{SURFACE}; border-bottom:1px solid {BORDER};')
        tb = QtWidgets.QHBoxLayout(toolbar)
        tb.setContentsMargins(16, 0, 16, 0); tb.setSpacing(10)

        title = QtWidgets.QLabel('⚙  Motor Telemetry')
        title.setStyleSheet(f'color:{TEXT}; font-size:15px; font-weight:700;')
        tb.addWidget(title); tb.addSpacing(16)

        # ── BIG START / STOP button ───────────────────────────────────────────
        self.start_btn = QtWidgets.QPushButton('▶  Start Logging')
        self.start_btn.setObjectName('start_btn')
        self.start_btn.setProperty('active', 'false')
        self.start_btn.setFixedHeight(36)
        self.start_btn.clicked.connect(self._toggle_logging)
        tb.addWidget(self.start_btn)

        tb.addWidget(_sep_widget())

        # Decimation selector
        dec_lbl = QtWidgets.QLabel('Display:')
        dec_lbl.setStyleSheet(f'color:{TEXT_MUTED}; font-size:11px;')
        self.dec_combo = QtWidgets.QComboBox()
        self.dec_combo.addItems(
            ['Every packet', 'Every 2nd', 'Every 5th', 'Every 10th', 'Every 20th'])
        self.dec_combo.setCurrentIndex(2)   # default Every 5th
        self.dec_combo.setFixedWidth(120)
        self.dec_combo.currentIndexChanged.connect(self._on_decimate_change)
        tb.addWidget(dec_lbl); tb.addWidget(self.dec_combo)

        tb.addWidget(_sep_widget())

        self.clear_btn = QtWidgets.QPushButton('Clear All')
        self.clear_btn.clicked.connect(self._clear_all)
        tb.addWidget(self.clear_btn)

        tb.addStretch()

        self.ts_lbl = QtWidgets.QLabel('ts: —')
        self.ts_lbl.setStyleSheet(f'color:{TEXT_MUTED}; font-size:11px;')
        tb.addWidget(self.ts_lbl)
        root.addWidget(toolbar)

        # ── Motor cards ───────────────────────────────────────────────────────
        cards_w = QtWidgets.QWidget()
        cards_w.setStyleSheet(f'background:{BG};')
        cards_lay = QtWidgets.QHBoxLayout(cards_w)
        cards_lay.setContentsMargins(12, 12, 12, 8); cards_lay.setSpacing(10)
        self.cards = []
        for i in range(5):
            card = MotorCard(i)
            cards_lay.addWidget(card, 1)
            self.cards.append(card)
        cards_w.setFixedHeight(180)
        root.addWidget(cards_w)

        # ── Tabs ──────────────────────────────────────────────────────────────
        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self.log_tables = []
        self._freeze_btns = []
        self._tab_count_lbls = []

        for i in range(5):
            color  = ACCENT[i]
            tab_w  = QtWidgets.QWidget()
            tab_lay = QtWidgets.QVBoxLayout(tab_w)
            tab_lay.setContentsMargins(0, 0, 0, 0); tab_lay.setSpacing(0)

            # Per-tab toolbar
            tab_tb = QtWidgets.QWidget()
            tab_tb.setFixedHeight(36)
            tab_tb.setStyleSheet(
                f'background:{SURFACE2}; border-bottom:1px solid {BORDER};')
            ttb = QtWidgets.QHBoxLayout(tab_tb)
            ttb.setContentsMargins(12, 0, 12, 0); ttb.setSpacing(8)

            count_lbl = QtWidgets.QLabel('0 rows')
            count_lbl.setStyleSheet(f'color:{color}; font-size:11px; font-weight:600;')
            self._tab_count_lbls.append(count_lbl)
            ttb.addWidget(count_lbl)

            hint_lbl = QtWidgets.QLabel(
                '— CSV logs every packet · table shows decimated view')
            hint_lbl.setStyleSheet(f'color:{TEXT_MUTED}; font-size:10px;')
            ttb.addWidget(hint_lbl)
            ttb.addStretch()

            # Freeze button
            freeze_btn = QtWidgets.QPushButton('❄  Freeze')
            freeze_btn.setObjectName('freeze_btn')
            freeze_btn.setProperty('frozen', 'false')
            freeze_btn.setCheckable(True)
            freeze_btn.setFixedWidth(90)
            freeze_btn.clicked.connect(
                lambda checked, mid=i: self._toggle_freeze(mid, checked))
            self._freeze_btns.append(freeze_btn)

            clr_btn = QtWidgets.QPushButton('Clear')
            clr_btn.setFixedWidth(60)
            clr_btn.clicked.connect(
                lambda _, mid=i: self._clear_tab(mid))

            ttb.addWidget(freeze_btn)
            ttb.addWidget(clr_btn)
            tab_lay.addWidget(tab_tb)

            tbl = MotorLogTable(i)
            self.log_tables.append(tbl)
            tab_lay.addWidget(tbl)

            self.tabs.addTab(tab_w, f'  M{i}  ')
            self.tabs.tabBar().setTabTextColor(i, QtGui.QColor(color))

        root.addWidget(self.tabs, 1)

        # ── Status bar ────────────────────────────────────────────────────────
        self.status_bar = StatusBar()
        root.addWidget(self.status_bar)

    # ── Serial ────────────────────────────────────────────────────────────────
    def _start_reader(self):
        self.reader = SerialReader(self.port, self.baud, self.pkt_q)
        self.reader.start()
        self.status_bar.set_connected(f'{self.port} @ {self.baud}')

    # ── Start / Stop logging ──────────────────────────────────────────────────
    def _toggle_logging(self):
        if not self._logging:
            # ── START ──
            self._logging = True
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            fnames = []
            for i in range(5):
                fname = f'motor{i}_{ts}.csv'
                f = open(fname, 'w', newline='')
                w = csv.writer(f)
                w.writerow(['wall_time', 'timestamp_ms',
                             'angle_deg', 'target_deg', 'err_deg',
                             'rpm', 'state', 'crc_ok'])
                self._csv_files[i]   = f
                self._csv_writers[i] = w
                fnames.append(fname)
            self.start_btn.setText('■  Stop Logging')
            self.start_btn.setProperty('active', 'true')
            self.start_btn.style().unpolish(self.start_btn)
            self.start_btn.style().polish(self.start_btn)
            self.status_bar.log_lbl.setText(
                f'● Logging → motor0…motor4_{ts}.csv')
            self.status_bar.log_lbl.setStyleSheet(
                f'color:{GREEN}; font-size:11px; font-weight:600;')
        else:
            # ── STOP ──
            self._logging = False
            for i in range(5):
                if self._csv_files[i]:
                    self._csv_files[i].close()
                self._csv_files[i]   = None
                self._csv_writers[i] = None
            self.start_btn.setText('▶  Start Logging')
            self.start_btn.setProperty('active', 'false')
            self.start_btn.style().unpolish(self.start_btn)
            self.start_btn.style().polish(self.start_btn)
            self.status_bar.log_lbl.setText('Logs saved.')
            self.status_bar.log_lbl.setStyleSheet(
                f'color:{TEXT_MUTED}; font-size:11px;')

    # ── Decimation combo ──────────────────────────────────────────────────────
    def _on_decimate_change(self, idx):
        self._decimate = [1, 2, 5, 10, 20][idx]

    # ── Freeze ────────────────────────────────────────────────────────────────
    def _toggle_freeze(self, mid, frozen):
        self.log_tables[mid].frozen = frozen
        btn = self._freeze_btns[mid]
        btn.setText('▶  Live' if frozen else '❄  Freeze')
        btn.setProperty('frozen', 'true' if frozen else 'false')
        btn.style().unpolish(btn); btn.style().polish(btn)

    # ── Clear ─────────────────────────────────────────────────────────────────
    def _clear_tab(self, mid):
        self.log_tables[mid].setRowCount(0)
        self.log_tables[mid]._row_counter = 0
        self._tab_count_lbls[mid].setText('0 rows')

    def _clear_all(self):
        for i in range(5):
            self._clear_tab(i)

    # ── Poll loop ─────────────────────────────────────────────────────────────
    def _poll(self):
        count = 0
        while not self.pkt_q.empty() and count < 60:
            pkt = self.pkt_q.get_nowait()
            if 'error' in pkt:
                self.status_bar.conn_lbl.setText(f'ERR: {pkt["error"]}')
                return
            self.pkt_count += 1
            self._rate_buf.append(time.monotonic())
            ts = pkt['timestamp_ms']
            for i, m in enumerate(pkt['motors']):
                self.cards[i].buffer(m)
                self.log_tables[i].add_row(ts, m, self._decimate)
                rc = self.log_tables[i].rowCount()
                self._tab_count_lbls[i].setText(f'{rc} rows')
                if self._logging and self._csv_writers[i]:
                    self._write_csv(i, ts, m)
            self.ts_lbl.setText(f'ts: {ts} ms')
            count += 1
        self.status_bar.count_lbl.setText(f'{self.pkt_count} pkts')

    def _flush_cards(self):
        now = time.monotonic()
        for card in self.cards:
            card.flush_if_due(now)

    def _update_rate(self):
        now = time.monotonic()
        self._rate_buf = [t for t in self._rate_buf if t > now - 1.0]
        self.status_bar.rate_lbl.setText(f'{len(self._rate_buf)} pkt/s')

    def _write_csv(self, mid, ts_ms, m):
        wall = datetime.now().isoformat(timespec='milliseconds')
        err  = m['angle'] - m['target']
        self._csv_writers[mid].writerow([
            wall, ts_ms,
            f"{m['angle']:.3f}", f"{m['target']:.3f}", f"{err:+.3f}",
            f"{m['rpm']:.3f}", m['state'], m['crc_ok'],
        ])

    def closeEvent(self, event):
        if self._logging:
            self._toggle_logging()
        if hasattr(self, 'reader'):
            self.reader.running = False
        super().closeEvent(event)

# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser(description='Motor Telemetry Monitor')
    ap.add_argument('--port', required=True,
                    help='Serial port (e.g. COM3 or /dev/ttyUSB0)')
    ap.add_argument('--baud', type=int, default=115200)
    args = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle('Fusion')
    win = TelemetryWindow(args.port, args.baud)
    win.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()