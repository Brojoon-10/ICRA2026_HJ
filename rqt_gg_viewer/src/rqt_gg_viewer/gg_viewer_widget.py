"""GG Viewer Widget — matplotlib + table + diff for GGV diamond results."""

import json
import copy

import numpy as np
import rospy
from std_msgs.msg import String

from python_qt_binding.QtCore import Qt, Signal
from python_qt_binding.QtGui import QColor, QFont
from python_qt_binding.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QSplitter, QTabWidget,
    QComboBox, QCheckBox, QPushButton, QLabel, QTableWidget,
    QTableWidgetItem, QHeaderView, QStatusBar, QGroupBox,
)

import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class GGViewerWidget(QWidget):
    """Main widget: diamond plot + summary table + diff + slope tab."""

    # Thread-safe signals (ROS callback thread → Qt main thread)
    results_signal = Signal(str)
    status_signal = Signal(str)

    def __init__(self, context=None):
        super().__init__()
        self.setWindowTitle('GG Viewer')

        self.current_data = None
        self.baseline_data = None

        self._build_ui()
        self._connect_signals()
        self._subscribe_ros()

    # ------------------------------------------------------------------ #
    #  UI construction
    # ------------------------------------------------------------------ #
    def _build_ui(self):
        root = QVBoxLayout(self)

        # --- top toolbar ---
        toolbar = QHBoxLayout()
        toolbar.addWidget(QLabel('V:'))
        self.v_combo = QComboBox()
        self.v_combo.setMinimumWidth(80)
        toolbar.addWidget(self.v_combo)
        toolbar.addWidget(QLabel('g:'))
        self.g_combo = QComboBox()
        self.g_combo.setMinimumWidth(80)
        toolbar.addWidget(self.g_combo)
        self.diff_check = QCheckBox('Show diff')
        self.diff_check.setChecked(True)
        toolbar.addWidget(self.diff_check)
        self.baseline_btn = QPushButton('Set as Baseline')
        toolbar.addWidget(self.baseline_btn)
        self.auto_baseline_check = QCheckBox('Auto-baseline first')
        self.auto_baseline_check.setChecked(True)
        self.auto_baseline_check.setToolTip(
            'Automatically set the first received result as baseline')
        toolbar.addWidget(self.auto_baseline_check)
        toolbar.addStretch()
        root.addLayout(toolbar)

        # --- tabs ---
        self.tabs = QTabWidget()

        # Tab 1: Diamond plot + summary table
        main_tab = QWidget()
        main_layout = QHBoxLayout(main_tab)
        splitter = QSplitter(Qt.Horizontal)

        # left: matplotlib diamond plot
        plot_group = QGroupBox('GG Diamond')
        plot_layout = QVBoxLayout(plot_group)
        self.fig = Figure(figsize=(5, 5), dpi=100)
        self.canvas = FigureCanvas(self.fig)
        self.ax_plot = self.fig.add_subplot(111)
        plot_layout.addWidget(self.canvas)
        splitter.addWidget(plot_group)

        # right: summary table
        table_group = QGroupBox('Diamond Summary (velocity_frame)')
        table_layout = QVBoxLayout(table_group)
        self.table = QTableWidget()
        self.table.setFont(QFont('Monospace', 9))
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        table_layout.addWidget(self.table)
        splitter.addWidget(table_group)

        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)
        main_layout.addWidget(splitter)
        self.tabs.addTab(main_tab, 'Diamond + Table')

        # Tab 2: Slope analysis
        slope_tab = QWidget()
        slope_layout = QVBoxLayout(slope_tab)
        self.slope_fig = Figure(figsize=(6, 4), dpi=100)
        self.slope_canvas = FigureCanvas(self.slope_fig)
        self.slope_ax = self.slope_fig.add_subplot(111)
        slope_layout.addWidget(self.slope_canvas)
        self.slope_label = QLabel('No slope data yet. Enable slope in GGTuner rqt.')
        slope_layout.addWidget(self.slope_label)
        self.tabs.addTab(slope_tab, 'Slope Analysis')

        # Tab 3: Tuning params
        params_tab = QWidget()
        params_layout = QVBoxLayout(params_tab)
        self.params_table = QTableWidget()
        self.params_table.setColumnCount(3)
        self.params_table.setHorizontalHeaderLabels(['Param', 'Current', 'Baseline'])
        self.params_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        params_layout.addWidget(self.params_table)
        self.tabs.addTab(params_tab, 'Tuning Params')

        root.addWidget(self.tabs)

        # --- status bar ---
        self.status_bar = QStatusBar()
        self.status_bar.showMessage('Waiting for /gg_results ...')
        root.addWidget(self.status_bar)

    def _connect_signals(self):
        self.results_signal.connect(self._handle_results)
        self.status_signal.connect(self._handle_status)
        self.v_combo.currentIndexChanged.connect(self._on_combo_changed)
        self.g_combo.currentIndexChanged.connect(self._on_combo_changed)
        self.diff_check.stateChanged.connect(self._on_combo_changed)
        self.baseline_btn.clicked.connect(self._set_baseline)

    # ------------------------------------------------------------------ #
    #  ROS subscriptions
    # ------------------------------------------------------------------ #
    def _subscribe_ros(self):
        self._sub_results = rospy.Subscriber(
            '/gg_results', String, self._on_results_cb, queue_size=1)
        self._sub_status = rospy.Subscriber(
            '/gg_compute_status', String, self._on_status_cb, queue_size=5)

    def _on_results_cb(self, msg):
        self.results_signal.emit(msg.data)

    def _on_status_cb(self, msg):
        self.status_signal.emit(msg.data)

    # ------------------------------------------------------------------ #
    #  Qt slot handlers (main thread)
    # ------------------------------------------------------------------ #
    def _handle_results(self, json_str):
        try:
            data = json.loads(json_str)
        except json.JSONDecodeError as e:
            rospy.logwarn(f'[GGViewer] JSON parse error: {e}')
            return
        self.current_data = data
        if self.baseline_data is None and self.auto_baseline_check.isChecked():
            self.baseline_data = copy.deepcopy(data)
        self._populate_combos(data)
        self._update_display()
        self.status_bar.showMessage(
            f"Received: {data.get('vehicle_name', '?')} "
            f"@ {data.get('timestamp', '?')}")

    def _handle_status(self, text):
        self.status_bar.showMessage(text)

    def _set_baseline(self):
        if self.current_data is not None:
            self.baseline_data = copy.deepcopy(self.current_data)
            self.status_bar.showMessage('Baseline set from current data')
            self._update_display()

    def _on_combo_changed(self, _=None):
        self._update_display()

    # ------------------------------------------------------------------ #
    #  Populate combo boxes
    # ------------------------------------------------------------------ #
    def _populate_combos(self, data):
        v_list = data.get('v_list', [])
        g_list = data.get('g_list', [])

        prev_v = self.v_combo.currentText()
        prev_g = self.g_combo.currentText()

        self.v_combo.blockSignals(True)
        self.g_combo.blockSignals(True)
        self.v_combo.clear()
        self.g_combo.clear()
        for v in v_list:
            self.v_combo.addItem(f'{v:.2f}')
        for g in g_list:
            self.g_combo.addItem(f'{g:.2f}')

        # restore previous selection if possible
        idx_v = self.v_combo.findText(prev_v)
        if idx_v >= 0:
            self.v_combo.setCurrentIndex(idx_v)
        idx_g = self.g_combo.findText(prev_g)
        if idx_g >= 0:
            self.g_combo.setCurrentIndex(idx_g)

        self.v_combo.blockSignals(False)
        self.g_combo.blockSignals(False)

    # ------------------------------------------------------------------ #
    #  Update all displays
    # ------------------------------------------------------------------ #
    def _update_display(self):
        if self.current_data is None:
            return
        self._update_table()
        self._update_plot()
        self._update_slope_tab()
        self._update_params_tab()

    # ------------------------------------------------------------------ #
    #  Summary table
    # ------------------------------------------------------------------ #
    def _update_table(self):
        d = self.current_data['diamond']
        v_list = self.current_data['v_list']
        g_list = self.current_data['g_list']
        show_diff = (self.diff_check.isChecked() and self.baseline_data is not None)

        n_v = len(v_list)
        n_g = len(g_list)
        # rows = velocities, columns = g values × 3 metrics
        metrics = ['ax_max', 'ax_min', 'ay_max']
        n_cols = n_g * len(metrics)

        self.table.setRowCount(n_v)
        self.table.setColumnCount(n_cols)

        # headers
        col_headers = []
        for g in g_list:
            for m in metrics:
                col_headers.append(f'g={g:.1f}\n{m}')
        self.table.setHorizontalHeaderLabels(col_headers)

        row_headers = [f'V={v:.1f}' for v in v_list]
        self.table.setVerticalHeaderLabels(row_headers)

        base_d = self.baseline_data['diamond'] if show_diff else None

        for vi in range(n_v):
            for gi in range(n_g):
                for mi, m_key in enumerate(metrics):
                    col = gi * len(metrics) + mi
                    val = d[m_key][vi][gi]
                    text = f'{val:.3f}'

                    item = QTableWidgetItem()
                    if show_diff and base_d is not None:
                        base_val = self._get_baseline_val(base_d, m_key, vi, gi)
                        if base_val is not None:
                            delta = val - base_val
                            pct = (delta / abs(base_val) * 100) if abs(base_val) > 1e-9 else 0
                            text = f'{val:.3f}\n({delta:+.3f} {pct:+.1f}%)'
                            if delta > 0.001:
                                item.setBackground(QColor(200, 255, 200))  # green
                            elif delta < -0.001:
                                item.setBackground(QColor(255, 200, 200))  # red
                    item.setText(text)
                    item.setTextAlignment(Qt.AlignCenter)
                    item.setFlags(item.flags() & ~Qt.ItemIsEditable)
                    self.table.setItem(vi, col, item)

    def _get_baseline_val(self, base_d, key, vi, gi):
        try:
            return base_d[key][vi][gi]
        except (IndexError, KeyError):
            return None

    # ------------------------------------------------------------------ #
    #  Diamond GG plot
    # ------------------------------------------------------------------ #
    def _update_plot(self):
        self.ax_plot.clear()
        d = self.current_data['diamond']
        v_list = self.current_data['v_list']
        g_list = self.current_data['g_list']

        vi = self.v_combo.currentIndex()
        gi = self.g_combo.currentIndex()
        if vi < 0 or gi < 0:
            self.canvas.draw()
            return

        v_val = v_list[vi]
        g_val = g_list[gi]

        ax_max_val = d['ax_max'][vi][gi]
        ax_min_val = d['ax_min'][vi][gi]
        ay_max_val = d['ay_max'][vi][gi]
        gg_exp = d['gg_exponent'][vi][gi]

        # Draw current diamond
        self._draw_diamond(self.ax_plot, ax_max_val, ax_min_val, ay_max_val,
                           gg_exp, color='#2196F3', linestyle='-',
                           label='Current', alpha=1.0)

        # Draw baseline diamond
        show_diff = (self.diff_check.isChecked() and self.baseline_data is not None)
        if show_diff:
            bd = self.baseline_data['diamond']
            try:
                b_ax_max = bd['ax_max'][vi][gi]
                b_ax_min = bd['ax_min'][vi][gi]
                b_ay_max = bd['ay_max'][vi][gi]
                b_gg_exp = bd['gg_exponent'][vi][gi]
                self._draw_diamond(self.ax_plot, b_ax_max, b_ax_min, b_ay_max,
                                   b_gg_exp, color='gray', linestyle='--',
                                   label='Baseline', alpha=0.7)
            except (IndexError, KeyError):
                pass

        self.ax_plot.set_xlabel('Lateral ay [m/s²]')
        self.ax_plot.set_ylabel('Longitudinal ax [m/s²]')
        self.ax_plot.set_title(f'GG Diamond  V={v_val:.1f} m/s  g={g_val:.1f} m/s²')
        self.ax_plot.legend(loc='upper right', fontsize=8)
        self.ax_plot.grid(True, alpha=0.3)
        self.ax_plot.set_aspect('equal', adjustable='datalim')
        self.ax_plot.axhline(y=0, color='k', linewidth=0.5)
        self.ax_plot.axvline(x=0, color='k', linewidth=0.5)
        self.fig.tight_layout()
        self.canvas.draw()

    def _draw_diamond(self, ax, ax_max, ax_min, ay_max, gg_exp,
                      color='blue', linestyle='-', label=None, alpha=1.0):
        """Draw a GG diamond envelope using the diamond parameterization."""
        exp = max(gg_exp, 0.5)
        theta = np.linspace(0, 2 * np.pi, 200)
        # diamond parameterization: |ax/ax_lim|^exp + |ay/ay_max|^exp = 1
        ay_pts = ay_max * np.cos(theta)
        ax_pts = np.zeros_like(theta)
        for i, t in enumerate(theta):
            ay_frac = min(abs(ay_pts[i]) / ay_max, 1.0) if ay_max > 1e-9 else 0
            remaining = max(1.0 - ay_frac ** exp, 0.0)
            ax_lim = ax_max if np.sin(t) >= 0 else abs(ax_min)
            ax_pts[i] = ax_lim * remaining ** (1.0 / exp) * np.sign(np.sin(t))
        ax.plot(ay_pts, ax_pts, color=color, linestyle=linestyle,
                linewidth=2, label=label, alpha=alpha)

    # ------------------------------------------------------------------ #
    #  Slope tab
    # ------------------------------------------------------------------ #
    def _update_slope_tab(self):
        slope_data = self.current_data.get('slope_results')
        if slope_data is None:
            self.slope_label.setText(
                'No slope data. Enable "enable_slope" in GGTuner rqt.')
            self.slope_label.show()
            self.slope_ax.clear()
            self.slope_canvas.draw()
            return
        self.slope_label.hide()
        self.slope_ax.clear()

        vi = self.v_combo.currentIndex()
        gi = self.g_combo.currentIndex()
        if vi < 0 or gi < 0:
            self.slope_canvas.draw()
            return

        slopes_deg = slope_data.get('slope_list_deg', [])
        ax_max_s = slope_data.get('ax_max', [])
        ax_min_s = slope_data.get('ax_min', [])
        ay_max_s = slope_data.get('ay_max', [])

        try:
            ax_max_vs = [ax_max_s[vi][gi][si] for si in range(len(slopes_deg))]
            ax_min_vs = [ax_min_s[vi][gi][si] for si in range(len(slopes_deg))]
            ay_max_vs = [ay_max_s[vi][gi][si] for si in range(len(slopes_deg))]
        except (IndexError, KeyError):
            self.slope_label.setText('Slope data dimension mismatch.')
            self.slope_label.show()
            self.slope_canvas.draw()
            return

        v_val = self.current_data['v_list'][vi]
        g_val = self.current_data['g_list'][gi]

        self.slope_ax.plot(slopes_deg, ax_max_vs, 'r-o', markersize=4, label='ax_max')
        self.slope_ax.plot(slopes_deg, ax_min_vs, 'b-o', markersize=4, label='ax_min')
        self.slope_ax.plot(slopes_deg, ay_max_vs, 'g-o', markersize=4, label='ay_max')
        self.slope_ax.set_xlabel('Slope angle [deg]')
        self.slope_ax.set_ylabel('Acceleration [m/s²]')
        self.slope_ax.set_title(f'Slope effect  V={v_val:.1f} m/s  g={g_val:.1f} m/s²')
        self.slope_ax.legend(fontsize=8)
        self.slope_ax.grid(True, alpha=0.3)
        self.slope_ax.axvline(x=0, color='k', linewidth=0.5, linestyle='--')
        self.slope_fig.tight_layout()
        self.slope_canvas.draw()

    # ------------------------------------------------------------------ #
    #  Tuning params tab
    # ------------------------------------------------------------------ #
    def _update_params_tab(self):
        cur_p = self.current_data.get('tuning_params', {})
        base_p = self.baseline_data.get('tuning_params', {}) if self.baseline_data else {}
        all_keys = sorted(set(list(cur_p.keys()) + list(base_p.keys())))

        self.params_table.setRowCount(len(all_keys))
        for i, key in enumerate(all_keys):
            self.params_table.setItem(i, 0, QTableWidgetItem(key))

            cur_val = cur_p.get(key, '')
            base_val = base_p.get(key, '')
            cur_item = QTableWidgetItem(f'{cur_val}')
            base_item = QTableWidgetItem(f'{base_val}')

            if cur_val != '' and base_val != '' and cur_val != base_val:
                cur_item.setBackground(QColor(255, 255, 180))  # highlight changed
            self.params_table.setItem(i, 1, cur_item)
            self.params_table.setItem(i, 2, base_item)

    # ------------------------------------------------------------------ #
    #  Shutdown
    # ------------------------------------------------------------------ #
    def shutdown(self):
        self._sub_results.unregister()
        self._sub_status.unregister()
