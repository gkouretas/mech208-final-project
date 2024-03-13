from __future__ import annotations

import sys
import time
import copy
from PyQt5 import QtWidgets, QtCore
from packet_manager import QueueContainer, PacketManager, SystemPacket, SystemType
from typing import NamedTuple
from collections import deque
import pyqtgraph as pg

_BALL_RADIUS = 2                # [cm]
_TUBE_RADIUS = 2.25             # [cm]
_ALLOWABLE_LIMITS = (5, 30)     # "allowable" limits (how far we allow our commands to go)
_WORKSPACE_LIMITS = (0, 35)     # "workspace" limits (how far we expect the ball CAN go)
_TIMEOUT = 1.0 / 60.0           # 1/2 FPS

class PlotContainer(NamedTuple):
    kp: pg.PlotDataItem
    ki: pg.PlotDataItem
    kd: pg.PlotDataItem
    ff: pg.PlotDataItem

class ContribContainer(NamedTuple):
    kp: deque
    ki: deque
    kd: deque
    ff: deque

class ContributionPlots:
    def __init__(self, fan_widget: pg.PlotWidget, beam_widget: pg.PlotWidget) -> None:
        fan_widget.addLegend()
        self.fan = PlotContainer(
            fan_widget.plot(symbol = 'o', pen = 'r', symbolBrush = 'r', name="kp"),
            fan_widget.plot(symbol = 'o', pen = 'g', symbolBrush = 'g', name="ki"),
            fan_widget.plot(symbol = 'o', pen = 'b', symbolBrush = 'b', name="kd"),
            fan_widget.plot(symbol = 'o', pen = 'purple', symbolBrush = 'purple', name="ff")
        )

        self.fan_queue = ContribContainer(
            deque(maxlen = 50),
            deque(maxlen = 50),
            deque(maxlen = 50),
            deque(maxlen = 50)
        )

        self.beam = PlotContainer(
            beam_widget.plot(symbol = 'o', pen = 'r', symbolBrush = 'r', name="kp"),
            beam_widget.plot(symbol = 'o', pen = 'g', symbolBrush = 'g', name="ki"),
            beam_widget.plot(symbol = 'o', pen = 'b', symbolBrush = 'b', name="kd"),
            beam_widget.plot(symbol = 'o', pen = 'purple', symbolBrush = 'purple', name="ff")
        )

        self.beam_queue = ContribContainer(
            deque(maxlen = 50),
            deque(maxlen = 50),
            deque(maxlen = 50),
            deque(maxlen = 50)
        )

        self._time_fan = deque(maxlen = 50)
        self._time_beam = deque(maxlen = 50)

    def plot_data(self, data: SystemPacket, ts: float):
        if data.sys == SystemType.FAN:
            plot_ = self.fan
            queue_ = self.fan_queue
            self._time_fan.append(ts)
            time_ = self._time_fan
        elif data.sys == SystemType.BEAM:
            plot_ = self.beam
            queue_ = self.beam_queue
            time_ = self._time_beam
            self._time_beam.append(ts)
        for plot, q, contrib in zip(plot_, queue_, (data.kp_contrib, data.ki_contrib, data.kd_contrib, data.ff_contrib)):
            q.append(contrib)
            plot.setData(time_, q)

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, queues: QueueContainer, *args, **kwargs):
        self._app = QtWidgets.QApplication([])
        super().__init__(*args, **kwargs)
        self._queues = queues

        self.plot_widget = QtWidgets.QWidget()

        self._is_started = False

        self._layout = QtWidgets.QGridLayout(self.plot_widget)
        self.setLayout(self._layout)

        self.ball_plot_widget = pg.PlotWidget()
        self.ball_plot_widget.setTitle("System Status/Command")
        self.ball_plot_widget.setRange(
            xRange = [-_WORKSPACE_LIMITS[1]/2, _WORKSPACE_LIMITS[1]/2], 
            yRange = _WORKSPACE_LIMITS
        )

        self.fan_contrib_plot_widget = pg.PlotWidget()
        self.fan_contrib_plot_widget.setTitle("Fan Control Contributions")

        self.beam_contrib_plot_widget = pg.PlotWidget()
        self.beam_contrib_plot_widget.setTitle("Beam Control Contributions")

        self.contrib_plots = ContributionPlots(self.fan_contrib_plot_widget, self.beam_contrib_plot_widget)
        
        self._layout.addWidget(self.ball_plot_widget, 0, self._layout.columnCount())
        self._layout.addWidget(self.fan_contrib_plot_widget, 0, self._layout.columnCount())
        self._layout.addWidget(self.beam_contrib_plot_widget, 0, self._layout.columnCount())

        self.ball_plot_widget.addLegend()

        self.primary_position: pg.PlotDataItem = \
            self.ball_plot_widget.plot(
                symbol = 'o', 
                symbolBrush = 'blue', 
                symbolSize = _BALL_RADIUS*2, 
                pxMode = False,
                name = "Fan"
            )
        
        self.primary_position.setData(
            [0],[0]
        ) # start @ (0, 0) initially

        self.secondary_position: pg.PlotDataItem = \
            self.ball_plot_widget.plot(
                symbol = 'o', 
                symbolBrush = 'yellow', 
                symbolSize = _BALL_RADIUS*2, 
                pxMode = False,
                name = "Beam"
            )
        
        self.secondary_position.setData(
            [0],[0.5]
        ) # start @ (0, 0) initially

        self.setpoint: pg.PlotDataItem = \
            self.ball_plot_widget.plot(pen = 'r', color = "red", symbolBrush = 'red', name = "Setpoint")
        self.setpoint.setData(
            [-_BALL_RADIUS, _BALL_RADIUS], [0, 0]
        ) # place setpoint @ (0,0) initially

        # Invert y-axis since distance sensor is pointed at the ground
        self.ball_plot_widget.getPlotItem().getViewBox().invertY(True)

        # Boundaries of tube
        _left_outer_tube_plot: pg.PlotDataItem = \
            self.ball_plot_widget.plot()
        _left_outer_tube_plot.setData([-_TUBE_RADIUS, -_TUBE_RADIUS], [_ALLOWABLE_LIMITS[0], _ALLOWABLE_LIMITS[1]])

        _right_outer_tube_plot: pg.PlotDataItem = \
            self.ball_plot_widget.plot()
        _right_outer_tube_plot.setData([_TUBE_RADIUS, _TUBE_RADIUS], [_ALLOWABLE_LIMITS[0], _ALLOWABLE_LIMITS[1]])

        self.ball_plot_widget.setAspectLocked(True)

        self.setCentralWidget(self.plot_widget)
    
    def update_plot(self):
        primary = None
        secondary = None

        for q in self._queues:
            entry = time.process_time()
            if q is None: 
                continue
            while len(q) == 0 and time.process_time() - entry < _TIMEOUT:
                pass
            if len(q) == 0:
                continue

            data = q.pop()
            if data.is_primary: 
                primary = copy.deepcopy(data)
            else: 
                secondary = copy.deepcopy(data)

        if primary is not None:
            if primary.sys == SystemType.FAN:
                self.primary_position.setData([0], [primary.actual])
            elif primary.sys == SystemType.BEAM:
                self.secondary_position.setData([0], [primary.actual])
            self.setpoint.setData([-_BALL_RADIUS, _BALL_RADIUS], [primary.target, primary.target])
            self.contrib_plots.plot_data(primary, primary.ts)

        if secondary is not None:
            if secondary.sys == SystemType.FAN:
                self.primary_position.setData([0], [secondary.actual])
            elif secondary.sys == SystemType.BEAM:
                self.secondary_position.setData([0], [secondary.actual])
            self.contrib_plots.plot_data(secondary, secondary.ts)

    def run(self):
        # Plot refresh, update @ configured FPS
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.setInterval(1000 // 30) # 30 FPS
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start()

        self.show()
        sys.exit(self._app.exec_())

def main():
    queues = QueueContainer(deque(maxlen = 1), deque(maxlen=1))
    PacketManager(queues = queues, simulated = False).run()
    main = MainWindow(
        queues = queues
    )

    main.run()

if __name__ == '__main__':
    main()
