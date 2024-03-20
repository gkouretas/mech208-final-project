from __future__ import annotations

import sys
import time
import copy
from PyQt5 import QtWidgets, QtCore
from packet_manager import QueueContainer, PacketManager, SystemPacket, SystemType
from typing import NamedTuple
from collections import deque
import pyqtgraph as pg

_FAN_BALL_RADIUS = 2                # [cm]
_BEAM_BALL_RADIUS = 1                # [cm]
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
        self._fan_widget = fan_widget
        self._beam_widget = beam_widget
        
        fan_widget.addLegend()
        beam_widget.addLegend()
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

        self.fan_values: list[pg.TextItem] = None

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

        self.beam_values: list[pg.TextItem] = None

        self._time_fan = deque(maxlen = 50)
        self._time_beam = deque(maxlen = 50)

    def plot_data(self, data: SystemPacket, ts: float):    
        if data.sys == SystemType.FAN:
            plot_ = self.fan
            queue_ = self.fan_queue
            self._time_fan.append(ts)
            time_ = self._time_fan
            if self.fan_values is None:
                self.fan_values = []
                for c in ("r", "g", "b", "purple"):
                    text_item = pg.TextItem("", anchor=(0, 0), color = "black", fill = c)
                    self._fan_widget.addItem(text_item)
                    self.fan_values.append(text_item)
            values_ = self.fan_values
        elif data.sys == SystemType.BEAM:
            plot_ = self.beam
            queue_ = self.beam_queue
            time_ = self._time_beam
            self._time_beam.append(ts)
            if self.beam_values is None:
                self.beam_values = []
                for c in ("r", "g", "b", "purple"):
                    text_item = pg.TextItem("", anchor=(0, 0), color = "black", fill = c)
                    self._beam_widget.addItem(text_item)
                    self.beam_values.append(text_item)
            values_ = self.beam_values
        
        for plot, q, contrib, text in zip(plot_, queue_, (data.kp_contrib, data.ki_contrib, data.kd_contrib, data.ff_contrib), values_):
            q.append(contrib)
            plot.setData(time_, q)
            text.setPos(time_[-1] - 100.0, contrib)
            text.setText(f"{contrib:.2f}")

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
                symbolSize = _FAN_BALL_RADIUS*2, 
                pxMode = False,
                name = "Fan"
            )
        
        self.primary_position.setData(
            [0],[0]
        ) # start @ (0, 0) initially

        self.primary_position_text: pg.TextItem = None

        self.secondary_position: pg.PlotDataItem = \
            self.ball_plot_widget.plot(
                symbol = 'o', 
                symbolBrush = 'gray', 
                symbolSize = _BEAM_BALL_RADIUS*2, 
                pxMode = False,
                name = "Beam"
            )
        
        self.secondary_position.setData(
            [0],[0.5]
        ) # start @ (0, 0) initially

        self.secondary_position_text: pg.TextItem = None

        self.setpoint: pg.PlotDataItem = \
            self.ball_plot_widget.plot(pen = 'r', color = "red", symbolBrush = 'red', name = "Setpoint")
        self.setpoint.setData(
            [-_TUBE_RADIUS, _TUBE_RADIUS], [0, 0]
        ) # place setpoint @ (0,0) initially

        self.setpoint_position_text: pg.TextItem = None

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
            self.setpoint.setData([-_TUBE_RADIUS, _TUBE_RADIUS], [primary.target, primary.target])
            self.contrib_plots.plot_data(primary, primary.ts)
            if self.primary_position_text is None:
                self.primary_position_text = pg.TextItem("", anchor=(0.5, 0.5), color = "black", fill = "white")
                self.ball_plot_widget.addItem(self.primary_position_text)
            self.primary_position_text.setPos(-2*_TUBE_RADIUS, primary.actual)
            self.primary_position_text.setText(f"{primary.actual:.2f}")
            # self.primary_position_text.setColor("blue" if primary.sys == SystemType.FAN else "gray")

            if self.setpoint_position_text is None:
                self.setpoint_position_text = pg.TextItem("", anchor=(0.5, 0.5), color = "black", fill = "red")
                self.ball_plot_widget.addItem(self.setpoint_position_text)
            self.setpoint_position_text.setPos(2*_TUBE_RADIUS, primary.target)
            self.setpoint_position_text.setText(f"{primary.target:.2f}")

        if secondary is not None:
            if secondary.sys == SystemType.FAN:
                self.primary_position.setData([0], [secondary.actual])
            elif secondary.sys == SystemType.BEAM:
                self.secondary_position.setData([0], [secondary.actual])
            self.contrib_plots.plot_data(secondary, secondary.ts)
            if self.secondary_position_text is None:
                self.secondary_position_text = pg.TextItem("", anchor=(0.5, 0.5), color = "black", fill = "white")
                self.ball_plot_widget.addItem(self.secondary_position_text)
            self.secondary_position_text.setPos(-2*_TUBE_RADIUS, secondary.actual)
            self.secondary_position_text.setText(f"{secondary.actual:.2f}")
            # self.secondary_position_text.setColor("blue" if secondary.sys == SystemType.FAN else "gray")

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
    PacketManager(queues = queues, simulated = True).run()
    main = MainWindow(
        queues = queues
    )

    main.run()

if __name__ == '__main__':
    main()