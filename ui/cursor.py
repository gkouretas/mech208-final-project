from __future__ import annotations

import sys
import timeit
from PyQt5 import QtWidgets, QtCore
from packet_manager import QueueContainer, PacketManager

from collections import deque
import pyqtgraph as pg

_BALL_RADIUS = 2                # [cm]
_TUBE_RADIUS = 2.25             # [cm]
_ALLOWABLE_LIMITS = (5, 25)     # "allowable" limits (how far we allow our commands to go)
_WORKSPACE_LIMITS = (0, 35)     # "workspace" limits (how far we expect the ball CAN go)
_TIMEOUT = 1.0 / 60.0           # 1/2 FPS

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, queues: QueueContainer, *args, **kwargs):
        self._app = QtWidgets.QApplication([])
        self._queues = queues
        super().__init__(*args, **kwargs)

        self._is_started = False

        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setRange(
            xRange = [-_WORKSPACE_LIMITS[1]/2, _WORKSPACE_LIMITS[1]/2], 
            yRange = _WORKSPACE_LIMITS
        )

        self.setCentralWidget(self.plot_widget)

        self.primary_position: pg.PlotDataItem = \
            self.plot_widget.plot(
                symbol = 'o', 
                symbolBrush = 'blue', 
                symbolSize = _BALL_RADIUS*2, 
                pxMode = False
            )
        
        self.primary_position.setData(
            [0],[0]
        ) # start @ (0, 0) initially

        self.secondary_position: pg.PlotDataItem = \
            self.plot_widget.plot(
                symbol = 'o', 
                symbolBrush = 'yellow', 
                symbolSize = _BALL_RADIUS*2, 
                pxMode = False
            )
        
        self.secondary_position.setData(
            [0],[0.5]
        ) # start @ (0, 0) initially

        self.setpoint: pg.PlotDataItem = \
            self.plot_widget.plot(pen = 'r', color = "red", symbolBrush = 'red')
        self.setpoint.setData(
            [-_BALL_RADIUS, _BALL_RADIUS], [0, 0]
        ) # place setpoint @ (0,0) initially

        # Invert y-axis since distance sensor is pointed at the ground
        self.plot_widget.getPlotItem().getViewBox().invertY(True)

        # Boundaries of tube
        _left_outer_tube_plot: pg.PlotDataItem = \
            self.plot_widget.plot()
        _left_outer_tube_plot.setData([-_TUBE_RADIUS, -_TUBE_RADIUS], [_ALLOWABLE_LIMITS[0], _ALLOWABLE_LIMITS[1]])

        _right_outer_tube_plot: pg.PlotDataItem = \
            self.plot_widget.plot()
        _right_outer_tube_plot.setData([_TUBE_RADIUS, _TUBE_RADIUS], [_ALLOWABLE_LIMITS[0], _ALLOWABLE_LIMITS[1]])

        self.plot_widget.setAspectLocked(True)
    
    def update_plot(self):
        primary = None
        secondary = None

        for i, q in enumerate(self._queues):
            entry = timeit.default_timer()
            if q is None: 
                continue
            while len(q) == 0 and timeit.default_timer() - entry < _TIMEOUT:
                pass
            if len(q) == 0:
                continue

            if i == 0: primary = q.pop()
            else: secondary = q.pop()

        if primary is not None:
            self.primary_position.setData([0], [primary.actual])
            self.setpoint.setData([-_BALL_RADIUS, _BALL_RADIUS], [primary.target, primary.target])
        if secondary is not None:
            self.secondary_position.setData([0], [secondary.actual])

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
    PacketManager(queues = queues).run()
    main = MainWindow(
        queues = queues
    )

    main.run()

if __name__ == '__main__':
    main()
