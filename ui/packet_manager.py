import serial
import time
import threading

from typing import NamedTuple
from collections import deque
from dataclasses import dataclass
from enum import Enum

class SystemType(Enum):
    FAN = 0
    BEAM = 1

@dataclass(frozen = True)
class SystemPacket:
    sys: SystemType
    target: float
    actual: float
    kp: float
    ki: float
    kd: float
    kp_contrib: float
    ki_contrib: float
    kd_contrib: float
    ff_contrib: float

class QueueContainer(NamedTuple):
    fan: deque[SystemPacket] | None
    beam: deque[SystemPacket] | None

num_items = len(SystemPacket.__match_args__)

class PacketManager:
    def __init__(self, queues: QueueContainer) -> None:
        self._fan_queue = queues.fan
        self._beam_queue = queues.beam
        self._com = serial.Serial(port = "COM18", baudrate = 115200)
        
    def run(self): 
        threading.Thread(target = self._poll, daemon = True).start()

    def _populate_packet(self, info: list, offset: int):
        return SystemPacket(
            sys         = SystemType(int(info[offset])),
            target      = float(info[offset+1]),
            actual      = float(info[offset+2]),
            kp          = float(info[offset+3]),
            ki          = float(info[offset+4]),
            kd          = float(info[offset+5]),
            kp_contrib  = float(info[offset+6]),
            ki_contrib  = float(info[offset+7]),
            kd_contrib  = float(info[offset+8]),
            ff_contrib  = float(info[offset+9])
        )

    def _poll(self):
        while True:
            try:
                buffer = self._com.readline()
                info = buffer.decode().replace("\r\n", "").split(" ")
                if len(info) == num_items + 1:      # single system
                    if self._fan_queue is not None:
                        print(info)
                        self._fan_queue.append(
                            self._populate_packet(info = info, offset = 0)
                        )
                elif len(info) == 2*num_items + 1:
                    if self._fan_queue is not None:
                        self._fan_queue.append(
                            self._populate_packet(info = info, offset = 0)
                        )
                    if self._beam_queue is not None:
                        self._beam_queue.append(
                            self._populate_packet(info = info, offset = num_items)
                        )
                else:
                    raise ValueError(f"Unexpected number of items: {len(info)}")

            except Exception:
                import traceback
                traceback.print_exc()
            
            time.sleep(0.005) # ~200 Hz