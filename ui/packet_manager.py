import serial
import time
import threading
import copy

from typing import NamedTuple
from collections import deque
from dataclasses import dataclass
from enum import Enum
from telemetry_logger import DataLogger

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
    ts: float
    is_primary: bool

class QueueContainer(NamedTuple):
    fan: deque[SystemPacket] | None
    beam: deque[SystemPacket] | None

num_items = len(SystemPacket.__match_args__) - 2

class PacketManager:
    def __init__(self, queues: QueueContainer, simulated: bool = False) -> None:
        self._fan_queue = queues.fan
        self._beam_queue = queues.beam
        self._simulated = simulated
        
        self._fan_logger = DataLogger(SystemPacket)
        self._beam_logger = DataLogger(SystemPacket)

        self._com: serial.Serial = None
        if not self._simulated:
            self._com = serial.Serial(port = "COM3", baudrate = 115200)
        
    def run(self): 
        threading.Thread(target = self._poll, daemon = True).start()

    def _populate_packet(self, info: list, offset: int, is_primary: bool, ts_index: int = None):
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
            ff_contrib  = float(info[offset+9]),
            ts          = float(info[ts_index if ts_index is not None else offset+10]),
            is_primary  = is_primary
        )
        
    def _process_packet(self, packet: SystemPacket):
        if packet.sys == SystemType.FAN:
            q = self._fan_queue
            logger = self._fan_logger
        else:
            q = self._beam_queue
            logger = self._beam_queue
            
        if q is not None: q.append(copy.deepcopy(packet))
        
        logger.log(copy.deepcopy(packet))
        
    def _poll(self):
        while True:
            try:
                if self._simulated:
                    info = [
                        "0",
                        "10",
                        "10",
                        "0",
                        "0",
                        "0",
                        "5",
                        "6",
                        "7",
                        "8",
                        "9",
                        str(time.time())
                    ]

                    is_primary = True
                else:
                    buffer = self._com.readline()
                    info = buffer.decode().replace("\r\n", "").split(" ")
                    is_primary = bool(int(info.pop(0)))
                if len(info) == num_items + 1:      # single system
                    packet = self._populate_packet(info = info, offset = 0, is_primary = is_primary)
                    self._process_packet(packet)
                        
                elif len(info) == 2*num_items + 1:
                    packet = self._populate_packet(info = info, offset = 0, ts_index = 2*num_items, is_primary = is_primary)
                    self._process_packet(packet)
                    
                    packet = self._populate_packet(info = info, offset = num_items, is_primary = not is_primary)
                    self._process_packet(packet)
                        
                else:
                    raise ValueError(f"Unexpected number of items: {len(info)}")

            except Exception:
                import traceback
                traceback.print_exc()
            
            time.sleep(0.005) # ~200 Hz