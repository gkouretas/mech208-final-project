"""
George Kouretas
MECH 208

CSV logger for data
"""
import csv
import threading
import os
import datetime

class DataLogger:
    def __init__(self, name: str, obj: object) -> None:
        """Simple CSV logger for thermal chamber."""
        os.makedirs(os.path.join(os.getcwd(), "logs"), exist_ok = True)
        self.path = os.path.join(
            os.getcwd(), f"logs/{name}_" + datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S") + ".csv"
        )
        
        self._lock = threading.RLock()
        
        self._fp = open(self.path, "w", newline='')
        
        # Initialize writer
        self.writer = csv.writer(self._fp)
        
        # Write header
        self.writer.writerow(obj.__annotations__.keys())
        
        print(f"Logging to {self.path}")
        
    def log(self, parsed_buffer: object):
        """Log data"""
        self.writer.writerow(list(parsed_buffer.__dict__.values()))
        self._fp.flush()
        
    def __del__(self):
        """Destructor. Flush and close file"""
        if not self._fp.closed:
            self._fp.flush()
            self._fp.close()