import time
import socket
import threading
import weakref

import rosservice
from PySide.QtCore import *

class ServiceMonitor(threading.Thread, QObject):
    refresh = Signal(list)

    def __init__(self, sm):
        threading.Thread.__init__(self)
        QObject.__init__(self)
        self.sm = weakref.ref(sm)
        self.daemon = True
        self.refresh.connect(sm.refresh)

    def run(self):
        state = None
        while True:
            sm = self.sm()
            if sm is None:
                break
            time.sleep(0.2)
            try:
                new_state = rosservice.get_service_list()
                if new_state != state:
                    state = new_state
                    self.refresh.emit(state)
            except Exception:
                pass

