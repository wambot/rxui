import time
import socket
import threading
import weakref

import rospy
from PySide.QtCore import *

class ParamMonitor(threading.Thread, QObject):
    refresh = Signal(list)

    def __init__(self, pe):
        threading.Thread.__init__(self)
        QObject.__init__(self)
        self.pe = weakref.ref(pe)
        self.daemon = True
        self.refresh.connect(pe.setupModelData)

    def run(self):
        state = None
        while True:
            pe = self.pe()
            if pe is None:
                break
            time.sleep(0.2)
            try:
                new_state = rospy.get_param("")
                if new_state != state:
                    state = new_state
                    self.refresh.emit(state)
            except socket.error:
                pass
