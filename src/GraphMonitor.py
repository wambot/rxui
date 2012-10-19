import weakref
import socket
import time
import threading

from PySide.QtCore import *

class GraphMonitor(threading.Thread, QObject):
    refresh = Signal(list)

    def __init__(self, ui):
        threading.Thread.__init__(self)
        QObject.__init__(self)
        self.ui = weakref.ref(ui)
        self.daemon = True
        self.refresh.connect(ui.refresh)

    def run(self):
        state = None
        new_state = None
        while True:
            ui = self.ui()
            if ui is None:
                break
            time.sleep(0.2)
            try:
                new_state = ui.master.getSystemState()
                if new_state != state:
                    state = new_state
                    #ui.refresh(state)
                    self.refresh.emit(state)
            except socket.error:
                pass

