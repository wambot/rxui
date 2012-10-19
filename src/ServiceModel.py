import threading

from PySide.QtCore import *
from PySide.QtGui import *

from ServiceItem import ServiceItem, ServiceItemBranch, ServiceItemLeaf
from ServiceMonitor import ServiceMonitor

class ServiceModel(QAbstractItemModel):
    def __init__(self, parent=None):
        super(ServiceModel, self).__init__(parent)
        self.lock = threading.RLock()
        self.root_item = ServiceItemBranch(self, "", None)
        self.headers = ["Service", "Arguments"]
        self.monitor = ServiceMonitor(self)
        self.monitor.start()

    def columnCount(self, parent=QModelIndex()):
        return 2

    def data(self, index, role):
        with self.lock:
            if not index.isValid():
                return None
            return index.internalPointer().data(index.column(), role)

    def headerData(self, section, orientation, role):
        with self.lock:
            if orientation == Qt.Horizontal and role == Qt.DisplayRole:
                return self.headers[section]
            return None

    def flags(self, index):
        return Qt.ItemIsEnabled | Qt.ItemIsSelectable

    def index(self, row, col, parent):
        with self.lock:
            if not self.hasIndex(row, col, parent):
                return QModelIndex()

            if parent.isValid():
                return parent.internalPointer().index(row, col)
            return self.root_item.index(row, col)

    def parent(self, index):
        with self.lock:
            if not index.isValid():
                return QModelIndex()
            return index.internalPointer().parent()

    def rowCount(self, parent):
        with self.lock:
            if parent.column() > 0:
                return 0

            if parent.isValid():
                return parent.internalPointer().rowCount()

            return self.root_item.rowCount()

    def setArgs(self, index, args):
        with self.lock:
            if index.isValid():
                index.internalPointer().last_args = args

    def getArgs(self, index):
        with self.lock:
            if index.isValid():
                return index.internalPointer().last_args

    def getFullName(self, index):
        with self.lock:
            if index.isValid():
                return index.internalPointer().fullname()
            return ""

    def isService(self, index):
        with self.lock:
            if index.isValid():
                return type(index.internalPointer()) == ServiceItemLeaf
            return False

    def refresh(self, state):
        with self.lock:
            servs = [s.split("/")[1:] for s in state]

            def create_dicttree(servs):
                if servs == [[]]:
                    return {}
                ret = {}
                for s in servs:
                    ret[s[0]] = []
                for s in servs:
                    ret[s[0]].append(s[1:])
                for k in ret.keys():
                    ret[k] = create_dicttree(ret[k])
                return ret
            dicttree = create_dicttree(servs)

            def create_item(dt, name, parent):
                if len(dt) == 0:
                    return ServiceItemLeaf(self, name, parent)
                ret = ServiceItemBranch(self, name, parent)
                for k, v in dt.items():
                    ret.children[k] = create_item(v, k, ret)
                return ret
            item = create_item(dicttree, "", None)
            self.layoutAboutToBeChanged.emit()
            self.root_item.merge(item)
            self.layoutChanged.emit()
        
