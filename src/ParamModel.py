import threading
import rospy

from PySide.QtCore import *
from PySide.QtGui import *

from ParamItem import ParamItem
from ParamMonitor import ParamMonitor

class ParamModel(QAbstractItemModel):
    def __init__(self, parent=None):
        super(ParamModel, self).__init__(parent)
        self.lock = threading.RLock()
        self.rootItem = ParamItem("", None, None, None, self)
        self.headers = ["Parameter", "Value"]
        self.monitor = ParamMonitor(self)
        self.monitor.start()

    def columnCount(self, parent=QModelIndex()):
        return 2

    def data(self, index, role):
        with self.lock:
            if not index.isValid():
                return None
            if role == Qt.DisplayRole:
                item = index.internalPointer()
                if index.column() == 0:
                    return item.name
                if index.column() == 1:
                    return item.data
                #return "row, col: %s, %s" % (index.row(), index.column())
                #return index.internalPointer().data()
            return None

    def setData(self, index, value):
        with self.lock:
            rospy.set_param(index.internalPointer().fullpath(), value)

    def headerData(self, section, orientation, role):
        with self.lock:
            if orientation == Qt.Horizontal and role == Qt.DisplayRole:
                return self.headers[section]
            return None

    def flags(self, index):
        with self.lock:
            if not index.isValid():
                return Qt.ItemIsEnabled | Qt.ItemIsSelectable

            if index.internalPointer().childCount() == 0 and index.column() == 1:
                return Qt.ItemIsEnabled | Qt.ItemIsSelectable | Qt.ItemIsEditable
            else:
                return Qt.ItemIsEnabled | Qt.ItemIsSelectable
            

    def index(self, row, col, parent):
        with self.lock:
            if not self.hasIndex(row, col, parent):
                return QModelIndex()

            if parent.isValid():
                parentItem = parent.internalPointer()
            else:
                parentItem = self.rootItem

            childItem = parentItem.child(row)
            if childItem:
                return childItem.createIndex(col)
            return QModelIndex()
    
    def parent(self, index):
        with self.lock:
            if not index.isValid():
                return QModelIndex()

            childItem = index.internalPointer()
            parentItem = childItem.parent()
        
            return parentItem.createIndex(0)

    def rowCount(self, parent):
        with self.lock:
            if parent.column() > 0:
                return 0

            if parent.isValid():
                parentItem = parent.internalPointer()
            else:
                parentItem = self.rootItem

            return parentItem.childCount()

    def setupModelData(self, data):
        with self.lock:
            #self.modelAboutToBeReset.emit()
            self.rootItem.merge(ParamItem("", None, data, None, self))
            #self.modelReset.emit()

