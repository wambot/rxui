import rosservice
from PySide.QtCore import *
from PySide.QtGui import *

class ServiceItem(object):
    def __init__(self, model, name, parent):
        self.model = model
        self.name = name
        self.parent_item = parent

    def parent(self):
        if self.parent_item is None:
            return QModelIndex()
        return self.parent_item.createIndex(0)

    def createIndex(self, col):
        row = 0
        if self.parent_item:
            keys = self.parent_item.children.keys()
            keys = sorted(keys)
            row = keys.index(self.name)
        return self.model.createIndex(row, col, self)

    def fullname(self):
        if self.parent_item:
            return self.parent_item.fullname() + "/" + self.name
        return ""

class ServiceItemBranch(ServiceItem):
    def __init__(self, model, name, parent):
        ServiceItem.__init__(self, model, name, parent)
        self.children = {}

    def data(self, col, role):
        if role == Qt.DisplayRole:
            if col == 0:
                return self.name
        return None

    def index(self, row, col):
        keys = self.children.keys()
        keys = sorted(keys)
        if row < len(keys):
            return self.children[keys[row]].createIndex(col)
        return QModelIndex()

    def rowCount(self):
        return len(self.children)

    def merge(self, item):
        keys_old = set(self.children.keys())
        keys_new = set(item.children.keys())

        add = keys_new - keys_old
        remove = keys_old - keys_new
        keep = keys_old & keys_new

        changed = False
        for k in add:
            self.children[k] = item.children[k]
        for k in remove:
            del self.children[k]
        for k in keep:
            if type(self.children[k]) == ServiceItemBranch and type(item.children[k]) == ServiceItemBranch:
                self.children[k].merge(item.children[k])
            elif type(self.children[k]) != type(item.children[k]):
                self.children[k] = item.children[k]
                changed |= True
        changed |= len(add) > 0 or len(remove) > 0
        if changed:
            index = self.createIndex(0)
            self.model.dataChanged.emit(index, index)

class ServiceItemLeaf(ServiceItem):
    def __init__(self, model, name, parent):
        ServiceItem.__init__(self, model, name, parent)
        try:
            self.args = rosservice.get_service_args(self.fullname())
        except rosservice.ROSServiceIOException:
            self.args = "Error communication with service."
        self.last_args = ""

    def data(self, col, role):
        if role == Qt.DisplayRole:
            if col == 0:
                return self.name
            if col == 1:
                return self.args
        return None

    def index(self, row, col):
        return QModelIndex()

    def rowCount(self):
        return 0
        
