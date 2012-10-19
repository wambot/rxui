from PySide.QtCore import *
from PySide.QtGui import *

class ParamItem(object):
    def __init__(self, name, parent, data, meta, model):
        self.name = name
        self.parentItem = parent
        self.items = {}
        self.model = model
        if type(data) == dict:
            for k, v in data.items():
                if not k.endswith("__meta"):
                    childdata = v
                    childmeta = data.get(k + "__meta")
                    self.items[k] = ParamItem(k, self, childdata, childmeta, model)
            self.data = None
        else:
            self.data = data
        self.meta = meta

    def merge(self, item):
        if item.name != self.name:
            raise ValueError("Error using ParamItem.merge")
        self.meta = item.meta
        if item.data is not None:
            emit = self.data != item.data or self.items != {}
            self.data = item.data
            self.items = {}
            if emit:
                index = self.createIndex(1)
                self.model.dataChanged.emit(index, index)
        else:
            oldkeys = set(self.items.keys())
            newkeys = set(item.items.keys())
            add = newkeys - oldkeys
            remove = oldkeys - newkeys
            keep = newkeys & oldkeys
            for k in add:
                self.items[k] = item.items[k]
            for k in remove:
                del self.items[k]
            for k in keep:
                self.items[k].merge(item.items[k])
            if len(add) > 0 or len(remove) > 0:
                index = self.createIndex(0)
                print "change this?"
                self.model.dataChanged.emit(index, index)

    def createIndex(self, col):
        if self.parentItem:
            return self.model.createIndex(self.row(), col, self)
        return QModelIndex()

    def child(self, row):
        return self.items[sorted(self.items.keys())[row]]

    def childCount(self):
        return len(self.items)

    def parent(self):
        return self.parentItem

    def row(self):
        if self.parentItem:
            return sorted(self.parentItem.items.keys()).index(self.name)

    def fullpath(self):
        if self.parentItem:
            return self.parentItem.fullpath() + "/" + self.name
        return ""
