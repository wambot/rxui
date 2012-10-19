from PySide.QtGui import *

class StackItem(object):
    def __init__(self, name, parent):
        self.name = name
        self.packages = []
        self.root = parent

    def addPackage(self, pack):
        self.packages.append(pack)

    def child(self, row):
        return self.packages[row]

    def childCount(self):
        return len(self.packages)

    def data(self):
        return self.name

    def icon(self):
        return QApplication.style().standardIcon(QStyle.SP_DriveHDIcon)

    def parent(self):
        return self.root

    def row(self):
        return self.root.stacks.index(self)


