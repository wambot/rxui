from PySide.QtGui import *

class PackageItem(object):
    def __init__(self, name, parent):
        self.name = name
        self.stack = parent
        self.nodes = []

    def addNode(self, node):
        self.nodes.append(node)
    
    def child(self, row):
        return self.nodes[row]

    def childCount(self):
        return len(self.nodes)

    def data(self):
        return self.name

    def icon(self):
        return QApplication.style().standardIcon(QStyle.SP_DirIcon)

    def parent(self):
        return self.stack

    def row(self):
        return self.stack.packages.index(self)


