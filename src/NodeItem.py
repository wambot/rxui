from PySide.QtGui import *

class NodeItem(object):
    def __init__(self, name, parent):
        self.name = name
        self.package = parent
        self.last_args = ""
    
    def childCount(self):
        return 0

    def data(self):
        return self.name

    def icon(self):
        return QApplication.style().standardIcon(QStyle.SP_FileIcon)

    def parent(self):
        return self.package

    def row(self):
        return self.package.index(self)


