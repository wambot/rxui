import os

from PySide.QtCore import *
from PySide.QtGui import *

from RootItem import *
from StackItem import *
from PackageItem import *
from NodeItem import *

class StackPackageNodeModel(QAbstractItemModel):
    def __init__(self, parent=None):
        super(StackPackageNodeModel, self).__init__(parent)
        self.rootItem = RootItem()
        self.setupModelData()

    def columnCount(self, parent):
        return 1

    def data(self, index, role):
        if not index.isValid():
            return None
        if role == Qt.DecorationRole:
            return index.internalPointer().icon()
        if role == Qt.DisplayRole:
            return index.internalPointer().data()
        return None

    def flags(self, index):
        if not index.isValid():
            return Qt.NoItemFlags

        return Qt.ItemIsEnabled | Qt.ItemIsSelectable
    
    def headerData(self, section, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return self.rootItem.data()
        return None

    def index(self, row, col, parent):
        if not self.hasIndex(row, col, parent):
            return QModelIndex()

        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        childItem = parentItem.child(row)
        if childItem:
            return self.createIndex(row, col, childItem)
        return QModelIndex()

    def parent(self, index):
        if not index.isValid():
            return QModelIndex()
        
        childItem = index.internalPointer()
        parentItem = childItem.parent()

        if parentItem == self.rootItem:
            return QModelIndex()
        return self.createIndex(parentItem.row(), 0, parentItem)

    def rowCount(self, parent):
        if parent.column() > 0:
            return 0

        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        return parentItem.childCount()

    def setupModelData(self):
        #print env
        for path in os.environ["ROS_PACKAGE_PATH"].split(":"):
            if os.path.isdir(path):
                for stack in sorted(os.listdir(path)):
                    stackpath = path + "/" + stack
                    if os.path.isdir(stackpath) and stack[0] != ".":
                        stackitem = StackItem(stack, self.rootItem)
                        self.rootItem.addStack(stackitem)
                        for package in sorted(os.listdir(stackpath)):
                            packagepath = stackpath + "/" + package
                            if os.path.isdir(packagepath) and package[0] != ".":
                                packageitem = PackageItem(package, stackitem)
                                stackitem.addPackage(packageitem)
                                for node in sorted(os.listdir(packagepath)):
                                    nodepath = packagepath + "/" + node
                                    if not os.path.isdir(nodepath) and os.access(nodepath, os.X_OK):
                                        nodeitem = NodeItem(node, packageitem)
                                        packageitem.addNode(nodeitem)
                                if os.path.isdir(packagepath + "/bin"):
                                    for node in sorted(os.listdir(packagepath + "/bin")):
                                        nodepath = packagepath + "/bin/" + node
                                        if not os.path.isdir(nodepath) and os.access(nodepath, os.X_OK):
                                            nodeitem = NodeItem(node, packageitem)
                                            packageitem.addNode(nodeitem)
                                if os.path.isdir(packagepath + "/nodes"):
                                    for node in sorted(os.listdir(packagepath + "/nodes")):
                                        nodepath = packagepath + "/nodes/" + node
                                        if not os.path.isdir(nodepath) and os.access(nodepath, os.X_OK):
                                            nodeitem = NodeItem(node, packageitem)
                                            packageitem.addNode(nodeitem)


