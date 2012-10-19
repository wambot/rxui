from PySide.QtCore import *
from PySide.QtGui import *

from ParamModel import ParamModel
from ParamDelegate import ParamDelegate

class ParamEdit(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)

        tree_view = QTreeView()
        model = ParamModel()
        delegate = ParamDelegate()
        tree_view.setModel(model)
        tree_view.setItemDelegate(delegate)
        tree_view.setEditTriggers(QAbstractItemView.AllEditTriggers)
        
        layout = QVBoxLayout()
        layout.addWidget(tree_view)
        self.setLayout(layout)

