import subprocess

from PySide.QtCore import *
from PySide.QtGui import *

from StackPackageNodeModel import StackPackageNodeModel
from NodeItem import NodeItem

class NodeEdit(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)

        tree_view = QTreeView()
        model = StackPackageNodeModel()
        tree_view.setModel(model)
        tree_view.selectionModel().selectionChanged.connect(self.tree_view_selection_changed)

        create_node_button = QPushButton("Run")
        create_node_button.clicked.connect(self.create_node)
        create_node_button.setEnabled(False)

        args_label = QLabel("Args: ")
        args_lineedit = QLineEdit()
        args_lineedit.textEdited.connect(self.args_lineedit_edited)
        args_lineedit.setEnabled(False)

        create_node_layout = QHBoxLayout()
        create_node_layout.addWidget(create_node_button)
        create_node_layout.addWidget(args_label)
        create_node_layout.addWidget(args_lineedit)

        layout = QVBoxLayout()
        layout.addWidget(tree_view)
        layout.addLayout(create_node_layout)

        self.tree_view = tree_view
        self.create_node_button = create_node_button
        self.args_lineedit = args_lineedit
        self.setLayout(layout)

    def create_node(self):
        nodeitem = self.tree_view.selectedIndexes()[0].internalPointer()
        if not type(nodeitem) == NodeItem:
            return None
        node = nodeitem.data()
        package = nodeitem.parent().data()

        command = "rosrun " + package + " " + node + " " + self.args_lineedit.text()
        print "command: %s" % command
        subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)

    def args_lineedit_edited(self):
        nodeitem = self.tree_view.selectedIndexes()[0].internalPointer()
        if not type(nodeitem) == NodeItem:
            return
        nodeitem.last_args = self.args_lineedit.text()

    def tree_view_selection_changed(self):
        nodeitem = self.tree_view.selectedIndexes()[0].internalPointer()
        if not type(nodeitem) == NodeItem:
            self.create_node_button.setEnabled(False)
            self.args_lineedit.setEnabled(False)
            self.args_lineedit.setText("")
            return
        self.create_node_button.setEnabled(True)
        self.args_lineedit.setEnabled(True)
        self.args_lineedit.setText(nodeitem.last_args)

