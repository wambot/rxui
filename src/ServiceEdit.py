#!/usr/bin/env python

import math

import rospy
import rosservice
import rosgraph

from PySide.QtCore import *
from PySide.QtGui import *

from ServiceModel import ServiceModel

class ServiceEdit(QWidget):
    def __init__(self, parent=None):
        QWidget.__init__(self, parent)

        tree_view = QTreeView()
        model = ServiceModel()
        tree_view.setModel(model)
        tree_view.selectionModel().selectionChanged.connect(self.tree_view_selection_changed)

        call_service_button = QPushButton("Call Service")
        call_service_button.clicked.connect(self.call_service)
        call_service_button.setEnabled(False)
        call_service_button.setDefault(True)
        args_label = QLabel("Args: ")
        args_lineedit = QLineEdit()
        args_lineedit.textEdited.connect(self.args_lineedit_edited)
        args_lineedit.setEnabled(False)
        args_lineedit.returnPressed.connect(self.call_service)
        response_label = QLabel()

        call_service_layout = QHBoxLayout()
        call_service_layout.addWidget(args_label)
        call_service_layout.addWidget(args_lineedit)
        call_service_layout.addWidget(call_service_button)
        
        layout = QVBoxLayout()
        layout.addWidget(tree_view)
        layout.addLayout(call_service_layout)
        layout.addWidget(response_label)

        self.tree_view = tree_view
        self.model = model
        self.call_service_button = call_service_button
        self.args_lineedit = args_lineedit
        self.response_label = response_label
        self.setLayout(layout)

    def call_service(self):
        sel = self.tree_view.selectedIndexes()[0]
        service_name = self.model.getFullName(sel)
        service_name = rosgraph.names.script_resolve_name("rosservice", service_name)
        service_class = rosservice.get_service_class_by_name(service_name)
        glob = {
            "math": math
        }
        try:
            service_args = eval("[" + self.args_lineedit.text() + "]", glob, {})
            #service_args = eval(self.args_lineedit.text(), glob, {})
            #if type(service_args) == tuple:
                #service_args = list(service_args)
            #else:
                #service_args = [service_args]
            service_proxy = rospy.ServiceProxy(service_name, service_class)
            #req, res = rosservice.call_service(service_name, service_args, service_class)
            res = service_proxy(*service_args)
            print "response: %s" % res
        except Exception as e:
            self.response_label.setText("<i>" + str(e) + "</i>")
            print "exception: %s" % e
            return
        self.response_label.setText(str(res))

    def args_lineedit_edited(self):
        sel = self.tree_view.selectedIndexes()[0]
        self.model.setArgs(sel, self.args_lineedit.text())

    def tree_view_selection_changed(self):
        sel = self.tree_view.selectedIndexes()[0]
        print sel
        if self.model.isService(sel):
            args = self.model.getArgs(sel)
            self.args_lineedit.setText(args)
            self.call_service_button.setEnabled(True)
            self.args_lineedit.setEnabled(True)
        else:
            self.args_lineedit.setText("")
            self.call_service_button.setEnabled(False)
            self.args_lineedit.setEnabled(False)
            
if __name__ == "__main__":
    import sys
    import roslib
    roslib.load_manifest("roslib")
    app = QApplication(sys.argv)
    win = ServiceEdit()
    win.show()
    app.exec_()

