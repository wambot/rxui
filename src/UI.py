#!/usr/bin/env python

import sys
import threading

import rosnode
import rosgraph

from PySide.QtCore import *
from PySide.QtGui import *

from Node import Node
from Topic import Topic
from ParamEditor import ParamEditor
from ParamEdit import ParamEdit
from ServiceEdit import ServiceEdit
from NodeEdit import NodeEdit

from GraphMonitor import GraphMonitor

class UI(QDialog):
    def __init__(self):
        super(UI, self).__init__(None)
        self.lock = threading.RLock()
        with self.lock:
            self.setWindowTitle("rxui")

            self.scene = QGraphicsScene()
            self.scene.selectionChanged.connect(self.selection_changed)
            self.graphics = QGraphicsView(self.scene)
            self.node_edit = NodeEdit()
            self.param_edit = ParamEdit()
            self.service_edit = ServiceEdit()
            self.tab = QTabWidget()
            self.tab.addTab(self.node_edit, "Run")
            self.tab.addTab(self.param_edit, "Parameters")
            self.tab.addTab(self.service_edit, "Services")
            self.info_label = QLabel()
            self.edit_params_button = QPushButton("&Edit Parameters")
            self.edit_params_button.clicked.connect(self.edit_params)

            self.right_col_layout = QVBoxLayout()
            self.right_col_layout.align = Qt.AlignTop
            self.right_col_layout.addWidget(self.tab)
            self.right_col_layout.addWidget(self.info_label)
            self.right_col_layout.addWidget(self.edit_params_button)
            self.right_col_widget = QWidget()
            self.right_col_widget.setLayout(self.right_col_layout)
            self.splitter = QSplitter()
            self.splitter.setHandleWidth(8)
            self.splitter.addWidget(self.graphics)
            self.splitter.addWidget(self.right_col_widget)
            self.layout = QHBoxLayout()
            self.layout.addWidget(self.splitter)

            self.setLayout(self.layout)

            self.resize(1000, 700)

            self.master = rosgraph.Master("rxui")

            self.namespace = ""
            self.nodes = {}
            self.topics = {}
            #self.arrows = {}
            #self.pubs = {}
            #self.subs = {}
            #self.services = {}
            self.monitor = GraphMonitor(self)
            self.monitor.start()

    def refresh(self, state):
        with self.lock:
            repaint = False

            nodes_remaining = set(self.nodes.keys())
            topics_remaining = set(self.topics.keys())
            
            all_nodes = set.union(*([set([nod for nod in lis if nod.startswith(self.namespace)]) for top, lis in (state[0] + state[1])] + [set()]))
            all_topics = set([top for top, lis in (state[0] + state[1])])

            for n in all_nodes:
                nodes_remaining.discard(n)
                if not self.nodes.has_key(n):
                    newnode = Node(self, n)
                    self.nodes[n] = newnode
                    repaint |= True
                else:
                    self.nodes[n].update()

            for t in all_topics:
                topics_remaining.discard(t)
                if not self.topics.has_key(t):
                    newtopic = Topic(self, t)
                    self.topics[t] = newtopic
                    repaint |= True
                else:
                    self.topics[t].update()
                    #self.scene.addItem(newtopic)

            for n in nodes_remaining:
                self.scene.update(self.nodes[n].rect)
                self.scene.removeItem(self.nodes[n])
                del self.nodes[n]

            for t in topics_remaining:
                self.scene.update(self.topics[t].rect)
                self.scene.removeItem(self.topics[t])
                del self.topics[t]

            repaint |= bool(nodes_remaining)
            repaint |= bool(topics_remaining)

            for node in self.nodes.values():
                node.refresh(state)

            for topic in self.topics.values():
                topic.refresh(state)

            if repaint:
                self.scene.update()

            #self.scene.addItem(Arrow(self, "/usb_cam/image_raw", self.nodes["/usb_cam"], self.nodes["/sdl_viewer"]))

    def edit_params(self):
        sel = self.scene.selectedItems()
        if(len(sel) == 1):
            item = sel[0]
            if type(item) == Node:
                p = ParamEditor(item.name, self)
                p.show()
                p.raise_()
                p.activateWindow()

    def selection_changed(self):
        sel = self.scene.selectedItems()
        if(len(sel) == 1):
            item = sel[0]
            #self.node_info.set_node(item)
            #self.info_label.setText(item.info_text)
        else:
            #self.node_info.set_node(None)
            #self.info_label.setText("<i>No node selected.</i>")
            pass

if __name__ == "__main__":
    import roslib
    roslib.load_manifest("roslib")
    app = QApplication(sys.argv)
    win = UI()
    win.show()
    app.exec_()

