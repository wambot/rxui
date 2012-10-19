import time
import rosnode

from PySide.QtCore  import *
from PySide.QtGui   import *

class Node(QGraphicsItem):
    def __init__(self, ui, name):
        self.ui = ui
        self.info_text = ""
        self.name = name
        self.rect = QRect(0, 0, 128, 64)
        while [i for i in ui.scene.items(self.rect) if i != self] != []:
            self.rect.translate(0, 80)
        QGraphicsItem.__init__(self, None, ui.scene)
        self.setCursor(Qt.OpenHandCursor)
        self.setFlags(QGraphicsItem.ItemIsSelectable | QGraphicsItem.ItemIsMovable | QGraphicsItem.ItemIsFocusable)
        self.setZValue(1)

        self.pubs           = []
        self.subs           = []
        #self.refresh()
        #self.test = QGraphicsLineItem(0, 0, 140, 140, self, ui.scene)

    def refresh(self, state):
        self.info_text = rosnode.get_node_info_description(self.name)
        self.setToolTip(self.info_text)
        self.pubs = [self.ui.topics[t] for t, l in state[0] if ((self.name in l) and (self.ui.topics.has_key(t)))]
        self.subs = [self.ui.topics[t] for t, l in state[1] if ((self.name in l) and (self.ui.topics.has_key(t)))]

    def paint(self, painter, option, widget=None):
        irect = self.rect.adjusted(5, 5, -5, -5)
        col = QColor(0, 0, 0)
        if self.isSelected():
            col.setRed(255)
        """
        if self.isUnderMouse():
            col.setBlue(255)
        """
        painter.setPen(col)
        painter.drawRoundedRect(self.rect, 2, 2)
        painter.drawText(irect, Qt.AlignLeft | Qt.AlignTop | Qt.TextSingleLine, self.name)
        """
        sdx = self.rect.right() - self.rect.left() / (len(self.pubs) + 1)
        start = QPointF(self.rect.left() + sdx, self.rect.bottom())
        for p in self.pubs:
            if self.ui.subs.has_key(p):
                for s in self.ui.subs[p]:
                    edx = s.rect.right() - s.rect.left() / (len(s.subs) + 1)
                    end = QPointF(s.rect.left() + edx * (s.subs.index(p) + 1), s.rect.top())
                    painter.drawLine(start, end + s.scenePos() - self.scenePos())
        """

    def boundingRect(self):
        return self.rect
    
    def mousePressEvent(self, event):
        QGraphicsItem.mousePressEvent(self, event)
        self.setCursor(Qt.ClosedHandCursor)

    def mouseMoveEvent(self, event):
        QGraphicsItem.mouseMoveEvent(self, event)
        for s in self.subs:
            s.update()
        for p in self.pubs:
            p.update()
        #self.setPos(self.pos() + event.pos() - event.lastPos())
        """
        state = self.ui.master.getSystemState()
        for p in self.pubs:
            p.refresh(state)
        for s in self.subs:
            s.refresh(state)
        """

    def mouseReleaseEvent(self, event):
        QGraphicsItem.mouseReleaseEvent(self, event)
        self.setCursor(Qt.OpenHandCursor)

    def keyPressEvent(self, event):
        QGraphicsItem.keyPressEvent(self, event)
        if event.key() == Qt.Key_Delete:
            res = rosnode.kill_nodes([self.name])
            #if res == ([self.name], []):
                #del self.ui.nodes[self.name]
                #self.ui.scene.removeItem(self)
                #self.ui.refresh()

