#!/usr/bin/python

import itertools
import datetime

from PySide.QtCore  import *
from PySide.QtGui   import *

class Topic(QGraphicsItem):
    def __init__(self, ui, name):
        QGraphicsItem.__init__(self, None, ui.scene)
        self.GraphicsItemFlag = QGraphicsItem.ItemIsSelectable
        self.ui = ui
        self.name = name
        self.pubs = []
        self.subs = []
        self.rect = QRect(0, 0, 1, 1)
        #self.lines = []
    
    def refresh(self, state):
        pubnames = dict(state[0]).get(self.name, [])
        self.pubs = []
        for p in pubnames:
            if self.ui.nodes.has_key(p):
                self.pubs.append(self.ui.nodes[p])

        subnames = dict(state[1]).get(self.name, [])
        self.subs = []
        for s in subnames:
            if self.ui.nodes.has_key(s):
                self.subs.append(self.ui.nodes[s])
    
    def lines(self):
        ret = []
        for pub, sub in itertools.product(set(self.pubs), set(self.subs)):
            pubdx = (pub.rect.right() - pub.rect.left()) / (len(pub.pubs) + 1)
            subdx = (sub.rect.right() - sub.rect.left()) / (len(sub.subs) + 1)
            pubinds = [k for k, e in enumerate(pub.pubs) if e == self]
            subinds = [k for k, e in enumerate(sub.subs) if e == self]
            pubx = [pub.pos().x() + pub.rect.left() + pubdx * (k + 1) for k in pubinds]
            subx = [sub.pos().x() + sub.rect.left() + subdx * (k + 1) for k in subinds]
            py = pub.pos().y() + pub.rect.bottom()
            sy = sub.pos().y() + sub.rect.top()
            for px, sx in itertools.product(pubx, subx):
                ret.append((QPoint(px, py),QPoint(sx, sy)))
        return ret

    def boundingRect(self):
        lines = self.lines()
        def encompass(acc, line):
            s, e = line
            rs = QRect(s, QSize(1, 1))
            re = QRect(e, QSize(1, 1))
            return acc | rs | re
        new_rect = reduce(encompass, lines, QRect())
        ret = new_rect | self.rect
        self.rect = new_rect
        return ret

    def paint(self, painter, option, widget=None):
        lines = self.lines()
        for line in lines:
            s, e = line
            painter.drawLine(s, e)
        painter.setPen(QColor(0, 0, 255))
        

