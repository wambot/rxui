#!/usr/bin/python

from __future__ import division

import sys
import math

import rospy
import rostopic
import rosparam

from PySide.QtCore import *
from PySide.QtGui import *

def create_control(path, data, meta=None):
    if meta:
        ctype = meta.get("type")
        if ctype == "int":
            return ParamEditorInt(path, data, meta)
        if ctype == "float":
            return ParamEditorFloat(path, data, meta)
        if ctype == "bool":
            return ParamEditorBool(path, data, meta)
        if ctype == "string":
            return ParamEditorString(path, data, meta)

    if type(data) == int:
        return ParamEditorInt(path, data)
    if type(data) == float:
        return ParamEditorFloat(path, data)
    if type(data) == bool:
        return ParamEditorBool(path, data)
    if type(data) == str:
        return ParamEditorString(path, data)
    if type(data) == dict:
        return ParamEditorDict(path, data)
            
class ParamEditorInt(QWidget):
    def __init__(self, path, data, meta={}, parent=None):
        QWidget.__init__(self, parent)
        name = path.rstrip("/").rsplit("/", 1)[-1]

        magic = 97612893

        min_value = meta.get("min_value", -0x80000000)
        max_value = meta.get("max_value",  0x7fffffff)
        increment = meta.get("increment",  1         )
        default   = meta.get("default"  ,  data      )
        scale     = meta.get("scale"    , "linear"   )

        label = QLabel(name)
        spin = QSpinBox()
        slide = QSlider(Qt.Horizontal)

        def update(x):
            rospy.set_param(path, x)

        spin.setMinimum(min_value)
        spin.setMaximum(max_value)
        spin.setValue(data)
        spin.setSingleStep(increment)
        spin.valueChanged.connect(update)
        if scale == "logarithmic" and min_value > 0 and max_value > 0:
            spin.valueChanged.connect(lambda x: slide.setValue(math.log(x) * magic))
            slide.setMinimum(math.log(min_value) * magic)
            slide.setMaximum(math.log(max_value) * magic)
            slide.setValue(math.log(data) * magic)
            print slide.value(), slide.maximum(), slide.minimum()
            slide.valueChanged.connect(lambda x: update(math.exp(x / magic)))
            slide.valueChanged.connect(lambda x: spin.setValue(math.exp(x / magic)))
        else:
            spin.valueChanged.connect(slide.setValue)
            slide.setMinimum(min_value)
            slide.setMaximum(max_value)
            slide.setValue(data)
            slide.valueChanged.connect(update)
            slide.valueChanged.connect(spin.setValue)

        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(spin)
        layout.addWidget(slide)
        self.setLayout(layout)
        self.default = default

class ParamEditorFloat(QWidget):
    def __init__(self, path, data, meta={}, parent=None):
        QWidget.__init__(self, parent)
        name = path.rstrip("/").rsplit("/", 1)[-1]
        magic = 3025550

        min_value = meta.get("min_value", sys.float_info.min)
        max_value = meta.get("max_value", sys.float_info.max)
        increment = meta.get("increment", 1                 )
        default   = meta.get("default"  , data              )
        scale     = meta.get("scale"    , "linear"          )

        decimals = 0
        factor = 1
        i = float(increment)
        while i != math.floor(i):
            i *= 10
            factor *= 10
            decimals += 1
        try:
            factor = int(min(factor, abs(sys.float_info.max / max_value)))
        except:
            pass
        try:
            factor = int(min(factor, abs(sys.float_info.max / min_value)))
        except:
            pass

        label = QLabel(name)
        spin = QDoubleSpinBox()
        slide = QSlider(Qt.Horizontal)

        def update(x):
            rospy.set_param(path, x)

        spin.setMinimum(min_value)
        spin.setMaximum(max_value)
        spin.setValue(data)
        spin.setSingleStep(increment)
        spin.setDecimals(decimals)
        spin.valueChanged.connect(update)
        if scale == "logarithmic" and min_value > 0 and max_value > 0:
            spin.valueChanged.connect(lambda x: slide.setValue(math.log(x) * magic))
            slide.setMinimum(math.log(min_value) * magic)
            slide.setMaximum(math.log(max_value) * magic)
            slide.setValue(math.log(data) * magic)
            slide.valueChanged.connect(lambda x: update(math.exp(x / magic)))
            slide.valueChanged.connect(lambda x: spin.setValue(math.exp(x / magic)))
        else:
            spin.valueChanged.connect(lambda x: slide.setValue(x * factor))
            slide.setMinimum(min_value * factor)
            print "max_value", max_value
            print "factor: %s" % factor
            print "path: %s" % path
            slide.setMaximum(max_value * factor)
            slide.setValue(data * factor)
            slide.valueChanged.connect(lambda x: update(x / factor))
            slide.valueChanged.connect(lambda x: spin.setValue(x / factor))

        layout = QVBoxLayout()
        layout.addWidget(label)
        layout.addWidget(spin)
        layout.addWidget(slide)
        self.setLayout(layout)
        self.default = default

class ParamEditorString(QWidget):
    def __init__(self, path, data, meta={}, parent=None):
        QWidget.__init__(self, parent)
        name = path.rstrip("/").rsplit("/", 1)[-1]

        default    = meta.get("default", data)
        defines    = meta.get("defines")
        topic_type = meta.get("topic_type")

        print defines, topic_type
        print meta

        label = QLabel(name)
        layout = QVBoxLayout()
        layout.addWidget(label)

        if defines == "topic" and topic_type != None:
            combo = QComboBox()
            combo.addItems([""] + rostopic.find_by_type(topic_type))
            combo.currentIndexChanged.connect(lambda ind: rospy.set_param(path, combo.itemText(ind)))
            combo.setCurrentIndex(combo.findText(data))
            layout.addWidget(combo)
        else:
            text = QLineEdit(data)
            text.textChanged.connect(lambda s: rospy.set_param(path, s))
            layout.addWidget(text)
            
        self.setLayout(layout)
        self.default = default
        
class ParamEditorBool(QWidget):
    def __init__(self, path, data, meta={}, parent=None):
        QWidget.__init__(self, parent)
        name = path.rstrip("/").rsplit("/", 1)[-1]

        default = meta.get("default", data)

        checkbox = QCheckBox(name)
        if data:
            checkbox.setCheckState(Qt.Checked)
        else:
            checkbox.setCheckState(Qt.Unchecked)
        checkbox.stateChanged.connect(lambda c: rospy.set_param(path, c == 2))
        layout = QHBoxLayout()
        layout.addWidget(checkbox)
        self.setLayout(layout)
        self.default = default

class ParamEditorDict(QWidget):
    def __init__(self, path, data, parent=None):
        QWidget.__init__(self, parent)
        name = path.rstrip("/").rsplit("/", 1)[-1]
        
        group = QGroupBox(name)
        sublayout = QVBoxLayout()
        layout = QVBoxLayout()
        for k in filter(lambda s: not s.endswith("__meta"), data.keys()):
            meta = data.get(k + "__meta")
            if meta != None:
                print "got meta for %s" % k
                print "meta %s" % meta
            widget = create_control(path + "/" + k, data[k], meta)
            sublayout.addWidget(widget)
        group.setLayout(sublayout)
        layout.addWidget(group)
        self.setLayout(layout)

class ParamEditor(QDialog):
    def __init__(self, path, parent=None):
        QDialog.__init__(self, parent)

        path = path.rstrip("/")
        self.close_button = QPushButton("&Close")
        self.close_button.clicked.connect(self.close_clicked)

        layout = QVBoxLayout()
        layout.addWidget(create_control(path, rospy.get_param(path)))
        layout.addWidget(self.close_button)
        self.setLayout(layout)

    def close_clicked(self):
        self.done(0)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        path = "/"
    app = QApplication(sys.argv)
    win = ParamEditor(path)
    win.show()
    app.exec_()


