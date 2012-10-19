from PySide.QtCore import *
from PySide.QtGui import *

class IntEditor(QWidget):
    valueChanged = Signal(int)

    def __init__(self, parent=None):
        QWidget.__init__(self, parent)
        spin_box = QSpinBox()
        slider = QSlider(Qt.Horizontal)
        layout = QVBoxLayout()
        layout.addWidget(spin_box)
        layout.addWidget(slider)

        spin_box.valueChanged.connect(self.setValue)
        slider.valueChanged.connect(self.setValue)
        spin_box.valueChanged.connect(slider.setValue)
        slider.valueChanged.connect(spin_box.setValue)

        self.spin_box = spin_box
        self.slider = slider
        self.setLayout(layout)

    def setValue(self, value):
        self.spin_box.setValue(value)
        self.slider.setValue(value)
        self.valueChanged.emit(value)

    def setMinimum(self, minvalue):
        self.spin_box.setMinimum(minvalue)
        self.slider.setMinimum(minvalue)

    def setMaximum(self, maxvalue):
        self.spin_box.setMaximum(maxvalue)
        self.slider.setMaximum(maxvalue)

    def value(self):
        return self.spin_box.value()

    def setGeometry(self, rect):
        self.spin_box.setGeometry(rect)
        self.slider.setGeometry(rect)
        r = self.geometry()
        #QWidget.setGeometry(self, r.translated(1, 1))
        print r
        #QWidget.setGeometry(self, r.translated(100, 14))
        #QWidget.setGeometry(self, r.adjusted(100, 14, 769, 100))
        print self.geometry()
        print self.spin_box.geometry()
