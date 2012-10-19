import weakref

from PySide.QtCore import *
from PySide.QtGui import *

from IntEditor import IntEditor

class ParamDelegate(QItemDelegate):
    def __init__(self, parent=None):
        QItemDelegate.__init__(self, parent)
        self.selected = None
        self.last_editor = None

    def createEditor(self, parent, option, index):
        self.last_editor = None
        data = index.internalPointer().data
        meta = index.internalPointer().meta
        if type(meta) == dict:
            if meta.has_key("type"):
                if meta["type"] == "int":
                    return self.createIntEditor(parent, data, meta)
                return None

        if type(data) == int:
            return self.createIntEditor(parent, data)

        return None

    def createIntEditor(self, parent, data, meta={}):
        ret = IntEditor(parent)
        ret.setMinimum(meta.get("min", min(0, data)))
        ret.setMaximum(meta.get("max", max(100, data)))
        ret.setValue(data)
        ret.setMinimumSize(ret.sizeHint())
        self.last_editor = weakref.ref(ret)
        return ret

    def setEditorData(self, editor, index):
        if type(editor) == IntEditor:
            editor.setValue(index.internalPointer().data)

    def setModelData(self, editor, model, index):
        if type(editor) == IntEditor:
            index.internalPointer().data = editor.value()
            model.setData(index, editor.value())

    def updateEditorGeometry(self, editor, option, index):
        print "\tupdating geo"
        editor.setGeometry(option.rect)
        
    def paint(self, painter, option, index):
        QItemDelegate.paint(self, painter, option, index)
        if option.state & QStyle.State_Selected:
            new_selected = index.internalPointer()
            if new_selected != self.selected:
                self.selected = new_selected
                self.selected.model.layoutChanged.emit()

    def sizeHint(self, option, index):
        if index.internalPointer() == self.selected and self.last_editor:
            editor = self.last_editor()
            if editor:
                return editor.sizeHint()
        return QItemDelegate.sizeHint(self, option, index)
        
        
