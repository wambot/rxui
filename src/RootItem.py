class RootItem(object):
    def __init__(self):
        self.stacks = []

    def addStack(self, stack):
        self.stacks.append(stack)

    def child(self, row):
        return self.stacks[row]

    def childCount(self):
        return len(self.stacks)

    def data(self):
        return "Package Browser"

    def parent(self):
        return None

    def row(self):
        return 0


