"""Readable matrix cells and browsable definitions; original exports stay exact."""
import re
from urdf2dt.ui.number_display import decimal, residual


def compact_expression(text):
    """Only shorten floating literals for a clearly labelled approximate preview."""
    pattern = r'(?<![\w.])(?:\d+\.\d*|\.\d+)(?:[eE][+-]?\d+)?(?![\w.])'
    return re.sub(pattern, lambda m: residual(float(m.group())), text)


def fill_matrix(qt, table, values, rows=None, columns=None, numeric=False):
    table.clear()
    table.setRowCount(len(values))
    table.setColumnCount(len(values[0]) if values else 0)
    table.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
    if rows:
        table.setVerticalHeaderLabels(rows)
    if columns:
        table.setHorizontalHeaderLabels(columns)
    for i, row in enumerate(values):
        for j, value in enumerate(row):
            item = qt.QTableWidgetItem(decimal(value) if numeric else compact_expression(value))
            item.setToolTip(str(value))
            table.setItem(i,j,item)
    table.resizeColumnsToContents()
    for j in range(table.columnCount()):
        table.setColumnWidth(j, max(72, table.columnWidth(j)))


class MatrixView:
    def __init__(self, e, raw_output):
        self.e = e
        qt = e.qt
        self.widget = qt.QWidget()
        layout = qt.QVBoxLayout(self.widget)
        layout.setContentsMargins(0,0,0,0)
        self.mode = qt.QComboBox()
        self.mode.addItems(['Matrix', 'Definitions', 'Original full-precision text'])
        layout.addWidget(self.mode)
        self.note = qt.QLabel('Readable preview: decimal constants shortened; small terms use scientific notation. Copy/export retains original precision.')
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        self.key = qt.QLabel()
        self.key.setWordWrap(True)
        layout.addWidget(self.key)
        self.pages = qt.QStackedWidget()
        layout.addWidget(self.pages,1)
        self.matrix = qt.QTableWidget()
        self.pages.addWidget(self.matrix)
        definitions_page = qt.QWidget()
        dl = qt.QVBoxLayout(definitions_page)
        dl.setContentsMargins(0,0,0,0)
        self.definitions = qt.QTableWidget()
        self.definitions.setColumnCount(2)
        self.definitions.setHorizontalHeaderLabels(['Symbol','Expression'])
        self.definitions.setEditTriggers(qt.QAbstractItemView.NoEditTriggers)
        self.definitions.setSelectionBehavior(qt.QAbstractItemView.SelectRows)
        dl.addWidget(self.definitions,1)
        self.detail = qt.QPlainTextEdit()
        self.detail.setReadOnly(True)
        self.detail.setLineWrapMode(qt.QPlainTextEdit.WidgetWidth)
        self.detail.setMaximumHeight(150)
        dl.addWidget(self.detail)
        self.pages.addWidget(definitions_page)
        self.pages.addWidget(raw_output)
        self.mode.currentIndexChanged.connect(self.pages.setCurrentIndex)
        self.definitions.currentCellChanged.connect(self.show_definition)
        self.matrix.cellClicked.connect(self.follow_entry)
        self.data = None

    def set_data(self, data):
        self.data = data
        self.matrix.clear()
        self.matrix.setRowCount(0)
        self.definitions.setRowCount(0)
        self.detail.clear()
        self.key.clear()
        if not data:
            return
        values = data['matrix']
        is_j = data['label'] == 'J'
        rows = ['vx','vy','vz','ωx','ωy','ωz'] if is_j else ['x','y','z','w']
        cols = [f'q{i+1}' for i in range(len(values[0]))] if is_j else ['X','Y','Z','Position']
        fill_matrix(self.e.qt,self.matrix,values,rows,cols)
        self.key.setText(f"{len(data['joint_key'])} joints • hover here for the q-to-joint mapping")
        self.key.setToolTip('\n'.join(data['joint_key']))
        self.definitions.setRowCount(len(data['definitions']))
        for i, (symbol, expression) in enumerate(data['definitions']):
            for j, text in enumerate((symbol, compact_expression(expression))):
                item = self.e.qt.QTableWidgetItem(text)
                item.setToolTip(expression)
                self.definitions.setItem(i,j,item)
        self.definitions.setColumnWidth(0,65)
        self.definitions.setColumnWidth(1,400)

    def show_definition(self, row, *_):
        if self.data and 0 <= row < len(self.data['definitions']):
            symbol, expression = self.data['definitions'][row]
            self.detail.setPlainText(symbol + ' ≈ ' + compact_expression(expression))

    def follow_entry(self, row, column):
        name = self.matrix.item(row,column).toolTip()
        if self.data:
            for i, (symbol, _) in enumerate(self.data['definitions']):
                if symbol == name:
                    self.mode.setCurrentIndex(1)
                    self.definitions.setCurrentCell(i,0)
                    self.definitions.scrollToItem(self.definitions.item(i,0))
                    break
