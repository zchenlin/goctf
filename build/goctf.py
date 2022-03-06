from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from subprocess import Popen, PIPE, STDOUT
import sys
import os
import subprocess
import shlex

class MainWindow(QWidget):


    def __init__(self,parent=None):
        super().__init__(parent)
 
        flo = QFormLayout()

        self.b1 = QPushButton('Locate Micrographs Directory')
        self.b1.clicked.connect(self.open_file)

        self.b3 = QLineEdit()

        flo.addWidget(self.b1)
        flo.addRow(self.b3)

        self.e11 = QLineEdit()
        flo.addRow('Output file name', self.e11)

        self.e1 = QLineEdit('2.7')
        flo.addRow('Cs(mm)', self.e1)
        
        self.e2 = QComboBox()
        self.e2.addItems(['200','300'])
        flo.addRow('Voltage(kV)', self.e2)
        self.e2.currentTextChanged.connect(self.text_changed)

        self.e3 = QLineEdit('0.1')
        flo.addRow('Amplitude contrast', self.e3)

        self.e4 = QLineEdit()
        flo.addRow('Pixel size(A)', self.e4)

        self.e5 = QLineEdit('512')
        flo.addRow('FFT box size', self.e5)

        self.e6 = QLineEdit('30')
        flo.addRow('Min Res(A)', self.e6)

        self.e7 = QLineEdit('8')
        flo.addRow('Max Res', self.e7)

        self.e8 = QLineEdit('50000')
        flo.addRow('Min defcous(A)', self.e8)

        self.e9 = QLineEdit('500000')
        flo.addRow('Max defcous(A)', self.e9)

        self.e10 = QLineEdit('500')
        flo.addRow('Defcous step size(A)', self.e10)

        self.e12 = QComboBox()
        self.e12.addItems(['No','Yes'])
        flo.addRow("Particle refinement", self.e12)

        self.b2 = QPushButton('Run goCFT')
        self.b2.clicked.connect(self.click_b2)
        flo.addWidget(self.b2)

        self.setLayout(flo)
        self.setWindowTitle('goCTF')
        self.setFixedWidth(500)
        self.setFixedHeight(500)
        self.show()

        sys.exit(app.exec_())


    def open_file(self):
        path = QFileDialog.getOpenFileName(self, 'Open a file', '','All Files (*.*)')
        if path != ('', ''):
            print(path[0])
        self.b3.setText('{}'.format(path[0].split('/')[-1]))


    def click_b2(self):
        input_para = self.b3.text()+"\n"+self.e11.text()+"\n"+self.e4.text()+"\n"+self.e2.currentText()+"\n"+self.e1.text()+"\n"+self.e3.text()+"\n"+self.e5.text()+"\n"+self.e6.text()+"\n"+self.e7.text()+"\n"+self.e8.text()+"\n"+self.e9.text()+"\n"+self.e10.text()+"\n"+self.e12.currentText()
        subprocess.run(['./goctf'],input=input_para.encode())


    def text_changed(self, s):
        if s == '200': 
            self.e3.setText('0.1')
        else: 
            self.e3.setText('0.07')

if __name__ == "__main__":
    app = QApplication(sys.argv)
    Win = MainWindow()
    Win.show()
    sys.exit( app.exec_() )