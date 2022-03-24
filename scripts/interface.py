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

        self.b1 = QPushButton('Locate particles.star Directory')
        self.b1.clicked.connect(self.open_file)

        self.b3 = QLineEdit()

        flo.addWidget(self.b1)
        flo.addRow(self.b3)

        

        self.b2 = QPushButton('Run')
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
        #input_para = self.b3.text()+"\n"+self.e11.text()+"\n"+self.e4.text()+"\n"+self.e2.currentText()+"\n"+self.e1.text()+"\n"+self.e3.text()+"\n"+self.e5.text()+"\n"+self.e6.text()+"\n"+self.e7.text()+"\n"+self.e8.text()+"\n"+self.e9.text()+"\n"+self.e10.text()+"\n"+self.e12.currentText()
        #subprocess.run('python3 particles_split.py -f ' + self.b3.text())
        os.system('python3 particles_split.py -f ' + self.b3.text())



if __name__ == "__main__":
    app = QApplication(sys.argv)
    Win = MainWindow()
    Win.show()
    sys.exit( app.exec_() )