import sys
from PyQt4 import QtGui, QtCore
from matplotlib.backends.backend_qt4agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt4agg import NavigationToolbar2QT as NavigationToolbar
import matplotlib.pyplot as plt
from matplotlib import style
from collections import deque
import re
style.use('ggplot')
import serial
import time
import random



class Window(QtGui.QMainWindow):

    def __init__(self):
        super(Window,self).__init__()
# some variable

        self.dati = []
        self.Vepm = deque(range(0, 200, 2), 100)
        self.Tepm = deque(range(0, 100, 1), 100)
        self.Vipm = deque(range(0, 300, 3), 100)
        self.Tipm = deque(range(0, 800, 8), 100)

        self.loop = 0
        self.flag = 0
        self.tempaccio = deque([0] * 100, 100)
        self.clock = 5;

# set geometry of the window
        self.setGeometry(50,50,1250,650)
        self.setWindowTitle("Shielding Platform Control Interface")
        self.setWindowIcon(QtGui.QIcon('stormy.jpg'))
# make the menu
        comunica = QtGui.QAction("&Open Serial", self)
        comunica.setShortcut("Ctrl+G")
        comunica.setStatusTip('Start comunicating with STM32')
        comunica.triggered.connect(self.start_serial)

        basta_parlare = QtGui.QAction("&Close Serial", self)
        basta_parlare.setShortcut("Ctrl+H")
        basta_parlare.setStatusTip('Stop comunicating with STM32')
        basta_parlare.triggered.connect(self.stop_serial)

        chiudi = QtGui.QAction("&Exit", self)
        chiudi.setShortcut("Ctrl+Q")
        chiudi.setStatusTip('Leave the app')
        chiudi.triggered.connect(self.close_application)
        #
        saveFile = QtGui.QAction("&Save File", self)
        saveFile.setShortcut("Ctrl + S")
        saveFile.setStatusTip('Save File')
        saveFile.triggered.connect(self.file_save)

        self.statusBar()
# start a timer based acquisition
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tempo)
        self.timer.start(self.clock)

        mainMenu = self.menuBar()
        fileMenu = mainMenu.addMenu('&File')
        fileMenu.addAction(comunica)
        fileMenu.addAction(basta_parlare)
        fileMenu.addAction(saveFile)
        fileMenu.addAction(chiudi)

        self.home()

    def home(self):

        self.main_widget = Main_widget(self)
        self.setCentralWidget(self.main_widget)

        #set title and axis labels on the 4 plots
        self.main_widget.EPM.ax1.set_title('EPM velocity')
        self.main_widget.EPM.ax1.set_xlabel('time [ms]')
        self.main_widget.EPM.ax1.set_ylabel('Velocity [RPM]')

        self.main_widget.EPM.ax2.set_title('EPM Torque')
        self.main_widget.EPM.ax2.set_xlabel('time[ms]')
        self.main_widget.EPM.ax2.set_ylabel('Torque [mNm]')

        self.main_widget.IPM.ax1.set_title('IPM velocity')
        self.main_widget.IPM.ax1.set_xlabel('time[ms]')
        self.main_widget.IPM.ax1.set_ylabel('Velocity [RPM]')

        self.main_widget.IPM.ax2.set_title('EPM Torque')
        self.main_widget.IPM.ax2.set_xlabel('time[ms]')
        self.main_widget.IPM.ax2.set_ylabel('Torque [mNm]')

        self.show()

    def file_save(self):

        name = QtGui.QFileDialog.getSaveFileName(self,'Save File')
        file = open(name,'w')
        title = "Vepm\tTepm\tVipm\tTipm\n\n"
        file.write(title)
        text = ""
        for i in range(len(self.Vepm)):
            text += str(self.Vepm[i]) + "\t" + str(self.Tepm[i]) + "\t" + str(self.Vipm[i]) + "\t" + str(self.Tipm[i]) + "\n"
        file.write(text)
        print(text)
        file.close()

    def start_serial(self):
        self.result = 'COM9'
        self.s = serial.Serial(self.result,9600)
        # self.s.open()
        # self.s.write("8")
        self.flag = 1

    def stop_serial(self,s):
        self.s.close()
        self.flag = 0

    def close_application(self):
        choice = QtGui.QMessageBox.question(self,'Extract!', "Wanna get the duck out?",
                                            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
        if choice == QtGui.QMessageBox.Yes:
            print("Extracting naaaaaaaaaaaaaaow!")
            sys.exit()
        else:
            pass

    def tempo(self):
        # self.dati = self.s.read(2)
        if self.flag:
            self.dati=self.s.readline()
            # print(self.dati)

            data = re.findall(r"[-+]?\d*\.\d+|\d+", self.dati)
            print(data)
            # print(type(data))

            self.Vepm.append(float(data[0]))
            self.Tepm.append(float(data[1]))
            self.Vipm.append(float(data[2]))
            self.Tipm.append(float(data[3]))

            # print(self.Vepm)
            # print(type(self.Vepm))
            # [random.random() for i in range(10)]
            self.main_widget.EPM.plot(self.tempaccio,self.Vepm, 1)
            self.main_widget.EPM.plot(self.tempaccio,self.Tepm, 2)
            self.main_widget.IPM.plot(self.tempaccio,self.Vipm, 1)
            self.main_widget.IPM.plot(self.tempaccio,self.Tipm, 2)

            # self.main_widget.EPM.plot(self.tempaccio, self.Tepm, self.main_widget.EPM.plot_torque,
            #                           self.main_widget.EPM.ax2, self.main_widget.EPM.canvas2)
            # self.main_widget.EPM.plot(self.tempaccio,self.Tepm)
            # print(self.loop)
            self.loop += 1
            self.main_widget.loopcicle.display(self.loop)
            self.tempaccio.append(self.tempaccio[-1]+self.clock)

            # print(self.tempaccio)

        else:
            print('comunication blocked\n')



class Magnet(QtGui.QWidget):
    def __init__(self, id, parent=None):
        super(Magnet, self).__init__(parent)
        self.parent = parent
        self.id = id
        # two figures instance to plot on Velocity and Torque
        self.figure1 = plt.figure()
        self.ax1 = self.figure1.add_subplot(111)
        self.figure2 = plt.figure()
        self.ax2 = self.figure2.add_subplot(111)
        self.ax1.hold(False)
        self.ax2.hold(False)

        self.canvas1 = FigureCanvas(self.figure1)
        self.canvas2 = FigureCanvas(self.figure2)
        self.toolbar1 = NavigationToolbar(self.canvas1, self)
        self.toolbar2 = NavigationToolbar(self.canvas2,self)

        self.PWM = PWM_slider(self)

        # set the layout
        layout = QtGui.QVBoxLayout()
        layout.addWidget(self.toolbar1)
        layout.addWidget(self.canvas1)
        layout.addWidget(self.PWM)
        layout.addWidget(self.toolbar2)
        layout.addWidget(self.canvas2)
        self.setLayout(layout)

        # ax = self.figure1.add_subplot(111)
        # ax.hold(False)
        self.plot_velocity = self.ax1.plot([], '*-')[0]
        self.plot_torque = self.ax2.plot([],'*-')[0]

    def plot(self, tempaccio, data, whichplot):

        if whichplot == 1:
            # data = [random.random() for i in range(10)]
            self.plot_velocity.set_ydata(data)
            self.plot_velocity.set_xdata(tempaccio)
            self.ax1.set_xlim(tempaccio[0],tempaccio[-1])
            self.ax1.set_ylim(min(data)*.8,max(data)*1.2)
            self.canvas1.draw()
        else:
            if whichplot == 2:
                self.plot_torque.set_ydata(data)
                self.plot_torque.set_xdata(tempaccio)
                self.ax2.set_xlim(tempaccio[0], tempaccio[-1])
                self.ax2.set_ylim(min(data) * .8, max(data) * 1.2)
                self.canvas2.draw()
            else:
                print("Specify better your plot")

class PWM_slider (QtGui.QWidget):
    def __init__(self, parent=None):
        super(PWM_slider,self).__init__(parent)
        self.parent = parent

        # self.button = QtGui.QPushButton('Plot')
        # self.button.clicked.connect(self.plot)

        self.lcd = QtGui.QLCDNumber(self)
        self.sld = QtGui.QSlider(QtCore.Qt.Horizontal, self)
        self.sld.valueChanged.connect(self.lcd.display)
        self.sld.valueChanged.connect(self.parla)
        # print(self.sld.valueChanged[int])
        # ciccia = self.sld.value

        layout = QtGui.QHBoxLayout()
        layout.addWidget(self.lcd)
        layout.addWidget(self.sld)
        self.setLayout(layout)

    def parla(self):
        # print('1' + str(self.sld.value()))
        print(str(self.sld.value()))
        if self.parent.parent.parent.flag:

            if self.sld.value() < 10:
                print(str(self.parent.id) + '0' + str(self.sld.value()))
                self.parent.parent.parent.s.write(str(self.parent.id) + '0' + str(self.sld.value()))
                time.sleep(0.08)

            else:
                print(str(self.parent.id) + str(self.sld.value()))
                self.parent.parent.parent.s.write(str(self.parent.id) + str(self.sld.value()))
                time.sleep(0.08)


            # if (self.s.isOpen()):
            #     self.s.write('1')
            #     self.s.write(value)
            # else:
            #     pass


class Main_widget(QtGui.QWidget):
    def __init__(self, parent=None):
        super(Main_widget, self).__init__(parent)
        self.parent = parent

        self.EPM = Magnet(1,self)
        self.IPM = Magnet(2,self)
        self.loopcicle = QtGui.QLCDNumber(self)

        layout = QtGui.QHBoxLayout()
        layout.addWidget(self.EPM)
        layout.addWidget(self.loopcicle)
        layout.addWidget(self.IPM)

        self.setLayout(layout)

def run():
    app = QtGui.QApplication(sys.argv)
    GUI = Window()
    sys.exit(app.exec_())

run()




