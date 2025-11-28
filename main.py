
#System
import sys
import os.path
import time
import collections

#Interface
from PyQt5 import QtCore, QtGui, QtWidgets, uic
layout_form = uic.loadUiType("designer.ui")[0]
#CurvePlot
from guiqwt.plot import CurveWidget
from guiqwt.builder import make
#USB communication
import serial
from serial.tools import list_ports
#Others
import pickle
import numpy as np

class Plot():
    """ Plot Class to show the temperature as a function of time in the screen"""
    def __init__(self, widget_plot, toolbar):
        """ Initializes the plot Widget and adds a item"""
        self.widget_plot = widget_plot
        self.toolbar = toolbar
        self.data =  [0,0] #Initial data, arbitrary
        #Curve widget
        self.plotWidget = CurveWidget(self.widget_plot, xlabel=('Time'),
                                                ylabel=('Counts'), xunit=('us'))
        #self.plotWidget.add_toolbar(self.toolbar, "default") #Removed ToolBar in this version for now
        #self.plotWidget.register_all_curve_tools()
        #Curve item
        self.item = make.curve(np.asarray(range(len(self.data))),np.asarray(self.data))
        #Curve plot
        self.plot = self.plotWidget.get_plot()
        self.plot.add_item(self.item)
        self.plotWidget.resize(self.widget_plot.size())
        self.plotWidget.show()

    def setData(self, data):
        """ Change the data of the item and replot """
        self.item.set_data(np.asarray(data[0]),np.asarray(data[1]))
        self.plot.do_autoscale()
        self.plot.replot()

class Worker_TCSPC(QtCore.QObject):
    """ Thread doing the backend """
    #Attributes
    parent = None
    paired = False
    started = False
    ser = None

    #Signals
    updateData = QtCore.pyqtSignal(list)
    emitError = QtCore.pyqtSignal(str)

    def loopWork(self):
        """ Checks if the device is paired and atualize the device's list """
        while 1:
            if self.paired == True and self.started == True:
                time.sleep(1)
                self.get()
            else:
                time.sleep(1)


    def change_HRTIM_preesc(self, preesc):
        try:
            text = "hrtim_preesc " + preesc + "\n"
            self.ser.write(text.encode())
            print("HRTIM Preescale changed")
        except Exception as erro:
            errorMessage = str(erro.args[0])
            print("Error on changing HRTIM: ", errorMessage)

    def change_HRTIM_cycles(self, cycles):
        try:
            text = "trigger_cycle " + str(cycles) + "\n"
            self.ser.write(text.encode())
            print("Number of trigger cycles changed")
        except Exception as erro:
            errorMessage = str(erro.args[0])
            print("Error on changing Trigger Cycles: ", errorMessage)

            
    def start(self):
        try:
            self.ser.write("start_hrtim\n".encode())
            print("Counter started")
        except Exception as erro:
            errorMessage = str(erro.args[0])
            print("Error on start: ", errorMessage)

    def stop(self):
        try:
            self.ser.write("stop_hrtim\n".encode())
            print("Counter stoped")
        except Exception as erro:
            errorMessage = str(erro.args[0])
            print("Error on stop: ", errorMessage)
            
    def get(self):
        """ Read and process the data sent by the device """
        try:
            self.ser.write("get_hrtim\n".encode())
            
            expected = 65535 * 4
            raw = self.ser.read(expected)

            #while len(raw) < expected:
            #    raw += self.ser.read(expected - len(raw))
            
            data = np.frombuffer(raw, dtype='<u4')  # '<u4' = little-endian uint32
            if self.started == True: self.updateData.emit(list(data[1:]))

        except Exception as erro:
            errorMessage = str(erro.args[0])
            print("Error on acquiring: ", errorMessage)

class Worker_CNT(QtCore.QObject):
    """ Thread doing the backend """
    #Attributes
    parent = None
    paired = False
    started = False
    ser = None

    #Signals
    counts =  collections.deque(maxlen=100)
    int_time = 1

    updateData = QtCore.pyqtSignal(list)
    emitError = QtCore.pyqtSignal(str)

    def loopWork(self):
        """ Checks if the device is paired and atualize the device's list """
        while 1:
            if self.paired == True and self.started == True:
                self.get()
                #print("in")
            else:
                time.sleep(0.1)
                #print("out")


    def change_CNT_delay(self, preesc):
        try:
            text = "cnt_preesc " + str(preesc) + "\n"
            self.ser.write(text.encode())
            self.int_time = preesc
            print("COUNTER delay changed to " + str(preesc))
        except Exception as erro:
            errorMessage = str(erro.args[0])
            print("Error on changing COUNTER delay: ", errorMessage)
            
    def start(self):
        try:
            self.clearPlot()
            #self.change_CNT_delay(self.parent.CNT_spinBox_intTime.value()) #attention
            self.ser.write("start_cnt\n".encode())
            time.sleep(0.5)
            self.ser.write("get_cnt\n".encode())
            time.sleep(0.5)

            print("Counter started")
        except Exception as erro:
            errorMessage = str(erro.args[0])
            print("Error on start: ", errorMessage)

    def stop(self):
        try:
            self.ser.write("stop_cnt\n".encode())
            print("Counter stoped")
        except Exception as erro:
            errorMessage = str(erro.args[0])
            print("Error on stop: ", errorMessage)
    
    def clearPlot(self):
        self.counts =  collections.deque(maxlen=100)

    def get(self):
        """ Read and process the data sent by the device """
        try:
            #self.ser.reset_input_buffer()
            #self.ser.reset_output_buffer()
            #self.ser.write("get_cnt".encode())
            #self.ser.flush()
            #time.sleep(self.int_time/1000)
            #expected = 4
            raw = self.ser.readline()
            #print(raw)

            #data = np.frombuffer(raw, dtype='<u4')[0]  # '<u4' = little-endian uint32
            self.counts.append(int(raw))
            if self.started == True: self.updateData.emit(list(self.counts))
            
        

        except Exception as erro:
            errorMessage = str(erro.args[0])
            print("Error on acquiring: ", errorMessage)


class Main(QtWidgets.QMainWindow, layout_form):
    """ Thread responsible by the user interface """
    def __init__(self, parent=None):
        QtWidgets.QWidget.__init__(self,parent)
        self.setupUi(self)
        
        #Serial Related
        self.updateListPorts()
        self.plot_time_preesc = 1/480
        
        #Plot
        self.mainToolbar = self.addToolBar("Plot")
        self.plot = Plot(self.widget_plot, self.mainToolbar)

        ##### WIDGET ACTIONS #####
        #~Buttons
        self.pushButton_connect.clicked.connect(self.pair_unpair)
        self.pushButton_start.clicked.connect(self.start_stop)
        self.pushButton_update.clicked.connect(self.updateListPorts)
        self.pushButton_clearPlot.clicked.connect(self.clearPlot)
        
        self.pushButton_saveTxt.clicked.connect(self.saveFileTxt)
        self.pushButton_savePng.clicked.connect(self.saveFilePng)
        
        self.TCSPC_radioButton_HRTIMprresc0.clicked.connect(lambda: self.changeHRTIMpreesc(self.TCSPC_radioButton_HRTIMprresc0))
        self.TCSPC_radioButton_HRTIMprresc1.clicked.connect(lambda: self.changeHRTIMpreesc(self.TCSPC_radioButton_HRTIMprresc1))
        self.TCSPC_radioButton_HRTIMprresc2.clicked.connect(lambda: self.changeHRTIMpreesc(self.TCSPC_radioButton_HRTIMprresc2))
        self.TCSPC_radioButton_HRTIMprresc3.clicked.connect(lambda: self.changeHRTIMpreesc(self.TCSPC_radioButton_HRTIMprresc3))
        self.TCSPC_spinBox_triggerCycles.valueChanged.connect(lambda: self.changeHRTIMcycles(self.TCSPC_spinBox_triggerCycles.value()))
                
        self.CNT_spinBox_intTime.valueChanged.connect(lambda: self.changeCNTdelay(self.CNT_spinBox_intTime.value()))

        #Initial Settings
        self.get_Worker(self.tabWidget.currentIndex())
        self.tabWidget.currentChanged.connect(lambda: self.change_Worker(self.tabWidget.currentIndex()))

    ##### FUNCTIONS #####
    def clearPlot(self):
        self.plot.setData(([0],[0]))
        self.routine.clearPlot()

    def get_Worker(self, tab_index):
        self.current_tab = self.tabWidget.tabText(tab_index)
        print(self.current_tab)
        if self.current_tab == "Counter":
            self.routine = Worker_CNT()
        
        elif self.current_tab == "TCSPC":
            self.routine = Worker_TCSPC()

    def change_Worker(self, tab_index):
        current_ser = self.routine.ser
        self.get_Worker(tab_index)
        self.routine.ser = current_ser
    
    ##INTERFACE
    def changeCNTdelay(self, delay):
        self.routine.change_CNT_delay(delay)

    def changeHRTIMcycles(self, cycles):
        self.routine.change_HRTIM_cycles(cycles)
         
    def changeHRTIMpreesc(self, button):
        gateTime = button.text()
        
        if gateTime == '2.08 ns': preesc = '0'
        elif gateTime == '4.17 ns': preesc = '1'
        elif gateTime == '8.33 ns': preesc = '2'
        elif gateTime == '16.66 ns': preesc = '3'

        self.routine.change_HRTIM_preesc(preesc)
        self.plot_time_preesc = 1/(480/2**(int(preesc)))
        print(self.plot_time_preesc)
    
    def showError(self, error):
        self.statusBar().showMessage(error)

    def updateData(self, data):
        if self.current_tab == "TCSPC":
            data= np.array(data)
            non_zero_data = data[np.nonzero(data)][:-3]
            self.TCSPC_lcdNumber_counts.display(non_zero_data.mean())
            self.plot.setData(([x*self.plot_time_preesc for x in range(len(non_zero_data))], non_zero_data))

        elif self.current_tab == 'Counter':
            data= np.array(data)

            self.CNT_lcdNumber_counts.display(data[-1])
            self.plot.setData((range(len(data)), data))


    #SERIAL USB COMMUNICATION
    def pair_unpair(self):
        if  self.pushButton_connect.isChecked() == True:
            if self.comboBox_port.currentText() != 'None':
                portName = self.comboBox_port.currentText().split('-')[0]
                boudRate = self.lineEdit_boud.text()
                try:
                    self.get_Worker(self.tabWidget.currentIndex())
                    self.routine.ser = serial.Serial('/dev/'+portName, int(boudRate), timeout=5, write_timeout=5)
                    self.routine.paired = True

                    time.sleep(0.5)
                    
                    self.start_Thread()
                    print("Device paired")
                    self.statusBar().showMessage("Device paired successfully")
                    self.pushButton_connect.setText("Unpair")
                    
                    self.pushButton_update.setDisabled(True)
                    self.pushButton_start.setDisabled(False)
                    self.lineEdit_boud.setDisabled(True)
                    self.comboBox_port.setDisabled(True)
                    self.TCSPC_widget_chooseTimer.setDisabled(False)
                    self.TCSPC_widget_timeGate.setDisabled(False)
                    self.CNT_spinBox_intTime.setDisabled(False)
                    self.tabWidget.setDisabled(False)
                    self.widget_trigger_cycles.setDisabled(False)
                    self.TCSPC_spinBox_triggerCycles.setDisabled(False)

                except Exception as erro:
                    self.pushButton_connect.setChecked(False)
                    errorMessage =  erro.args[0]
                    self.statusBar().showMessage("Error on pairing: " + str(errorMessage))
        else:
            self.routine.paired = False
            time.sleep(0.5)
            self.routine.ser.close()
            
            print("Device unpaired")
            self.quit_Thread()
            self.statusBar().showMessage("Device unpaired")
            self.pushButton_connect.setText("Pair")
            
            self.pushButton_update.setDisabled(False)
            self.pushButton_start.setDisabled(True)
            self.lineEdit_boud.setDisabled(False)
            self.comboBox_port.setDisabled(False)
            self.TCSPC_widget_chooseTimer.setDisabled(True)
            self.TCSPC_widget_timeGate.setDisabled(True)
            self.CNT_spinBox_intTime.setDisabled(True)
            self.tabWidget.setDisabled(True)
            self.widget_trigger_cycles.setDisabled(True)
            self.TCSPC_spinBox_triggerCycles.setDisabled(True)


    def start_Thread(self):
        self.routine.parent = self
        self.executionThread = QtCore.QThread(parent=self)
        self.routine.moveToThread(self.executionThread)
        self.executionThread.started.connect(self.routine.loopWork)
        self.executionThread.start()

        #Thread Signals
        self.routine.updateData.connect(self.updateData)
        self.routine.emitError.connect(self.showError)

    def quit_Thread(self):
        self.executionThread.quit()

    ## START STOP ACQUISITION
    def start_stop(self):
        #conectar ou desconectar, dependendo do estado do botão
        if  self.pushButton_start.isChecked() == True:
            try:
                self.routine.start()
                self.routine.started = True
                # Routine
                self.statusBar().showMessage("Acquisiton started")
                self.pushButton_start.setText("Stop")
                
                self.pushButton_connect.setDisabled(True)   
                self.TCSPC_widget_chooseTimer.setDisabled(True)
                self.TCSPC_widget_timeGate.setDisabled(True)
                self.CNT_spinBox_intTime.setDisabled(True)
                self.widget_trigger_cycles.setDisabled(True)


            except Exception as erro:
                self.pushButton_start.setChecked(False)
                errorMessage =  str(erro.args[0])
                self.statusBar().showMessage("Error on starting/stopping: " + errorMessage)
        else:
            self.routine.stop()
            self.routine.started = False
            self.pushButton_start.setDisabled(True)
            time.sleep(5)
            self.pushButton_start.setDisabled(False)
            # Routine
            self.statusBar().showMessage("Acquisiton stoped")
            self.pushButton_start.setText("Start")
            
            self.pushButton_connect.setDisabled(False)
            self.TCSPC_widget_chooseTimer.setDisabled(False)
            self.TCSPC_widget_timeGate.setDisabled(False)
            self.CNT_spinBox_intTime.setDisabled(False)
            self.tabWidget.setDisabled(False)
            self.widget_trigger_cycles.setDisabled(False)
    
    
    def updateListPorts(self):
        self.comboBox_port.clear()
        self.listPorts = serial.tools.list_ports.comports()
        
        for port in self.listPorts:
            if not port.description == 'n/a':
                self.comboBox_port.addItem(port.name + '-' + port.description)

    def saveFileTxt(self):
        fileName, _ = QtWidgets.QFileDialog.getSaveFileName(self,"Select File Path", '', "Text (*.txt)")
        if fileName:
            #Arquivo completo
            try:
                np.savetxt(fileName+'.txt', self.plot.item.get_data())
                print("TXT saved")
            except Exception as erro:
                errorMessage =  str(erro.args[0])
                print("Error on saving txt: " + errorMessage)

    def saveFilePng(self):
        fileName, _ = QtWidgets.QFileDialog.getSaveFileName(self,"Select File Path", '', "Image (*.png)")
        if fileName:
            #Arquivo completo
            try:
                self.plot.plot.save_widget(fileName+'.png')
                print("PNG saved")
            except Exception as erro:
                errorMessage =  str(erro.args[0])
                print("Error on saving png: " + errorMessage)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = Main()
    window.show()
    sys.exit(app.exec_())
