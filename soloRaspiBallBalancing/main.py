from PyQt5 import QtGui,QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap,QFont,QImage
import sys
import matplotlib
matplotlib.use("Qt5Agg")  # 声明使用QT5
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
import cv2
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import numpy as np
import globals
from control_sys import *
from setting_PY import *
import csv


#01020304050
class VideoThread(QThread,vision_servo): # multi - inherit
    change_pixmap_signal = pyqtSignal(np.ndarray)
    def run(self):
        self.camera()
        
class message_thread(QThread,Ethernet):
    def run(self):
        self.ip_type = globals.ip_type
        self.IP_set()
        self.socket_init = self.socket_set()
        print(self.socket_init)
        conn, addr = self.socket_init.accept()
        print("{},{}".format(conn, addr))
        print('success')
        RPI_communication(conn)  # receive/send data packet
class App(QWidget,extra_fn): #multi-inherit
    def __init__(self):
        super().__init__()
        self.setWindowTitle("CHT interface ver.1")
        self.display_width = 640
        self.display_height = 480
        #create Label
        # create the label that holds the image
        self.image_label = QLabel(self)
        self.pix = QPixmap("ballBalancingNTUST.jpg")
        self.pix = self.pix.scaled(self.display_height, self.display_height)
        self.image_label.setPixmap(self.pix)
        self.image_label.resize(self.display_width, self.display_width)
        #interaction setting signal
        self.start_signal = False
        self.chart_signal = False
        self.diagnosis_signal = False

        #normal label
        self.function_label = QLabel("Functions block",self)
        self.function_label.setFont(QFont("Microsoft New Tai Lue",18))
        self.vision_label = QLabel("Ball Balancing Platform")
        self.vision_label.setFont(QFont("Microsoft New Tai Lue", 18))
        self.internet_select_label = QLabel("Choose internet connection:",self)
        self.internet_select_label.setFont(QFont("Microsoft New Tai Lue", 18))
        self.video_information = QLabel("video information",self)
        self.video_information.setFont(QFont("Microsoft New Tai Lue", 18))
        self.sys_information = QLabel("System information",self)
        self.sys_information.setFont(QFont("Microsoft New Tai Lue", 18))
        #create textbrowser
        self.sys_textBrowser = QTextBrowser(self)
        self.info_textBrowser = QTextBrowser(self)
        #create combobox
        self.internet_combobox = QComboBox(self)
        information = ["WLAN","Wifi","5G","4G"]
        self.internet_combobox.addItems(information)
        self.internet_combobox.currentIndexChanged.connect(self.internet_select)
        # create button
        self.Save_data_pushButton = QPushButton("save data(.csv)")
        self.Save_data_pushButton.clicked.connect(self.fn_block_save_data)
        self.Compare_pushButton = QPushButton("Line Chart")
        self.Compare_pushButton.clicked.connect(self.fn_block_line_chart)
        self.diagnosis_pushButton = QPushButton("diagnosis")
        self.Compare_pushButton.clicked.connect(self.fn_block_diagnosis)
        self.start_button = QPushButton("Finish setting and Start ")
        self.start_button.clicked.connect(self.start_Video_thread)
        self.end_button = QPushButton("End and Reset")
        self.end_button.clicked.connect(self.end_Video_thread)
        # create a vertical box layout and add the two labels
        image_box = QVBoxLayout()
        right_part_box = QVBoxLayout()
        start_end_box = QHBoxLayout()
        left_part_box = QVBoxLayout()
        function_box = QVBoxLayout()
        internet_select_box = QVBoxLayout()
        info_box =QVBoxLayout()
        All_box = QHBoxLayout()
        #right part interface
        image_box.addWidget(self.vision_label)
        image_box.addWidget(self.internet_select_label)
        image_box.addWidget(self.internet_combobox)
        image_box.addWidget(self.image_label)
        #start end block
        start_end_box.addWidget(self.start_button)
        start_end_box.addWidget(self.end_button)
        #
        #function
        function_box.addWidget(self.function_label)
        function_box.addWidget(self.Save_data_pushButton)
        function_box.addWidget(self.Compare_pushButton)
        function_box.addWidget(self.diagnosis_pushButton)
        # internet select
        # internet_select_box.addWidget(self.internet_select_label)
        # internet_select_box.addWidget(self.internet_combobox)
        #information box
        info_box.addWidget(self.sys_information)
        info_box.addWidget(self.sys_textBrowser)
        info_box.addWidget(self.video_information)
        info_box.addWidget(self.info_textBrowser)
        # left part interface
        # left_part_box.addLayout(image_label_box)
        left_part_box.addLayout(image_box)
        left_part_box.addLayout(start_end_box)
        #right part interface
        right_part_box.addLayout(internet_select_box)
        right_part_box.addLayout(function_box)
        right_part_box.addLayout(info_box)

        All_box.addLayout(left_part_box)
        All_box.addLayout(right_part_box)
        # set the vbox layout as the widgets layout
        self.setLayout(All_box)

        # # create the video capture thread
        # self.thread = VideoThread()
        # # connect its signal to the update_image slot
        # self.thread.change_pixmap_signal.connect(self.update_image)
        # # start the thread
        # self.thread.start()

    @pyqtSlot(np.ndarray)
    def update_image(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.image_label.setPixmap(qt_img)

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
        p = convert_to_Qt_format.scaled(self.display_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def internet_select(self): #object interaction: setting internet type
        internet_TEXT = self.internet_combobox.currentText()
        self.sys_textBrowser.append("->Internet interface select:  "+internet_TEXT)
        if internet_TEXT == "4G":
            globals.ip_type = 0
        elif internet_TEXT == "5G":
            globals.ip_type = 3
        elif internet_TEXT == "WLAN":
            globals.ip_type = 1
        elif internet_TEXT == "Wifi":
            globals.ip_type = 2

    def fn_block_save_data(self): # save data in  csv file.
        globals.save_file_signal = True
        globals.end_signal = False
        self.Save_data_pushButton.setStyleSheet("background-color: red")
        csvfile = open('CHT_platform.csv','w',newline='')
        writer = csv.writer(csvfile)
        writer.writerow(['TIME', 'X error', 'Y error','X','Y', 'thetaX', 'thetaY', 'slidingY', 'slidingY', 'error_x_dot', 'error_y_dot', 'saturation_x', 'saturation_y'])
        csvfile.close()
        time.sleep(0.001)
        #globals.start_TIME = time.time()
        self.sys_textBrowser.append("->Start record... ")

    def fn_block_line_chart(self):
        self.chart_signal = True
        # create sub window
        with open('CHT_platform.csv', newline='') as f:
            reader = csv.reader(f)
            data = list(reader)
        data = data[1:-1]
        np_data = np.asarray(data)
        print(np.shape(data))
        self.sub_window = SubWindow(np_data)
        self.sub_window.show()
        self.sys_textBrowser.append("->Start record... ")

    def fn_block_diagnosis(self):
        print("ddd")
    def start_Video_thread(self):
        if not self.start_signal:
            self.start_signal = True
            globals.end_signal = False
            globals.camera_file_open = True
            # create the video capture thread
            self.thread = VideoThread()
            #create communication thread
            self.message_thread = message_thread()

            # connect its signal to the update_image slot
            self.thread.change_pixmap_signal.connect(self.update_image)
            # start the thread
            self.thread.start()
            self.message_thread.start()

    def end_Video_thread(self):
        globals.end_signal = True
        self.start_signal = False

        self.sys_textBrowser.append("ready to stop...")
        time.sleep(0.05)
        self.thread.terminate()

        #self.message_thread.socket_init.shutdown(socket.SHUT_RDWR)
        self.message_thread.socket_init.close()
        self.message_thread.terminate()
        globals.save_file_signal = False
        #self.image_label.setPixmap(self.pix)
        self.Save_data_pushButton.setStyleSheet("background-color: light gray")
        print("END video.")

class SubWindow(QDialog):
    def __init__(self,data):
        super().__init__()
        self.setFixedSize(800, 600)
        self.figure = plt.figure() #圖片
        self.canvas = FigureCanvas(self.figure) #圖片放在畫布上
    # 以上兩行是將matplotlib 帶進qt的接口
        self.toolbar = NavigationToolbar(self.canvas, self)  # 加入matplotlib 的 toolbar
        layout = QVBoxLayout()  # 在視窗上增加放toolbar和畫布的地方
        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        self.data = data
        self.plot()
    def plot(self):
        ax = self.figure.add_subplot(111)
        #ax2 = self.figure.add_subplot(211)
        ax.plot(self.data[:,0],self.data[:,1],'-o')
        #ax2.plot(self.data[:, 0], self.data[:, 2], '-o')
        self.canvas.draw()

if __name__ == "__main__":
    globals.global_variable_init()
    app = QApplication(sys.argv)
    a = App()
    a.show()
    sys.exit(app.exec_())
