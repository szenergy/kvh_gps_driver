#!/usr/bin/env python
from __future__ import print_function
import rospy
import std_msgs.msg as rosmsg
import nav_msgs.msg as navmsg
import sensor_msgs.msg as senmsg
import pyqtgraph as pg
import pyqtgraph.Qt as qtgqt
import numpy as np
import pyqtgraph.dockarea as darea
#import threading

#lck = threading.Lock()

class PlotHandler(object):
    def __init__(self, gps):
        super(PlotHandler, self).__init__()
        self.gps = gps
        self.app = qtgqt.QtGui.QApplication([])
        self.area = darea.DockArea()
        self.win = qtgqt.QtGui.QMainWindow()

    def initializePlot(self):
        self.win.setWindowTitle("szenergy gps visualizer")
        self.win.resize(1200,600)
        self.win.setCentralWidget(self.area)
        self.dleft1 = darea.Dock("left 1", size = (1,1))  # give this dock minimum possible size
        self.dleft2 = darea.Dock("left 2", size = (500,400)) # size is only a suggestion
        self.dright1 = darea.Dock("right 1", size=(500,200))
        self.dright2 = darea.Dock("right 2", size=(500,200))
        self.area.addDock(self.dleft1, "left")
        self.area.addDock(self.dleft2, "bottom", self.dleft1)
        self.area.addDock(self.dright1, "right")
        self.area.addDock(self.dright2, "below", self.dright1)   ## place dright2 at top edge of dright1
        self.area.moveDock(self.dright2, "below", self.dright1)
        self.wleft1 = pg.LayoutWidget()
        self.mainlabel = qtgqt.QtGui.QLabel("No data\n\n")
        self.seclabel = qtgqt.QtGui.QLabel("No data\n\n")
        self.saveBtn = qtgqt.QtGui.QPushButton("Save dock state")
        self.restoreBtn = qtgqt.QtGui.QPushButton("Restore dock state")
        self.rndBtn = qtgqt.QtGui.QPushButton("Random")
        self.restoreBtn.setEnabled(False)
        self.wleft1.addWidget(self.mainlabel, row=0, col=0)
        self.wleft1.addWidget(self.seclabel, row=0, col=1)
        self.wleft1.addWidget(self.saveBtn, row=1, col=0)
        self.wleft1.addWidget(self.restoreBtn, row=2, col=0)
        self.wleft1.addWidget(self.rndBtn, row=3, col=2)
        self.wleft1.setStyleSheet("background-color: rgb(40, 44, 52); color: rgb(171, 178, 191);")
        self.dleft1.setStyleSheet("background-color: rgb(18, 20, 23);")
        self.mainlabel.setStyleSheet("font: 30pt; background-color: rgb(44, 48, 56)")
        self.dleft1.addWidget(self.wleft1)
        self.state = None
        self.wleft2 = pg.PlotWidget(title="Plot left 2 (bottom)")
        self.plot_left2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(6,106,166,255))
        self.wleft2.showGrid(x=True, y=True)
        self.wleft2.addItem(self.plot_left2)
        self.dleft2.addWidget(self.wleft2)
        self.rndBtn.clicked.connect(self.rnd)
        self.saveBtn.clicked.connect(self.save)
        self.restoreBtn.clicked.connect(self.load)
        self.wright1 = pg.PlotWidget(title="Plot right 1")
        self.wright1.plot(np.random.normal(size=20))
        self.wright1.showGrid(x=True, y=True)
        self.dright1.addWidget(self.wright1)
        self.wright2 = pg.PlotWidget(title="Plot right")
        self.plot_right2 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(200,66,66,255))
        self.wright2.showGrid(x=True, y=True)
        self.wright2.addItem(self.plot_right2)
        self.dright2.addWidget(self.wright2)

        self.win.show()

    def updateFirstPlot(self):
        self.plot_left2.addPoints(self.gps.gps_data_x, self.gps.gps_data_y)
        # buffer only 200 points (include the starting point as a reference) # todo this can be done better
        if(len(self.plot_left2.data)) > 200:
            self.plot_left2.data = np.delete(self.plot_left2.data, 1)
    def updateSecondPlot(self):
        self.plot_right2.addPoints(self.gps.odom_data_x[0], self.gps.odom_data_y)

    def updateMainLabel(self):
        self.mainlabel.setText("x: %.6f\ny: %.6f" %(self.gps.gps_data_x, self.gps.gps_data_y))

    def rnd(self):
        print("hello")
        self.seclabel.setText("ox: %.6f\noy: %.6f" %(self.gps.odom_data_x, self.gps.odom_data_y))
        #self.mainlabel.setText("hello")

    def save(self):
        self.state = self.area.saveState()
        self.restoreBtn.setEnabled(True)

    def load(self):
        self.area.restoreState(self.state)

class GpsSubscriber(object):
    def __init__(self):
        self.odom_data_x = None
        self.odom_data_y = None
        self.gps_data_x = None
        self.gps_data_y = None
        self.utm_zone = None
        rospy.Subscriber('odom', navmsg.Odometry, self.odometryCallBack)
        rospy.Subscriber("gps/fix", senmsg.NavSatFix, self.gpsFixCallBack)
        rospy.Subscriber("gps/utmzone", rosmsg.String, self.utmCallback)

    def utmCallback(self, msg):
        self.utm_zone = msg.data
        #print("UTM zone: %s" %  (msg.data))

    def odometryCallBack(self, msg):
        #lck.acquire()
        self.odom_data_x = np.array([msg.pose.pose.position.x])
        self.odom_data_y = np.array([msg.pose.pose.position.y])
        #print("odom: %.4f %.4f " % (msg.pose.pose.position.x, msg.pose.pose.position.y))
        #lck.release()

    def gpsFixCallBack(self, msg):
        self.gps_data_x = np.array([msg.latitude])
        self.gps_data_y = np.array([msg.longitude])
        #print("gps: %.4f %.4f " % (msg.latitude, msg.longitude))


if __name__ == "__main__":
    import sys
    print(__file__, "- gps message reader started ... ")
    # In ROS, nodes are uniquely named. The anonymous=True flag means that rospy will choose a unique
    # name for our "listener" node so that multiple listeners can run simultaneously.
    rospy.init_node("gps_visualizer", anonymous=True)
    gpsSub = GpsSubscriber()
    ph = PlotHandler(gpsSub)
    ph.initializePlot()
    timer1 = qtgqt.QtCore.QTimer()
    timer1.timeout.connect(ph.updateSecondPlot)
    timer1.start(30)
    timer2 = qtgqt.QtCore.QTimer()
    timer2.timeout.connect(ph.updateFirstPlot)
    timer2.start(30)
    timer3 = qtgqt.QtCore.QTimer()
    timer3.timeout.connect(ph.updateMainLabel)
    timer3.start(30)


    if (sys.flags.interactive != 1) or not hasattr(qtgqt.QtCore, "PYQT_VERSION"):
        qtgqt.QtGui.QApplication.instance().exec_()
