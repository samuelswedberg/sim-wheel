# Form implementation generated from reading ui file 'gui/wheelbase.ui'
#
# Created by: PyQt6 UI code generator 6.4.2
#
# WARNING: Any manual changes made to this file will be lost when pyuic6 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt6 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1150, 698)
        self.centralwidget = QtWidgets.QWidget(parent=MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.frame_settings = QtWidgets.QFrame(parent=self.centralwidget)
        self.frame_settings.setGeometry(QtCore.QRect(290, 100, 831, 371))
        self.frame_settings.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_settings.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_settings.setObjectName("frame_settings")
        self.frame_ffbsettings = QtWidgets.QFrame(parent=self.frame_settings)
        self.frame_ffbsettings.setGeometry(QtCore.QRect(10, 10, 381, 161))
        self.frame_ffbsettings.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_ffbsettings.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_ffbsettings.setObjectName("frame_ffbsettings")
        self.label_ffb = QtWidgets.QLabel(parent=self.frame_ffbsettings)
        self.label_ffb.setGeometry(QtCore.QRect(10, 60, 121, 16))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_ffb.setFont(font)
        self.label_ffb.setObjectName("label_ffb")
        self.sldFF = QtWidgets.QSlider(parent=self.frame_ffbsettings)
        self.sldFF.setGeometry(QtCore.QRect(10, 90, 281, 22))
        self.sldFF.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.sldFF.setObjectName("sldFF")
        self.sldSmooth = QtWidgets.QSlider(parent=self.frame_ffbsettings)
        self.sldSmooth.setGeometry(QtCore.QRect(10, 140, 281, 22))
        self.sldSmooth.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.sldSmooth.setObjectName("sldSmooth")
        self.label_smooth = QtWidgets.QLabel(parent=self.frame_ffbsettings)
        self.label_smooth.setGeometry(QtCore.QRect(10, 110, 121, 16))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_smooth.setFont(font)
        self.label_smooth.setObjectName("label_smooth")
        self.label_ffbStg = QtWidgets.QLabel(parent=self.frame_ffbsettings)
        self.label_ffbStg.setGeometry(QtCore.QRect(10, 10, 251, 41))
        font = QtGui.QFont()
        font.setPointSize(22)
        self.label_ffbStg.setFont(font)
        self.label_ffbStg.setObjectName("label_ffbStg")
        self.pPercent = QtWidgets.QLabel(parent=self.frame_ffbsettings)
        self.pPercent.setGeometry(QtCore.QRect(250, 50, 51, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.pPercent.setFont(font)
        self.pPercent.setObjectName("pPercent")
        self.cbReverseFFD = QtWidgets.QCheckBox(parent=self.frame_ffbsettings)
        self.cbReverseFFD.setGeometry(QtCore.QRect(160, 60, 81, 20))
        self.cbReverseFFD.setObjectName("cbReverseFFD")
        self.frame_steer = QtWidgets.QFrame(parent=self.centralwidget)
        self.frame_steer.setGeometry(QtCore.QRect(19, 100, 261, 371))
        self.frame_steer.setFrameShape(QtWidgets.QFrame.Shape.StyledPanel)
        self.frame_steer.setFrameShadow(QtWidgets.QFrame.Shadow.Raised)
        self.frame_steer.setObjectName("frame_steer")
        self.sldAngle = QtWidgets.QSlider(parent=self.frame_steer)
        self.sldAngle.setGeometry(QtCore.QRect(20, 170, 161, 22))
        self.sldAngle.setOrientation(QtCore.Qt.Orientation.Horizontal)
        self.sldAngle.setObjectName("sldAngle")
        self.label_angle = QtWidgets.QLabel(parent=self.frame_steer)
        self.label_angle.setGeometry(QtCore.QRect(20, 140, 121, 21))
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_angle.setFont(font)
        self.label_angle.setObjectName("label_angle")
        self.label_steerStg = QtWidgets.QLabel(parent=self.frame_steer)
        self.label_steerStg.setGeometry(QtCore.QRect(20, 20, 251, 41))
        font = QtGui.QFont()
        font.setPointSize(22)
        self.label_steerStg.setFont(font)
        self.label_steerStg.setObjectName("label_steerStg")
        self.pDeg = QtWidgets.QLabel(parent=self.frame_steer)
        self.pDeg.setGeometry(QtCore.QRect(200, 160, 51, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.pDeg.setFont(font)
        self.pDeg.setObjectName("pDeg")
        self.pCurrentDeg = QtWidgets.QLabel(parent=self.frame_steer)
        self.pCurrentDeg.setGeometry(QtCore.QRect(80, 60, 91, 91))
        font = QtGui.QFont()
        font.setPointSize(48)
        self.pCurrentDeg.setFont(font)
        self.pCurrentDeg.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        self.pCurrentDeg.setObjectName("pCurrentDeg")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(parent=MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_ffb.setText(_translate("MainWindow", "Force Feedback"))
        self.label_smooth.setText(_translate("MainWindow", "Smoothness"))
        self.label_ffbStg.setText(_translate("MainWindow", "Feedback Settings"))
        self.pPercent.setText(_translate("MainWindow", "100%"))
        self.cbReverseFFD.setText(_translate("MainWindow", "Reverse FFB"))
        self.label_angle.setText(_translate("MainWindow", "Angle"))
        self.label_steerStg.setText(_translate("MainWindow", "Steer Settings"))
        self.pDeg.setText(_translate("MainWindow", "900^"))
        self.pCurrentDeg.setText(_translate("MainWindow", "0^"))


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec())