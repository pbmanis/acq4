# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'acq4/devices/DAQGeneric/DeviceTemplate.ui'
#
# Created by: PyQt5 UI code generator 5.8.2
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(240, 194)
        self.gridLayout = QtWidgets.QGridLayout(Form)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setObjectName("gridLayout")
        self.holdingSpin = SpinBox(Form)
        self.holdingSpin.setObjectName("holdingSpin")
        self.gridLayout.addWidget(self.holdingSpin, 2, 2, 1, 1)
        self.scaleSpin = SpinBox(Form)
        self.scaleSpin.setObjectName("scaleSpin")
        self.gridLayout.addWidget(self.scaleSpin, 3, 2, 1, 1)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.inputRadio = QtWidgets.QRadioButton(Form)
        self.inputRadio.setObjectName("inputRadio")
        self.horizontalLayout_2.addWidget(self.inputRadio)
        self.outputRadio = QtWidgets.QRadioButton(Form)
        self.outputRadio.setObjectName("outputRadio")
        self.horizontalLayout_2.addWidget(self.outputRadio)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem)
        self.gridLayout.addLayout(self.horizontalLayout_2, 1, 3, 1, 2)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(Form)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.channelCombo = QtWidgets.QComboBox(Form)
        self.channelCombo.setObjectName("channelCombo")
        self.horizontalLayout.addWidget(self.channelCombo)
        self.gridLayout.addLayout(self.horizontalLayout, 1, 0, 1, 3)
        self.scaleDefaultBtn = QtWidgets.QPushButton(Form)
        self.scaleDefaultBtn.setObjectName("scaleDefaultBtn")
        self.gridLayout.addWidget(self.scaleDefaultBtn, 3, 3, 1, 1)
        self.offsetDefaultBtn = QtWidgets.QPushButton(Form)
        self.offsetDefaultBtn.setObjectName("offsetDefaultBtn")
        self.gridLayout.addWidget(self.offsetDefaultBtn, 4, 3, 1, 1)
        self.invertCheck = QtWidgets.QCheckBox(Form)
        self.invertCheck.setObjectName("invertCheck")
        self.gridLayout.addWidget(self.invertCheck, 2, 3, 1, 1)
        self.nameLabel = QtWidgets.QLabel(Form)
        font = Qt.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.nameLabel.setFont(font)
        self.nameLabel.setObjectName("nameLabel")
        self.gridLayout.addWidget(self.nameLabel, 0, 0, 1, 3)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.gridLayout.addItem(spacerItem1, 5, 3, 1, 1)
        self.offsetSpin = SpinBox(Form)
        self.offsetSpin.setObjectName("offsetSpin")
        self.gridLayout.addWidget(self.offsetSpin, 4, 2, 1, 1)
        self.holdingLabel = QtWidgets.QLabel(Form)
        self.holdingLabel.setAlignment(Qt.Qt.AlignRight|Qt.Qt.AlignTrailing|Qt.Qt.AlignVCenter)
        self.holdingLabel.setObjectName("holdingLabel")
        self.gridLayout.addWidget(self.holdingLabel, 2, 1, 1, 1)
        self.scaleLabel = QtWidgets.QLabel(Form)
        self.scaleLabel.setAlignment(Qt.Qt.AlignRight|Qt.Qt.AlignTrailing|Qt.Qt.AlignVCenter)
        self.scaleLabel.setObjectName("scaleLabel")
        self.gridLayout.addWidget(self.scaleLabel, 3, 1, 1, 1)
        self.offsetLabel = QtWidgets.QLabel(Form)
        self.offsetLabel.setAlignment(Qt.Qt.AlignRight|Qt.Qt.AlignTrailing|Qt.Qt.AlignVCenter)
        self.offsetLabel.setObjectName("offsetLabel")
        self.gridLayout.addWidget(self.offsetLabel, 4, 1, 1, 1)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem2, 2, 0, 3, 1)
        spacerItem3 = QtWidgets.QSpacerItem(42, 1, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.gridLayout.addItem(spacerItem3, 2, 4, 3, 1)
        self.gridLayout.setColumnStretch(1, 1)
        self.gridLayout.setColumnStretch(2, 2)
        self.gridLayout.setColumnStretch(3, 2)
        self.gridLayout.setColumnStretch(4, 4)

        self.retranslateUi(Form)
        Qt.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = Qt.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.inputRadio.setText(_translate("Form", "Input"))
        self.outputRadio.setText(_translate("Form", "Output"))
        self.label.setText(_translate("Form", "Channel:"))
        self.scaleDefaultBtn.setText(_translate("Form", "Default"))
        self.offsetDefaultBtn.setText(_translate("Form", "Default"))
        self.invertCheck.setText(_translate("Form", "Invert"))
        self.nameLabel.setText(_translate("Form", "ChannelName"))
        self.holdingLabel.setText(_translate("Form", "Holding:"))
        self.scaleLabel.setText(_translate("Form", "Scale:"))
        self.offsetLabel.setText(_translate("Form", "Offset:"))

from acq4.pyqtgraph import SpinBox