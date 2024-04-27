# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainUI.ui'
##
## Created by: Qt User Interface Compiler version 5.15.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(600, 600)
        sizePolicy = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(MainWindow.sizePolicy().hasHeightForWidth())
        MainWindow.setSizePolicy(sizePolicy)
        MainWindow.setMinimumSize(QSize(600, 600))
        self.gridLayout = QGridLayout(MainWindow)
        self.gridLayout.setObjectName(u"gridLayout")
        self.verticalLayout_main = QVBoxLayout()
        self.verticalLayout_main.setObjectName(u"verticalLayout_main")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.configuration = QPushButton(MainWindow)
        self.configuration.setObjectName(u"configuration")
        self.configuration.setCheckable(True)

        self.horizontalLayout.addWidget(self.configuration)

        self.regenerate = QPushButton(MainWindow)
        self.regenerate.setObjectName(u"regenerate")

        self.horizontalLayout.addWidget(self.regenerate)

        self.label_2 = QLabel(MainWindow)
        self.label_2.setObjectName(u"label_2")

        self.horizontalLayout.addWidget(self.label_2)

        self.contributor = QLineEdit(MainWindow)
        self.contributor.setObjectName(u"contributor")

        self.horizontalLayout.addWidget(self.contributor)

        self.save_metrics = QPushButton(MainWindow)
        self.save_metrics.setObjectName(u"save_metrics")

        self.horizontalLayout.addWidget(self.save_metrics)

        self.quit = QPushButton(MainWindow)
        self.quit.setObjectName(u"quit")

        self.horizontalLayout.addWidget(self.quit)


        self.verticalLayout_main.addLayout(self.horizontalLayout)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_main.addItem(self.verticalSpacer)

        self.label = QLabel(MainWindow)
        self.label.setObjectName(u"label")
        sizePolicy1 = QSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.MinimumExpanding)
        sizePolicy1.setHorizontalStretch(2)
        sizePolicy1.setVerticalStretch(2)
        sizePolicy1.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy1)
        self.label.setMinimumSize(QSize(500, 500))
        self.label.setMaximumSize(QSize(10240, 7680))
        self.label.setSizeIncrement(QSize(2, 2))
        self.label.setFrameShape(QFrame.Box)
        self.label.setScaledContents(True)

        self.verticalLayout_main.addWidget(self.label)


        self.gridLayout.addLayout(self.verticalLayout_main, 0, 0, 1, 1)


        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Social Navigation Dataset Generator", None))
        self.configuration.setText(QCoreApplication.translate("MainWindow", u"configuration", None))
        self.regenerate.setText(QCoreApplication.translate("MainWindow", u"regenerate", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"contributor's unique id:", None))
        self.contributor.setText(QCoreApplication.translate("MainWindow", u"default", None))
        self.save_metrics.setText(QCoreApplication.translate("MainWindow", u"save metrics", None))
        self.quit.setText(QCoreApplication.translate("MainWindow", u"quit", None))
        self.label.setText("")
    # retranslateUi

