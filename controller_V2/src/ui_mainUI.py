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


class Ui_guiDlg(object):
    def setupUi(self, guiDlg):
        if not guiDlg.objectName():
            guiDlg.setObjectName(u"guiDlg")
        guiDlg.resize(600, 600)
        sizePolicy = QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(guiDlg.sizePolicy().hasHeightForWidth())
        guiDlg.setSizePolicy(sizePolicy)
        guiDlg.setMinimumSize(QSize(600, 600))
        self.verticalLayout = QVBoxLayout(guiDlg)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalSpacer_5 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_5)

        self.configuration = QPushButton(guiDlg)
        self.configuration.setObjectName(u"configuration")
        self.configuration.setCheckable(True)

        self.horizontalLayout.addWidget(self.configuration)

        self.horizontalSpacer = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer)

        self.regenerate = QPushButton(guiDlg)
        self.regenerate.setObjectName(u"regenerate")

        self.horizontalLayout.addWidget(self.regenerate)

        self.horizontalSpacer_4 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_4)

        self.label_2 = QLabel(guiDlg)
        self.label_2.setObjectName(u"label_2")

        self.horizontalLayout.addWidget(self.label_2)

        self.contributor = QLineEdit(guiDlg)
        self.contributor.setObjectName(u"contributor")

        self.horizontalLayout.addWidget(self.contributor)

        self.horizontalSpacer_6 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_6)

        self.save_metrics = QPushButton(guiDlg)
        self.save_metrics.setObjectName(u"save_metrics")

        self.horizontalLayout.addWidget(self.save_metrics)

        self.horizontalSpacer_3 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_3)

        self.quit = QPushButton(guiDlg)
        self.quit.setObjectName(u"quit")

        self.horizontalLayout.addWidget(self.quit)

        self.horizontalSpacer_2 = QSpacerItem(40, 20, QSizePolicy.Expanding, QSizePolicy.Minimum)

        self.horizontalLayout.addItem(self.horizontalSpacer_2)


        self.verticalLayout.addLayout(self.horizontalLayout)

        self.label = QLabel(guiDlg)
        self.label.setObjectName(u"label")
        sizePolicy1 = QSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
        sizePolicy1.setHorizontalStretch(2)
        sizePolicy1.setVerticalStretch(2)
        sizePolicy1.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy1)
        self.label.setMinimumSize(QSize(500, 500))
        self.label.setMaximumSize(QSize(10240, 7680))
        self.label.setSizeIncrement(QSize(2, 2))
        self.label.setFrameShape(QFrame.Box)
        self.label.setScaledContents(True)

        self.verticalLayout.addWidget(self.label)


        self.retranslateUi(guiDlg)

        QMetaObject.connectSlotsByName(guiDlg)
    # setupUi

    def retranslateUi(self, guiDlg):
        guiDlg.setWindowTitle(QCoreApplication.translate("guiDlg", u"Social Navigation Dataset Generator", None))
        self.configuration.setText(QCoreApplication.translate("guiDlg", u"configuration", None))
        self.regenerate.setText(QCoreApplication.translate("guiDlg", u"regenerate", None))
        self.label_2.setText(QCoreApplication.translate("guiDlg", u"contributor's unique id:", None))
        self.contributor.setText(QCoreApplication.translate("guiDlg", u"default", None))
        self.save_metrics.setText(QCoreApplication.translate("guiDlg", u"save metrics", None))
        self.quit.setText(QCoreApplication.translate("guiDlg", u"quit", None))
        self.label.setText("")
    # retranslateUi

