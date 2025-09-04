# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'starting_positions.ui'
##
## Created by: Qt User Interface Compiler version 6.8.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (
    QCoreApplication,
    QDate,
    QDateTime,
    QLocale,
    QMetaObject,
    QObject,
    QPoint,
    QRect,
    QSize,
    QTime,
    QUrl,
    Qt,
)
from PySide6.QtGui import (
    QBrush,
    QColor,
    QConicalGradient,
    QCursor,
    QFont,
    QFontDatabase,
    QGradient,
    QIcon,
    QImage,
    QKeySequence,
    QLinearGradient,
    QPainter,
    QPalette,
    QPixmap,
    QRadialGradient,
    QTransform,
)
from PySide6.QtWidgets import (
    QAbstractButton,
    QApplication,
    QCheckBox,
    QDialog,
    QDialogButtonBox,
    QGraphicsView,
    QGridLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QWidget,
)

from autoloader_data import *
from autoloader_interfaces.msg import Landmarks

from .starting_positions_grid import InputCarrierGridView, OutputCarrierGridView


class Ui_Starting_Pos(object):
    def setupUi(self, Dialog, landmarks: Landmarks):
        if not Dialog.objectName():
            Dialog.setObjectName("Dialog")
        Dialog.resize(999, 542)
        self.gridLayout = QGridLayout(Dialog)
        self.gridLayout.setObjectName("gridLayout")
        self.label_3 = QLabel(Dialog)
        self.label_3.setObjectName("label_3")
        font = QFont()
        font.setBold(True)
        font.setItalic(True)
        self.label_3.setFont(font)
        self.label_3.setAlignment(Qt.AlignLeading | Qt.AlignLeft | Qt.AlignVCenter)

        self.gridLayout.addWidget(self.label_3, 0, 0, 1, 2)

        self.buttonBox = QDialogButtonBox(Dialog)
        self.buttonBox.setObjectName("buttonBox")
        self.buttonBox.setOrientation(Qt.Horizontal)
        self.buttonBox.setStandardButtons(QDialogButtonBox.Cancel | QDialogButtonBox.Ok)

        self.gridLayout.addWidget(self.buttonBox, 7, 0, 1, 2)

        self.output_grid = OutputCarrierGridView(
            carrier=landmarks.smd_carrier, parent=self
        )
        self.output_grid.setObjectName("output_grid")

        self.gridLayout.addWidget(self.output_grid, 6, 1, 1, 1)

        self.label_2 = QLabel(Dialog)
        self.label_2.setObjectName("label_2")
        font1 = QFont()
        font1.setPointSize(11)
        font1.setBold(True)
        font1.setUnderline(True)
        self.label_2.setFont(font1)

        self.gridLayout.addWidget(self.label_2, 5, 1, 1, 1)

        self.label = QLabel(Dialog)
        self.label.setObjectName("label")
        self.label.setFont(font1)

        self.gridLayout.addWidget(self.label, 5, 0, 1, 1)

        self.input_grid = InputCarrierGridView(
            carrier=landmarks.smd_waffle,
            output_carrier=landmarks.smd_carrier,
            parent=self,
        )
        self.input_grid.setObjectName("input_grid")

        self.gridLayout.addWidget(self.input_grid, 6, 0, 1, 1)

        self.reset_positions_button = QPushButton(Dialog)
        self.reset_positions_button.setObjectName("reset_positions_button")
        palette = QPalette()
        brush = QBrush(QColor(0, 0, 0, 255))
        brush.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.WindowText, brush)
        brush1 = QBrush(QColor(230, 32, 84, 255))
        brush1.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Button, brush1)
        brush2 = QBrush(QColor(255, 125, 159, 255))
        brush2.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Light, brush2)
        brush3 = QBrush(QColor(242, 78, 121, 255))
        brush3.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Midlight, brush3)
        brush4 = QBrush(QColor(115, 16, 42, 255))
        brush4.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Dark, brush4)
        brush5 = QBrush(QColor(153, 21, 56, 255))
        brush5.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.Mid, brush5)
        palette.setBrush(QPalette.Active, QPalette.Text, brush)
        brush6 = QBrush(QColor(255, 255, 255, 255))
        brush6.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.BrightText, brush6)
        palette.setBrush(QPalette.Active, QPalette.ButtonText, brush)
        palette.setBrush(QPalette.Active, QPalette.Base, brush6)
        palette.setBrush(QPalette.Active, QPalette.Window, brush1)
        palette.setBrush(QPalette.Active, QPalette.Shadow, brush)
        brush7 = QBrush(QColor(242, 143, 169, 255))
        brush7.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.AlternateBase, brush7)
        brush8 = QBrush(QColor(255, 255, 220, 255))
        brush8.setStyle(Qt.SolidPattern)
        palette.setBrush(QPalette.Active, QPalette.ToolTipBase, brush8)
        palette.setBrush(QPalette.Active, QPalette.ToolTipText, brush)
        brush9 = QBrush(QColor(0, 0, 0, 128))
        brush9.setStyle(Qt.SolidPattern)
        # if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette.Active, QPalette.PlaceholderText, brush9)
        # endif
        palette.setBrush(QPalette.Inactive, QPalette.WindowText, brush)
        palette.setBrush(QPalette.Inactive, QPalette.Button, brush1)
        palette.setBrush(QPalette.Inactive, QPalette.Light, brush2)
        palette.setBrush(QPalette.Inactive, QPalette.Midlight, brush3)
        palette.setBrush(QPalette.Inactive, QPalette.Dark, brush4)
        palette.setBrush(QPalette.Inactive, QPalette.Mid, brush5)
        palette.setBrush(QPalette.Inactive, QPalette.Text, brush)
        palette.setBrush(QPalette.Inactive, QPalette.BrightText, brush6)
        palette.setBrush(QPalette.Inactive, QPalette.ButtonText, brush)
        palette.setBrush(QPalette.Inactive, QPalette.Base, brush6)
        palette.setBrush(QPalette.Inactive, QPalette.Window, brush1)
        palette.setBrush(QPalette.Inactive, QPalette.Shadow, brush)
        palette.setBrush(QPalette.Inactive, QPalette.AlternateBase, brush7)
        palette.setBrush(QPalette.Inactive, QPalette.ToolTipBase, brush8)
        palette.setBrush(QPalette.Inactive, QPalette.ToolTipText, brush)
        # if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette.Inactive, QPalette.PlaceholderText, brush9)
        # endif
        palette.setBrush(QPalette.Disabled, QPalette.WindowText, brush4)
        palette.setBrush(QPalette.Disabled, QPalette.Button, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Light, brush2)
        palette.setBrush(QPalette.Disabled, QPalette.Midlight, brush3)
        palette.setBrush(QPalette.Disabled, QPalette.Dark, brush4)
        palette.setBrush(QPalette.Disabled, QPalette.Mid, brush5)
        palette.setBrush(QPalette.Disabled, QPalette.Text, brush4)
        palette.setBrush(QPalette.Disabled, QPalette.BrightText, brush6)
        palette.setBrush(QPalette.Disabled, QPalette.ButtonText, brush4)
        palette.setBrush(QPalette.Disabled, QPalette.Base, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Window, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.Shadow, brush)
        palette.setBrush(QPalette.Disabled, QPalette.AlternateBase, brush1)
        palette.setBrush(QPalette.Disabled, QPalette.ToolTipBase, brush8)
        palette.setBrush(QPalette.Disabled, QPalette.ToolTipText, brush)
        # if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette.Disabled, QPalette.PlaceholderText, brush9)
        # endif
        self.reset_positions_button.setPalette(palette)
        font2 = QFont()
        font2.setPointSize(11)
        font2.setBold(True)
        self.reset_positions_button.setFont(font2)

        self.gridLayout.addWidget(self.reset_positions_button, 1, 0, 1, 1)

        self.select_failures_checkbox = QCheckBox(Dialog)
        self.select_failures_checkbox.setObjectName("select_failures_checkbox")
        self.select_failures_checkbox.setFont(font2)

        self.gridLayout.addWidget(self.select_failures_checkbox, 1, 1, 1, 1)

        self.retranslateUi(Dialog)
        self.buttonBox.accepted.connect(Dialog.accept)
        self.buttonBox.rejected.connect(Dialog.reject)

        QMetaObject.connectSlotsByName(Dialog)

    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", "Dialog", None))
        self.label_3.setText(
            QCoreApplication.translate(
                "Dialog", "Select Slot Positions for Picking & Placing!", None
            )
        )
        self.label_2.setText(
            QCoreApplication.translate("Dialog", "Output Carrier Positions", None)
        )
        self.label.setText(
            QCoreApplication.translate("Dialog", "Input Carrier Positions", None)
        )
        self.reset_positions_button.setText(
            QCoreApplication.translate("Dialog", "Reset Positions", None)
        )
        self.select_failures_checkbox.setText(
            QCoreApplication.translate("Dialog", "Select Failed Devices / Slots", None)
        )

    # retranslateUi
