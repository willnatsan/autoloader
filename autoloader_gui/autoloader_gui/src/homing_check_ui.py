# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'homing_check.ui'
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
    QDialog,
    QDialogButtonBox,
    QGridLayout,
    QLabel,
    QSizePolicy,
    QSpacerItem,
    QWidget,
)


class Ui_Homing(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName("Dialog")
        Dialog.resize(504, 138)
        self.gridLayout = QGridLayout(Dialog)
        self.gridLayout.setObjectName("gridLayout")
        self.label_2 = QLabel(Dialog)
        self.label_2.setObjectName("label_2")
        font = QFont()
        font.setFamilies(["Noto Sans Mono"])
        self.label_2.setFont(font)

        self.gridLayout.addWidget(self.label_2, 2, 0, 1, 2)

        self.label = QLabel(Dialog)
        self.label.setObjectName("label")
        font1 = QFont()
        font1.setFamilies(["Noto Sans Mono"])
        font1.setPointSize(20)
        font1.setBold(True)
        font1.setUnderline(True)
        self.label.setFont(font1)

        self.gridLayout.addWidget(self.label, 0, 0, 1, 1)

        self.verticalSpacer = QSpacerItem(
            20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding
        )

        self.gridLayout.addItem(self.verticalSpacer, 1, 0, 1, 1)

        self.verticalSpacer_2 = QSpacerItem(
            20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding
        )

        self.gridLayout.addItem(self.verticalSpacer_2, 3, 0, 1, 1)

        self.dialog_button_box = QDialogButtonBox(Dialog)
        self.dialog_button_box.setObjectName("dialog_button_box")
        self.dialog_button_box.setStandardButtons(
            QDialogButtonBox.No | QDialogButtonBox.Yes
        )
        self.dialog_button_box.setCenterButtons(True)

        self.gridLayout.addWidget(self.dialog_button_box, 4, 0, 1, 2)

        self.retranslateUi(Dialog)

        QMetaObject.connectSlotsByName(Dialog)

    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", "Dialog", None))
        self.label_2.setText(
            QCoreApplication.translate(
                "Dialog", "Has the machine Homed correctly (Reached Bottom Left)?", None
            )
        )
        self.label.setText(QCoreApplication.translate("Dialog", "Homing Check", None))

    # retranslateUi
