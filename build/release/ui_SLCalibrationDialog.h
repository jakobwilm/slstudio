/********************************************************************************
** Form generated from reading UI file 'SLCalibrationDialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SLCALIBRATIONDIALOG_H
#define UI_SLCALIBRATIONDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QListWidget>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>
#include <QtGui/QWidget>
#include "SLVideoWidget.h"

QT_BEGIN_NAMESPACE

class Ui_SLCalibrationDialog
{
public:
    SLVideoWidget *videoWidget;
    QPushButton *snapButton;
    QListWidget *listWidget;
    QPushButton *calibrateButton;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QSpinBox *checkerSizeBox;
    QWidget *layoutWidget1;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_3;
    QSpinBox *checkerRowsBox;
    QWidget *layoutWidget2;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_4;
    QSpinBox *checkerColsBox;
    QWidget *layoutWidget3;
    QHBoxLayout *horizontalLayout_4;
    QPushButton *cancelButton;
    QPushButton *saveButton;

    void setupUi(QDialog *SLCalibrationDialog)
    {
        if (SLCalibrationDialog->objectName().isEmpty())
            SLCalibrationDialog->setObjectName(QString::fromUtf8("SLCalibrationDialog"));
        SLCalibrationDialog->resize(890, 540);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(SLCalibrationDialog->sizePolicy().hasHeightForWidth());
        SLCalibrationDialog->setSizePolicy(sizePolicy);
        SLCalibrationDialog->setMinimumSize(QSize(890, 540));
        SLCalibrationDialog->setMaximumSize(QSize(890, 540));
        SLCalibrationDialog->setSizeGripEnabled(false);
        videoWidget = new SLVideoWidget(SLCalibrationDialog);
        videoWidget->setObjectName(QString::fromUtf8("videoWidget"));
        videoWidget->setEnabled(true);
        videoWidget->setGeometry(QRect(10, 10, 651, 471));
        snapButton = new QPushButton(SLCalibrationDialog);
        snapButton->setObjectName(QString::fromUtf8("snapButton"));
        snapButton->setGeometry(QRect(205, 500, 241, 27));
        listWidget = new QListWidget(SLCalibrationDialog);
        listWidget->setObjectName(QString::fromUtf8("listWidget"));
        listWidget->setGeometry(QRect(680, 10, 181, 281));
        calibrateButton = new QPushButton(SLCalibrationDialog);
        calibrateButton->setObjectName(QString::fromUtf8("calibrateButton"));
        calibrateButton->setEnabled(false);
        calibrateButton->setGeometry(QRect(718, 430, 111, 27));
        layoutWidget = new QWidget(SLCalibrationDialog);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(680, 330, 181, 29));
        horizontalLayout = new QHBoxLayout(layoutWidget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout->addWidget(label);

        checkerSizeBox = new QSpinBox(layoutWidget);
        checkerSizeBox->setObjectName(QString::fromUtf8("checkerSizeBox"));

        horizontalLayout->addWidget(checkerSizeBox);

        layoutWidget1 = new QWidget(SLCalibrationDialog);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(680, 360, 181, 29));
        horizontalLayout_2 = new QHBoxLayout(layoutWidget1);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_3 = new QLabel(layoutWidget1);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        horizontalLayout_2->addWidget(label_3);

        checkerRowsBox = new QSpinBox(layoutWidget1);
        checkerRowsBox->setObjectName(QString::fromUtf8("checkerRowsBox"));

        horizontalLayout_2->addWidget(checkerRowsBox);

        layoutWidget2 = new QWidget(SLCalibrationDialog);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(680, 390, 181, 29));
        horizontalLayout_3 = new QHBoxLayout(layoutWidget2);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalLayout_3->setContentsMargins(0, 0, 0, 0);
        label_4 = new QLabel(layoutWidget2);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        horizontalLayout_3->addWidget(label_4);

        checkerColsBox = new QSpinBox(layoutWidget2);
        checkerColsBox->setObjectName(QString::fromUtf8("checkerColsBox"));

        horizontalLayout_3->addWidget(checkerColsBox);

        layoutWidget3 = new QWidget(SLCalibrationDialog);
        layoutWidget3->setObjectName(QString::fromUtf8("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(690, 480, 181, 29));
        horizontalLayout_4 = new QHBoxLayout(layoutWidget3);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        horizontalLayout_4->setContentsMargins(0, 0, 0, 0);
        cancelButton = new QPushButton(layoutWidget3);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));

        horizontalLayout_4->addWidget(cancelButton);

        saveButton = new QPushButton(layoutWidget3);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));
        saveButton->setEnabled(false);

        horizontalLayout_4->addWidget(saveButton);


        retranslateUi(SLCalibrationDialog);
        QObject::connect(cancelButton, SIGNAL(clicked()), SLCalibrationDialog, SLOT(close()));

        QMetaObject::connectSlotsByName(SLCalibrationDialog);
    } // setupUi

    void retranslateUi(QDialog *SLCalibrationDialog)
    {
        SLCalibrationDialog->setWindowTitle(QApplication::translate("SLCalibrationDialog", "SL Calibration", 0, QApplication::UnicodeUTF8));
        snapButton->setText(QApplication::translate("SLCalibrationDialog", "Snap", 0, QApplication::UnicodeUTF8));
        calibrateButton->setText(QApplication::translate("SLCalibrationDialog", "Calibrate", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("SLCalibrationDialog", "Size (mm):", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("SLCalibrationDialog", "Rows:", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("SLCalibrationDialog", "Cols:", 0, QApplication::UnicodeUTF8));
        cancelButton->setText(QApplication::translate("SLCalibrationDialog", "&Cancel", 0, QApplication::UnicodeUTF8));
        saveButton->setText(QApplication::translate("SLCalibrationDialog", "&Save", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SLCalibrationDialog: public Ui_SLCalibrationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SLCALIBRATIONDIALOG_H
