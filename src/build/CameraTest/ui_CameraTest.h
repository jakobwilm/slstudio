/********************************************************************************
** Form generated from reading UI file 'CameraTest.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CAMERATEST_H
#define UI_CAMERATEST_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QVBoxLayout>
#include "../SLVideoWidget.h"

QT_BEGIN_NAMESPACE

class Ui_CameraTest
{
public:
    QVBoxLayout *verticalLayout;
    SLVideoWidget *widget;
    QPushButton *pushButton;

    void setupUi(QDialog *CameraTest)
    {
        if (CameraTest->objectName().isEmpty())
            CameraTest->setObjectName(QString::fromUtf8("CameraTest"));
        CameraTest->resize(640, 460);
        verticalLayout = new QVBoxLayout(CameraTest);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        widget = new SLVideoWidget(CameraTest);
        widget->setObjectName(QString::fromUtf8("widget"));

        verticalLayout->addWidget(widget);

        pushButton = new QPushButton(CameraTest);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout->addWidget(pushButton);


        retranslateUi(CameraTest);
        QObject::connect(pushButton, SIGNAL(clicked()), CameraTest, SLOT(close()));

        QMetaObject::connectSlotsByName(CameraTest);
    } // setupUi

    void retranslateUi(QDialog *CameraTest)
    {
        CameraTest->setWindowTitle(QApplication::translate("CameraTest", "Dialog", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("CameraTest", "Quit", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class CameraTest: public Ui_CameraTest {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CAMERATEST_H
