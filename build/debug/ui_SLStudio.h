/********************************************************************************
** Form generated from reading UI file 'SLStudio.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SLSTUDIO_H
#define UI_SLSTUDIO_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>
#include "SLPointCloudWidget.h"

QT_BEGIN_NAMESPACE

class Ui_SLStudio
{
public:
    QAction *actionQuit;
    QAction *actionStart;
    QAction *actionStop;
    QAction *actionCalibration;
    QAction *actionLoadCalibration;
    QAction *actionExportCalibration;
    QAction *actionPreferences;
    QAction *actionSavePointCloud;
    QAction *actionSaveScreenshot;
    QAction *actionTracking;
    QAction *actionAbout;
    QAction *actionTracker;
    QAction *actionUpdatePointClouds;
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    SLPointCloudWidget *pointCloudWidget;
    QMenuBar *menuBar;
    QMenu *menuCalibration;
    QMenu *menuSLStudio;
    QMenu *menuScan;
    QMenu *menuView;
    QToolBar *toolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *SLStudio)
    {
        if (SLStudio->objectName().isEmpty())
            SLStudio->setObjectName(QString::fromUtf8("SLStudio"));
        SLStudio->resize(824, 537);
        SLStudio->setAutoFillBackground(false);
        actionQuit = new QAction(SLStudio);
        actionQuit->setObjectName(QString::fromUtf8("actionQuit"));
        actionQuit->setMenuRole(QAction::QuitRole);
        actionStart = new QAction(SLStudio);
        actionStart->setObjectName(QString::fromUtf8("actionStart"));
        actionStop = new QAction(SLStudio);
        actionStop->setObjectName(QString::fromUtf8("actionStop"));
        actionStop->setEnabled(false);
        actionCalibration = new QAction(SLStudio);
        actionCalibration->setObjectName(QString::fromUtf8("actionCalibration"));
        actionLoadCalibration = new QAction(SLStudio);
        actionLoadCalibration->setObjectName(QString::fromUtf8("actionLoadCalibration"));
        actionExportCalibration = new QAction(SLStudio);
        actionExportCalibration->setObjectName(QString::fromUtf8("actionExportCalibration"));
        actionPreferences = new QAction(SLStudio);
        actionPreferences->setObjectName(QString::fromUtf8("actionPreferences"));
        actionSavePointCloud = new QAction(SLStudio);
        actionSavePointCloud->setObjectName(QString::fromUtf8("actionSavePointCloud"));
        actionSavePointCloud->setEnabled(false);
        actionSaveScreenshot = new QAction(SLStudio);
        actionSaveScreenshot->setObjectName(QString::fromUtf8("actionSaveScreenshot"));
        actionSaveScreenshot->setEnabled(false);
        actionTracking = new QAction(SLStudio);
        actionTracking->setObjectName(QString::fromUtf8("actionTracking"));
        actionTracking->setEnabled(false);
        actionAbout = new QAction(SLStudio);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        actionAbout->setMenuRole(QAction::AboutRole);
        actionTracker = new QAction(SLStudio);
        actionTracker->setObjectName(QString::fromUtf8("actionTracker"));
        actionTracker->setCheckable(true);
        actionTracker->setChecked(true);
        actionUpdatePointClouds = new QAction(SLStudio);
        actionUpdatePointClouds->setObjectName(QString::fromUtf8("actionUpdatePointClouds"));
        actionUpdatePointClouds->setCheckable(true);
        actionUpdatePointClouds->setChecked(true);
        centralWidget = new QWidget(SLStudio);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setAcceptDrops(false);
        centralWidget->setLayoutDirection(Qt::LeftToRight);
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        pointCloudWidget = new SLPointCloudWidget(centralWidget);
        pointCloudWidget->setObjectName(QString::fromUtf8("pointCloudWidget"));
        pointCloudWidget->setEnabled(true);
        pointCloudWidget->setLayoutDirection(Qt::LeftToRight);
        pointCloudWidget->setAutoFillBackground(false);

        gridLayout->addWidget(pointCloudWidget, 0, 0, 1, 1);

        SLStudio->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(SLStudio);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 824, 25));
        menuCalibration = new QMenu(menuBar);
        menuCalibration->setObjectName(QString::fromUtf8("menuCalibration"));
        menuSLStudio = new QMenu(menuBar);
        menuSLStudio->setObjectName(QString::fromUtf8("menuSLStudio"));
        menuScan = new QMenu(menuBar);
        menuScan->setObjectName(QString::fromUtf8("menuScan"));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QString::fromUtf8("menuView"));
        SLStudio->setMenuBar(menuBar);
        toolBar = new QToolBar(SLStudio);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        toolBar->setEnabled(true);
        QFont font;
        font.setItalic(false);
        font.setStrikeOut(false);
        toolBar->setFont(font);
        toolBar->setMovable(false);
        toolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        SLStudio->addToolBar(Qt::TopToolBarArea, toolBar);
        statusBar = new QStatusBar(SLStudio);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        SLStudio->setStatusBar(statusBar);

        menuBar->addAction(menuSLStudio->menuAction());
        menuBar->addAction(menuScan->menuAction());
        menuBar->addAction(menuCalibration->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuCalibration->addAction(actionLoadCalibration);
        menuCalibration->addAction(actionExportCalibration);
        menuCalibration->addAction(actionCalibration);
        menuSLStudio->addAction(actionPreferences);
        menuSLStudio->addAction(actionQuit);
        menuSLStudio->addAction(actionAbout);
        menuScan->addAction(actionStart);
        menuScan->addAction(actionStop);
        menuScan->addAction(actionSavePointCloud);
        menuScan->addAction(actionSaveScreenshot);
        menuView->addAction(actionUpdatePointClouds);
        toolBar->addAction(actionStart);
        toolBar->addAction(actionStop);
        toolBar->addSeparator();
        toolBar->addAction(actionSavePointCloud);
        toolBar->addAction(actionSaveScreenshot);
        toolBar->addAction(actionCalibration);
        toolBar->addSeparator();

        retranslateUi(SLStudio);
        QObject::connect(actionStart, SIGNAL(triggered()), SLStudio, SLOT(onActionStart()));
        QObject::connect(actionStop, SIGNAL(triggered()), SLStudio, SLOT(onActionStop()));
        QObject::connect(actionQuit, SIGNAL(triggered()), SLStudio, SLOT(close()));
        QObject::connect(actionCalibration, SIGNAL(triggered()), SLStudio, SLOT(onActionCalibration()));
        QObject::connect(actionPreferences, SIGNAL(triggered()), SLStudio, SLOT(onActionPreferences()));
        QObject::connect(actionExportCalibration, SIGNAL(triggered()), SLStudio, SLOT(onActionExportCalibration()));
        QObject::connect(actionSaveScreenshot, SIGNAL(triggered()), pointCloudWidget, SLOT(saveScreenShot()));
        QObject::connect(actionSavePointCloud, SIGNAL(triggered()), pointCloudWidget, SLOT(savePointCloud()));
        QObject::connect(actionLoadCalibration, SIGNAL(triggered()), SLStudio, SLOT(onActionLoadCalibration()));
        QObject::connect(actionAbout, SIGNAL(triggered()), SLStudio, SLOT(onActionAbout()));

        QMetaObject::connectSlotsByName(SLStudio);
    } // setupUi

    void retranslateUi(QMainWindow *SLStudio)
    {
        SLStudio->setWindowTitle(QApplication::translate("SLStudio", "SLStudio", 0, QApplication::UnicodeUTF8));
        actionQuit->setText(QApplication::translate("SLStudio", "&Quit", 0, QApplication::UnicodeUTF8));
        actionStart->setText(QApplication::translate("SLStudio", "Start Scan", 0, QApplication::UnicodeUTF8));
        actionStop->setText(QApplication::translate("SLStudio", "Stop Scan", 0, QApplication::UnicodeUTF8));
        actionCalibration->setText(QApplication::translate("SLStudio", "Perform Calibration", 0, QApplication::UnicodeUTF8));
        actionLoadCalibration->setText(QApplication::translate("SLStudio", "Load Calibration", 0, QApplication::UnicodeUTF8));
        actionExportCalibration->setText(QApplication::translate("SLStudio", "Export Calibration", 0, QApplication::UnicodeUTF8));
        actionPreferences->setText(QApplication::translate("SLStudio", "&Preferences", 0, QApplication::UnicodeUTF8));
        actionSavePointCloud->setText(QApplication::translate("SLStudio", "Save Point Cloud", 0, QApplication::UnicodeUTF8));
        actionSaveScreenshot->setText(QApplication::translate("SLStudio", "Save Screenshot", 0, QApplication::UnicodeUTF8));
        actionTracking->setText(QApplication::translate("SLStudio", "Start Tracking", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("SLStudio", "About", 0, QApplication::UnicodeUTF8));
        actionTracker->setText(QApplication::translate("SLStudio", "Tracker", 0, QApplication::UnicodeUTF8));
        actionUpdatePointClouds->setText(QApplication::translate("SLStudio", "Update Point Clouds", 0, QApplication::UnicodeUTF8));
        menuCalibration->setTitle(QApplication::translate("SLStudio", "Calibration", 0, QApplication::UnicodeUTF8));
        menuSLStudio->setTitle(QApplication::translate("SLStudio", "SLStudio", 0, QApplication::UnicodeUTF8));
        menuScan->setTitle(QApplication::translate("SLStudio", "Scan", 0, QApplication::UnicodeUTF8));
        menuView->setTitle(QApplication::translate("SLStudio", "View", 0, QApplication::UnicodeUTF8));
        toolBar->setWindowTitle(QApplication::translate("SLStudio", "toolBar", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SLStudio: public Ui_SLStudio {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SLSTUDIO_H
