/********************************************************************************
** Form generated from reading UI file 'SLPreferenceDialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SLPREFERENCEDIALOG_H
#define UI_SLPREFERENCEDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QRadioButton>
#include <QtGui/QSpinBox>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_SLPreferenceDialog
{
public:
    QDialogButtonBox *buttonBox;
    QLabel *cameraLabel;
    QLabel *projectorLabel;
    QComboBox *cameraComboBox;
    QLabel *patternModeLabel;
    QComboBox *patternModeComboBox;
    QComboBox *projectorComboBox;
    QLabel *triggerModeLabel;
    QWidget *layoutWidget;
    QHBoxLayout *triggerModeLayout;
    QRadioButton *triggerHardwareRadioButton;
    QRadioButton *triggerSoftwareRadioButton;
    QLabel *writeToDiskLabel;
    QWidget *layoutWidget1;
    QHBoxLayout *shutterLayout;
    QLabel *shutterLabel;
    QDoubleSpinBox *shutterDoubleSpinBox;
    QLabel *delayMsLabel_2;
    QWidget *layoutWidget2;
    QHBoxLayout *triggerLayout;
    QHBoxLayout *shiftLayout;
    QLabel *shiftLabel;
    QSpinBox *shiftSpinBox;
    QHBoxLayout *delayLayout;
    QLabel *delayLabel;
    QSpinBox *delaySpinBox;
    QLabel *delayMsLabel;
    QWidget *layoutWidget3;
    QHBoxLayout *writeToDiskLayout;
    QCheckBox *pointCloudsCheckBox;
    QCheckBox *trackingCheckBox;
    QWidget *layoutWidget4;
    QHBoxLayout *diamondPatternLayout;
    QCheckBox *diamondPatternCheckBox;
    QLabel *aquisitionLabel;
    QWidget *layoutWidget_2;
    QHBoxLayout *aquisitionLayout;
    QRadioButton *aquisitioncontinuousRadioButton;
    QRadioButton *aquisitionSingleRadioButton;
    QWidget *layoutWidget_3;
    QHBoxLayout *patternModeLayout;
    QCheckBox *patternHorizontalCheckBox;
    QCheckBox *patternVerticalCheckBox;

    void setupUi(QDialog *SLPreferenceDialog)
    {
        if (SLPreferenceDialog->objectName().isEmpty())
            SLPreferenceDialog->setObjectName(QString::fromUtf8("SLPreferenceDialog"));
        SLPreferenceDialog->resize(362, 619);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(SLPreferenceDialog->sizePolicy().hasHeightForWidth());
        SLPreferenceDialog->setSizePolicy(sizePolicy);
        buttonBox = new QDialogButtonBox(SLPreferenceDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(80, 570, 261, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);
        cameraLabel = new QLabel(SLPreferenceDialog);
        cameraLabel->setObjectName(QString::fromUtf8("cameraLabel"));
        cameraLabel->setGeometry(QRect(20, 280, 67, 17));
        projectorLabel = new QLabel(SLPreferenceDialog);
        projectorLabel->setObjectName(QString::fromUtf8("projectorLabel"));
        projectorLabel->setGeometry(QRect(20, 180, 131, 17));
        cameraComboBox = new QComboBox(SLPreferenceDialog);
        cameraComboBox->setObjectName(QString::fromUtf8("cameraComboBox"));
        cameraComboBox->setGeometry(QRect(20, 300, 321, 27));
        patternModeLabel = new QLabel(SLPreferenceDialog);
        patternModeLabel->setObjectName(QString::fromUtf8("patternModeLabel"));
        patternModeLabel->setGeometry(QRect(20, 80, 121, 17));
        patternModeComboBox = new QComboBox(SLPreferenceDialog);
        patternModeComboBox->setObjectName(QString::fromUtf8("patternModeComboBox"));
        patternModeComboBox->setGeometry(QRect(20, 100, 321, 27));
        projectorComboBox = new QComboBox(SLPreferenceDialog);
        projectorComboBox->setObjectName(QString::fromUtf8("projectorComboBox"));
        projectorComboBox->setGeometry(QRect(20, 200, 321, 27));
        triggerModeLabel = new QLabel(SLPreferenceDialog);
        triggerModeLabel->setObjectName(QString::fromUtf8("triggerModeLabel"));
        triggerModeLabel->setGeometry(QRect(20, 400, 121, 17));
        layoutWidget = new QWidget(SLPreferenceDialog);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(40, 420, 261, 24));
        triggerModeLayout = new QHBoxLayout(layoutWidget);
        triggerModeLayout->setObjectName(QString::fromUtf8("triggerModeLayout"));
        triggerModeLayout->setContentsMargins(0, 0, 0, 0);
        triggerHardwareRadioButton = new QRadioButton(layoutWidget);
        triggerHardwareRadioButton->setObjectName(QString::fromUtf8("triggerHardwareRadioButton"));
        triggerHardwareRadioButton->setChecked(true);

        triggerModeLayout->addWidget(triggerHardwareRadioButton);

        triggerSoftwareRadioButton = new QRadioButton(layoutWidget);
        triggerSoftwareRadioButton->setObjectName(QString::fromUtf8("triggerSoftwareRadioButton"));

        triggerModeLayout->addWidget(triggerSoftwareRadioButton);

        writeToDiskLabel = new QLabel(SLPreferenceDialog);
        writeToDiskLabel->setObjectName(QString::fromUtf8("writeToDiskLabel"));
        writeToDiskLabel->setGeometry(QRect(20, 500, 131, 17));
        layoutWidget1 = new QWidget(SLPreferenceDialog);
        layoutWidget1->setObjectName(QString::fromUtf8("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(40, 340, 201, 29));
        shutterLayout = new QHBoxLayout(layoutWidget1);
        shutterLayout->setObjectName(QString::fromUtf8("shutterLayout"));
        shutterLayout->setContentsMargins(0, 0, 0, 0);
        shutterLabel = new QLabel(layoutWidget1);
        shutterLabel->setObjectName(QString::fromUtf8("shutterLabel"));
        shutterLabel->setEnabled(true);

        shutterLayout->addWidget(shutterLabel);

        shutterDoubleSpinBox = new QDoubleSpinBox(layoutWidget1);
        shutterDoubleSpinBox->setObjectName(QString::fromUtf8("shutterDoubleSpinBox"));
        shutterDoubleSpinBox->setDecimals(3);
        shutterDoubleSpinBox->setMaximum(999.99);

        shutterLayout->addWidget(shutterDoubleSpinBox);

        delayMsLabel_2 = new QLabel(layoutWidget1);
        delayMsLabel_2->setObjectName(QString::fromUtf8("delayMsLabel_2"));

        shutterLayout->addWidget(delayMsLabel_2);

        layoutWidget2 = new QWidget(SLPreferenceDialog);
        layoutWidget2->setObjectName(QString::fromUtf8("layoutWidget2"));
        layoutWidget2->setGeometry(QRect(40, 450, 261, 31));
        triggerLayout = new QHBoxLayout(layoutWidget2);
        triggerLayout->setObjectName(QString::fromUtf8("triggerLayout"));
        triggerLayout->setContentsMargins(0, 0, 0, 0);
        shiftLayout = new QHBoxLayout();
        shiftLayout->setObjectName(QString::fromUtf8("shiftLayout"));
        shiftLabel = new QLabel(layoutWidget2);
        shiftLabel->setObjectName(QString::fromUtf8("shiftLabel"));

        shiftLayout->addWidget(shiftLabel);

        shiftSpinBox = new QSpinBox(layoutWidget2);
        shiftSpinBox->setObjectName(QString::fromUtf8("shiftSpinBox"));
        shiftSpinBox->setMinimum(0);

        shiftLayout->addWidget(shiftSpinBox);


        triggerLayout->addLayout(shiftLayout);

        delayLayout = new QHBoxLayout();
        delayLayout->setObjectName(QString::fromUtf8("delayLayout"));
        delayLabel = new QLabel(layoutWidget2);
        delayLabel->setObjectName(QString::fromUtf8("delayLabel"));
        delayLabel->setEnabled(true);

        delayLayout->addWidget(delayLabel);

        delaySpinBox = new QSpinBox(layoutWidget2);
        delaySpinBox->setObjectName(QString::fromUtf8("delaySpinBox"));
        delaySpinBox->setMaximum(9999);

        delayLayout->addWidget(delaySpinBox);

        delayMsLabel = new QLabel(layoutWidget2);
        delayMsLabel->setObjectName(QString::fromUtf8("delayMsLabel"));

        delayLayout->addWidget(delayMsLabel);


        triggerLayout->addLayout(delayLayout);

        layoutWidget3 = new QWidget(SLPreferenceDialog);
        layoutWidget3->setObjectName(QString::fromUtf8("layoutWidget3"));
        layoutWidget3->setGeometry(QRect(40, 520, 261, 24));
        writeToDiskLayout = new QHBoxLayout(layoutWidget3);
        writeToDiskLayout->setObjectName(QString::fromUtf8("writeToDiskLayout"));
        writeToDiskLayout->setContentsMargins(0, 0, 0, 0);
        pointCloudsCheckBox = new QCheckBox(layoutWidget3);
        pointCloudsCheckBox->setObjectName(QString::fromUtf8("pointCloudsCheckBox"));

        writeToDiskLayout->addWidget(pointCloudsCheckBox);

        trackingCheckBox = new QCheckBox(layoutWidget3);
        trackingCheckBox->setObjectName(QString::fromUtf8("trackingCheckBox"));

        writeToDiskLayout->addWidget(trackingCheckBox);

        layoutWidget4 = new QWidget(SLPreferenceDialog);
        layoutWidget4->setObjectName(QString::fromUtf8("layoutWidget4"));
        layoutWidget4->setGeometry(QRect(40, 240, 251, 24));
        diamondPatternLayout = new QHBoxLayout(layoutWidget4);
        diamondPatternLayout->setObjectName(QString::fromUtf8("diamondPatternLayout"));
        diamondPatternLayout->setContentsMargins(0, 0, 0, 0);
        diamondPatternCheckBox = new QCheckBox(layoutWidget4);
        diamondPatternCheckBox->setObjectName(QString::fromUtf8("diamondPatternCheckBox"));
        diamondPatternCheckBox->setChecked(true);

        diamondPatternLayout->addWidget(diamondPatternCheckBox);

        aquisitionLabel = new QLabel(SLPreferenceDialog);
        aquisitionLabel->setObjectName(QString::fromUtf8("aquisitionLabel"));
        aquisitionLabel->setGeometry(QRect(20, 20, 121, 17));
        layoutWidget_2 = new QWidget(SLPreferenceDialog);
        layoutWidget_2->setObjectName(QString::fromUtf8("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(40, 40, 267, 24));
        aquisitionLayout = new QHBoxLayout(layoutWidget_2);
        aquisitionLayout->setObjectName(QString::fromUtf8("aquisitionLayout"));
        aquisitionLayout->setContentsMargins(0, 0, 0, 0);
        aquisitioncontinuousRadioButton = new QRadioButton(layoutWidget_2);
        aquisitioncontinuousRadioButton->setObjectName(QString::fromUtf8("aquisitioncontinuousRadioButton"));
        aquisitioncontinuousRadioButton->setChecked(true);

        aquisitionLayout->addWidget(aquisitioncontinuousRadioButton);

        aquisitionSingleRadioButton = new QRadioButton(layoutWidget_2);
        aquisitionSingleRadioButton->setObjectName(QString::fromUtf8("aquisitionSingleRadioButton"));

        aquisitionLayout->addWidget(aquisitionSingleRadioButton);

        layoutWidget_3 = new QWidget(SLPreferenceDialog);
        layoutWidget_3->setObjectName(QString::fromUtf8("layoutWidget_3"));
        layoutWidget_3->setGeometry(QRect(40, 140, 261, 24));
        patternModeLayout = new QHBoxLayout(layoutWidget_3);
        patternModeLayout->setObjectName(QString::fromUtf8("patternModeLayout"));
        patternModeLayout->setContentsMargins(0, 0, 0, 0);
        patternHorizontalCheckBox = new QCheckBox(layoutWidget_3);
        patternHorizontalCheckBox->setObjectName(QString::fromUtf8("patternHorizontalCheckBox"));
        patternHorizontalCheckBox->setChecked(true);

        patternModeLayout->addWidget(patternHorizontalCheckBox);

        patternVerticalCheckBox = new QCheckBox(layoutWidget_3);
        patternVerticalCheckBox->setObjectName(QString::fromUtf8("patternVerticalCheckBox"));

        patternModeLayout->addWidget(patternVerticalCheckBox);

        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        layoutWidget->raise();
        buttonBox->raise();
        cameraLabel->raise();
        projectorLabel->raise();
        cameraComboBox->raise();
        patternModeLabel->raise();
        patternModeComboBox->raise();
        projectorComboBox->raise();
        triggerModeLabel->raise();
        writeToDiskLabel->raise();
        aquisitionLabel->raise();
        layoutWidget_2->raise();
        layoutWidget_3->raise();

        retranslateUi(SLPreferenceDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), SLPreferenceDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), SLPreferenceDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(SLPreferenceDialog);
    } // setupUi

    void retranslateUi(QDialog *SLPreferenceDialog)
    {
        SLPreferenceDialog->setWindowTitle(QApplication::translate("SLPreferenceDialog", "Preferences", 0, QApplication::UnicodeUTF8));
        cameraLabel->setText(QApplication::translate("SLPreferenceDialog", "Camera:", 0, QApplication::UnicodeUTF8));
        projectorLabel->setText(QApplication::translate("SLPreferenceDialog", "Projector:", 0, QApplication::UnicodeUTF8));
        patternModeLabel->setText(QApplication::translate("SLPreferenceDialog", "Pattern Mode:", 0, QApplication::UnicodeUTF8));
        triggerModeLabel->setText(QApplication::translate("SLPreferenceDialog", "Trigger Mode:", 0, QApplication::UnicodeUTF8));
        triggerHardwareRadioButton->setText(QApplication::translate("SLPreferenceDialog", "Hardware", 0, QApplication::UnicodeUTF8));
        triggerSoftwareRadioButton->setText(QApplication::translate("SLPreferenceDialog", "Software", 0, QApplication::UnicodeUTF8));
        writeToDiskLabel->setText(QApplication::translate("SLPreferenceDialog", "Write to disk:", 0, QApplication::UnicodeUTF8));
        shutterLabel->setText(QApplication::translate("SLPreferenceDialog", "Shutter:", 0, QApplication::UnicodeUTF8));
        delayMsLabel_2->setText(QApplication::translate("SLPreferenceDialog", "ms", 0, QApplication::UnicodeUTF8));
        shiftLabel->setText(QApplication::translate("SLPreferenceDialog", "Shift:", 0, QApplication::UnicodeUTF8));
        delayLabel->setText(QApplication::translate("SLPreferenceDialog", "Delay:", 0, QApplication::UnicodeUTF8));
        delayMsLabel->setText(QApplication::translate("SLPreferenceDialog", "ms", 0, QApplication::UnicodeUTF8));
        pointCloudsCheckBox->setText(QApplication::translate("SLPreferenceDialog", "Point Clouds", 0, QApplication::UnicodeUTF8));
        trackingCheckBox->setText(QApplication::translate("SLPreferenceDialog", "Tracking Data", 0, QApplication::UnicodeUTF8));
        diamondPatternCheckBox->setText(QApplication::translate("SLPreferenceDialog", "Diamond Pixel Pattern", 0, QApplication::UnicodeUTF8));
        aquisitionLabel->setText(QApplication::translate("SLPreferenceDialog", "Aquisition:", 0, QApplication::UnicodeUTF8));
        aquisitioncontinuousRadioButton->setText(QApplication::translate("SLPreferenceDialog", "Continuous", 0, QApplication::UnicodeUTF8));
        aquisitionSingleRadioButton->setText(QApplication::translate("SLPreferenceDialog", "Single Point Cloud", 0, QApplication::UnicodeUTF8));
        patternHorizontalCheckBox->setText(QApplication::translate("SLPreferenceDialog", "Horizontal", 0, QApplication::UnicodeUTF8));
        patternVerticalCheckBox->setText(QApplication::translate("SLPreferenceDialog", "Vertical", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SLPreferenceDialog: public Ui_SLPreferenceDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SLPREFERENCEDIALOG_H
