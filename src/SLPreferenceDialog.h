/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#ifndef SLPREFERENCEDIALOG_H
#define SLPREFERENCEDIALOG_H

#include <QDialog>

namespace Ui {
    class SLPreferenceDialog;
}

class SLPreferenceDialog : public QDialog{
    Q_OBJECT
    
    public:
        explicit SLPreferenceDialog(QWidget *parent = 0);
        ~SLPreferenceDialog();

    private slots:
        void on_buttonBox_accepted();

        void on_triggerHardwareRadioButton_clicked();

        void on_triggerSoftwareRadioButton_clicked();

        void on_cameraComboBox_currentIndexChanged(const QString &arg1);

        void on_patternHorizontalCheckBox_clicked();

        void on_patternVerticalCheckBox_clicked();

private:
        Ui::SLPreferenceDialog *ui;
};

#endif // SLPREFERENCEDIALOG_H
