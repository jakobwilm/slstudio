/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include <QDialog>

namespace Ui {
class PreferenceDialog;
}

class PreferenceDialog : public QDialog {
  Q_OBJECT

public:
  explicit PreferenceDialog(QWidget *parent = 0);
  ~PreferenceDialog();

private slots:
  void on_buttonBox_accepted();

  void on_triggerHardwareRadioButton_clicked();

  void on_triggerSoftwareRadioButton_clicked();

  void on_cameraComboBox_currentIndexChanged(const QString &arg1);

  void on_patternHorizontalCheckBox_clicked();

  void on_patternVerticalCheckBox_clicked();

private:
  Ui::PreferenceDialog *ui;
};
