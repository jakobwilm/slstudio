/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#ifndef SLABOUTDIALOG_H
#define SLABOUTDIALOG_H

#include <QDialog>

namespace Ui {
class SLAboutDialog;
}

class SLAboutDialog : public QDialog {
  Q_OBJECT

public:
  explicit SLAboutDialog(QWidget *parent = 0);
  ~SLAboutDialog();

private:
  Ui::SLAboutDialog *ui;
};

#endif // SLABOUTDIALOG_H
