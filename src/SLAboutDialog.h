/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLABOUTDIALOG_H
#define SLABOUTDIALOG_H

#include <QDialog>

namespace Ui {
class SLAboutDialog;
}

class SLAboutDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SLAboutDialog(QWidget *parent = 0);
    ~SLAboutDialog();

private:
    Ui::SLAboutDialog *ui;
};

#endif // SLABOUTDIALOG_H
