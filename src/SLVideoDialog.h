/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLVideoDialog_H
#define SLVideoDialog_H

#include <QDialog>

#include <opencv2/opencv.hpp>

namespace Ui {
class SLVideoDialog;
}

class SLVideoDialog : public QDialog{
    Q_OBJECT

    public:
        explicit SLVideoDialog(const QString &title, QWidget *parent = 0);
        ~SLVideoDialog();
        QAction* toggleViewAction();
        void showEvent(QShowEvent *);
        void closeEvent(QCloseEvent *);
    public slots:
        void showImageCV(cv::Mat image);

    private:
        Ui::SLVideoDialog *ui;
        QAction *action;
};

#endif // SLVideoDialog_H
