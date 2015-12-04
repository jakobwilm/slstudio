#include "SLVideoDialog.h"
#include "ui_SLVideoDialog.h"

#include <QSettings>

SLVideoDialog::SLVideoDialog(const QString &title, QWidget *parent) : QDialog(parent),ui(new Ui::SLVideoDialog){
    ui->setupUi(this);

    //this->setWindowFlags(Qt::Window);

    // Set window title
    this->setWindowTitle(title);

    // Create QDockWidget like action associated with dialog
    action = new QAction(title, this);
    action->setCheckable(true);
    connect(action, SIGNAL(toggled(bool)), this, SLOT(setShown(bool)));
}

// QDockWidget like, return a checkable action in sync with visibility
QAction* SLVideoDialog::toggleViewAction(){
    return action;
}

void SLVideoDialog::showImageCV(cv::Mat image){
    if(!action->isChecked())
        return;

    ui->videoWidget->showFrameCV(image);

}

void SLVideoDialog::showImageSeqCV(std::vector<cv::Mat> imageSeq){
    if(!action->isChecked())
        return;

    cv::Mat image;
    cv::hconcat(imageSeq, image);

    ui->videoWidget->showFrameCV(image);

}

void SLVideoDialog::showEvent(QShowEvent *){
    if(!action->isChecked())
        action->setChecked(true);
}

void SLVideoDialog::closeEvent(QCloseEvent *){
    action->setChecked(false);
}

SLVideoDialog::~SLVideoDialog(){
    delete ui;

}
