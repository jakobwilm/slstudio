#include "VideoDialog.h"
#include "ui_VideoDialog.h"

#include <QAction>
#include <QSettings>

VideoDialog::VideoDialog(const QString &title, QWidget *parent)
    : QDialog(parent), ui(new Ui::VideoDialog) {
  ui->setupUi(this);

  // this->setWindowFlags(Qt::Window);

  // Set window title
  this->setWindowTitle(title);

  // Create QDockWidget like action associated with dialog
  action = new QAction(title, this);
  action->setCheckable(true);
  connect(action, &QAction::toggled, this, &QDialog::setVisible);
}

// QDockWidget like, return a checkable action in sync with visibility
QAction *VideoDialog::toggleViewAction() { return action; }

void VideoDialog::showImageCV(cv::Mat image) {
  if (!action->isChecked())
    return;

  ui->videoWidget->showFrameCV(image);
}

void VideoDialog::showImageSeqCV(std::vector<cv::Mat> imageSeq) {
  if (!action->isChecked())
    return;

  cv::Mat image;
  cv::hconcat(imageSeq, image);

  ui->videoWidget->showFrameCV(image);
}

void VideoDialog::showEvent(QShowEvent *) {
  if (!action->isChecked()) {
    action->setChecked(true);
  }
}

void VideoDialog::closeEvent(QCloseEvent *) { action->setChecked(false); }

VideoDialog::~VideoDialog() { delete ui; }
