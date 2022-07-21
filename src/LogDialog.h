/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#pragma once

#include <QAction>
#include <QDialog>

namespace Ui {
class LogDialog;
}

class LogDialog : public QDialog, public std::basic_streambuf<char> {
  Q_OBJECT

public:
  explicit LogDialog(QWidget *parent = nullptr);
  QAction *toggleViewAction();
  void showEvent(QShowEvent *);
  ~LogDialog();
public slots:
  void onlogMessage(const QString &msg);

private slots:
  void on_LogDialog_finished(int result);
signals:
  void logMessage(const QString &msg);

private:
  Ui::LogDialog *ui;
  QAction *action;
  std::string m_string;

  int overflow(int v) override;
  std::streamsize xsputn(const char *p, std::streamsize n) override;
};
