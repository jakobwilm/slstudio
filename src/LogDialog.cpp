#include "LogDialog.h"
#include "ui_LogDialog.h"
#include <iostream>

LogDialog::LogDialog(QWidget *parent) : QDialog(parent), ui(new Ui::LogDialog) {
  ui->setupUi(this);

  ui->plainTextEdit->setFont({"Monospace"});

  // set this to be the buffer of std::cout and std::cerr
  std::cout.rdbuf(this);
  std::cerr.rdbuf(this);

  connect(this, &LogDialog::logMessage, this, &LogDialog::onlogMessage);

  // Create QDockWidget like action associated with dialog
  action = new QAction("Log", this);
  action->setCheckable(true);
  connect(action, &QAction::toggled, this, &QDialog::setVisible);
}

// QDockWidget like, return a checkable action in sync with visibility
QAction *LogDialog::toggleViewAction() { return action; }

void LogDialog::showEvent(QShowEvent *) {
  if (!action->isChecked()) {
    action->setChecked(true);
  }
}

LogDialog::~LogDialog() { delete ui; }

void LogDialog::onlogMessage(const QString &msg) {
  ui->plainTextEdit->appendPlainText(msg);
  ui->plainTextEdit->moveCursor(QTextCursor::End, QTextCursor::MoveAnchor);
}

int LogDialog::overflow(int v) {

  if (v == '\n') {
    emit logMessage(QString::fromStdString(m_string));
    // ui->plainTextEdit->appendPlainText(QString::fromStdString(m_string));
    m_string.erase(m_string.begin(), m_string.end());

  } else {
    m_string += v;
  }
  return v;
}

std::streamsize LogDialog::xsputn(const char *p, std::streamsize n) {
  m_string.append(p, p + n);
  long pos = 0;
  while (pos != static_cast<long>(std::string::npos)) {
    pos = static_cast<long>(m_string.find('\n'));
    if (pos != static_cast<long>(std::string::npos)) {
      std::string tmp(m_string.begin(), m_string.begin() + pos);
      emit logMessage(QString::fromStdString(tmp));
      // ui->plainTextEdit->appendPlainText(QString::fromStdString(tmp));
      m_string.erase(m_string.begin(), m_string.begin() + pos + 1);
    }
  }
  return n;
}

void LogDialog::on_LogDialog_finished(int result) {
  Q_UNUSED(result)
  action->setChecked(false);
}
