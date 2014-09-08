#include "SLAboutDialog.h"
#include "ui_SLAboutDialog.h"

SLAboutDialog::SLAboutDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SLAboutDialog)
{
    ui->setupUi(this);
}

SLAboutDialog::~SLAboutDialog()
{
    delete ui;
}
