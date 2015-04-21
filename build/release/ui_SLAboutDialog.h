/********************************************************************************
** Form generated from reading UI file 'SLAboutDialog.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_SLABOUTDIALOG_H
#define UI_SLABOUTDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QDialogButtonBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QTextEdit>

QT_BEGIN_NAMESPACE

class Ui_SLAboutDialog
{
public:
    QDialogButtonBox *buttonBox;
    QLabel *label;
    QTextEdit *textEdit;

    void setupUi(QDialog *SLAboutDialog)
    {
        if (SLAboutDialog->objectName().isEmpty())
            SLAboutDialog->setObjectName(QString::fromUtf8("SLAboutDialog"));
        SLAboutDialog->resize(389, 248);
        buttonBox = new QDialogButtonBox(SLAboutDialog);
        buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
        buttonBox->setGeometry(QRect(30, 190, 341, 32));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Close);
        label = new QLabel(SLAboutDialog);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(290, 20, 71, 101));
        label->setPixmap(QPixmap(QString::fromUtf8(":/resources/logo.gif")));
        label->setScaledContents(true);
        textEdit = new QTextEdit(SLAboutDialog);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));
        textEdit->setGeometry(QRect(3, 7, 261, 221));

        retranslateUi(SLAboutDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), SLAboutDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), SLAboutDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(SLAboutDialog);
    } // setupUi

    void retranslateUi(QDialog *SLAboutDialog)
    {
        SLAboutDialog->setWindowTitle(QApplication::translate("SLAboutDialog", "About", 0, QApplication::UnicodeUTF8));
        label->setText(QString());
        textEdit->setHtml(QApplication::translate("SLAboutDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">SLStudio v. 1.0</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Open source software for Structured Light with a single camera/projector pair.</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" marg"
                        "in-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">(c) GPL 2014, Jakob Wilm, Image Analysis and Computer Graphics, DTU Compute, Denmark.</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">jakw@dtu.dk</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">http://compute.dtu.dk/~jakw/</p></body></html>", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class SLAboutDialog: public Ui_SLAboutDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_SLABOUTDIALOG_H
