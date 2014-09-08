#include "CameraTest.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    CameraTest w;
    w.show();

    return a.exec();
}
