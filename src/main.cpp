#include "SLStudio.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SLStudio w;
    w.show();
    
    return a.exec();
}
