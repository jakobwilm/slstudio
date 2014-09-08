#ifndef CAMERAVIEW_H
#define CAMERAVIEW_H

#include <QDialog>
#include "Camera.h"

#include "CameraWorker.h"

namespace Ui {
class CameraTest;
}

class CameraTest : public QDialog
{
    Q_OBJECT
    
public:
    explicit CameraTest(QWidget *parent = 0);
    ~CameraTest();
    void closeEvent(QCloseEvent *);
private:
    Ui::CameraTest *ui;
    Camera *camera;
    QThread *cameraThread;
    CameraWorker *cameraWorker;
};

#endif // CAMERAVIEW_H
