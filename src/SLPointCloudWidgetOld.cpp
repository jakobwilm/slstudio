#include "SLPointCloudWidget.h"
#include <cmath>
#ifdef __APPLE__
    #include <OpenGL/glu.h>
#else
    #include <GL/glu.h>
#endif

void SLPointCloudWidget::updatePointCloud(cv::Mat _pointCloud, cv::Mat _shading){
    pointCloud = _pointCloud;
    shading = _shading;

    this->updateGL();
    emit newPointCloudDisplayed();

}

void SLPointCloudWidget::initializeGL() {

    glClearColor(0.0, 0.0, 0.0, 0.0);

}

void SLPointCloudWidget::resizeGL(int w, int h) {
    //TODO: emulate camera viewpoint
    glViewport(0,0,w,h);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(26, (float)w/(float)h, 0, 1000);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);

}

void SLPointCloudWidget::paintGL(){

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
    glTranslatef(0.0, 0.0, 250.0);
    glRotatef(rotationX, 1.0, 0.0, 0.0);
    glRotatef(rotationY, 0.0, 1.0, 0.0);
    glRotatef(rotationZ, 0.0, 0.0, 1.0);
    glTranslatef(0.0, 0.0, -250.0);

    glClear(GL_COLOR_BUFFER_BIT);

    // Draw coordinate system
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINES);
        glVertex3i(0,0,0);
        glVertex3i(100,0,0);
    glEnd();
    glColor3f(0.0, 1.0, 0.0);
    glBegin(GL_LINES);
        glVertex3i(0,0,0);
        glVertex3i(0,100,0);
    glEnd();
    glColor3f(0.0, 0.0, 1.0);
    glBegin(GL_LINES);
        glVertex3i(0,0,0);
        glVertex3i(0,0,100);
    glEnd();

    // Draw points
    glColor3f(1.0, 1.0, 1.0);

    glBegin(GL_POINTS);
    for (int row = 0; row<pointCloud.rows; row++){
        for (int col = 0; col<pointCloud.cols; col++){
            const cv::Vec3f pnt = pointCloud.at<cv::Vec3f>(row,col);
            const float shade = shading.at<float>(row,col);
                glColor3f(shade, shade, shade);
                glVertex3f(pnt[0], pnt[1], pnt[2]);
        }
    }
    glEnd();
}


void SLPointCloudWidget::mousePressEvent(QMouseEvent *event){
    lastMousePos = event->pos();
}
void SLPointCloudWidget::mouseMoveEvent(QMouseEvent *event){
    GLfloat dx = GLfloat(event->x() - lastMousePos.x()) / width();
    GLfloat dy = GLfloat(event->y() - lastMousePos.y()) / height();
    if (event->buttons() & Qt::LeftButton) {
        rotationX += 50 * dy;
        rotationY += 50 * dx;
        updateGL();
    } else if (event->buttons() & Qt::RightButton) {
        rotationX += 50 * dy;
        rotationZ += 50 * dx;
        updateGL();
    }
    lastMousePos = event->pos();
}
void SLPointCloudWidget::mouseDoubleClickEvent(QMouseEvent*){
    rotationX = 0;
    rotationY = 0;
    rotationZ = 0;
    updateGL();
}
