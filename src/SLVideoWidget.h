/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLVIDEOWIDGET_H
#define SLVIDEOWIDGET_H

#include "glew.h"
#include <QGLWidget>
//#include <QOpenGLShaderProgram> Qt 5
#include <QtOpenGL/QGLShaderProgram>
//#include <QGraphicsScene>
#include "Camera.h"

#include <opencv2/opencv.hpp>

class SLVideoWidget : public QGLWidget {
    Q_OBJECT
    public:
        explicit SLVideoWidget(QWidget *parent = 0) : QGLWidget(parent){}
        void initializeGL();
        void resizeGL(int w, int h);

    signals:

    public slots:
        void showFrame(CameraFrame frame);
        void showFrameCV(cv::Mat frame);
        void paintGL();
    protected:
        void setGrayScale(bool enable);
    private:
        QGLShaderProgram *textureSwizzleProgram;
        bool grayScale;
        float aspectRatioTexture;
        float aspectRatioWidget;
};

#endif
