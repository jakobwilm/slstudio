/*
 *
 MVTracker - Multi View Tracking for Pose Estimation
 (c) 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLTraceWidget_H
#define SLTraceWidget_H

#include <QLabel>

#include <opencv2/opencv.hpp>

class SLTraceWidget : public QLabel {
    Q_OBJECT
    public:
        explicit SLTraceWidget(QWidget *parent = 0);
        ~SLTraceWidget();
    public slots:
        void addTrace(QString name);
        void addMeasurement(unsigned int id, float val);
        void addMeasurement(QString name, float val);
        void addMeasurements(std::vector<float> vals);
        void setBounds(float _tSpan, float _yMin, float _yMax);
        void draw();
    private:
        float tSpan, yMin, yMax;
        std::vector< QString > names;
        std::vector< std::vector<float> > traces;
        std::vector< cv::Scalar > colors;
};

#endif // SLTraceWidget_H
