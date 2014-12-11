#include "SLTraceWidget.h"

SLTraceWidget::SLTraceWidget(QWidget *parent): QLabel(parent), tSpan(100), yMin(-10), yMax(10){

    colors.push_back(cv::Scalar(255, 0, 0));
    colors.push_back(cv::Scalar(0, 255, 0));
    colors.push_back(cv::Scalar(0, 0, 255));
    colors.push_back(cv::Scalar(255, 255, 0));
}

void SLTraceWidget::addTrace(QString name){
    names.push_back(name);
    traces.push_back(std::vector<float>());
}

void SLTraceWidget::addMeasurements(std::vector<float> vals){

    if(vals.size() != traces.size())
        return;

    for(unsigned int i=0; i<vals.size(); i++)
        this->addMeasurement(i, vals[i]);
}

void SLTraceWidget::addMeasurement(unsigned int id, float val){

    if(id > traces.size())
        return;

    traces[id].push_back(val);

    if(traces[id].size() > tSpan)
        traces[id].erase(traces[id].begin());

}

void SLTraceWidget::addMeasurement(QString name, float val){
    for(unsigned int i=0; i<names.size(); i++){
        if(names[i].compare(name) == 0){
            addMeasurement(i, val);
            return;
        }
    }
}

void SLTraceWidget::setBounds(float _tSpan, float _yMin, float _yMax){
    tSpan = _tSpan;
    yMin = _yMin;
    yMax = _yMax;
}

void SLTraceWidget::draw(){

    // Primitive: image the size of coordinate system...
    cv::Mat im(300, 500, CV_8UC3);
    im.setTo(0.0);

    // Draw traces
    for(unsigned int i=0; i<traces.size(); i++){

        cv::Scalar color = colors[i % 4];

        for(unsigned int j=1; j< traces[i].size(); j++){

            float xLast = float(j-1)/tSpan * im.cols;
            float x = float(j)/tSpan * im.cols;

            float yLast = im.rows - ((traces[i][j-1]-yMin)/(yMax-yMin) * im.rows);
            float y = im.rows - ((traces[i][j]-yMin)/(yMax-yMin) * im.rows);

            cv::line(im, cv::Point2f(xLast, yLast), cv::Point2f(x, y), color, 2, 4);
        }

        // Write label
        cv::putText(im, names[i].toStdString(), cv::Point(10, (i+1)*20), cv::FONT_HERSHEY_PLAIN, 1.0, color, 1.8);
    }

    QImage img((const uchar*)im.data, im.cols, im.rows, im.step, QImage::Format_RGB888);
    img = img.rgbSwapped();
    QPixmap pix = QPixmap::fromImage(img);
    this->setPixmap(pix);
}

SLTraceWidget::~SLTraceWidget(){


}
