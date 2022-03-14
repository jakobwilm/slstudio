/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#ifndef SLPOSEWIDGET_H
#define SLPOSEWIDGET_H

#ifndef Q_MOC_RUN
#include <Eigen/Eigen>
#include <QVTKWidget.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif

class SLPoseWidget : public QVTKWidget {
  Q_OBJECT
public:
  explicit SLPoseWidget(QWidget *parent = 0);
  ~SLPoseWidget();
signals:

public slots:
  void showPoseEstimate(Eigen::Affine3f T);

private:
  pcl::visualization::PCLVisualizer *visualizer;
};

#endif // SLPOSEWIDGET_H
