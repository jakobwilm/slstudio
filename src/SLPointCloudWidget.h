/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) Jakob Wilm, jw@vision-consulting.dk
 *
*/

#ifndef SLPOINTCLOUDWIDGET_H
#define SLPOINTCLOUDWIDGET_H

#include <QElapsedTimer>

#ifndef Q_MOC_RUN
#include <Eigen/Eigen>
#include <QVTKRenderWidget.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif

#include <opencv2/opencv.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr PointCloudConstPtr;

class SLPointCloudWidget : public QVTKRenderWidget {
  Q_OBJECT
public:
  explicit SLPointCloudWidget(QWidget *parent = 0);
  ~SLPointCloudWidget();

protected:
  void keyPressEvent(QKeyEvent *event);
public slots:
  void updatePointCloud(PointCloudConstPtr _pointCloudPCL);
  void savePointCloud();
  void saveScreenShot();
  void updateCalibration();
signals:
  void newPointCloudDisplayed();

private:
  pcl::visualization::PCLVisualizer *visualizer;
  PointCloudConstPtr pointCloudPCL;
  pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB> *colorHandler;
  bool surfaceReconstruction;
  pcl::OrganizedFastMesh<pcl::PointXYZRGB> *reconstructor;
  QElapsedTimer timer;
};

#endif // SLPOINTCLOUDWIDGET_H
