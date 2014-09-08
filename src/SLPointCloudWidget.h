/*
 *
 SLStudio - Platform for Real-Time  Structured Light
 (c) 2013 -- 2014 Jakob Wilm, DTU, Kgs.Lyngby, Denmark
 *
*/

#ifndef SLPOINTCLOUDWIDGET_H
#define SLPOINTCLOUDWIDGET_H

#include <QTime>

#ifndef Q_MOC_RUN
    #include <QVTKWidget.h>
    #include <pcl/visualization/pcl_visualizer.h>
    #include <pcl/surface/organized_fast_mesh.h>
    #include <Eigen/Eigen>
#endif

#include <opencv2/opencv.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr PointCloudConstPtr;

class SLPointCloudWidget : public QVTKWidget {
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
    signals:
        void newPointCloudDisplayed();
    private:
        pcl::visualization::PCLVisualizer *visualizer;
        PointCloudConstPtr pointCloudPCL;
        pcl::visualization::PointCloudColorHandler<pcl::PointXYZRGB>* colorHandler;
        bool surfaceReconstruction;
        pcl::OrganizedFastMesh<pcl::PointXYZRGB> *reconstructor;
        QTime time;
};

#endif // SLPOINTCLOUDWIDGET_H
