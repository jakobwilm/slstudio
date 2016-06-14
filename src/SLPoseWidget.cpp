#include "SLPoseWidget.h"
#include <pcl/geometry/quad_mesh.h>

#include <vtkRenderWindow.h>

SLPoseWidget::SLPoseWidget(QWidget *parent) : QVTKWidget(parent) {

    visualizer = new pcl::visualization::PCLVisualizer("PCLVisualizer", false);
    this->SetRenderWindow(visualizer->getRenderWindow());

    visualizer->setShowFPS(false);
    this->setMouseTracking(false);

    // Create pose viewport
    visualizer->setBackgroundColor(0.3, 0.3, 0.3);

    // Add ply model
    bool success = visualizer->addModelFromPLYFile("/home/jakw/Code/Repos/SLStudio/src/resources/head.ply");

    if(!success){
        // Show cube as a fallback
        pcl::PointCloud<pcl::PointXYZ> square;
        square.resize(4);
        square[0] = pcl::PointXYZ(-10,-10,-10);
        square[1] = pcl::PointXYZ(10,-10,-10);
        square[2] = pcl::PointXYZ(10,10,-10);
        square[3] = pcl::PointXYZ(-10,10,-10);
        pcl::PlanarPolygon<pcl::PointXYZ> anterior;
        anterior.setContour(square);
        visualizer->addPolygon(anterior, 1.0, 0.0, 0.0, "anterior");

        square[0] = pcl::PointXYZ(-10,-10,10);
        square[1] = pcl::PointXYZ(10,-10,10);
        square[2] = pcl::PointXYZ(10,10,10);
        square[3] = pcl::PointXYZ(-10,10,10);
        pcl::PlanarPolygon<pcl::PointXYZ> posterior;
        posterior.setContour(square);
        visualizer->addPolygon(posterior, 1.0, 0.0, 0.0, "posterior");

        square[0] = pcl::PointXYZ(-10,-10,-10);
        square[1] = pcl::PointXYZ(-10,10,-10);
        square[2] = pcl::PointXYZ(-10,10,10);
        square[3] = pcl::PointXYZ(-10,-10,10);
        pcl::PlanarPolygon<pcl::PointXYZ> left;
        left.setContour(square);
        visualizer->addPolygon(left, 0.0, 1.0, 0.0, "left");

        square[0] = pcl::PointXYZ(10,-10,-10);
        square[1] = pcl::PointXYZ(10,10,-10);
        square[2] = pcl::PointXYZ(10,10,10);
        square[3] = pcl::PointXYZ(10,-10,10);
        pcl::PlanarPolygon<pcl::PointXYZ> right;
        right.setContour(square);
        visualizer->addPolygon(right, 0.0, 1.0, 0.0, "right");

        square[0] = pcl::PointXYZ(-10,10,-10);
        square[1] = pcl::PointXYZ(10,10,-10);
        square[2] = pcl::PointXYZ(10,10,10);
        square[3] = pcl::PointXYZ(-10,10,10);
        pcl::PlanarPolygon<pcl::PointXYZ> superior;
        superior.setContour(square);
        visualizer->addPolygon(superior, 0.0, 0.0, 1.0, "superior");

        square[0] = pcl::PointXYZ(-10,-10,-10);
        square[1] = pcl::PointXYZ(10,-10,-10);
        square[2] = pcl::PointXYZ(10,-10,10);
        square[3] = pcl::PointXYZ(-10,-10,10);
        pcl::PlanarPolygon<pcl::PointXYZ> inferior;
        inferior.setContour(square);
        visualizer->addPolygon(inferior, 0.0, 0.0, 1.0, "inferior");
    }

//    visualizer->setRepresentationToSurfaceForAllActors();
//#ifndef __APPLE__
    visualizer->setCameraPosition(0,0,-150,0,0,0,0,-1,0);
//#endif


}

void SLPoseWidget::showPoseEstimate(const Eigen::Affine3f & T){

    //SEGFAULT!!
//    visualizer->updateShapePose("cube", T);

//    // Show only rotation
//    T.translation() = Eigen::Vector3f(0.0, 0.0, 0.0);

    // Workaround
    Eigen::Vector3f posVector = T.rotation() * Eigen::Vector3f(0,0,-150);
    Eigen::Vector3f lookAtVector = -posVector;
    Eigen::Vector3f upVector = T.rotation() * Eigen::Vector3f(0, -1, 0);
    visualizer->setCameraPosition(posVector[0], posVector[1], posVector[2],
                                lookAtVector[0], lookAtVector[1], lookAtVector[2],
                                upVector[0], upVector[1], upVector[2]);

//    // Workaround 2
//    double dummy;
//    if(!visualizer->getPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, dummy, "pointCloudPose"))
//        visualizer->addPointCloud(pointCloudPCL, *colorHandler, "pointCloudPose", viewPortPose);
//    visualizer->updatePointCloudPose("pointCloudPose", T);

    this->update();
}


SLPoseWidget::~SLPoseWidget(){

    //delete visualizer;

}
