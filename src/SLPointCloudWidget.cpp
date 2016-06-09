#include "SLPointCloudWidget.h"

#include "calibrator/CalibrationData.h"
#include <opencv2/core/eigen.hpp>

#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
#include <vtkRenderWindow.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/geometry/quad_mesh.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/vtk_io.h>
#include <vtkPolyDataWriter.h>
#include <pcl/conversions.h>

#include <fstream>

#include <QFileDialog>
#include <QKeyEvent>

SLPointCloudWidget::SLPointCloudWidget(QWidget *parent) : QVTKWidget(parent), surfaceReconstruction(false) {

    visualizer = new pcl::visualization::PCLVisualizer("PCLVisualizer", false);
    this->SetRenderWindow(visualizer->getRenderWindow());

    visualizer->setShowFPS(false);

    this->updateCalibration();

    // Create point cloud viewport
    visualizer->setBackgroundColor(255, 255, 255);
    visualizer->addCoordinateSystem(50, 0);
    visualizer->setCameraPosition(0,0,-50,0,0,0,0,-1,0);
    visualizer->setCameraClipDistances(0.1, 10000);
    // Initialize point cloud color handler
    colorHandler = new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>();

    // Initialize surface reconstruction objection
    reconstructor = new pcl::OrganizedFastMesh<pcl::PointXYZRGB>;

    //reconstructor->setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
    //reconstructor->setMaxEdgeLength(3.0);
    //reconstructor->setTrianglePixelSize(2);
    time.start();
}

void SLPointCloudWidget::updateCalibration(){

    CalibrationData calibration;
    bool load_result = calibration.load("calibration.xml");
	if (!load_result)
		return;
    // Camera coordinate system
    visualizer->addCoordinateSystem(50, "camera", 0);

    // Projector coordinate system
    cv::Mat TransformPCV(4, 4, CV_32F, 0.0);
    cv::Mat(calibration.Rp).copyTo(TransformPCV.colRange(0, 3).rowRange(0, 3));
    cv::Mat(calibration.Tp).copyTo(TransformPCV.col(3).rowRange(0, 3));
    TransformPCV.at<float>(3, 3) = 1.0; // make it homogeneous 
    Eigen::Affine3f TransformP;
    cv::cv2eigen(TransformPCV, TransformP.matrix());

    visualizer->addCoordinateSystem(50, TransformP.inverse(), "projector", 0);
}

void SLPointCloudWidget::keyPressEvent(QKeyEvent *event){

//    std::cout << event->key() << std::endl;
    // Switch between color handlers
    switch(event->key()){
        case '1':
            surfaceReconstruction = false;
            visualizer->removePolygonMesh("meshPCL");
            colorHandler = new pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>();
            break;
        case '2':
            surfaceReconstruction = false;
            visualizer->removePolygonMesh("meshPCL");
            colorHandler = new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB>("z");
            break;
        case '3':
            surfaceReconstruction = true;
            visualizer->removePointCloud("pointCloudPCL");
            break;
    }

    updatePointCloud(pointCloudPCL);

    if(!(('0' <= event->key()) & (event->key() <= '9')))
        QVTKWidget::keyPressEvent(event);
}

void SLPointCloudWidget::updatePointCloud(PointCloudConstPtr _pointCloudPCL){

    if(!_pointCloudPCL || _pointCloudPCL->points.empty())
        return;

//    time.restart();

    pointCloudPCL = _pointCloudPCL;

    if(surfaceReconstruction){
        reconstructor->setInputCloud(pointCloudPCL);
        std::vector<pcl::Vertices> polygons;
        reconstructor->reconstruct(polygons);
        if(!visualizer->updatePolygonMesh<pcl::PointXYZRGB>(pointCloudPCL, polygons, "meshPCL")){
            visualizer->addPolygonMesh<pcl::PointXYZRGB>(pointCloudPCL, polygons, "meshPCL");
         }
    } else {
        // Note: using the color handler makes a copy of the rgb fields
        colorHandler->setInputCloud(pointCloudPCL);
        if(!visualizer->updatePointCloud(pointCloudPCL, *colorHandler, "pointCloudPCL")){
            visualizer->addPointCloud(pointCloudPCL, *colorHandler, "pointCloudPCL");
            visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "pointCloudPCL");
         }
    }

    this->update();
    emit newPointCloudDisplayed();

//    std::cout << "PCL Widget: " << time.restart() << "ms" << std::endl;
}

void SLPointCloudWidget::savePointCloud(){

    QString selectedFilter;
    QString fileName = QFileDialog::getSaveFileName(this, "Save Point Cloud", QString(), "*.pcd;;*.ply;;*.vtk;;*.png;;*.txt", &selectedFilter);
    QFileInfo info(fileName);
    QString type = info.suffix();
    if(type == ""){
        fileName.append(selectedFilter.remove(0,1));
        type = selectedFilter.remove(0,1);
    }

    if(type == "pcd"){
        pcl::io::savePCDFileASCII(fileName.toStdString(), *pointCloudPCL);
    } else if(type == "ply"){
        //pcl::io::savePLYFileBinary( fileName.toStdString(), *pointCloudPCL);
        pcl::PLYWriter w;
        // Write to ply in binary without camera
        w.write<pcl::PointXYZRGB> (fileName.toStdString(), *pointCloudPCL, true, false);
    } else if(type == "vtk"){
        pcl::PCLPointCloud2 pointCloud2;
        pcl::toPCLPointCloud2(*pointCloudPCL, pointCloud2);
        pcl::io::saveVTKFile(fileName.toStdString(), pointCloud2);

//        vtkPolyData polyData;
//        pcl::io::pointCloudTovtkPolyData(*pointCloudPCL, polyData);
//        vtkPolyDataWriter::Pointer writer = vtkPolyDataWriter::New();
//        writer->SetInput(polyData);
//        writer->SetFileName(fileName.toStdString());
//        writer->Update();
    } else if(type == "png"){
        pcl::io::savePNGFile(fileName.toStdString(), *pointCloudPCL, "rgb");
    } else if(type == "txt"){
        std::ofstream s(fileName.toLocal8Bit());
        for(unsigned int r=0; r<pointCloudPCL->height; r++){
            for(unsigned int c=0; c<pointCloudPCL->width; c++){
                pcl::PointXYZRGB p = pointCloudPCL->at(c, r);
                if(p.x == p.x)
                    s << p.x << " " << p.y << " " << p.z << "\r\n";
            }
        }
        std::flush(s);
        s.close();
    }

}

void SLPointCloudWidget::saveScreenShot(){

    vtkWindowToImageFilter* filter = vtkWindowToImageFilter::New();
    //filter->SetInput(visualizer->getRenderWindow());
    filter->Modified();

    QString fileName = QFileDialog::getSaveFileName(this, "Save Screen Shot", QString(), "*.png");
    QFileInfo info(fileName);
    QString type = info.suffix();
    if(type == "")
        fileName.append(".png");

    vtkPNGWriter* writer = vtkPNGWriter::New();
    writer->SetInputConnection(0, filter->GetOutputPort(0)); 
    writer->SetFileName(qPrintable(fileName));
    writer->Write();
    writer->Delete();
}

SLPointCloudWidget::~SLPointCloudWidget(){

    //delete visualizer;

}
