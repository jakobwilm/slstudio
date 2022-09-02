#include "PointCloudWidget.h"

#include "CalibrationData.h"
#include <opencv2/core/eigen.hpp>

#include <vtkPNGWriter.h>
#include <vtkRenderWindow.h>
#include <vtkWindowToImageFilter.h>

#include <pcl/common/io.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <fstream>
#include <pcl/conversions.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/vtk_io.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPolyDataWriter.h>

#include <QFileDialog>
#include <QKeyEvent>

PointCloudWidget::PointCloudWidget(QWidget *parent)
    : QVTKRenderWidget(parent), surfaceReconstruction(false) {

  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->GlobalWarningDisplayOff();

  renderWindow->AddRenderer(renderer);
  visualizer = new pcl::visualization::PCLVisualizer(renderer, renderWindow,
                                                     "PCLVisualizer", false);
  this->setRenderWindow(visualizer->getRenderWindow());

  // visualizer->setupInteractor(this->interactor(), this->renderWindow());

  // Disable double buffering (which is enabled per default in VTK6)
  visualizer->getRenderWindow()->SetDoubleBuffer(1);
  visualizer->getRenderWindow()->SetErase(1);
  // visualizer->setUseVbos(true);

  visualizer->setShowFPS(false);
  this->setUpdatesEnabled(true);

  this->updateCalibration();

  // Create point cloud viewport
  visualizer->setBackgroundColor(0, 0, 0);
  visualizer->addCoordinateSystem(50, "camera", 0);
  visualizer->setCameraPosition(0, 0, -50, 0, 0, 0, 0, -1, 0);
  visualizer->setCameraClipDistances(0.1, 10000);
  // Initialize point cloud color handler
  colorHandler = new pcl::visualization::PointCloudColorHandlerRGBField<
      pcl::PointXYZRGB>();

  // Initialize surface reconstruction objection
  reconstructor = new pcl::OrganizedFastMesh<pcl::PointXYZRGB>;

  //
  reconstructor->setTriangulationType(
      pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
  // reconstructor->setMaxEdgeLength(3.0);
  // reconstructor->setTrianglePixelSize(2);
  timer.start();
}

void PointCloudWidget::updateCalibration() {

  CalibrationData calibration;
  calibration.load("calibration.xml");

  visualizer->removeCoordinateSystem("camera", 0);
  visualizer->removeCoordinateSystem("projector", 0);

  // Camera coordinate system
  visualizer->addCoordinateSystem(50, "camera", 0);

  // Projector coordinate system
  cv::Mat TransformPCV(3, 4, CV_32F);
  cv::Mat(calibration.Rp).copyTo(TransformPCV.colRange(0, 3));
  cv::Mat(calibration.Tp).copyTo(TransformPCV.col(3));
  Eigen::Affine3f TransformP;
  cv::cv2eigen(TransformPCV, TransformP.matrix());

  visualizer->addCoordinateSystem(50, TransformP.inverse(), "projector", 0);
}

void PointCloudWidget::keyPressEvent(QKeyEvent *event) {

  //    std::cout << event->key() << std::endl;
  // Switch between color handlers
  switch (event->key()) {
  case '1':
    surfaceReconstruction = false;
    visualizer->removePolygonMesh("meshPCL");
    colorHandler = new pcl::visualization::PointCloudColorHandlerRGBField<
        pcl::PointXYZRGB>();
    break;
  case '2':
    surfaceReconstruction = false;
    visualizer->removePolygonMesh("meshPCL");
    colorHandler = new pcl::visualization::PointCloudColorHandlerGenericField<
        pcl::PointXYZRGB>("z");
    break;
  case '3':
    surfaceReconstruction = true;
    visualizer->removePointCloud("pointCloudPCL");
    break;
  }

  updatePointCloud(pointCloudPCL);

  if (!(('0' <= event->key()) & (event->key() <= '9')))
    QVTKRenderWidget::keyPressEvent(event);
}

void PointCloudWidget::updatePointCloud(PointCloudConstPtr _pointCloudPCL) {

  if (!_pointCloudPCL || _pointCloudPCL->points.empty())
    return;

  //    timer.start();

  pointCloudPCL = _pointCloudPCL;

  if (surfaceReconstruction) {
    reconstructor->setInputCloud(pointCloudPCL);
    std::vector<pcl::Vertices> polygons;
    reconstructor->reconstruct(polygons);
    if (!visualizer->updatePolygonMesh<pcl::PointXYZRGB>(pointCloudPCL,
                                                         polygons, "meshPCL")) {
      visualizer->addPolygonMesh<pcl::PointXYZRGB>(pointCloudPCL, polygons,
                                                   "meshPCL");
    }
  } else {
    // Note: using the color handler makes a copy of the rgb fields
    colorHandler->setInputCloud(pointCloudPCL);
    if (!visualizer->updatePointCloud(pointCloudPCL, *colorHandler,
                                      "pointCloudPCL")) {
      visualizer->addPointCloud(pointCloudPCL, *colorHandler, "pointCloudPCL");
      // visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
      // 1.0, "pointCloudPCL");
    }
  }

  this->GetRenderWindow()->Render();
  emit newPointCloudDisplayed();

  //    std::cout << "PCL Widget: " << time.restart() << "ms" << std::endl;
}

void PointCloudWidget::savePointCloud() {

  QString selectedFilter;
  QString fileName = QFileDialog::getSaveFileName(
      this, "Save Point Cloud", QString(), "*.pcd;;*.ply;;*.vtk;;*.png;;*.txt",
      &selectedFilter);
  QFileInfo info(fileName);
  QString type = info.suffix();
  if (type == "") {
    fileName.append(selectedFilter.remove(0, 1));
    type = selectedFilter.remove(0, 1);
  }

  if (type == "pcd") {
    pcl::io::savePCDFileASCII(fileName.toStdString(), *pointCloudPCL);
  } else if (type == "ply") {
    // pcl::io::savePLYFileBinary( fileName.toStdString(), *pointCloudPCL);
    pcl::PLYWriter w;
    // Write to ply in binary without camera
    w.write<pcl::PointXYZRGB>(fileName.toStdString(), *pointCloudPCL, true,
                              false);
  } else if (type == "vtk") {
    pcl::PCLPointCloud2 pointCloud2;
    pcl::toPCLPointCloud2(*pointCloudPCL, pointCloud2);
    pcl::io::saveVTKFile(fileName.toStdString(), pointCloud2);

    //        vtkPolyData polyData;
    //        pcl::io::pointCloudTovtkPolyData(*pointCloudPCL, polyData);
    //        vtkPolyDataWriter::Pointer writer = vtkPolyDataWriter::New();
    //        writer->SetInput(polyData);
    //        writer->SetFileName(fileName.toStdString());
    //        writer->Update();
  } else if (type == "png") {
    pcl::io::savePNGFile(fileName.toStdString(), *pointCloudPCL, "rgb");
  } else if (type == "txt") {
    std::ofstream s(fileName.toLocal8Bit());
    for (unsigned int r = 0; r < pointCloudPCL->height; r++) {
      for (unsigned int c = 0; c < pointCloudPCL->width; c++) {
        pcl::PointXYZRGB p = pointCloudPCL->at(c, r);
        if (p.x == p.x)
          s << p.x << " " << p.y << " " << p.z << "\r\n";
      }
    }
    std::flush(s);
    s.close();
  }
}

void PointCloudWidget::saveScreenShot() {

  vtkWindowToImageFilter *filter = vtkWindowToImageFilter::New();
  // filter->SetInput(visualizer->getRenderWindow());
  filter->Modified();

  QString fileName = QFileDialog::getSaveFileName(this, "Save Screen Shot",
                                                  QString(), "*.png");
  QFileInfo info(fileName);
  QString type = info.suffix();
  if (type == "")
    fileName.append(".png");

  vtkPNGWriter *writer = vtkPNGWriter::New();
  writer->SetInputConnection(filter->GetOutputPort());
  writer->SetFileName(qPrintable(fileName));
  writer->Write();
  writer->Delete();
}

PointCloudWidget::~PointCloudWidget() {

  // delete visualizer;
}
