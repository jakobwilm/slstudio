#pragma once

#include <QString>
#include <opencv2/core/core.hpp>

class CalibrationData {
public:
  CalibrationData();
  CalibrationData(cv::Matx33f _Kc, cv::Vec<float, 5> _kc, double _cam_error,
                  cv::Matx33f _Kp, cv::Vec<float, 5> _kp, double _proj_error,
                  cv::Matx33f _Rp, cv::Vec3f _Tp, double _stereo_error);
  // CalibrationData(const QString& filename){load(filename);}
  bool load(const QString &filename);
  bool save(const QString &filename);
  bool loadXML(const QString &filename);
  bool saveXML(const QString &filename);
  bool saveMatlab(const QString &filename);
  bool saveSLCALIB(const QString &filename);
  void print(std::ostream &stream);

  cv::Matx33f Kc;       // Intrinsic camera matrix
  cv::Vec<float, 5> kc; // Camera distortion coefficients
  double cam_error;
  std::vector<double> cam_stdint, cam_stdext, cam_pve;

  cv::Matx33f Kp;       // Intrinsic projector matrix
  cv::Vec<float, 5> kp; // Projector distortion coefficients
  double proj_error;
  std::vector<double> proj_stdint, proj_stdext, proj_pve;

  cv::Matx33f Rp; // Extrinsic camera rotation matrix
  cv::Vec3f Tp;   // Extrinsic camera rotation matrix

  double stereo_error;

  int frameWidth, frameHeight;
  int screenResX, screenResY;
  std::string calibrationDateTime;
};
