#include "CalibrationData.h"

#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QString>
#include <QTextStream>

#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/calib3d/calib3d.hpp>

CalibrationData::CalibrationData()
    : Kc(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), kc(0.0), cam_error(0.0),
      Kp(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), kp(0.0), proj_error(0.0),
      Rp(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), Tp(0.0),
      stereo_error(0.0), frameWidth(752), frameHeight(480), screenResX(0),
      screenResY(0),
      calibrationDateTime(
          QDateTime::currentDateTime().toString().toStdString()) {}

bool CalibrationData::load(const QString &filename) {
  QFileInfo info(filename);
  //  QString type = info.suffix();

  if (info.exists() && info.suffix() == "xml")
    return loadXML(filename);

  std::cerr << "TI2CalibrationData error: no such .xml file: "
            << filename.toStdString() << std::endl;
  return false;
}

bool CalibrationData::save(const QString &filename) {

  QFileInfo info(filename);

  QDir dir;
  dir.mkpath(info.absolutePath());

  QString type = info.suffix();

  if (type == "xml") {
    return saveXML(filename);
  }
  if (type == "slcalib") {
    return saveSLCALIB(filename);
  } else if (type == "m") {
    return saveMatlab(filename);
  } else {
    std::cerr << "TI2CalibrationData error save: unknown file extension: "
              << type.toStdString() << std::endl;
    return false;
  }

  return false;
}

bool CalibrationData::loadXML(const QString &filename) {
  cv::FileStorage fs(filename.toStdString(), cv::FileStorage::READ); //
  if (!fs.isOpened()) {
    std::cerr << "TI2CalibrationData error: could not open file "
              << filename.toStdString() << std::endl;
    return false;
  }

  cv::Mat temp;
  fs["Kc"] >> temp;
  Kc = temp;
  fs["kc"] >> temp;
  kc = temp;
  fs["Kp"] >> temp;
  Kp = temp;
  fs["kp"] >> temp;
  kp = temp;
  fs["Rp"] >> temp;
  Rp = temp;
  fs["Tp"] >> temp;
  Tp = temp;

  fs["cam_error"] >> cam_error;
  fs["proj_error"] >> proj_error;
  fs["stereo_error"] >> stereo_error;

  fs["frameWidth"] >> frameWidth;
  fs["frameHeight"] >> frameHeight;
  fs["screenResX"] >> screenResX;
  fs["screenResY"] >> screenResY;

  fs.release();

  return true;
}

struct DavidCalFileToken {
  DavidCalFileToken() : value(nullptr), found(false) {}
  DavidCalFileToken(QString n, double *v) : name(n), value(v), found(false) {}

  QString name;
  double *value;
  bool found;
};

bool loadDavidCalFile(QString filename, double &f, double &sx, double &cx,
                      double &cy, double &kappa1, cv::Matx33f &R,
                      cv::Vec3f &T) {

  if (filename.isEmpty()) {
    return false;
  }

  QFile file(filename);
  if (!file.open(QFile::Text | QFile::ReadOnly)) {
    return false;
  }

  double kc[5];
  double N[3], O[3], A[3], P[3];

  std::vector<DavidCalFileToken> tokens;
  {
    tokens.push_back(DavidCalFileToken("cx", &cx));
    tokens.push_back(DavidCalFileToken("cy", &cy));
    tokens.push_back(DavidCalFileToken("f", &f));
    tokens.push_back(DavidCalFileToken("sx", &sx));
    tokens.push_back(DavidCalFileToken("kappa1", &kappa1));
    tokens.push_back(DavidCalFileToken("nx", N));
    tokens.push_back(DavidCalFileToken("ny", N + 1));
    tokens.push_back(DavidCalFileToken("nz", N + 2));
    tokens.push_back(DavidCalFileToken("ox", O));
    tokens.push_back(DavidCalFileToken("oy", O + 1));
    tokens.push_back(DavidCalFileToken("oz", O + 2));
    tokens.push_back(DavidCalFileToken("ax", A));
    tokens.push_back(DavidCalFileToken("ay", A + 1));
    tokens.push_back(DavidCalFileToken("az", A + 2));
    tokens.push_back(DavidCalFileToken("px", P));
    tokens.push_back(DavidCalFileToken("py", P + 1));
    tokens.push_back(DavidCalFileToken("pz", P + 2));
  }

  QTextStream stream(&file);
  printf("CameraModel.file: '%s'\n", qPrintable(filename));

  for (size_t elementRead = 0;; ++elementRead) {
    QString line = file.readLine().trimmed();
    if (line.isEmpty())
      break;

    for (size_t i = 0; i < tokens.size(); ++i) {
      if (tokens[i].found)
        continue;

      QString prefix = QString("<%1>").arg(tokens[i].name);
      if (line.startsWith(prefix)) {
        line.remove(prefix);
        line.remove(QString("</%1>").arg(tokens[i].name));
        bool ok;
        assert(tokens[i].value);
        *tokens[i].value = line.toDouble(&ok);
        if (!ok) {
          // malformed file
          assert(false);
          return false;
        }

        tokens[i].found = true;
        continue;
      }
    }

    // special elements
    if (line.startsWith("<camera_model>")) {
      line.remove("<camera_model>");
      line.remove("</camera_model>");
      assert(line.toUpper() == "CAMERATSAI");
      printf("CameraModel.model: '%s'\n", qPrintable(line));
    } else if (line.startsWith("<MotionType>")) {
      line.remove("<MotionType>");
      line.remove("</MotionType>");
      assert(line.toUpper() == "FIXEDPOSE");
      printf("CameraModel.motion_type: '%s'\n", qPrintable(line));
    }
  }

  // check that we have everything we need
  for (size_t i = 0; i < tokens.size(); ++i) {
    if (!tokens[i].found)
      return false;
  }

  f *= sx;
  sx = 1.0 / sx;

// construct extrinsics matrix
#ifdef _DEBUG
  // check input vectors
  double dpOA = O[0] * A[0] + O[1] * A[1] + O[2] * A[2];
  assert(fabs(dpOA) < 1.0e-8);
  double dpNA = N[0] * A[0] + N[1] * A[1] + N[2] * A[2];
  assert(fabs(dpNA) < 1.0e-8);
  double dpON = O[0] * N[0] + O[1] * N[1] + O[2] * N[2];
  assert(fabs(dpON) < 1.0e-8);
  double normA = sqrt(A[0] * A[0] + A[1] * A[1] + A[2] * A[2]);
  assert(fabs(1.0 - normA) < 1.0e-8);
  double normO = sqrt(O[0] * O[0] + O[1] * O[1] + O[2] * O[2]);
  assert(fabs(1.0 - normO) < 1.0e-8);
  double normN = sqrt(N[0] * N[0] + N[1] * N[1] + N[2] * N[2]);
  assert(fabs(1.0 - normN) < 1.0e-8);
#endif

  R(0, 0) = N[0];
  R(0, 1) = N[1];
  R(0, 2) = N[2];
  R(1, 0) = O[0];
  R(1, 1) = O[1];
  R(1, 2) = O[2];
  R(2, 0) = A[0];
  R(2, 1) = A[1];
  R(2, 2) = A[2];

  T[0] = P[0];
  T[1] = P[1];
  T[2] = P[2];

  return true;
}

bool CalibrationData::saveSLCALIB(const QString &filename) {

  FILE *fp = fopen(qPrintable(filename), "w");
  if (!fp)
    return false;

  fprintf(fp, "#V1.0 SLStudio calibration\n");
  fprintf(fp, "#Calibration time: %s\n\n", calibrationDateTime.c_str());
  fprintf(fp, "Kc\n%f %f %f\n%f %f %f\n%f %f %f\n\n", Kc(0, 0), Kc(0, 1),
          Kc(0, 2), Kc(1, 0), Kc(1, 1), Kc(1, 2), Kc(2, 0), Kc(2, 1), Kc(2, 2));
  fprintf(fp, "kc\n%f %f %f %f %f\n\n", kc(0), kc(1), kc(2), kc(3), kc(4));
  fprintf(fp, "Kp\n%f %f %f\n%f %f %f\n%f %f %f\n\n", Kp(0, 0), Kp(0, 1),
          Kp(0, 2), Kp(1, 0), Kp(1, 1), Kp(1, 2), Kp(2, 0), Kp(2, 1), Kp(2, 2));
  fprintf(fp, "kp\n%f %f %f %f %f\n\n", kp(0), kp(1), kp(2), kp(3), kp(4));
  fprintf(fp, "Rp\n%f %f %f\n%f %f %f\n%f %f %f\n\n", Rp(0, 0), Rp(0, 1),
          Rp(0, 2), Rp(1, 0), Rp(1, 1), Rp(1, 2), Rp(2, 0), Rp(2, 1), Rp(2, 2));
  fprintf(fp, "Tp\n%f %f %f\n\n", Tp(0), Tp(1), Tp(2));

  fprintf(fp, "cam_error: %f\n\n", cam_error);
  fprintf(fp, "proj_error: %f\n\n", proj_error);
  fprintf(fp, "stereo_error: %f\n\n", stereo_error);

  fclose(fp);

  return true;
}

bool CalibrationData::saveXML(const QString &filename) {
  cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);
  if (!fs.isOpened())
    return false;

  fs << "Kc" << cv::Mat(Kc) << "kc" << cv::Mat(kc) << "Kp" << cv::Mat(Kp)
     << "kp" << cv::Mat(kp) << "Rp" << cv::Mat(Rp) << "Tp" << cv::Mat(Tp)
     << "cam_error" << cam_error << "proj_error" << proj_error << "stereo_error"
     << stereo_error << "frameWidth" << frameWidth << "frameHeight"
     << frameHeight << "screenResX" << screenResX << "screenResY" << screenResY;
  fs.release();

  return true;
}

bool CalibrationData::saveMatlab(const QString &filename) {

  std::ofstream file(qPrintable(filename));
  if (!file)
    return false;

  file << "%%SLStudio calibration" << std::endl;
  file << "Kc = " << Kc << ";" << std::endl;
  file << "kc = " << kc << ";" << std::endl;
  file << "Kp = " << Kp << ";" << std::endl;
  file << "kp = " << kp << ";" << std::endl;
  file << "Rp = " << Rp << ";" << std::endl;
  file << "Tp = " << Tp << ";" << std::endl;

  file.close();

  return true;
}

// void CalibrationData::print(std::ostream &stream) {

//  stream << std::setw(5) << std::setprecision(4)
//         << "========================================\n"
//         << "Camera Calibration: \n"
//         << "- cam_error:\n"
//         << cam_error << "\n"
//         << "- Kc:\n"
//         << Kc << "\n"
//         << "- kc:\n"
//         << kc << "\n"
//         << "Projector Calibration: "
//         << "\n"
//         << "- proj_error: \n"
//         << proj_error << "\n"
//         << "- Kp: \n"
//         << Kp << "\n"
//         << "- kp: \n"
//         << kp << "\n"
//         << "Stereo Calibration: \n"
//         << "- stereo_error:\n"
//         << stereo_error << "\n"
//         << "- Rp:\n"
//         << Rp << "\n"
//         << "- Tp:\n"
//         << Tp << std::endl;
//}

void CalibrationData::print(std::ostream &stream) {

  stream << "======== Sensor Calibration ========\n\n";
  stream << "Date: " << calibrationDateTime << '\n';
  stream << std::setprecision(6) << "Camera: " << '\n'
         << "\t focal length: \t[" << Kc(0, 0) << " " << Kc(1, 1) << "] "
         << "\t\t +/- [" << cam_stdint[0] << " " << cam_stdint[1] << "]" << '\n'
         << "\t central point: \t[" << Kc(0, 2) << " " << Kc(1, 2) << "] "
         << "\t\t\t +/- [" << cam_stdint[2] << " " << cam_stdint[3] << "]"
         << '\n'
         << "\t lens distortion: \t" << kc << "\t +/- [" << cam_stdint[4] << " "
         << cam_stdint[5] << " " << cam_stdint[6] << " " << cam_stdint[7] << " "
         << cam_stdint[8] << "]" << '\n'
         << "\t reprojection errors: " << '\n';
  for (unsigned int i = 0; i < cam_pve.size(); i++)
    stream << "\t\t\t" << cam_pve[i] << " px" << '\n';

  stream << "Projector: " << '\n'
         << "\t focal length: \t[" << Kp(0, 0) << " " << Kp(1, 1) << "] "
         << "\t\t +/- [" << proj_stdint[0] << " " << proj_stdint[1] << "]"
         << '\n'
         << "\t central point: \t[" << Kp(0, 2) << " " << Kp(1, 2) << "] "
         << "\t\t +/- [" << proj_stdint[2] << " " << proj_stdint[3] << "]"
         << '\n'
         << "\t lens distortion: \t" << kp << "\t +/- [" << proj_stdint[4]
         << " " << proj_stdint[5] << " " << proj_stdint[6] << " "
         << proj_stdint[7] << " " << proj_stdint[8] << "]" << '\n'
         << "\t reprojection errors: " << '\n';
  for (unsigned int i = 0; i < proj_pve.size(); i++)
    stream << "\t\t\t" << proj_pve[i] << " px" << '\n';

  stream << "Stereo Parameters: " << '\n'
         << "\t rotation: \t\t[" << Rp(0, 0) << " " << Rp(0, 1) << " "
         << Rp(0, 2) << ";\n\t\t\t" << Rp(1, 0) << " " << Rp(1, 1) << " "
         << Rp(1, 2) << ";\n\t\t\t" << Rp(2, 0) << " " << Rp(2, 1) << " "
         << Rp(2, 2) << "]" << '\n'
         << "\t translation: \t[" << Tp(0) << " " << Tp(1) << " " << Tp(2)
         << "]" << '\n'
         << "\t reprojection error: \n\t\t\t" << stereo_error << " px"
         << std::endl;
}
