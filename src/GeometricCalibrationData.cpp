#include "GeometricCalibrationData.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <opencv2/calib3d/calib3d.hpp>

GeometricCalibrationData::GeometricCalibrationData()
    : Kc(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), kc(0.0), cam_error(0.0),
      Kp(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), kp(0.0), proj_error(0.0),
      Rp(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0), Tp(0.0),
      stereo_error(0.0), frameWidth(752), frameHeight(480), screenResX(0),
      screenResY(0),
      calibrationDateTime() {}

GeometricCalibrationData::GeometricCalibrationData(cv::Matx33f cameraMatrix, cv::Vec<float, 5> cameraDistortion,
                                double cameraError, cv::Matx33f projectorMatrix,
                                cv::Vec<float, 5> projectorDistortion, double projectorError,
                                cv::Matx33f rotation, cv::Vec3f translation, double stereoError)
    : GeometricCalibrationData() {
  Kc = cameraMatrix;
  kc = cameraDistortion;
  cam_error = cameraError;
  Kp = projectorMatrix;
  kp = projectorDistortion;
  proj_error = projectorError;
  Rp = rotation;
  Tp = translation;
  stereo_error = stereoError;
}

bool GeometricCalibrationData::load(const std::filesystem::path &filename) {
  if (std::filesystem::exists(filename) && filename.extension() == ".xml")
    return loadXML(filename);

  std::cerr << "TI2CalibrationData error: no such .xml file: "
            << filename.string() << std::endl;
  return false;
}

bool GeometricCalibrationData::save(const std::filesystem::path &filename) {
  std::error_code error;
  if (filename.has_parent_path())
    std::filesystem::create_directories(filename.parent_path(), error);
  if (error)
    return false;
  const auto type = filename.extension();
  if (type == ".xml") {
    return saveXML(filename);
  }
  if (type == ".slcalib") {
    return saveSLCALIB(filename);
  } else if (type == ".m") {
    return saveMatlab(filename);
  } else {
    std::cerr << "TI2CalibrationData error save: unknown file extension: "
              << type.string() << std::endl;
    return false;
  }

  return false;
}

bool GeometricCalibrationData::loadXML(const std::filesystem::path &filename) try {
  cv::FileStorage fs(filename.string(), cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "TI2CalibrationData error: could not open file "
              << filename.string() << std::endl;
    return false;
  }

  GeometricCalibrationData loaded;
  auto readMatrix = [&](const char *key, int rows, int columns) {
    cv::Mat matrix;
    fs[key] >> matrix;
    if (matrix.empty() || matrix.channels() != 1 ||
        matrix.total() != static_cast<std::size_t>(rows * columns))
      throw std::invalid_argument("Invalid calibration matrix");
    if (rows == 3 && columns == 3 && matrix.size() != cv::Size(3, 3))
      throw std::invalid_argument("Invalid calibration matrix dimensions");
    matrix = matrix.reshape(1, rows);
    matrix.convertTo(matrix, CV_32F);
    if (!cv::checkRange(matrix)) throw std::invalid_argument("Non-finite calibration matrix");
    return matrix;
  };
  loaded.Kc = readMatrix("Kc", 3, 3);
  loaded.kc = readMatrix("kc", 5, 1);
  loaded.Kp = readMatrix("Kp", 3, 3);
  loaded.kp = readMatrix("kp", 5, 1);
  loaded.Rp = readMatrix("Rp", 3, 3);
  loaded.Tp = readMatrix("Tp", 3, 1);
  fs["cam_error"] >> loaded.cam_error;
  fs["proj_error"] >> loaded.proj_error;
  fs["stereo_error"] >> loaded.stereo_error;
  fs["frameWidth"] >> loaded.frameWidth;
  fs["frameHeight"] >> loaded.frameHeight;
  fs["screenResX"] >> loaded.screenResX;
  fs["screenResY"] >> loaded.screenResY;
  if (!fs["calibrationDateTime"].empty()) fs["calibrationDateTime"] >> loaded.calibrationDateTime;
  auto diagnostics = [&](const char *key, std::vector<double> &values) {
    if (fs[key].empty()) return;
    if (fs[key].isReal() || fs[key].isInt()) values = {static_cast<double>(fs[key])};
    else if (fs[key].isSeq()) fs[key] >> values;
    else throw std::invalid_argument("Invalid calibration diagnostics");
    for (const double value : values)
      if (!std::isfinite(value) || value < 0)
        throw std::invalid_argument("Non-finite or negative calibration uncertainty/error");
  };
  diagnostics("cam_stdint", loaded.cam_stdint);
  diagnostics("cam_stdext", loaded.cam_stdext);
  diagnostics("cam_pve", loaded.cam_pve);
  diagnostics("proj_stdint", loaded.proj_stdint);
  diagnostics("proj_stdext", loaded.proj_stdext);
  diagnostics("proj_pve", loaded.proj_pve);
  if (loaded.frameWidth < 1 || loaded.frameHeight < 1 ||
      loaded.screenResX < 0 || loaded.screenResY < 0 ||
      loaded.Kc(0, 0) <= 0 || loaded.Kc(1, 1) <= 0 ||
      loaded.Kp(0, 0) <= 0 || loaded.Kp(1, 1) <= 0)
    return false;
  *this = std::move(loaded);
  return true;
} catch (const cv::Exception &) {
  return false;
} catch (const std::invalid_argument &) {
  return false;
}

bool GeometricCalibrationData::saveSLCALIB(const std::filesystem::path &filename) {

  FILE *fp = fopen(filename.string().c_str(), "w");
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

bool GeometricCalibrationData::saveXML(const std::filesystem::path &filename) {
  cv::FileStorage fs(filename.string(), cv::FileStorage::WRITE);
  if (!fs.isOpened())
    return false;

  fs << "Kc" << cv::Mat(Kc) << "kc" << cv::Mat(kc) << "Kp" << cv::Mat(Kp)
     << "kp" << cv::Mat(kp) << "Rp" << cv::Mat(Rp) << "Tp" << cv::Mat(Tp)
     << "cam_error" << cam_error << "proj_error" << proj_error << "stereo_error"
     << stereo_error << "frameWidth" << frameWidth << "frameHeight"
     << frameHeight << "screenResX" << screenResX << "screenResY" << screenResY;
  fs << "calibrationDateTime" << calibrationDateTime;
  auto diagnostics = [&](const char *key, const std::vector<double> &values) {
    if (!values.empty()) fs << key << values;
  };
  diagnostics("cam_stdint", cam_stdint);
  diagnostics("cam_stdext", cam_stdext);
  diagnostics("cam_pve", cam_pve);
  diagnostics("proj_stdint", proj_stdint);
  diagnostics("proj_stdext", proj_stdext);
  diagnostics("proj_pve", proj_pve);
  fs.release();

  return true;
}

bool GeometricCalibrationData::saveMatlab(const std::filesystem::path &filename) {

  std::ofstream file(filename);
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

// void GeometricCalibrationData::print(std::ostream &stream) {

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

void GeometricCalibrationData::print(std::ostream &stream) {

  if (cam_stdint.size() < 9 || proj_stdint.size() < 9) {
    stream << "Camera: " << Kc << "\nProjector: " << Kp
           << "\nRotation: " << Rp << "\nTranslation: " << Tp << '\n';
    return;
  }

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
