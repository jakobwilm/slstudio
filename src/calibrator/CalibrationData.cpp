#include "CalibrationData.h"

#include <boost/filesystem.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/calib3d/calib3d.hpp>

CalibrationData::CalibrationData() : Kc(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), kc(0.0), cam_error(0.0),
                    Kp(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), kp(0.0), proj_error(0.0),
                    Rp(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), Tp(0.0), stereo_error(0.0),
                    frameWidth(640), frameHeight(512), screenResX(0), screenResY(0) {

}

CalibrationData::CalibrationData(cv::Matx33f _Kc, cv::Vec<float, 5> _kc, double _cam_error, cv::Matx33f _Kp, cv::Vec<float, 5> _kp,
                double _proj_error, cv::Matx33f _Rp, cv::Vec3f _Tp, double _stereo_error) :
                Kc(_Kc), kc(_kc), cam_error(_cam_error), Kp(_Kp), kp(_kp), proj_error(_proj_error), Rp(_Rp), Tp(_Tp), stereo_error(_stereo_error){

}

bool CalibrationData::load(const std::string& filename){
    if(boost::filesystem::exists(filename) && 
        boost::filesystem::path(filename).extension()=="xml") {
        return loadXML(filename);
    } else {
        std::cerr << "CalibrationData error: no such .xml file: " << filename << std::endl;
        return false;
    }
}

bool CalibrationData::save(const std::string& filename){
    std::string type = boost::filesystem::path(filename).extension().string();

    if (type=="xml"){
        return saveXML(filename);
    } else if(type=="slcalib"){
        return saveSLCALIB(filename);
    } else if (type=="m"){
        return saveMatlab(filename);
    } else {
        std::cerr << "CalibrationData error save: unknown file extension: " << type << std::endl;
        return false;
    }

    return false;
}

bool CalibrationData::loadXML(const std::string& filename){
    cv::FileStorage fs(filename, cv::FileStorage::READ); //
    if (!fs.isOpened())
    {
        std::cerr << "CalibrationData error: could not open file " << filename << std::endl;
        return false;
    }

    cv::Mat temp;
    fs["Kc"] >> temp; Kc = temp;
    fs["kc"] >> temp; kc = temp;
    fs["Kp"] >> temp; Kp = temp;
    fs["kp"] >> temp; kp = temp;
    fs["Rp"] >> temp; Rp = temp;
    fs["Tp"] >> temp; Tp = temp;

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

bool CalibrationData::saveSLCALIB(const std::string& filename){


    FILE * fp = fopen(filename.c_str(), "w");
    if (!fp)
        return false;

    fprintf(fp, "#V1.0 SLStudio calibration\n");
    fprintf(fp, "#Calibration time: %s\n\n", calibrationDateTime.c_str());
    fprintf(fp, "Kc\n%f %f %f\n%f %f %f\n%f %f %f\n\n", Kc(0,0), Kc(0,1), Kc(0,2), Kc(1,0), Kc(1,1), Kc(1,2), Kc(2,0), Kc(2,1), Kc(2,2));
    fprintf(fp, "kc\n%f %f %f %f %f\n\n", kc(0), kc(1), kc(2), kc(3), kc(4));
    fprintf(fp, "Kp\n%f %f %f\n%f %f %f\n%f %f %f\n\n", Kp(0,0), Kp(0,1), Kp(0,2), Kp(1,0), Kp(1,1), Kp(1,2), Kp(2,0), Kp(2,1), Kp(2,2));
    fprintf(fp, "kp\n%f %f %f %f %f\n\n", kp(0), kp(1), kp(2), kp(3), kp(4));
    fprintf(fp, "Rp\n%f %f %f\n%f %f %f\n%f %f %f\n\n", Rp(0,0), Rp(0,1), Rp(0,2), Rp(1,0), Rp(1,1), Rp(1,2), Rp(2,0), Rp(2,1), Rp(2,2));
    fprintf(fp, "Tp\n%f %f %f\n\n", Tp(0), Tp(1), Tp(2));

    fprintf(fp, "cam_error: %f\n\n", cam_error);
    fprintf(fp, "proj_error: %f\n\n", proj_error);
    fprintf(fp, "stereo_error: %f\n\n", stereo_error);

    fclose(fp);

    return true;

}

bool CalibrationData::saveXML(const std::string& filename){
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (!fs.isOpened())
        return false;

    fs << "Kc" << cv::Mat(Kc) << "kc" << cv::Mat(kc)
       << "Kp" << cv::Mat(Kp) << "kp" << cv::Mat(kp)
       << "Rp" << cv::Mat(Rp) << "Tp" << cv::Mat(Tp)
       << "cam_error" << cam_error
       << "proj_error" << proj_error
       << "stereo_error" << stereo_error
       << "frameWidth" << frameWidth
       << "frameHeight" << frameHeight
       << "screenResX" << screenResX
       << "screenResY" << screenResY;
    fs.release();

    return true;
}

bool CalibrationData::saveMatlab(const std::string& filename){

    std::ofstream file(filename);
    if (!file)
        return false;

    file << "%%SLStudio calibration"  << std::endl;
    file << "Kc = " << cv::Mat(Kc) << ";" << std::endl;
    file << "kc = " << cv::Mat(kc) << ";" << std::endl;
    file << "Kp = " << cv::Mat(Kp) << ";" << std::endl;
    file << "kp = " << cv::Mat(kp) << ";" << std::endl;
    file << "Rp = " << cv::Mat(Rp) << ";" << std::endl;
    file << "Tp = " << cv::Mat(Tp) << ";" << std::endl;

    file.close();

    return true;

}

void CalibrationData::print(std::ostream &stream){

    stream  << std::setw(5) << std::setprecision(4)
            << "========================================\n"
            << "Camera Calibration: \n"
            << "- cam_error:\n" << cam_error << "\n"
            << "- Kc:\n" << cv::Mat(Kc) << "\n"
            << "- kc:\n" << cv::Mat(kc) << "\n"
            << "Projector Calibration: " << "\n"
            << "- proj_error: \n" << proj_error << "\n"
            << "- Kp: \n" << cv::Mat(Kp) << "\n"
            << "- kp: \n" << cv::Mat(kp) << "\n"
            << "Stereo Calibration: \n"
            << "- stereo_error:\n" << stereo_error << "\n"
            << "- Rp:\n" << cv::Mat(Rp) << "\n"
            << "- Tp:\n" << cv::Mat(Tp) << std::endl;
}
