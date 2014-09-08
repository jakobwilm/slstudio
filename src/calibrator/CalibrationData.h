#ifndef CALIBRATIONDATA_H
#define CALIBRATIONDATA_H

#include <QString>
#include <opencv2/core/core.hpp>

class CalibrationData{
    public:
        CalibrationData() : Kc(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), kc(0.0), cam_error(0.0),
                            Kp(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), kp(0.0), proj_error(0.0),
                            Rp(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0), Tp(0.0), stereo_error(0.0),
                            frameWidth(0), frameHeight(0), screenResX(0), screenResY(0) {}
        CalibrationData(cv::Matx33f _Kc, cv::Vec<float, 5> _kc, double _cam_error, cv::Matx33f _Kp, cv::Vec<float, 5> _kp, double _proj_error, cv::Matx33f _Rp, cv::Vec3f _Tp, double _stereo_error) :
                        Kc(_Kc), kc(_kc), cam_error(_cam_error), Kp(_Kp), kp(_kp), proj_error(_proj_error), Rp(_Rp), Tp(_Tp), stereo_error(_stereo_error){}
        //CalibrationData(const QString& filename){load(filename);}
        bool load(const QString& filename);
        bool save(const QString& filename);
        bool loadXML(const QString& filename);
        bool saveXML(const QString& filename);
        bool saveMatlab(const QString& filename);
        bool saveSLCALIB(const QString& filename);
        void print(std::ostream &stream);

        cv::Matx33f Kc; // Intrinsic camera matrix
        cv::Vec<float, 5> kc; // Camera distortion coefficients
        double cam_error;

        cv::Matx33f Kp; // Intrinsic projector matrix
        cv::Vec<float, 5> kp; // Projector distortion coefficients
        double proj_error;

        cv::Matx33f Rp; // Extrinsic camera rotation matrix
        cv::Vec3f   Tp; // Extrinsic camera rotation matrix

        double stereo_error;

        int frameWidth, frameHeight;
        int screenResX, screenResY;
        std::string calibrationDateTime;
};

#endif
