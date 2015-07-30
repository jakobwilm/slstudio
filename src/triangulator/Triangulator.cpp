#include "Triangulator.h"
#include <math.h>
#include <iostream>


#ifdef WIN32
    #ifndef NAN
        static const unsigned long __nan[2] = {0xffffffff, 0x7fffffff};
    #define NAN (*(const float *) __nan)
    #endif
#endif

Triangulator::Triangulator(CalibrationData _calibration) : calibration(_calibration){

    // Precompute uc, vc maps
    uc.create(calibration.frameHeight, calibration.frameWidth, CV_32F);
    vc.create(calibration.frameHeight, calibration.frameWidth, CV_32F);

    for(unsigned int row=0; row<calibration.frameHeight; row++){
        for(unsigned int col=0; col<calibration.frameWidth; col++){
            uc.at<float>(row, col) = col;
            vc.at<float>(row, col) = row;
        }
    }

    // Precompute determinant tensor
    cv::Mat Pc(3,4,CV_32F,cv::Scalar(0.0));
    cv::Mat(calibration.Kc).copyTo(Pc(cv::Range(0,3), cv::Range(0,3)));

    cv::Mat Pp(3,4,CV_32F), temp(3,4,CV_32F);
    cv::Mat(calibration.Rp).copyTo(temp(cv::Range(0,3), cv::Range(0,3)));
    cv::Mat(calibration.Tp).copyTo(temp(cv::Range(0,3), cv::Range(3,4)));
    Pp = cv::Mat(calibration.Kp) * temp;

    cv::Mat e = cv::Mat::eye(4, 4, CV_32F);

    int sz[] = {4, 3, 3, 3};
    cv::Mat C(4, sz, CV_32F, cv::Scalar::all(0));
    for(int k=0; k<4; k++){
        for(int i=0; i<3; i++){
            for(int j=0; j<3; j++){
                for(int l=0; l<3; l++){
                    cv::Mat op(4, 4, CV_32F);
                    Pc.row(i).copyTo(op.row(0));
                    Pc.row(j).copyTo(op.row(1));
                    Pp.row(l).copyTo(op.row(2));
                    e.row(k).copyTo(op.row(3));
                    C.at<float>(cv::Vec4i(k,i,j,l)) = cv::determinant(op.t());
                }
            }
        }
    }
    determinantTensor = C;

    // Precompute lens correction maps
    cv::Mat eye = cv::Mat::eye(3, 3, CV_32F);
    cv::initUndistortRectifyMap(calibration.Kc, calibration.kc, eye, calibration.Kc, cv::Size(calibration.frameWidth, calibration.frameHeight), CV_32FC1, lensMap1, lensMap2);

    //cv::Mat map1, map2;
    //cv::normalize(lensMap1, map1, 0, 255, cv::NORM_MINMAX, CV_8U);
    //cv::normalize(lensMap2, map2, 0, 255, cv::NORM_MINMAX, CV_8U);
    //cv::imwrite("map1.png", map1);
    //cv::imwrite("map2.png", map2);
}

void Triangulator::triangulate(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading, cv::Mat &pointCloud){

    // Undistort up, mask and shading
    if(!up.empty()){
        cv::Mat upUndistort;
        cv::remap(up, upUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
        up = upUndistort;
    }
    if(!vp.empty()){
        cv::Mat vpUndistort;
        cv::remap(vp, vpUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
        vp = vpUndistort;
    }

    cv::Mat maskUndistort, shadingUndistort;
    cv::remap(mask, maskUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
    cv::remap(shading, shadingUndistort, lensMap1, lensMap2, cv::INTER_LINEAR);
    mask = maskUndistort;
    shading = shadingUndistort;

    // Triangulate
    cv::Mat xyz;
    if(!up.empty() && vp.empty())
        triangulateFromUp(up, xyz);
    else if(!vp.empty() && up.empty())
        triangulateFromVp(vp, xyz);
    else if(!up.empty() && !vp.empty())
        triangulateFromUpVp(up, vp, xyz);

    // Mask
    pointCloud = cv::Mat(up.size(), CV_32FC3, cv::Scalar(NAN, NAN, NAN));
    xyz.copyTo(pointCloud, mask);

}

void Triangulator::triangulateFromUp(cv::Mat &up, cv::Mat &xyz){

    // Solve for xyzw using determinant tensor
    cv::Mat C = determinantTensor;
    std::vector<cv::Mat> xyzw(4);
    for(unsigned int i=0; i<4; i++){
//        xyzw[i].create(up.size(), CV_32F);
        xyzw[i] = C.at<float>(cv::Vec4i(i,0,1,0)) - C.at<float>(cv::Vec4i(i,2,1,0))*uc - C.at<float>(cv::Vec4i(i,0,2,0))*vc -
                C.at<float>(cv::Vec4i(i,0,1,2))*up + C.at<float>(cv::Vec4i(i,2,1,2))*up.mul(uc) + C.at<float>(cv::Vec4i(i,0,2,2))*up.mul(vc);
    }

    // Convert to non homogenous coordinates
    for(unsigned int i=0; i<3; i++)
        xyzw[i] /= xyzw[3];

    // Merge
    cv::merge(std::vector<cv::Mat>(xyzw.begin(), xyzw.begin()+3), xyz);

}

void Triangulator::triangulateFromVp(cv::Mat &vp, cv::Mat &xyz){

    // Solve for xyzw using determinant tensor
    cv::Mat C = determinantTensor;
    std::vector<cv::Mat> xyzw(4);
    for(unsigned int i=0; i<4; i++){
//        xyzw[i].create(vp.size(), CV_32F);
        xyzw[i] = C.at<float>(cv::Vec4i(i,0,1,1)) - C.at<float>(cv::Vec4i(i,2,1,1))*uc - C.at<float>(cv::Vec4i(i,0,2,1))*vc -
                C.at<float>(cv::Vec4i(i,0,1,2))*vp + C.at<float>(cv::Vec4i(i,2,1,2))*vp.mul(uc) + C.at<float>(cv::Vec4i(i,0,2,2))*vp.mul(vc);
    }

    // Convert to non homogenous coordinates
    for(unsigned int i=0; i<3; i++)
        xyzw[i] /= xyzw[3];

    // Merge
    cv::merge(std::vector<cv::Mat>(xyzw.begin(), xyzw.begin()+3), xyz);

}

void Triangulator::triangulateFromUpVp(cv::Mat &up, cv::Mat &vp, cv::Mat &xyz){

    std::cerr << "WARNING! NOT FULLY IMPLEMENTED!" << std::endl;
    int N = up.rows * up.cols;

    cv::Mat projPointsCam(2, N, CV_32F);
    uc.reshape(0,1).copyTo(projPointsCam.row(0));
    vc.reshape(0,1).copyTo(projPointsCam.row(1));

    cv::Mat projPointsProj(2, N, CV_32F);
    up.reshape(0,1).copyTo(projPointsProj.row(0));
    vp.reshape(0,1).copyTo(projPointsProj.row(1));

    cv::Mat Pc(3,4,CV_32F,cv::Scalar(0.0));
    cv::Mat(calibration.Kc).copyTo(Pc(cv::Range(0,3), cv::Range(0,3)));

    cv::Mat Pp(3,4,CV_32F), temp(3,4,CV_32F);
    cv::Mat(calibration.Rp).copyTo(temp(cv::Range(0,3), cv::Range(0,3)));
    cv::Mat(calibration.Tp).copyTo(temp(cv::Range(0,3), cv::Range(3,4)));
    Pp = cv::Mat(calibration.Kp) * temp;

    cv::Mat xyzw;
    cv::triangulatePoints(Pc, Pp, projPointsCam, projPointsProj, xyzw);

    xyz.create(3, N, CV_32F);
    for(int i=0; i<N; i++){
        xyz.at<float>(0,i) = xyzw.at<float>(0,i)/xyzw.at<float>(3,i);
        xyz.at<float>(1,i) = xyzw.at<float>(1,i)/xyzw.at<float>(3,i);
        xyz.at<float>(2,i) = xyzw.at<float>(2,i)/xyzw.at<float>(3,i);
    }

    xyz = xyz.t();
    xyz = xyz.reshape(3, up.rows);
}


