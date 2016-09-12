#include "CalibratorRBF.h"
#include "CodecCalibration.h"

#include "cvtools.h"
#include "RBFInterpolator.h"


CalibratorRBF::CalibratorRBF(unsigned int _screenCols, unsigned int _screenRows): Calibrator(_screenCols, _screenRows){

    // Create encoder/decoder
    encoder = new EncoderCalibration(screenCols, screenRows, CodecDirBoth);
    decoder = new DecoderCalibration(screenCols, screenRows, CodecDirBoth);

    this->N = encoder->getNPatterns();

    frameSeqs.resize(N);

    for(unsigned int i=0; i<N; i++)
        patterns.push_back(encoder->getEncodingPattern(i));

}

CalibrationData CalibratorRBF::calibrate(const int checkerSize, const int checkerRows, const int checkerCols){
    // Number of saddle points on calibration pattern
    cv::Size patternSize(checkerCols,checkerRows);

    // Number of calibration sequences
    unsigned nFrameSeq = frameSeqs.size();

    vector<cv::Mat> up(nFrameSeq), vp(nFrameSeq), shading(nFrameSeq), mask(nFrameSeq);

    // Decode frame sequences
    for(unsigned int i=0; i<nFrameSeq; i++){
        vector<cv::Mat> frames = frameSeqs[i];
        for(unsigned int f=0; f<frames.size(); f++){
            decoder->setFrame(f, frames[f]);
            #if 0
                cv::imwrite(QString("frames[%1].png").arg(f).toStdString(), frames[f]);
            #endif
        }
        decoder->decodeFrames(up[i], vp[i], mask[i], shading[i]);
    }

    unsigned int frameWidth = frameSeqs[0][0].cols;
    unsigned int frameHeight = frameSeqs[0][0].rows;

    // Generate local calibration object coordinates [mm]
    vector<cv::Point3f> Qi;
    for (int h=0; h<patternSize.height; h++)
        for (int w=0; w<patternSize.width; w++)
            Qi.push_back(cv::Point3f(checkerSize * w, checkerSize* h, 0.0));

    // Find calibration point coordinates for camera and projector
    vector< vector<cv::Point2f> > qc, qp;
    vector< vector<cv::Point3f> > Q;
    for(unsigned int i=0; i<nFrameSeq; i++){
        vector<cv::Point2f> qci;
        // Extract checker corners
        bool success = cv::findChessboardCorners(shading[i], patternSize, qci, cv::CALIB_CB_ADAPTIVE_THRESH);
        if(!success)
            std::cout << "Calibrator: could not extract chess board corners on frame seqence " << i << std::endl << std::flush;
        else
            // Refine corner locations
            cv::cornerSubPix(shading[i], qci, cv::Size(5, 5), cv::Size(-1, -1),cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001));

        // Draw colored chessboard
        cv::Mat shadingColor;
        cv::cvtColor(shading[i], shadingColor, cv::COLOR_GRAY2RGB);
        cv::drawChessboardCorners(shadingColor, patternSize, qci, success);
#if 0
    //dump calibration information
    cv::imwrite("shadingColor.png", shadingColor);
#endif
        // Emit chessboard results
        newSequenceResult(shadingColor, i, success);

        if(success){

            // Vectors of accepted points for current view
            vector<cv::Point2f> qpi_a;
            vector<cv::Point2f> qci_a;
            vector<cv::Point3f> Qi_a;

            // Loop through checkerboard corners
            for(unsigned int j=0; j<qci.size(); j++){

                const cv::Point2f &qcij = qci[j];

                // Collect neighbor points
                const unsigned int WINDOW_SIZE = 5;
                std::vector<cv::Point2f> N_qcij, N_qpij;

                // avoid going out of bounds
                unsigned int starth = max(int(qcij.y+0.5)-WINDOW_SIZE, 0u);
                unsigned int stoph  = min(int(qcij.y+0.5)+WINDOW_SIZE, frameHeight-1);
                unsigned int startw = max(int(qcij.x+0.5)-WINDOW_SIZE, 0u);
                unsigned int stopw  = min(int(qcij.x+0.5)+WINDOW_SIZE, frameWidth-1);

                for(unsigned int h=starth; h<=stoph; h++){
                    for(unsigned int w=startw; w<=stopw; w++){
                        // stay within mask
                        if(mask[i].at<bool>(h,w)){
                            N_qcij.push_back(cv::Point2f(w, h));

                            float upijwh = up[i].at<float>(h,w);
                            float vpijwh = vp[i].at<float>(h,w);
                            N_qpij.push_back(cv::Point2f(upijwh, vpijwh));
                        }
                    }
                }

                // if enough valid points to build interpolator
                if(N_qpij.size() >= 50){
                    RBFInterpolator *rbf = new RBFInterpolator(RBF_GAUSSIAN, 100.0);

                    // translate qcij into qpij using interpolator
                    rbf->setDataPoints(N_qcij, N_qpij);
                    cv::Point2f qpij = rbf->interpolate(N_qcij, qcij);

                    qpi_a.push_back(qpij);
                    qci_a.push_back(qci[j]);
                    Qi_a.push_back(Qi[j]);
                }
            }

            // Store projector corner coordinates
            qp.push_back(qpi_a);

            // Store camera corner coordinates
            qc.push_back(qci_a);

            // Store world corner coordinates
            Q.push_back(Qi_a);
        }
    }    

#if 0
    //dump calibration information
    for(unsigned int i=0; i<nFrameSeq; i++){
        cv::imwrite(QString("shading[%1].png").arg(i).toStdString(), shading[i]);
        cvtools::writeMat(up[i], QString("up[%1].mat").arg(i).toLocal8Bit());
        cvtools::writeMat(vp[i], QString("vp[%1].mat").arg(i).toLocal8Bit());
    }
#endif

    if(Q.size() < 1){
        std::cerr << "Error: not enough calibration sequences!" << std::endl;
        CalibrationData nanData;
        return nanData;
    }

    //calibrate the camera
    cv::Mat Kc, kc;
    std::vector<cv::Mat> cam_rvecs, cam_tvecs;
    cv::Size frameSize(frameWidth, frameHeight);
    double cam_error = cv::calibrateCamera(Q, qc, frameSize, Kc, kc, cam_rvecs, cam_tvecs, cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_PRINCIPAL_POINT + cv::CALIB_FIX_K2 + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST,
                                           cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

    //calibrate the projector
    cv::Mat Kp, kp;
    std::vector<cv::Mat> proj_rvecs, proj_tvecs;
    cv::Size screenSize(screenCols, screenRows);
    double proj_error = cv::calibrateCamera(Q, qp, screenSize, Kp, kp, proj_rvecs, proj_tvecs, cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K2 + cv::CALIB_FIX_K3 + cv::CALIB_ZERO_TANGENT_DIST,
                                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON));

    //stereo calibration
    cv::Mat Rp, Tp, E, F;
#ifdef OPENCV2
    double stereo_error = cv::stereoCalibrate(Q, qc, qp, Kc, kc, Kp, kp, frameSize, Rp, Tp, E, F,
                                              cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON));
#else // OPENCV2
    double stereo_error = cv::stereoCalibrate(Q, qc, qp, Kc, kc, Kp, kp, frameSize, Rp, Tp, E, F, cv::CALIB_FIX_INTRINSIC,
                                              cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, DBL_EPSILON));
#endif // OPENCV2

    CalibrationData calData(Kc, kc, cam_error, Kp, kp, proj_error, Rp, Tp, stereo_error);

    calData.print(std::cout);

    // Determine per-view reprojection errors:
    std::vector<float> cam_error_per_view(Q.size());
    cam_error_per_view.resize(Q.size());
    std::vector<float> proj_error_per_view(Q.size());
    proj_error_per_view.resize(Q.size());
    vector< vector<cv::Point2f> > qc_proj;
    qc_proj.resize(qc.size());
    vector< vector<cv::Point2f> > qp_proj;
    qp_proj.resize(qp.size());
//    for(unsigned int i = 0; i < (unsigned int)Q.size(); ++i){
//        int n = (int)Q[i].size();

//        cv::projectPoints(cv::Mat(Q[i]), cam_rvecs[i], cam_tvecs[i], Kc, kc, qc_proj[i]);
//        float err = cv::norm(cv::Mat(qc[i]), cv::Mat(qc_proj[i]), cv::L2);
//        cam_error_per_view[i] = (float)std::sqrt(err*err/n);

//        cv::projectPoints(cv::Mat(Q[i]), proj_rvecs[i], proj_tvecs[i], Kp, kp, qp_proj[i]);
//        err = cv::norm(cv::Mat(qp[i]), cv::Mat(qp_proj[i]), cv::L2);
//        proj_error_per_view[i] = (float)std::sqrt(err*err/n);

//        // Print information to std::out
//        std::cout << "Seq error " << i+1 << " cam:" << cam_error_per_view[i] << " proj:" << proj_error_per_view[i] << std::endl;
//    }

//    // Write all information to file
//    QString fileName = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmsszzz");
//    fileName.append(".xml");
//    cv::FileStorage fs(fileName.toStdString(), cv::FileStorage::WRITE);
//    for(unsigned int i = 0; i < (unsigned int)Q.size(); ++i){
//        fs << QString("Q_%1").arg(i).toStdString() << cv::Mat(Q[i]);
//        fs << QString("qc_%1").arg(i).toStdString() << cv::Mat(qc[i]);
//        fs << QString("qp_%1").arg(i).toStdString() << cv::Mat(qp[i]);
//        fs << QString("cam_rvecs_%1").arg(i).toStdString() << cv::Mat(cam_rvecs[i]);
//        fs << QString("cam_tvecs_%1").arg(i).toStdString() << cv::Mat(cam_tvecs[i]);
//        fs << QString("proj_rvecs_%1").arg(i).toStdString() << cv::Mat(proj_rvecs[i]);
//        fs << QString("proj_tvecs_%1").arg(i).toStdString() << cv::Mat(proj_tvecs[i]);
//        fs << QString("qc_proj_%1").arg(i).toStdString() << cv::Mat(qc_proj[i]);
//        fs << QString("qp_proj_%1").arg(i).toStdString() << cv::Mat(qp_proj[i]);
//        fs << QString("cam_error_per_view_%1").arg(i).toStdString() << cam_error_per_view[i];
//        fs << QString("proj_error_per_view_%1").arg(i).toStdString() << proj_error_per_view[i];
//    }
//    fs.release();

    return calData;

}
