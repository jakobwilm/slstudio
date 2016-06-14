#include "CodecPhaseShiftMicro.h"
#include <math.h>

#include "cvtools.h"
#include "pstools.h"


#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

static const unsigned int F = 10;
//static const float frequencies[] = {32.54, 30.00, 30.44, 30.68, 34.04, 34.34, 35.99};
//static const float frequencies[] = {74, 70, 71, 72, 73, 75, 76, 77, 78, 79};
static const float frequencies[] = {75.02, 70.00, 71.32, 72.47, 73.72, 76.23, 77.35, 78.40, 79.22, 80.00};

// Encoder
EncoderPhaseShiftMicro::EncoderPhaseShiftMicro(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    N = F+2;

    // Precompute encoded patterns
    const float pi = M_PI;

    // Main frequency encoding patterns
    for(unsigned int i=0; i<3; i++){
        float phase = -2.0*pi/3.0 * i;
        float pitch = frequencies[0];
        cv::Mat patternI(1,1,CV_8U);
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patternI = patternI.t();
        patterns.push_back(patternI);
    }

    // Additional frequency patterns
    for(unsigned int i=1; i<F; i++){
        float phase = 0.0;
        if(i%2)
            phase = pi;
        float pitch = frequencies[i];
        cv::Mat patternI;
        patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patternI = patternI.t();
        patterns.push_back(patternI);
    }
   cvtools::writeMat(patterns[8], "patterns8.mat");
}

cv::Mat EncoderPhaseShiftMicro::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

// Decoder
DecoderPhaseShiftMicro::DecoderPhaseShiftMicro(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows, _dir){

    N = F+2;

    frames.resize(N);
}

void DecoderPhaseShiftMicro::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderPhaseShiftMicro::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    const float pi = M_PI;

//    std::vector<cv::Mat> framesMain(frames.begin(), frames.begin()+3);

//    // Horizontal decoding
//    up = pstools::getPhase(framesMain[0], framesMain[1], framesMain[2]);
//    shading = pstools::getMagnitude(framesMain[0], framesMain[1], framesMain[2]);

//    mask = shading > 25;

    int rows = frames[0].rows;
    int cols = frames[0].cols;

    // Construct system of equations
    cv::Mat Mmicro(F+2, F+2, CV_32F, cv::Scalar(0.0));
    Mmicro.at<float>(0,1) = 1.0;
    Mmicro.at<float>(0,2) = 0.0;
    Mmicro.at<float>(1,1) = cos(2.0*pi/3.0);
    Mmicro.at<float>(1,2) = -sin(2.0*pi/3.0);
    Mmicro.at<float>(2,1) = cos(4.0*pi/3.0);
    Mmicro.at<float>(2,2) = -sin(4.0*pi/3.0);
    Mmicro.col(0).setTo(1.0);
    for(int i=0; i<F-1; i++)
        Mmicro.at<float>(3+i, 3+i) = (i % 2)*2-1;

    cv::Mat Rmicro(F+2, rows*cols, CV_32F);
    for(int i=0; i<F+2; i++){
        frames[i].reshape(0, 1).copyTo(Rmicro.row(i));
    }

    // Solve
    cv::Mat Ufact;
    cv::solve(Mmicro, Rmicro, Ufact);

    // Shading
    cv::Mat amp;
    cv::magnitude(Ufact.row(1), Ufact.row(2), amp);
    shading = amp.reshape(0, rows);
    shading.convertTo(shading, CV_8U, 2.0);
    mask = shading > 20;

    cv::Mat CosSin(F+1, rows*cols, CV_32F);
    for(int i=0; i<F+1; i++){
        cv::divide(Ufact.row(i+1), amp, CosSin.row(i));
    }

//    cvtools::writeMat(CosSin, "CosSin.mat", "CosSin");

    // Reference CosSin values
    cv::Mat RefCosSin(F+1, screenCols, CV_32F);
    for(int i=0; i<screenCols; i++){
        RefCosSin.at<float>(0,i) = cos(2*pi*i/frequencies[0]);
        RefCosSin.at<float>(1,i) = sin(2*pi*i/frequencies[0]);
        for(int j=2; j<F+1; j++){
            RefCosSin.at<float>(j,i) = cos(2*pi*i/frequencies[j-1]);
        }
    }

//    cvtools::writeMat(RefCosSin, "RefCosSin.mat", "RefCosSin");

    // Find best match value
    cv::Mat upCueMatch(1, rows*cols, CV_32F);
    cv::Mat bestDistMatch(1, rows*cols, CV_32F);
    for(int i=0; i<rows*cols; i++){
        int bestMatch = -1;
        float bestDist = std::numeric_limits<float>::infinity();
        for(int j=0; j<screenCols; j++){
//            float dist = cv::norm(CosSin.col(i) - RefCosSin.col(j), cv::NORM_L2SQR);
            float dist = 0.0;
            for(int k=0; k<F+1; k++){
                float diff = CosSin.at<float>(k, i)-RefCosSin.at<float>(k, j);
                dist += diff*diff;
            }
            if(dist < bestDist){
                bestDist = dist;
                bestMatch = j;
            }
        }
        upCueMatch.at<float>(0, i) = bestMatch;
        bestDistMatch.at<float>(0, i) = bestDist;
    }

    cv::Mat upCue = upCueMatch.reshape(0, rows);

    bestDistMatch = bestDistMatch.reshape(0, rows);
    cvtools::writeMat(bestDistMatch, "bestDistMatch.mat", "bestDistMatch");

    upCue *= (2*pi)/screenCols;
    up = pstools::getPhase(frames[0], frames[2], frames[1]);
    up = pstools::unwrapWithCue(up, upCue, screenCols/frequencies[0]);
    up *= screenCols/(2*pi);
//up = upCue;
//    cvtools::writeMat(up, "up.mat", "up");

}
