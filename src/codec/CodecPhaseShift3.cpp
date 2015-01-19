#include "CodecPhaseShift3.h"
#include <math.h>

#include "pstools.h"
#include "cvtools.h"

#ifndef M_PI
    #define M_PI 3.14159265359
#endif

// Encoder
EncoderPhaseShift3::EncoderPhaseShift3(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    N = 3;

    // Precompute encoded patterns
    const float pi = M_PI;
    for(unsigned int i=0; i<N; i++){
        float phase = 2.0*pi/float(N) * i;
        float pitch = screenCols;

//        cv::Mat patternI = pstools::computePhaseVector(screenCols, phase, pitch);

        // Loop through vector
        cv::Mat phaseVector(screenCols, 1, CV_32F);
        for(int i=0; i<phaseVector.rows; i++){
            // Amplitude of channels
            float amp = 0.5*(1+cos(2*pi*i/pitch - phase));
            phaseVector.at<float>(i, 0) = amp;
        }

        phaseVector = phaseVector.t();

        // Repeat pattern
        phaseVector = cv::repeat(phaseVector, screenRows, 1);

        // Add noise
        cv::Mat noise(phaseVector.size(), CV_32F);
        cv::randn(noise, 0.0, 0.05);
        phaseVector += noise;

        phaseVector.convertTo(phaseVector, CV_8U, 255.0);

        // Repeat texture
        cv::Mat patternI(screenRows, screenCols, CV_8UC3);
        cv::cvtColor(phaseVector, patternI, cv::COLOR_GRAY2RGB);
        patterns.push_back(patternI);
    }
}

cv::Mat EncoderPhaseShift3::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

// Decoder
DecoderPhaseShift3::DecoderPhaseShift3(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows){
    N = 3;
    frames.resize(N);
}

void DecoderPhaseShift3::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderPhaseShift3::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    const float pi = M_PI;

    up = pstools::getPhase(frames[0], frames[1], frames[2]);

//    cvtools::writeMat(frames[0], "frames[0].mat");
//    cvtools::writeMat(frames[1], "frames[1].mat");
//    cvtools::writeMat(frames[2], "frames[2].mat");

    up *= screenCols/(2*pi);

//    cv::Mat upCopy = up.clone();
//    cv::bilateralFilter(upCopy, up, 7, 500, 400);
    cv::GaussianBlur(up, up, cv::Size(0,0), 3, 3);

    shading = pstools::getMagnitude(frames[0], frames[1], frames[2]);

    cv::Mat avg = 0.333*frames[0] + 0.333*frames[1] + 0.333*frames[2];

    // Create mask from modulation image and erode
    mask.create(shading.size(), cv::DataType<bool>::type);
    mask = (shading > avg) & (shading > 100) & (shading < 250);

    cv::Mat dx, dy;
    cv::Sobel(cv::Mat_<float>(shading), dx, -1, 1, 0, 3);
    cv::Sobel(cv::Mat_<float>(shading), dy, -1, 0, 1, 3);
    cv::Mat edgesShading = abs(dx) + abs(dy);
//    cv::magnitude(dx, dy, edgesShading);

    cv::Sobel(up, dx, -1, 1, 0, 3);
    cv::Sobel(up, dy, -1, 0, 1, 3);
    cv::Mat edgesUp = abs(dx) + abs(dy);
//    cv::magnitude(dx, dy, edgesUp);

//cvtools::writeMat(edges, "edges.mat", "edges");
    mask = mask &  (edgesUp < 80);

}
