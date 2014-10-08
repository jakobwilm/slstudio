#include "CodecPhaseShift4.h"
#include <math.h>

#include "cvtools.h"
#include "pstools.h"

#ifndef M_PI
    #define M_PI 3.14159265359
#endif

// Encoder
EncoderPhaseShift4::EncoderPhaseShift4(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    // Set N
    N = 4;

    // Precompute encoded patterns
    const float pi = M_PI;
    for(unsigned int i=0; i<N; i++){
        float phase = 2.0*pi/float(N) * i;
        float pitch = screenCols;
        cv::Mat patternI = pstools::computePhaseVector(screenCols, phase, pitch);
        patterns.push_back(patternI.t());
    }
}

cv::Mat EncoderPhaseShift4::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

// Decoder
DecoderPhaseShift4::DecoderPhaseShift4(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows){
    N = 4;
    frames.resize(N);
}

void DecoderPhaseShift4::setFrame(unsigned int depth, cv::Mat frame){
    frames[depth] = frame;
}

void DecoderPhaseShift4::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    const float pi = M_PI;

    std::vector<cv::Mat> fIcomp = pstools::getDFTComponents(frames);

    cv::phase(fIcomp[2], -fIcomp[3], up);
    up *= screenCols/(2.0*pi);

//    cv::Mat upCopy = up.clone();
//    cv::bilateralFilter(upCopy, up, 7, 500, 400);
    cv::GaussianBlur(up, up, cv::Size(0,0), 3, 3);

    cv::Mat X0, X1, X2;
    cv::magnitude(fIcomp[0], fIcomp[1], X0);
    cv::magnitude(fIcomp[2], fIcomp[3], X1);
    cv::magnitude(fIcomp[4], fIcomp[5], X2);

    // Threshold on high modulation and low energy at wrong frequencies
    mask = (X1/X0 > 0.30) & (X1 > 100) & (X2 < 50);

    cv::Mat dx, dy;
    cv::Sobel(up, dx, -1, 1, 0, 3);
    cv::Sobel(up, dy, -1, 0, 1, 3);
    cv::Mat edges;
    cv::magnitude(dx, dy, edges);

    mask = mask & (abs(edges) < 75);

    shading = X1;
    shading.convertTo(shading, CV_8U);
}
