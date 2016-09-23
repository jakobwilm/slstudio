#ifndef CODECPHASESHIFT4_H
#define CODECPHASESHIFT4_H

#include "Codec.h"

class SLALGORITHM_EXPORT EncoderPhaseShift4 : public Encoder {
    public:
        EncoderPhaseShift4(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
};

class SLALGORITHM_EXPORT DecoderPhaseShift4 : public Decoder {
    public:
        DecoderPhaseShift4(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
};

#endif // CODECPHASESHIFT4_H
