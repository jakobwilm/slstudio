#ifndef CODECPHASESHIFT3UNWRAP_H
#define CODECPHASESHIFT3UNWRAP_H

#include "Codec.h"

class SLALGORITHM_EXPORT EncoderPhaseShift3Unwrap : public Encoder {
    public:
        EncoderPhaseShift3Unwrap(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
};

class SLALGORITHM_EXPORT DecoderPhaseShift3Unwrap : public Decoder {
    public:
        DecoderPhaseShift3Unwrap(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
};

#endif // CODECPHASESHIFT3UNWRAP_H
