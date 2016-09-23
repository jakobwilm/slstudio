#ifndef CODECPHASESHIFT2X3_H
#define CODECPHASESHIFT2X3_H

#include "Codec.h"

class SLALGORITHM_EXPORT EncoderPhaseShift2x3 : public Encoder {
    public:
        EncoderPhaseShift2x3(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
};

class SLALGORITHM_EXPORT DecoderPhaseShift2x3 : public Decoder {
    public:
        DecoderPhaseShift2x3(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
};

#endif // CODECPHASESHIFT2X3_H
