#ifndef CODECPhaseShiftModulated_H
#define CODECPhaseShiftModulated_H

#include "Codec.h"

class SLALGORITHM_EXPORT EncoderPhaseShiftModulated : public Encoder {
    public:
        EncoderPhaseShiftModulated(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
};

class SLALGORITHM_EXPORT DecoderPhaseShiftModulated : public Decoder {
    public:
        DecoderPhaseShiftModulated(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
};

#endif // CODECPhaseShiftModulated_H
