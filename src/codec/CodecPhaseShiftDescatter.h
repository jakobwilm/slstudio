#ifndef CODECPhaseShiftDescatter_H
#define CODECPhaseShiftDescatter_H

#include "Codec.h"

class SLALGORITHM_EXPORT EncoderPhaseShiftDescatter : public Encoder {
    public:
        EncoderPhaseShiftDescatter(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
};

class SLALGORITHM_EXPORT DecoderPhaseShiftDescatter : public Decoder {
    public:
        DecoderPhaseShiftDescatter(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
};

#endif // CODECPhaseShiftDescatter_H
