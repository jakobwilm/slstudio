#ifndef CODECFastRatio_H
#define CODECFastRatio_H

#include "Codec.h"

class SLALGORITHM_EXPORT EncoderFastRatio : public Encoder {
    public:
        EncoderFastRatio(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
};

class SLALGORITHM_EXPORT DecoderFastRatio : public Decoder {
    public:
        DecoderFastRatio(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
};

#endif // CODECFastRatio_H
