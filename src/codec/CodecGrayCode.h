#ifndef CodecGrayCode_H
#define CodecGrayCode_H

#include "Codec.h"

class SLALGORITHM_EXPORT EncoderGrayCode : public Encoder {
    public:
        EncoderGrayCode(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
};

class SLALGORITHM_EXPORT DecoderGrayCode : public Decoder {
    public:
        DecoderGrayCode(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, const cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
};

#endif // CodecGrayCode_H
