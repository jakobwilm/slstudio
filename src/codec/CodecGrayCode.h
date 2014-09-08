#ifndef CodecGrayCode_H
#define CodecGrayCode_H

#include "Codec.h"

class EncoderGrayCode : public Encoder {
    public:
        EncoderGrayCode(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
        unsigned int Nhorz, Nvert;
};

class DecoderGrayCode : public Decoder {
    public:
        DecoderGrayCode(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, const cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
        unsigned int Nhorz, Nvert;
};

#endif // CodecGrayCode_H
