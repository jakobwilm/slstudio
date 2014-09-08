#ifndef CODECPhaseShift2p1_H
#define CODECPhaseShift2p1_H

#include "Codec.h"

class EncoderPhaseShift2p1 : public Encoder {
    public:
        EncoderPhaseShift2p1(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
};

class DecoderPhaseShift2p1 : public Decoder {
    public:
        DecoderPhaseShift2p1(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
        cv::Mat_<float> *lastShading;
};

#endif // CODECPhaseShift2p1_H
