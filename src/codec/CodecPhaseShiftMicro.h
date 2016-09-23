#ifndef CODECPhaseShiftMicro_H
#define CODECPhaseShiftMicro_H

#include "Codec.h"

class SLALGORITHM_EXPORT EncoderPhaseShiftMicro : public Encoder {
    public:
        EncoderPhaseShiftMicro(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
};

class SLALGORITHM_EXPORT DecoderPhaseShiftMicro : public Decoder {
    public:
        DecoderPhaseShiftMicro(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
};

#endif // CODECPhaseShiftMicro_H
