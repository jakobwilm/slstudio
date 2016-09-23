#ifndef CODECPhaseShiftNStep_H
#define CODECPhaseShiftNStep_H

#include "Codec.h"

// 8 step phase shifting codec with phase unwrapping

class SLALGORITHM_EXPORT EncoderPhaseShiftNStep : public Encoder {
    public:
        EncoderPhaseShiftNStep(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Encoding
        cv::Mat getEncodingPattern(unsigned int depth);
    private:
        std::vector<cv::Mat> patterns;
};

class SLALGORITHM_EXPORT DecoderPhaseShiftNStep : public Decoder {
    public:
        DecoderPhaseShiftNStep(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir);
        // Decoding
        void setFrame(unsigned int depth, cv::Mat frame);
        void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading);
    private:
        std::vector<cv::Mat> frames;
};

#endif // CODECPhaseShiftNStep_H
