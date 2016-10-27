#ifndef CODEC_H
#define CODEC_H

#include <vector>
#include <opencv2/opencv.hpp>

#include "slalgorithm_export.h"

enum CodecType {
    codecTypePhaseShift2x3,
    codecTypePhaseShift3,
    codecTypePhaseShift3FastWrap,
    codecTypePhaseShift3Unwrap,
    codecTypePhaseShift4,
    codecTypeGrayCode,
    codecTypePhaseShift2p1,
    codecTypeFastRatio,
    codecTypePhaseShiftModulated,
    codecTypePhaseShiftMicro,
    codecTypePhaseShiftNStep
};

enum CodecDir {CodecDirNone = 0,
               CodecDirHorizontal = 1 << 0,
               CodecDirVertical = 1 << 1,
               CodecDirBoth = CodecDirHorizontal | CodecDirVertical};

// Base class for all encoders
class SLALGORITHM_EXPORT Encoder {
    public:
        static Encoder* NewEncoder(CodecType codec, unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir = CodecDirHorizontal);
    
        Encoder(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir = CodecDirHorizontal) : N(0),screenCols(_screenCols), screenRows(_screenRows),  dir(_dir){}
        unsigned int getNPatterns(){return N;}
        CodecDir getDir(){return dir;}
        // Encoding
        virtual cv::Mat getEncodingPattern(unsigned int depth) = 0;
        virtual ~Encoder(){}
    protected:
        unsigned int N;
        unsigned int screenCols, screenRows;
        CodecDir dir;
};

class SLALGORITHM_EXPORT Decoder {
    public:
        static Decoder* NewDecoder(CodecType codec, unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir = CodecDirHorizontal);
        
        Decoder(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir = CodecDirHorizontal) : N(0), screenCols(_screenCols), screenRows(_screenRows),  dir(_dir){}
        unsigned int getNPatterns(){return N;}
        CodecDir getDir(){return dir;}
        // Decoding
        virtual void setFrame(unsigned int depth, const cv::Mat frame) = 0;
        virtual void decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading) = 0;
        virtual ~Decoder(){}
    protected:
        unsigned int N;
        unsigned int screenCols, screenRows;
        CodecDir dir;
//        cv::Mat lastShading();
};

#endif // CODEC_H
