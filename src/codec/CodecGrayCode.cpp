#include "CodecGrayCode.h"
#include <cmath>
#include <iomanip>

#include "cvtools.h"

static unsigned int Nhorz = 10;
static unsigned int Nvert = 6;

#ifndef log2f
#define log2f(x) (log(x)/log(2.0))
#endif

using namespace std;

/*
 * The purpose of this function is to convert an unsigned
 * binary number to reflected binary Gray code.
 *
 * The operator >> is shift right. The operator ^ is exclusive or.
 * Source: http://en.wikipedia.org/wiki/Gray_code
 */
static unsigned int binaryToGray(unsigned int num) {
    return (num >> 1) ^ num;
}

/*
 * From Wikipedia: http://en.wikipedia.org/wiki/Gray_code
 * The purpose of this function is to convert a reflected binary
 * Gray code number to a binary number.
 */
static unsigned grayToBinary(unsigned num, unsigned numBits)
{
    for (unsigned shift = 1; shift < numBits; shift <<= 1){
        num ^= num >> shift;
    }
    return num;
}

/*
 * Function takes the decimal number
 * Function takes the Nth bit (1 to 31)
 * Return the value of Nth bit from decimal
 * Source: http://icfun.blogspot.com/2009/04/get-n-th-bit-value-of-any-integer.html
 */
static int get_bit(int decimal, int N){

    // Shifting the 1 for N-1 bits
    int constant = 1 << (N-1);

    // If the bit is set, return 1
    if( decimal & constant ){
        return 1;
    }

    // If the bit is not set, return 0
    return 0;
}

static inline int powi(int num, unsigned int exponent){
    // NOT EQUIVALENT TO pow()
    if(exponent == 0)
        return 1;

    float res = num;
    for(unsigned int i=0; i<exponent-1; i++)
        res *= num;

    return res;
}

// Encoder
EncoderGrayCode::EncoderGrayCode(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Encoder(_screenCols, _screenRows, _dir){

    N = 2;

    // Set total pattern number
    if(dir & CodecDirHorizontal)
        this->N += Nhorz;

    if(dir & CodecDirVertical)
        this->N += Nvert;

    // Encode every pixel column
    int NbitsHorz = ceilf(log2f((float)screenCols));

    // Number of vertical encoding patterns
    int NbitsVert = ceilf(log2f((float)screenRows));

    cv::Mat patternOn(1, 1, CV_8UC3, cv::Scalar(0));
    patternOn.at<cv::Vec3b>(0,0) = cv::Vec3b(255, 255, 255);
    patterns.push_back(patternOn);

    cv::Mat patternOff(1, 1, CV_8UC3, cv::Scalar(0));
    patterns.push_back(patternOff);

    if(dir & CodecDirHorizontal){
        // Precompute horizontally encoding patterns
        for(unsigned int p=0; p<Nhorz; p++){
            cv::Mat patternP(1, screenCols, CV_8UC3);
            // Loop through columns in first row
            for(unsigned int j=0; j<screenCols; j++){
                unsigned int jGray = binaryToGray(j);
                // Amplitude of channels
                float amp = get_bit(jGray, NbitsHorz-p);
                patternP.at<cv::Vec3b>(0,j) = cv::Vec3b(255.0*amp,255.0*amp,255.0*amp);
            }
            patterns.push_back(patternP);
        }
    }
    if(dir & CodecDirVertical){
        // Precompute vertical encoding patterns
        for(unsigned int p=0; p<Nvert; p++){
            cv::Mat patternP(screenRows, 1, CV_8UC3);

            // Loop through rows in first column
            for(unsigned int i=0; i<screenRows; i++){

                unsigned int iGray = binaryToGray(i);

                // Amplitude of channels
                float amp = get_bit(iGray, NbitsVert-p);
                patternP.at<cv::Vec3b>(i,0) = cv::Vec3b(255.0*amp,255.0*amp,255.0*amp);
            }
            patterns.push_back(patternP);
        }
    }

    #if 0
        for(unsigned int i=0; i<patterns.size(); i++){
            std::stringstream fileNameStream;
            fileNameStream << "pattern_" << std::setw(2) << std::setfill('0') << i << ".bmp";
            cv::imwrite(fileNameStream.str(), patterns[i]);
        }

    #endif
}

cv::Mat EncoderGrayCode::getEncodingPattern(unsigned int depth){
    return patterns[depth];
}

// Decoder
DecoderGrayCode::DecoderGrayCode(unsigned int _screenCols, unsigned int _screenRows, CodecDir _dir) : Decoder(_screenCols, _screenRows, _dir){

    N = 2;

    if(dir & CodecDirHorizontal)
        this->N += Nhorz;

    if(dir & CodecDirVertical)
        this->N += Nvert;

    frames.resize(N);

}

void DecoderGrayCode::setFrame(unsigned int depth, const cv::Mat frame){
    frames[depth] = frame;
}

void DecoderGrayCode::decodeFrames(cv::Mat &up, cv::Mat &vp, cv::Mat &mask, cv::Mat &shading){

    // min image
    cv::Mat minImage = frames[1];

    // shading image
    shading = frames[0] - minImage;

    // Threshold shading image for mask
    mask = (shading > 20);

    // Binarize frames. TODO: subpixel interpolation.
    vector<cv::Mat> framesBinary(frames.size());
    for(unsigned int i=0; i<frames.size(); i++){
        // Foreground pixels 1, background 0
        //cv::threshold(frames[i], framesBinary[i], 80, 1, cv::THRESH_BINARY);
        framesBinary[i].create(frames[0].size(), CV_8U);
        framesBinary[i] = cv::abs(shading-frames[i]) < cv::abs(frames[i]-minImage);
        cv::threshold(framesBinary[i], framesBinary[i], 1, 1, cv::THRESH_BINARY);
    }

    // Encode every pixel column
    int NbitsHorz = ceilf(log2f((float)screenCols));

    // Number of vertical encoding patterns
    int NbitsVert = ceilf(log2f((float)screenRows));

    if(dir & CodecDirHorizontal){
        vector<cv::Mat> framesHorz(framesBinary.begin()+2, framesBinary.begin()+Nhorz+2);

        // Construct up image.
        for(int i = 0; i < up.rows; i++){
            for(int j = 0; j < up.cols; j++){
                unsigned int enc = 0;
                std::size_t maxF = std::min(std::size_t(NbitsHorz), framesHorz.size());
                for(unsigned int f=0; f<maxF; f++){
                    // Gray decimal
                    enc += powi(2, NbitsHorz-f-1)*framesHorz[f].at<unsigned char>(i,j);
                }
                // Standard decimal
                enc = grayToBinary(enc, Nhorz);
                up.at<float>(i,j) = enc;

            }
        }
    }

//    cvtools::writeMat(up, "up.mat", "up");

    if(dir & CodecDirVertical){
        vector<cv::Mat> framesVert(framesBinary.end()-Nvert, framesBinary.end());

        // Construct vp image.
        for(int i = 0; i < vp.rows; i++){
            for(int j = 0; j < vp.cols; j++){
                unsigned int enc = 0;
                std::size_t maxF = std::min(std::size_t(NbitsVert), framesVert.size());
                for(unsigned int f=0; f<maxF; f++){
                    // Gray decimal
                    enc += powi(2, NbitsVert-f-1)*framesVert[f].at<unsigned char>(i,j);
                }
                // Standard decimal
                enc = grayToBinary(enc, Nvert);
                vp.at<float>(i,j) = enc;
            }
        }
    }
}
