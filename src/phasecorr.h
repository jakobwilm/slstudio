
#include <opencv2/opencv.hpp>

namespace phasecorrelation{

    using namespace cv;

    static void magSpectrums( InputArray _src, OutputArray _dst);
    static void divSpectrums( InputArray _srcA, InputArray _srcB, OutputArray _dst, int flags, bool conjB);
    static void fftShift(InputOutputArray _out);
    static Point2d weightedCentroid(InputArray _src, cv::Point peakLocation, cv::Size weightBoxSize, double* response);
    cv::Point2d phaseCorrelate(InputArray _src1, InputArray _src2, InputArray _window, double* response = NULL);
    void createHanningWindow(OutputArray _dst, cv::Size winSize, int type);

}
