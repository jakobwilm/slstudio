
#include <opencv2/opencv.hpp>

cv::Mat DLP3000InterpImage(cv::Mat I){
    //DLP3000INTERPIMAGE Interpolates a 684 x 608 image for projection on the DLP3000
    //diamond pixel array from a FWVGA image at 853 x 480.
    //
    //  Input parameter:
    //      C (853 x 480 pixels): input image of type double
    //
    //  Output parameter:
    //      I (684 x 608 matrix): output image
    //
    //  Note that using a 853 x 480 provides the correct aspect ratio for projecting on the DLP3000.
    //  Note that the code assumes that the image is projected 1:1 aka "short-axis flip".
    //
    //  DTU 2013, Jakob Wilm

    cv::Mat ret(684, 608, CV_8U);

    if(I.rows != 480 || I.cols != 853){
        std::cerr << "Input image has wrong dimensions" << std::endl;
        return ret;
    }

    // interpolation points
    float mdiag = 853.0/607.5; // mirror diagonal in input pixels

    cv::Mat mapX(684, 608, CV_32F), mapY(684, 608, CV_32F);

    for(unsigned int row=0; row<684; row++){
        for(unsigned int col=0; col<608; col++){
            mapY.at<float>(row,col) = row * 479.0/683.0;
            mapX.at<float>(row,col) = col*(852.0-0.5*mdiag)/607.0 + (row%2)*0.5*mdiag;
        }
    }

    cv::remap(I, ret, mapX, mapY, cv::INTER_LINEAR);

    return ret;
}
