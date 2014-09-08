#include "RBFInterpolator.h"

#include "cvtools.h"

inline float psiGaussian(const cv::Point2f p1, const cv::Point2f p2){
    float rSqr = (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y);

    if (rSqr == 0.0)
        return 1.0;
    else
        return std::exp(-0.2*rSqr);
}

inline float psiThinPlateSpline(const cv::Point2f p1, const cv::Point2f p2){
    float rSqr = (p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y);

    if (rSqr == 0.0)
        return 0.0;
    else
        return (rSqr * std::log(sqrt(rSqr)));
}

void RBFInterpolator::setDataPoints(const std::vector<cv::Point2f> x, const std::vector<cv::Point2f> f){

    unsigned int dim = 2;
    unsigned int N = x.size();

    // basis function matrix
    cv::Mat_<float> Psi(N, N, 0.0);
    for (unsigned int i = 0; i < N; i++){
        for (unsigned int j = 0; j <= i; j++){
            float psi = psiGaussian(x[i], x[j]);
            Psi(i,j) = psi;
            Psi(j,i) = psi;
        }
    }

    // polynomial matrix P
    cv::Mat_<float> P(N, dim+1, 1.0);
    for (unsigned int i = 0; i < N; i++){
        P(i, 0) = 1;
        P(i, 1) = x[i].x;
        P(i, 2) = x[i].y;
    }

    // complete coefficient matrix
    cv::Mat_<float> A(N+dim+1, N+dim+1, 0.0);
    A(cv::Range(0, N), cv::Range(0, N)) = Psi + cv::Mat::eye(N, N, CV_32F)*regularizationK; // add regularization
    P.copyTo(A(cv::Range(0, N), cv::Range(N, N+dim+1))); // needs copyTo because P is not a temporary
    P = P.t();
    P.copyTo(A(cv::Range(N, N+dim+1), cv::Range(0, N)));
//cvtools::writeMat(A, "A.mat");
    // RHS
    cv::Mat_<float> b(N+dim+1, dim, 0.0);
    for (unsigned int i = 0; i < N; i++){
        b(i,0) = f[i].x;
        b(i,1) = f[i].y;
    }

    cv::solve(A, b, lambda, cv::DECOMP_LU);
//cvtools::writeMat(lambda, "lambda.mat");
}

cv::Point2f RBFInterpolator::interpolate(const std::vector<cv::Point2f> x, const cv::Point2f xStar){

    unsigned int N = x.size();
    cv::Point2f fTilde(0.0, 0.0);

    for(unsigned int i=0; i<N; i++){
        float psi = psiGaussian(x[i], xStar);

        fTilde.x += lambda(i,0)*psi;
        fTilde.y += lambda(i,1)*psi;
    }

    fTilde.x += lambda(N,0) + xStar.x*lambda(N+1,0) + xStar.y*lambda(N+2,0);
    fTilde.y += lambda(N,1) + xStar.x*lambda(N+1,1) + xStar.y*lambda(N+2,1);

    return fTilde;
}
